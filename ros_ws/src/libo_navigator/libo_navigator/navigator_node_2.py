#!/usr/bin/env python3
import rclpy
import yaml
import math
import heapq
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus # GoalStatus를 action_msgs.msg에서 가져옵니다.
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# libo_interfaces에 정의된 우리만의 서비스 타입들을 import합니다.
from libo_interfaces.srv import SetGoal, CancelNavigation 
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from enum import Enum
from ament_index_python.packages import get_package_share_directory

class NavigatorState(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    ERROR = "ERROR"

class LiboNavigator(Node):
    def __init__(self):
        super().__init__('libo_navigator')
        
        self.current_state = NavigatorState.IDLE
        self.current_goal_pose = None
        self.waypoints = {}
        self.robot_current_pose = None
        self.initial_pose_received = False
        self.nav_goal_handle = None # 현재 진행중인 Nav2 임무 핸들
        self.status_check_timer = None  # 타이머 초기화

        # BasicNavigator 초기화 (중요!)
        self.navigator = BasicNavigator()

        # 여러 콜백(서비스, 구독)이 동시에 처리될 수 있도록 ReentrantCallbackGroup을 사용합니다.
        self.callback_group = ReentrantCallbackGroup()
        
        # ActionClient는 제거 (BasicNavigator가 내부적으로 처리)
        # self.nav_action_client = ActionClient(
        #     self, FollowWaypoints, 'follow_waypoints', callback_group=self.callback_group
        # )
        
        amcl_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, amcl_qos_profile,
            callback_group=self.callback_group
        )
        
        # Costmap 구독 설정 (장애물 감지용)
        from nav2_msgs.msg import Costmap
        self.create_subscription(
            Costmap,
            '/local_costmap/costmap',
            self.costmap_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 동적 재계획 관련 변수들
        self.current_costmap = None
        self._replanning = False
        self.blocked_waypoints = set()  # 막힌 웨이포인트들을 추적
        
        # --- 외부와 통신하기 위한 서비스 서버들을 생성합니다 ---
        self.create_service(
            SetGoal, 'set_navigation_goal', self.set_goal_callback, callback_group=self.callback_group
        )
        self.create_service(
            CancelNavigation, 'cancel_navigation', self.cancel_navigation_callback, callback_group=self.callback_group
        )
        # ---

        self.load_waypoints()
        self.get_logger().info('Libo Navigator 서비스 시작. /set_navigation_goal 요청 대기 중...')

    def amcl_pose_callback(self, msg):
        """AMCL로부터 로봇의 현재 위치를 계속해서 업데이트합니다."""
        if msg.pose.covariance[0] > 0.25:
            return
        self.robot_current_pose = msg.pose.pose
        if not self.initial_pose_received:
            self.get_logger().info('AMCL로부터 유효한 첫 위치 정보를 받았습니다! 시스템 준비 완료.')
            self.initial_pose_received = True

    # --- 서비스 콜백 함수들 (우리 노드의 새로운 '메인 진입점') ---
    def set_goal_callback(self, request, response):
        """[API] 외부로부터 목표 지점 요청을 받으면 이 함수가 실행됩니다."""
        if self.current_state == NavigatorState.NAVIGATING:
            self.get_logger().error("이미 다른 주행 임무가 진행 중입니다. 새 목표를 거부합니다.")
            response.success = False
            response.message = "이미 주행 중"
            return response

        if not self.initial_pose_received:
            self.get_logger().error("아직 로봇의 위치가 파악되지 않았습니다. 목표를 설정할 수 없습니다.")
            response.success = False
            response.message = "로봇 위치 미파악"
            return response

        # 요청받은 x, y 좌표로 목표 Pose 생성
        goal_pose = Pose()
        goal_pose.position.x = request.x
        goal_pose.position.y = request.y
        goal_pose.orientation.w = 1.0
        self.current_goal_pose = goal_pose

        self.get_logger().info(f'새로운 목표 수신: ({request.x:.2f}, {request.y:.2f})')
        
        path_found = self.plan_and_navigate()
        response.success = path_found
        response.message = "경로 탐색 및 주행 시작" if path_found else "경로 탐색 실패"
        return response

    def cancel_navigation_callback(self, request, response):
        """[API] 외부로부터 주행 취소 요청을 받으면 이 함수가 실행됩니다."""
        if self.current_state != NavigatorState.NAVIGATING:
            response.success = False
            response.message = "현재 진행 중인 주행이 없습니다."
            return response

        self.get_logger().warn("외부 요청에 의해 현재 주행을 취소합니다...")
        
        # 타이머가 존재하면 먼저 정리
        if hasattr(self, 'status_check_timer') and self.status_check_timer is not None:
            self.destroy_timer(self.status_check_timer)
            self.status_check_timer = None
        
        # BasicNavigator를 통해 취소
        self.navigator.cancelTask()
        
        # 상태를 즉시 IDLE로 변경
        self.current_state = NavigatorState.IDLE
        
        # navigation.cancelTask()가 호출되면, 진행중인 followWaypoints는 CANCELED 상태가 됩니다.
        # navigation_result_callback이 자동으로 호출되어 상태를 IDLE로 변경할 것입니다.
        
        response.success = True
        response.message = "주행 취소 요청을 보냈습니다."
        return response

    # --- 경로 계획 및 주행 로직 ---
    def plan_and_navigate(self):
        """경로를 계산하고 실제 주행을 시작하는 통합 함수입니다."""
        start_wp = self.get_closest_waypoint(self.robot_current_pose)
        goal_wp = self.get_closest_waypoint(self.current_goal_pose)
        
        if not start_wp or not goal_wp:
            self.get_logger().error(f"시작({start_wp}) 또는 목표({goal_wp}) 웨이포인트를 찾을 수 없습니다.")
            return False

        path_wp_names = self.find_path_astar(start_wp, goal_wp)
        
        if not path_wp_names:
            self.get_logger().error("최적 경로를 찾지 못했습니다!")
            return False
            
        self.get_logger().info(f"계산된 최적 경로: {path_wp_names}")
        
        waypoint_poses = []
        for name in path_wp_names:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = self.waypoints[name]['position']['x']
            pose.pose.position.y = self.waypoints[name]['position']['y']
            pose.pose.orientation.w = 1.0
            waypoint_poses.append(pose)
        
        self.start_navigation(waypoint_poses)
        return True

    def start_navigation(self, waypoint_poses):
        """Nav2에 웨이포인트 주행을 요청합니다."""
        self.get_logger().info(f"{len(waypoint_poses)}개의 지점으로 주행을 시작합니다...")
        self.current_state = NavigatorState.NAVIGATING
        
        self.navigator.followWaypoints(waypoint_poses)
        # isTaskComplete()를 주기적으로 확인하여 완료 시 콜백을 직접 호출하는 방식으로 변경
        self.status_check_timer = self.create_timer(1.0, self.check_navigation_status)

    def check_navigation_status(self):
        """1초마다 Nav2 주행 상태를 확인합니다."""
        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'주행 진행률: 웨이포인트 {feedback.current_waypoint + 1}')
            return
        
        # 주행이 끝났으면 타이머를 멈추고 결과 처리
        if self.status_check_timer is not None:
            self.destroy_timer(self.status_check_timer)
            self.status_check_timer = None
            
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('주행 완료! 최종 목적지에 도착했습니다.')
            self.current_state = NavigatorState.IDLE
        elif result == TaskResult.CANCELED:
            self.get_logger().info('주행이 외부 요청에 의해 취소되었습니다.')
            self.current_state = NavigatorState.IDLE
        else: # FAILED
            self.get_logger().error(f'주행 실패! 최종 상태: {result}')
            self.current_state = NavigatorState.ERROR

    # --- load_waypoints, get_closest_waypoint, find_path_astar 함수는 이전과 동일 ---
    def load_waypoints(self):
        try:
            share_dir = get_package_share_directory('libo_navigator')
            waypoint_file_path = f'{share_dir}/config/waypoints.yaml'
            with open(waypoint_file_path, 'r') as file:
                self.waypoints = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"웨이포인트 파일 로딩 실패: {e}")

    def get_closest_waypoint(self, pose):
        min_dist = float('inf')
        closest_wp_name = None
        if not self.waypoints: return None
        for name, data in self.waypoints.items():
            dist = math.sqrt((pose.position.x - data['position']['x'])**2 + (pose.position.y - data['position']['y'])**2)
            if dist < min_dist:
                min_dist = dist
                closest_wp_name = name
        return closest_wp_name

    def find_path_astar(self, start_wp_name, goal_wp_name):
        # A* 로직 (변경 없음)
        open_set = []
        heapq.heappush(open_set, (0, start_wp_name))
        came_from = {}
        g_score = {name: float('inf') for name in self.waypoints}
        g_score[start_wp_name] = 0
        f_score = {name: float('inf') for name in self.waypoints}
        goal_pos = self.waypoints[goal_wp_name]['position']
        start_pos = self.waypoints[start_wp_name]['position']
        f_score[start_wp_name] = math.sqrt((goal_pos['x'] - start_pos['x'])**2 + (goal_pos['y'] - start_pos['y'])**2)
        while open_set:
            _, current_name = heapq.heappop(open_set)
            if current_name == goal_wp_name:
                path = []
                while current_name in came_from:
                    path.append(current_name)
                    current_name = came_from[current_name]
                path.append(start_wp_name)
                return path[::-1]
            current_pos = self.waypoints[current_name]['position']
            for neighbor_name in self.waypoints[current_name].get('neighbors', []):
                neighbor_pos = self.waypoints[neighbor_name]['position']
                tentative_g_score = g_score[current_name] + math.sqrt((neighbor_pos['x'] - current_pos['x'])**2 + (neighbor_pos['y'] - current_pos['y'])**2)
                if tentative_g_score < g_score[neighbor_name]:
                    came_from[neighbor_name] = current_name
                    g_score[neighbor_name] = tentative_g_score
                    h_score = math.sqrt((goal_pos['x'] - neighbor_pos['x'])**2 + (goal_pos['y'] - neighbor_pos['y'])**2)
                    f_score[neighbor_name] = tentative_g_score + h_score
                    if neighbor_name not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor_name], neighbor_name))
        return None

def main(args=None):
    rclpy.init(args=args)
    navigator_node = LiboNavigator()  # 변수명 변경하여 클래스와 구분
    
    try:
        rclpy.spin(navigator_node)
    except KeyboardInterrupt:
        navigator_node.get_logger().info('키보드 인터럽트로 종료합니다...')
    finally:
        # 정리 작업
        if hasattr(navigator_node, 'status_check_timer') and navigator_node.status_check_timer is not None:
            navigator_node.destroy_timer(navigator_node.status_check_timer)
        
        navigator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

"""
Libo Navigator Node
로봇의 네비게이션을 담당하는 노드입니다.
메인 서비스로부터 목적지를 받아 웨이포인트를 따라 주행합니다.
"""

import rclpy
import yaml
import math
import heapq
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from libo_interfaces.srv import SetGoal, CancelNavigation, NavigationResult  # 커스텀 서비스 메시지 import
# ROS2 메시지 타입들
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.msg import Costmap
from std_msgs.msg import String

# 기타 필요한 라이브러리들
import numpy as np
from enum import Enum
from ament_index_python.packages import get_package_share_directory


class NavigatorState(Enum):
    """네비게이터의 상태를 정의하는 열거형"""
    IDLE = "IDLE"                    # 대기 상태
    NAVIGATING = "NAVIGATING"        # 주행 중
    WAITING_FOR_GOAL = "WAITING_FOR_GOAL"  # 목적지 대기 중
    ERROR = "ERROR"                  # 에러 상태


class LiboNavigator(Node):
    """
    Libo 로봇의 네비게이션을 담당하는 메인 클래스
    """
    
    def __init__(self):
        """노드 초기화"""
        super().__init__('libo_navigator')
        
        # 노드 상태 초기화
        self.current_state = NavigatorState.IDLE
        self.current_goal = None
        self.waypoint_list = []
        self.waypoints = {}  # 웨이포인트 그래프
        self.robot_current_pose = None
        self.initial_pose_received = False
        
        # 콜백 그룹 설정 (동시에 여러 액션을 처리하기 위해)
        self.callback_group = ReentrantCallbackGroup()
        
        # Nav2 액션 클라이언트 설정
        self.nav_action_client = ActionClient(
            self, 
            FollowWaypoints, 
            'follow_waypoints',
            callback_group=self.callback_group
        )
        
        # AMCL 위치 구독 설정
        amcl_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            amcl_qos_profile
        )
        
        # Costmap 구독 설정 (장애물 감지용)
        self.create_subscription(
            Costmap,
            '/local_costmap/costmap',
            self.costmap_callback,
            10
        )
        
        # 로그 메시지 출력
        self.get_logger().info('Libo Navigator Node가 시작되었습니다!')
        self.get_logger().info(f'현재 상태: {self.current_state.value}')
        self.get_logger().info('서비스 준비 완료: /set_navigation_goal')

        # 웨이포인트 로드
        self.load_waypoints()
        
        # Nav2 액션 서버가 준비될 때까지 대기
        self.wait_for_nav2_server()
                
        # 추가: 목표 지점 설정을 위한 서비스 서버 생성
        self.set_goal_service = self.create_service(
            SetGoal,
            'set_navigation_goal',
            self.set_goal_service_callback
        )
        
        # 네비게이션 취소를 위한 서비스 서버 생성
        self.cancel_navigation_service = self.create_service(
            CancelNavigation,
            'cancel_navigation',
            self.cancel_navigation_callback
        )

        # 재계획 플래그 추가
        self._replanning = False
        self.current_waypoint_names = []
        self.blocked_waypoints = {}

        # Costmap 저장용 변수 추가
        self.current_costmap = None

    # 네비게이션 목표입력 서비스 콜백함수
    def set_goal_service_callback(self, request, response):
        """
        목표 지점 설정 서비스 콜백 함수입니다.
        요청된 좌표를 목표 지점으로 설정하고 네비게이션을 시작합니다.
        """
        try:
            goal_pose = Pose()
            goal_pose.position.x = request.x
            goal_pose.position.y = request.y
            goal_pose.orientation.w = 1.0
            
            self.get_logger().info(f'서비스 요청으로 목표 좌표 수신: ({request.x}, {request.y})')
            
            # 목적지 설정 및 네비게이션 시작
            result = self.set_goal(goal_pose)
            
            # 응답 설정
            response.success = result
            if result:
                response.message = "목표 지점 설정 및 네비게이션 시작 성공"
            else:
                response.message = "목표 지점 설정 실패"
                
            return response
            
        except Exception as e:
            self.get_logger().error(f'서비스 처리 중 오류 발생: {e}')
            response.success = False
            response.message = f"오류: {str(e)}"
            return response

    # 네비게이션 취소 서비스 콜백함수
    def cancel_navigation_callback(self, request, response):
        """
        네비게이션 취소 서비스 콜백 함수입니다.
        현재 진행 중인 네비게이션을 취소합니다.
        """
        try:
            if self.current_state == NavigatorState.NAVIGATING:
                self.get_logger().info('진행 중인 네비게이션을 취소합니다...')
                # 현재 Nav2 액션 취소 요청
                self.nav_action_client.cancel_goal_async()
                # 웨이포인트 리스트 초기화 (임무 자체 취소)
                self.waypoint_list = []
                # 상태를 IDLE로 명확하게 설정
                self.current_state = NavigatorState.IDLE
                # 취소 완료 로그
                self.get_logger().info('네비게이션이 완전히 취소되었습니다.')
                
                # 리보서비스에 취소 알림 (선택적)
                self.notify_navigation_done("CANCELED")
                
                response.success = True
                response.message = "네비게이션이 취소되었습니다."
            else:
                response.success = False
                response.message = "현재 진행 중인 네비게이션이 없습니다."
                
            return response
            
        except Exception as e:
            self.get_logger().error(f'서비스 처리 중 오류 발생: {e}')
            response.success = False
            response.message = f"오류: {str(e)}"
            return response
    
    def amcl_pose_callback(self, msg):
        """AMCL로부터 로봇의 현재 위치를 받습니다."""
        # amcl이 발행하는 초기 (0,0) 위치는 부정확하므로 무시하는 로직
        if msg.pose.covariance[0] > 0.1:
            self.get_logger().warn('AMCL 위치의 불확실성이 너무 높습니다. 무시합니다.', throttle_duration_sec=5)
            return
            
        self.robot_current_pose = msg.pose.pose
        if not self.initial_pose_received:
            self.get_logger().info('AMCL로부터 유효한 첫 위치 정보를 받았습니다!')
            self.initial_pose_received = True
    
    def costmap_callback(self, msg):
        """Costmap 정보를 저장하고 장애물을 감지합니다."""
        # 🔥 핵심: costmap 데이터를 클래스 변수에 저장
        self.current_costmap = msg

        # 기본 조건 체크
        if not self.waypoint_list or self.current_state != NavigatorState.NAVIGATING:
            return

        # 🔥 수정: 디버깅 로그 레벨 조정 (너무 자주 출력되지 않도록)
        self.get_logger().debug('costmap_callback 호출됨')  # info -> debug

        # 현재 목표 웨이포인트의 위치
        current_target = self.waypoint_list[0]

        # 장애물 감지 로직 (기존과 동일)
        width = msg.metadata.size_x
        height = msg.metadata.size_y
        resolution = msg.metadata.resolution
        origin_x = msg.metadata.origin.position.x
        origin_y = msg.metadata.origin.position.y

        wx = int((current_target.pose.position.x - origin_x) / resolution)
        wy = int((current_target.pose.position.y - origin_y) / resolution)

        check_radius = 3
        blocked_cells = 0
        total_cells = 0

        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                x = wx + dx
                y = wy + dy
                if 0 <= x < width and 0 <= y < height:
                    index = y * width + x
                    total_cells += 1
                    if msg.data[index] >= 80:
                        blocked_cells += 1

        blocked_ratio = blocked_cells / total_cells if total_cells > 0 else 0
        is_blocked = blocked_ratio > 0.3

        if is_blocked:
            self.get_logger().warn('장애물 감지! Costmap 기반 재계획을 시작합니다!')
            self.smart_replan_with_costmap()
    
    def load_waypoints(self):
        """웨이포인트 yaml 파일을 로드합니다."""
        try:
            # 🔥 수정: libo_navigator 패키지에서 파일 로드
            share_dir = get_package_share_directory('libo_navigator')
            waypoint_file_path = f'{share_dir}/config/waypoints.yaml'
            self.get_logger().info(f"웨이포인트 파일 로딩: {waypoint_file_path}")
            
            with open(waypoint_file_path, 'r') as file:
                data = yaml.safe_load(file)
                self.waypoints = data
                if not self.waypoints:
                    self.get_logger().error("웨이포인트 데이터를 로드하지 못했습니다!")
                    return False
                self.get_logger().info(f"성공적으로 {len(self.waypoints)}개의 웨이포인트를 로드했습니다.")
                return True
        except Exception as e:
            self.get_logger().error(f"웨이포인트 파일 로딩 실패: {e}")
            return False
    
    def get_closest_waypoint(self, pose):
        """주어진 위치에서 가장 가까운 웨이포인트를 찾습니다."""
        min_dist = float('inf')
        closest_wp_name = None
        if not self.waypoints:
            return None
        for name, data in self.waypoints.items():
            dist = math.sqrt((pose.position.x - data['position']['x'])**2 +
                             (pose.position.y - data['position']['y'])**2)
            if dist < min_dist:
                min_dist = dist
                closest_wp_name = name
        return closest_wp_name
    
    def find_path_astar(self, start_wp_name, goal_wp_name):
        """Costmap을 반영한 A* 알고리즘으로 최적 경로를 찾습니다."""
        open_set = []
        heapq.heappush(open_set, (0, start_wp_name))
        came_from = {}
        g_score = {name: float('inf') for name in self.waypoints}
        g_score[start_wp_name] = 0
        f_score = {name: float('inf') for name in self.waypoints}
        
        goal_pos = self.waypoints[goal_wp_name]['position']
        start_pos = self.waypoints[start_wp_name]['position']
        f_score[start_wp_name] = self.calculate_heuristic(start_pos, goal_pos)
        
        while open_set:
            _, current_name = heapq.heappop(open_set)
            if current_name == goal_wp_name:
                path = []
                while current_name in came_from:
                    path.append(current_name)
                    current_name = came_from[current_name]
                path.append(start_wp_name)
                return path[::-1]
            
            current_pos = self.waypoints[current_name]['position']
            for neighbor_name in self.waypoints[current_name].get('neighbors', []):
                neighbor_pos = self.waypoints[neighbor_name]['position']
                
                # 🔥 핵심: Costmap 위험도를 반영한 비용 계산
                edge_cost = self.calculate_edge_cost_with_costmap(current_pos, neighbor_pos)
                tentative_g_score = g_score[current_name] + edge_cost
                
                if tentative_g_score < g_score[neighbor_name]:
                    came_from[neighbor_name] = current_name
                    g_score[neighbor_name] = tentative_g_score
                    h_score = self.calculate_heuristic(neighbor_pos, goal_pos)
                    f_score[neighbor_name] = tentative_g_score + h_score
                    if neighbor_name not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor_name], neighbor_name))
        return None
    
    def wait_for_nav2_server(self):
        """Nav2 액션 서버가 준비될 때까지 대기"""
        self.get_logger().info('Nav2 액션 서버를 기다리는 중...')
        
        while not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Nav2 액션 서버가 아직 준비되지 않았습니다. 대기 중...')
        
        self.get_logger().info('Nav2 액션 서버가 준비되었습니다!')
    
    # def start_coordinate_input(self):
    #     """좌표 입력을 시작합니다."""
    #     self.get_logger().info('좌표 입력을 시작합니다...')
    #     self.get_user_goal()
    #     # 타이머 제거 부분을 주석 처리 또는 삭제
    #     # self.destroy_timer(self.get_timer())
    
    # def get_user_goal(self):
    #     """사용자로부터 목적지 좌표를 입력받습니다."""
    #     try:
    #         self.get_logger().info('목적지 좌표를 입력해주세요.')
    #         x = float(input("X 좌표를 입력하세요: "))
    #         y = float(input("Y 좌표를 입력하세요: "))
            
    #         # Pose 객체 생성
    #         goal_pose = Pose()
    #         goal_pose.position.x = x
    #         goal_pose.position.y = y
    #         goal_pose.orientation.w = 1.0
            
    #         # 목적지 설정 및 네비게이션 시작
    #         self.set_goal(goal_pose)
            
    #     except ValueError:
    #         self.get_logger().error('잘못된 좌표입니다. 다시 시도해주세요.')
    #         self.current_state = NavigatorState.ERROR
    
    def set_goal(self, goal_pose: Pose):
        """
        새로운 목적지를 설정하고 네비게이션을 시작합니다.
        
        Args:
            goal_pose (Pose): 목적지 좌표
            
        Returns:
            bool: 네비게이션 시작 성공 여부
        """
        self.get_logger().info(f'새로운 목적지가 설정되었습니다: ({goal_pose.position.x}, {goal_pose.position.y})')
        self.current_goal = goal_pose
        self.current_state = NavigatorState.WAITING_FOR_GOAL
        
        # 목적지에 대한 웨이포인트 리스트 생성
        self.generate_waypoints()
        
        # 웨이포인트 생성 완료 후 주행 시작
        if self.waypoint_list:
            self.start_navigation()
            return True
        else:
            self.get_logger().error('웨이포인트 생성에 실패했습니다.')
            self.current_state = NavigatorState.ERROR
            return False
    #최단 웨이포인트 리스트 생성
    def generate_waypoints(self):
        """목적지까지의 웨이포인트 리스트를 생성합니다."""
        self.get_logger().info('웨이포인트 리스트를 생성합니다...')
        
        if self.current_goal is None:
            self.get_logger().error('목적지가 설정되지 않았습니다.')
            return
        
        if not self.initial_pose_received:
            self.get_logger().error('로봇의 현재 위치를 알 수 없습니다.')
            return
        
        # A* 알고리즘을 사용한 경로 계산
        start_wp = self.get_closest_waypoint(self.robot_current_pose)
        goal_wp = self.get_closest_waypoint(self.current_goal)
        
        if not start_wp or not goal_wp:
            self.get_logger().error(f"시작({start_wp}) 또는 목표({goal_wp}) 웨이포인트를 찾을 수 없습니다.")
            return
        
        self.get_logger().info(f"경로 탐색 시작: {start_wp} -> {goal_wp}")
        path_wp_names = self.find_path_astar(start_wp, goal_wp)
        
        if not path_wp_names:
            self.get_logger().error("최적 경로를 찾지 못했습니다!")
            return
        
        self.get_logger().info(f"계산된 최적 경로: {path_wp_names}")
        
        # 웨이포인트 리스트 생성
        waypoint_poses = []
        for i, name in enumerate(path_wp_names):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = self.waypoints[name]['position']['x']
            pose.pose.position.y = self.waypoints[name]['position']['y']
            
            # 다음 웨이포인트가 있으면 그 방향을 향하도록 orientation 설정
            if i < len(path_wp_names) - 1:
                next_name = path_wp_names[i+1]
                next_x = self.waypoints[next_name]['position']['x']
                next_y = self.waypoints[next_name]['position']['y']
                
                # 현재 웨이포인트에서 다음 웨이포인트를 향하는 방향 계산
                dx = next_x - pose.pose.position.x
                dy = next_y - pose.pose.position.y
                yaw = math.atan2(dy, dx)
                
                # 쿼터니언으로 변환 (yaw만 사용)
                q = self.euler_to_quaternion(0, 0, yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
            else:
                # 마지막 웨이포인트는 이전 방향을 유지하거나 기본값 사용
                pose.pose.orientation.w = 1.0
                
            waypoint_poses.append(pose)
        
        # 웨이포인트 이름 목록 저장 (추가)
        self.current_waypoint_names = path_wp_names.copy()
        
        self.waypoint_list = waypoint_poses
        self.get_logger().info(f'웨이포인트가 생성되었습니다: {len(self.waypoint_list)}개')
    
    def start_navigation(self):
        """웨이포인트를 따라 주행을 시작합니다."""
        if not self.waypoint_list:
            self.get_logger().error('웨이포인트 리스트가 비어있습니다!')
            return
        
        self.get_logger().info('주행을 시작합니다...')
        self.current_state = NavigatorState.NAVIGATING
        
        # Nav2에 웨이포인트 주행 요청
        goal_msg = FollowWaypoints.Goal()
        
        # 사용 가능한 속성들 확인
        self.get_logger().info(f'FollowWaypoints.Goal의 속성들: {dir(goal_msg)}')
        
        # 올바른 속성 이름 찾기
        if hasattr(goal_msg, 'poses'):
            goal_msg.poses = self.waypoint_list
            self.get_logger().info('poses 속성 사용')
        else:
            self.get_logger().error('사용 가능한 속성을 찾을 수 없습니다.')
            return
        
        # 액션 요청 전송 (수정된 부분)
        future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        future.add_done_callback(self.navigation_goal_response_callback)
        
        self.get_logger().info('Nav2에 웨이포인트 주행 요청을 전송했습니다.')
    
    def navigation_goal_response_callback(self, future):
        """Goal 응답을 처리하고 Result를 기다립니다."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal이 거부되었습니다!')
            self.current_state = NavigatorState.ERROR
            return
        
        self.get_logger().info('Goal이 수락되었습니다. 결과를 기다립니다...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """주행 중 피드백을 처리합니다."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'주행 진행률: {feedback.current_waypoint}/{len(self.waypoint_list)}')
    
    def navigation_result_callback(self, future):
        """주행 완료 결과를 처리합니다."""
        try:
            result = future.result()
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info('주행이 완료되었습니다!')
                self.current_state = NavigatorState.IDLE
                # 리보서비스에 완료 알림
                self.notify_navigation_done("SUCCEEDED")
            else:
                self.get_logger().error(f'주행 실패! status: {result.status}')
                self.current_state = NavigatorState.ERROR
                self.notify_navigation_done("FAILED")
        except Exception as e:
            self.get_logger().error(f'주행 중 오류가 발생했습니다: {e}')
            self.current_state = NavigatorState.ERROR
            self.notify_navigation_done("FAILED")

    def notify_navigation_done(self, result_str):
        # 리보서비스의 NavigationDone 서비스 클라이언트 생성 및 호출
        client = self.create_client(NavigationResult, 'navigation_result')
        req = NavigationResult.Request()
        req.result = result_str
        if client.wait_for_service(timeout_sec=2.0):
            future = client.call_async(req)
            # 필요하다면 응답 처리
        else:
            self.get_logger().warn('navigation_done 서비스가 준비되지 않았습니다.')
    
    def smart_replan_with_costmap(self):
        """Costmap을 반영한 스마트 재계획"""
        if self._replanning:
            return
        
        self._replanning = True
        try:
            # 현재 Nav2 액션 취소
            self.nav_action_client.cancel_goal_async()
            
            # Costmap 기반 새 경로 계산
            current_wp = self.get_closest_waypoint(self.robot_current_pose)
            goal_wp = self.get_closest_waypoint(self.current_goal)
            
            self.get_logger().info(f'Costmap 기반 경로 재계산: {current_wp} -> {goal_wp}')
            
            # 🔥 수정: 통합된 find_path_astar 함수 사용
            path_wp_names = self.find_path_astar(current_wp, goal_wp)
            
            if path_wp_names:
                self.current_waypoint_names = path_wp_names.copy()
                
                # 새로운 웨이포인트 리스트 생성
                waypoint_poses = []
                for i, name in enumerate(path_wp_names):
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = self.waypoints[name]['position']['x']
                    pose.pose.position.y = self.waypoints[name]['position']['y']
                    
                    # 방향 계산
                    if i < len(path_wp_names) - 1:
                        next_name = path_wp_names[i+1]
                        next_x = self.waypoints[next_name]['position']['x']
                        next_y = self.waypoints[next_name]['position']['y']
                        
                        dx = next_x - pose.pose.position.x
                        dy = next_y - pose.pose.position.y
                        yaw = math.atan2(dy, dx)
                        
                        q = self.euler_to_quaternion(0, 0, yaw)
                        pose.pose.orientation.x = q[0]
                        pose.pose.orientation.y = q[1]
                        pose.pose.orientation.z = q[2]
                        pose.pose.orientation.w = q[3]
                    else:
                        pose.pose.orientation.w = 1.0
                    
                    waypoint_poses.append(pose)
                
                self.waypoint_list = waypoint_poses
                self.start_navigation()
                self.get_logger().info(f'Costmap 기반 새 경로: {path_wp_names}')
            else:
                self.get_logger().error('Costmap 기반 경로를 찾을 수 없습니다!')
                self.current_state = NavigatorState.ERROR
                
        finally:
            self._replanning = False

   

    def calculate_edge_cost_with_costmap(self, pos1, pos2):
        """두 웨이포인트 사이의 costmap 기반 비용을 계산합니다."""
        # 기본 유클리드 거리
        base_distance = math.sqrt((pos2['x'] - pos1['x'])**2 + (pos2['y'] - pos1['y'])**2)
        
        # costmap 데이터가 없으면 기본 거리만 반환
        if self.current_costmap is None:
            return base_distance
        
        # 두 점 사이의 직선 경로에서 costmap 위험도 샘플링
        danger_penalty = self.sample_costmap_danger(pos1, pos2)
        
        # 최종 비용 = 기본 거리 + 위험도 패널티
        return base_distance + danger_penalty

    def sample_costmap_danger(self, pos1, pos2):
        """두 점 사이 직선 경로의 costmap 위험도를 샘플링합니다."""
        try:
            costmap = self.current_costmap
            width = costmap.metadata.size_x
            height = costmap.metadata.size_y
            resolution = costmap.metadata.resolution
            origin_x = costmap.metadata.origin.position.x
            origin_y = costmap.metadata.origin.position.y
            
            # 직선 경로를 여러 점으로 샘플링
            num_samples = 20
            total_danger = 0
            valid_samples = 0  # 🔥 추가: 유효한 샘플 수 카운트
            
            for i in range(num_samples + 1):
                ratio = i / num_samples
                sample_x = pos1['x'] + ratio * (pos2['x'] - pos1['x'])
                sample_y = pos1['y'] + ratio * (pos2['y'] - pos1['y'])
                
                # 맵 좌표를 costmap 인덱스로 변환
                grid_x = int((sample_x - origin_x) / resolution)
                grid_y = int((sample_y - origin_y) / resolution)
                
                # 🔥 개선: 범위 체크 강화
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    index = grid_y * width + grid_x
                    if 0 <= index < len(costmap.data):  # 추가 안전성 체크
                        cost_value = costmap.data[index]
                        valid_samples += 1
                        
                        # 위험도에 따른 패널티 계산
                        if cost_value >= 99:      # 장애물
                            total_danger += 1000  # 매우 높은 패널티
                        elif cost_value >= 80:    # 위험 지역
                            total_danger += 100   # 높은 패널티
                        elif cost_value >= 50:    # 약간 위험
                            total_danger += 10    # 낮은 패널티
        
            # 🔥 개선: 유효한 샘플이 있을 때만 평균 계산
            return total_danger / valid_samples if valid_samples > 0 else 0
            
        except Exception as e:
            self.get_logger().warn(f'Costmap 샘플링 중 오류: {e}')
            return 0

    def calculate_heuristic(self, pos1, pos2):
        """목적지까지의 휴리스틱 (유클리드 거리)"""
        return math.sqrt((pos2['x'] - pos1['x'])**2 + (pos2['y'] - pos1['y'])**2)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """오일러 각을 쿼터니언으로 변환합니다."""
        import math
        
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        return [qx, qy, qz, qw]


def main(args=None):
    """메인 함수 - 노드를 초기화하고 실행합니다."""
    rclpy.init(args=args)
    navigator = LiboNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('키보드 인터럽트로 종료합니다...')
    except Exception as e:
        navigator.get_logger().error(f'예상치 못한 오류로 종료합니다: {e}')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()
        print("Libo Navigator가 완전히 종료되었습니다.")

if __name__ == '__main__':
    main()