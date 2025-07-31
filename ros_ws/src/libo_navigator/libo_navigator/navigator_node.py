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
from libo_interfaces.srv import SetGoal  # 커스텀 서비스 메시지 import
# ROS2 메시지 타입들
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.msg import Costmap
from std_msgs.msg import String

 # 커스텀 서비스 메시지 import
from libo_interfaces.srv import SetGoal

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
        """Costmap 정보를 받아 장애물을 감지합니다."""
        # Costmap 데이터 처리
        width = msg.metadata.size_x
        height = msg.metadata.size_y
        resolution = msg.metadata.resolution
        origin_x = msg.metadata.origin.position.x
        origin_y = msg.metadata.origin.position.y
        
        # 현재 목표 웨이포인트가 있는지 확인
        if not self.waypoint_list or self.current_state != NavigatorState.NAVIGATING:
            return
            
        # 현재 목표 웨이포인트의 위치
        current_target = self.waypoint_list[0]  # 현재 향하고 있는 웨이포인트
        
        # 웨이포인트의 맵 좌표계 상의 위치를 Costmap 인덱스로 변환
        wx = int((current_target.pose.position.x - origin_x) / resolution)
        wy = int((current_target.pose.position.y - origin_y) / resolution)
        
        # 해당 위치 주변의 비용 확인 (장애물 여부 판단)
        check_radius = 5  # 웨이포인트 주변 5셀 반경 확인
        is_blocked = False
        
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                x = wx + dx
                y = wy + dy
                if 0 <= x < width and 0 <= y < height:
                    index = y * width + x
                    if msg.data[index] >= 99:  # 장애물로 판단되는 임계값
                        is_blocked = True
                        break
            if is_blocked:
                break
        
        if is_blocked:
            self.get_logger().warn('현재 목표 웨이포인트가 장애물로 막혀있습니다. 경로를 재계산합니다.')
            self.pause_navigation()
            self.replan_path()
    
    def load_waypoints(self):
        """웨이포인트 yaml 파일을 로드합니다."""
        try:
            share_dir = get_package_share_directory('libo_waypoint_runner')
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
        """A* 알고리즘으로 최적 경로를 찾습니다."""
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
        for name in path_wp_names:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = self.waypoints[name]['position']['x']
            pose.pose.position.y = self.waypoints[name]['position']['y']
            pose.pose.orientation.w = 1.0
            waypoint_poses.append(pose)
        
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
            self.get_logger().info('주행이 완료되었습니다!')
            self.current_state = NavigatorState.IDLE
            
        except Exception as e:
            self.get_logger().error(f'주행 중 오류가 발생했습니다: {e}')
            self.current_state = NavigatorState.ERROR

    def replan_path(self):
        """현재 위치에서 목적지까지 새로운 경로를 계산합니다."""
        if not self.initial_pose_received or not self.current_goal:
            return
        
        # 현재 위치와 목표 위치에서 가장 가까운 웨이포인트 찾기
        current_wp = self.get_closest_waypoint(self.robot_current_pose)
        goal_wp = self.get_closest_waypoint(self.current_goal)
        
        if not current_wp or not goal_wp:
            self.get_logger().error('현재 위치 또는 목표 위치 근처에서 웨이포인트를 찾을 수 없습니다.')
            return
        
        # 현재 진행 중인 네비게이션 취소
        if self.current_state == NavigatorState.NAVIGATING:
            # Nav2 goal 취소
            self.nav_action_client.cancel_goal_async()
        
        # 새로운 경로 계산
        self.get_logger().info(f'새로운 경로 계산 시작: {current_wp} -> {goal_wp}')
        path_wp_names = self.find_path_astar(current_wp, goal_wp)
        
        if not path_wp_names:
            self.get_logger().error('새로운 경로를 찾을 수 없습니다!')
            return
        
        # 새로운 웨이포인트 리스트 생성
        waypoint_poses = []
        for name in path_wp_names:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = self.waypoints[name]['position']['x']
            pose.pose.position.y = self.waypoints[name]['position']['y']
            pose.pose.orientation.w = 1.0
            waypoint_poses.append(pose)
        
        self.waypoint_list = waypoint_poses
        self.get_logger().info(f'새로운 경로가 생성되었습니다: {path_wp_names}')
        
        # 새로운 경로로 네비게이션 시작
        self.start_navigation()

    def pause_navigation(self):
        """현재 네비게이션을 일시 중지합니다."""
        if self.current_state == NavigatorState.NAVIGATING:
            self.nav_action_client.cancel_goal_async()
            self.get_logger().info('네비게이션이 일시 중지되었습니다.')


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    # 네비게이터 노드 생성
    navigator = LiboNavigator()
    
    try:
        # 노드 실행
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()