#해당코드 헤딩각 자동변환 사라짐
#!/usr/bin/env python3 
import rclpy
import yaml
import math
import heapq
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# libo_interfaces에 정의된 우리만의 서비스 타입들을 import합니다.
from libo_interfaces.srv import SetGoal, CancelNavigation, NavigationResult
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.msg import Costmap
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
        self.nav_goal_handle = None
        self.status_check_timer = None

        # BasicNavigator 초기화 (중요!)
        self.navigator = BasicNavigator()

        # 여러 콜백(서비스, 구독)이 동시에 처리될 수 있도록 ReentrantCallbackGroup을 사용합니다.
        self.callback_group = ReentrantCallbackGroup()
        
        # AMCL QoS 설정
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
        self.current_waypoint_names = []
        self.current_waypoint_poses = []
        self.current_waypoint_index = 0
        
        # --- 외부와 통신하기 위한 서비스 서버들을 생성합니다 ---
        self.create_service(
            SetGoal, 'set_navigation_goal', self.set_goal_callback, callback_group=self.callback_group
        )
        self.create_service(
            CancelNavigation, 'cancel_navigation', self.cancel_navigation_callback, callback_group=self.callback_group
        )

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

    def costmap_callback(self, msg):
        """Costmap 정보를 저장하고 장애물을 감지합니다."""
        # costmap 데이터를 클래스 변수에 저장
        self.current_costmap = msg

        # 기본 조건 체크 - 주행 중이 아니면 무시
        if self.current_state != NavigatorState.NAVIGATING:
            return

        # 현재 목표 웨이포인트의 장애물 검사
        if hasattr(self, 'current_waypoint_index') and hasattr(self, 'current_waypoint_poses'):
            if self.current_waypoint_index < len(self.current_waypoint_poses):
                current_target = self.current_waypoint_poses[self.current_waypoint_index]
                
                if self.is_waypoint_blocked(current_target, msg):
                    self.get_logger().warn(f'웨이포인트 {self.current_waypoint_index}에서 장애물 감지! 재계획을 시작합니다.')
                    self.handle_dynamic_obstacle()

    def is_waypoint_blocked(self, waypoint_pose, costmap_msg):
        """특정 웨이포인트가 장애물로 막혔는지 확인합니다."""
        try:
            width = costmap_msg.metadata.size_x
            height = costmap_msg.metadata.size_y
            resolution = costmap_msg.metadata.resolution
            origin_x = costmap_msg.metadata.origin.position.x
            origin_y = costmap_msg.metadata.origin.position.y

            # 웨이포인트의 맵 좌표를 costmap 인덱스로 변환
            wx = int((waypoint_pose.pose.position.x - origin_x) / resolution)
            wy = int((waypoint_pose.pose.position.y - origin_y) / resolution)

            # 웨이포인트 주변의 일정 반경 검사
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
                        if index < len(costmap_msg.data) and costmap_msg.data[index] >= 80:  # 장애물 임계값
                            blocked_cells += 1

            # 30% 이상이 막혔으면 장애물로 판단
            blocked_ratio = blocked_cells / total_cells if total_cells > 0 else 0
            return blocked_ratio > 0.3
            
        except Exception as e:
            self.get_logger().warn(f'장애물 검사 중 오류: {e}')
            return False

    def handle_dynamic_obstacle(self):
        """동적 장애물이 감지되었을 때 처리하는 함수"""
        if self._replanning:
            return  # 이미 재계획 중이면 무시
        
        self._replanning = True
        try:
            self.get_logger().info('동적 장애물로 인한 재계획을 시작합니다...')
            
            # 1. 현재 네비게이션 취소
            self.navigator.cancelTask()
            
            # 2. 타이머 정리
            if self.status_check_timer is not None:
                self.destroy_timer(self.status_check_timer)
                self.status_check_timer = None
            
            # 3. 막힌 웨이포인트 추가 (현재 목표 웨이포인트)
            if hasattr(self, 'current_waypoint_names') and hasattr(self, 'current_waypoint_index'):
                if self.current_waypoint_index < len(self.current_waypoint_names):
                    blocked_wp = self.current_waypoint_names[self.current_waypoint_index]
                    self.blocked_waypoints.add(blocked_wp)
                    self.get_logger().warn(f'웨이포인트 {blocked_wp}를 막힌 지점으로 등록했습니다.')
            
            # 4. 새로운 경로 계산 및 주행 시작
            self.replan_with_blocked_waypoints()
            
        except Exception as e:
            self.get_logger().error(f'동적 장애물 처리 중 오류: {e}')
            self.current_state = NavigatorState.ERROR
        finally:
            self._replanning = False

    def replan_with_blocked_waypoints(self):
        """막힌 웨이포인트를 제외하고 새로운 경로를 계산합니다."""
        try:
            # 현재 위치에서 목표까지 새로운 경로 계산
            start_wp = self.get_closest_waypoint(self.robot_current_pose)
            goal_wp = self.get_closest_waypoint(self.current_goal_pose)
            
            if not start_wp or not goal_wp:
                self.get_logger().error("재계획을 위한 웨이포인트를 찾을 수 없습니다.")
                self.current_state = NavigatorState.ERROR
                return
            
            # 막힌 웨이포인트를 제외한 A* 경로 계산
            path_wp_names = self.find_path_astar_with_blocked(start_wp, goal_wp)
            
            if not path_wp_names:
                self.get_logger().error("막힌 웨이포인트를 제외한 경로를 찾을 수 없습니다!")
                self.current_state = NavigatorState.ERROR
                return
            
            self.get_logger().info(f"재계산된 경로: {path_wp_names}")
            self.get_logger().info(f"제외된 웨이포인트: {list(self.blocked_waypoints)}")
            
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
            
            # 새로운 주행 시작
            self.current_waypoint_names = path_wp_names
            self.current_waypoint_poses = waypoint_poses
            self.current_waypoint_index = 0
            self.start_navigation(waypoint_poses)
            
        except Exception as e:
            self.get_logger().error(f'재계획 중 오류: {e}')
            self.current_state = NavigatorState.ERROR

    def find_path_astar_with_blocked(self, start_wp_name, goal_wp_name):
        """막힌 웨이포인트를 제외하고 A* 알고리즘으로 경로를 찾습니다."""
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
                # 막힌 웨이포인트는 건너뛰기
                if neighbor_name in self.blocked_waypoints:
                    continue
                    
                neighbor_pos = self.waypoints[neighbor_name]['position']
                tentative_g_score = g_score[current_name] + math.sqrt(
                    (neighbor_pos['x'] - current_pos['x'])**2 + 
                    (neighbor_pos['y'] - current_pos['y'])**2
                )
                
                if tentative_g_score < g_score[neighbor_name]:
                    came_from[neighbor_name] = current_name
                    g_score[neighbor_name] = tentative_g_score
                    h_score = math.sqrt(
                        (goal_pos['x'] - neighbor_pos['x'])**2 + 
                        (goal_pos['y'] - neighbor_pos['y'])**2
                    )
                    f_score[neighbor_name] = tentative_g_score + h_score
                    if neighbor_name not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor_name], neighbor_name))
        
        return None

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
        
        # 취소 시 막힌 웨이포인트 목록도 초기화
        self.blocked_waypoints.clear()
        
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
        
        # 웨이포인트 이름 저장 (재계획에 필요)
        self.current_waypoint_names = path_wp_names
        
        self.start_navigation(waypoint_poses)
        return True

    def start_navigation(self, waypoint_poses):
        """Nav2에 웨이포인트 주행을 요청합니다."""
        self.get_logger().info(f"{len(waypoint_poses)}개의 지점으로 주행을 시작합니다...")
        self.current_state = NavigatorState.NAVIGATING
        
        # 웨이포인트 추적을 위한 변수 설정
        self.current_waypoint_poses = waypoint_poses
        self.current_waypoint_index = 0
        
        self.navigator.followWaypoints(waypoint_poses)
        # isTaskComplete()를 주기적으로 확인하여 완료 시 콜백을 직접 호출하는 방식으로 변경
        self.status_check_timer = self.create_timer(1.0, self.check_navigation_status)

    def check_navigation_status(self):
        """1초마다 Nav2 주행 상태를 확인합니다."""
        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # 현재 웨이포인트 인덱스 업데이트
                self.current_waypoint_index = feedback.current_waypoint
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
            # 성공적으로 완료되면 막힌 웨이포인트 목록 초기화
            self.blocked_waypoints.clear()
            # 리보서비스에 완료 알림
            self.notify_navigation_done("SUCCEEDED")
        elif result == TaskResult.CANCELED:
            self.get_logger().info('주행이 외부 요청에 의해 취소되었습니다.')
            self.current_state = NavigatorState.IDLE
            # 리보서비스에 취소 알림
            self.notify_navigation_done("CANCELED")
        else: # FAILED
            self.get_logger().error(f'주행 실패! 최종 상태: {result}')
            self.current_state = NavigatorState.ERROR
            # 리보서비스에 실패 알림
            self.notify_navigation_done("FAILED")

    def notify_navigation_done(self, result_str):
        """리보서비스에 네비게이션 완료 상태를 알립니다."""
        try:
            client = self.create_client(NavigationResult, 'navigation_result')
            req = NavigationResult.Request()
            req.result = result_str
            if client.wait_for_service(timeout_sec=2.0):
                future = client.call_async(req)
                self.get_logger().info(f'네비게이션 결과 알림: {result_str}')
            else:
                self.get_logger().warn('navigation_result 서비스가 준비되지 않았습니다.')
        except Exception as e:
            self.get_logger().warn(f'네비게이션 결과 알림 중 오류: {e}')

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
            valid_samples = 0
            
            for i in range(num_samples + 1):
                ratio = i / num_samples
                sample_x = pos1['x'] + ratio * (pos2['x'] - pos1['x'])
                sample_y = pos1['y'] + ratio * (pos2['y'] - pos1['y'])
                
                # 맵 좌표를 costmap 인덱스로 변환
                grid_x = int((sample_x - origin_x) / resolution)
                grid_y = int((sample_y - origin_y) / resolution)
                
                if 0 <= grid_x < width and 0 <= grid_y < height:
                    index = grid_y * width + grid_x
                    if 0 <= index < len(costmap.data):
                        cost_value = costmap.data[index]
                        valid_samples += 1
                        
                        # 위험도에 따른 패널티 계산
                        if cost_value >= 99:      # 장애물
                            total_danger += 1000  # 매우 높은 패널티
                        elif cost_value >= 80:    # 위험 지역
                            total_danger += 100   # 높은 패널티
                        elif cost_value >= 50:    # 약간 위험
                            total_danger += 10    # 낮은 패널티
        
            return total_danger / valid_samples if valid_samples > 0 else 0
            
        except Exception as e:
            self.get_logger().warn(f'Costmap 샘플링 중 오류: {e}')
            return 0

    def euler_to_quaternion(self, roll, pitch, yaw):
        """오일러 각을 쿼터니언으로 변환합니다."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

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
        """Costmap을 반영한 A* 알고리즘으로 최적 경로를 찾습니다."""
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
                
                # Costmap 위험도를 반영한 비용 계산
                edge_cost = self.calculate_edge_cost_with_costmap(current_pos, neighbor_pos)
                tentative_g_score = g_score[current_name] + edge_cost
                
                if tentative_g_score < g_score[neighbor_name]:
                    came_from[neighbor_name] = current_name
                    g_score[neighbor_name] = tentative_g_score
                    h_score = math.sqrt(
                        (goal_pos['x'] - neighbor_pos['x'])**2 + 
                        (goal_pos['y'] - neighbor_pos['y'])**2
                    )
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
