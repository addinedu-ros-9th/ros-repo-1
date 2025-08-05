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
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from enum import Enum
from ament_index_python.packages import get_package_share_directory
import time

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

        # Nav2 충돌 방지를 위한 네임스페이스 확인
        self.declare_parameter('use_namespace', False)
        self.namespace_mode = self.get_parameter('use_namespace').value
        
        if self.namespace_mode:
            self.get_logger().info('네임스페이스 모드로 실행 - 기존 Nav2와 독립 동작')
        else:
            self.get_logger().info('기본 모드로 실행 - 기존 Nav2 스택 활용')

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
        
        # Global Costmap 구독 설정 (더 넓은 범위 장애물 감지용)
        self.create_subscription(
            Costmap,
            '/global_costmap/costmap',
            self.costmap_callback,  # 동일한 콜백 함수 사용
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
        
        # 취소 후 새 목표 처리를 위한 큐 시스템
        self.pending_goal = None
        self.is_canceling = False
        
        # ETA 기반 우회로 감지 시스템
        self.navigation_start_time = None
        self.initial_eta_estimate = None
        self.eta_monitoring_active = False
        self.eta_spike_threshold = 2.0  # ETA가 2배 이상 증가하면 우회로로 판단 (더 민감하게)
        self.min_monitoring_time = 1.0  # 최소 1초 후부터 ETA 모니터링 시작
        self.initial_direct_distance = None  # 초기 직선 거리
        self.current_planned_path = None  # Nav2의 계획된 경로
        
        # Nav2 경로 구독 (실제 계획된 경로 모니터링용)
        self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Nav2 액션 클라이언트 (ETA 정보 모니터링용)
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
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

    def path_callback(self, msg):
        """Nav2의 계획된 경로를 수신하여 ETA 모니터링에 활용합니다."""
        self.current_planned_path = msg
        
        if self.eta_monitoring_active and self.navigation_start_time:
            # 계획된 경로 길이 계산
            path_length = self.calculate_path_length(msg)
            
            if path_length > 0 and self.initial_direct_distance:
                # 우회로 비율 계산 (계획된 경로 / 직선 거리)
                detour_ratio = path_length / self.initial_direct_distance
                
                self.get_logger().debug(f'📊 경로 모니터링 - 직선: {self.initial_direct_distance:.2f}m, 계획: {path_length:.2f}m, 우회비율: {detour_ratio:.2f}')
                
                # 우회로 감지 기준: 직선 거리의 1.5배 이상이면 우회로로 판단
                if detour_ratio >= 1.5:
                    self.get_logger().warn(f'🚨 우회로 감지! 직선({self.initial_direct_distance:.2f}m) vs 계획({path_length:.2f}m) = x{detour_ratio:.1f}')
                    
                    # ETA 모니터링 중단
                    self.eta_monitoring_active = False
                    
                    # 즉시 Nav2 중단하고 웨이포인트 재최적화
                    self.handle_eta_spike_detection()

    def calculate_path_length(self, path_msg):
        """Nav2의 계획된 경로 길이를 계산합니다."""
        if not path_msg.poses or len(path_msg.poses) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(path_msg.poses) - 1):
            p1 = path_msg.poses[i].pose.position
            p2 = path_msg.poses[i + 1].pose.position
            
            distance = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            total_length += distance
        
        return total_length

    def costmap_callback(self, msg):
        """Costmap 정보를 저장하고 장애물을 감지합니다."""
        self.current_costmap = msg

        # 디버그: costmap 수신 확인
        self.get_logger().debug(f'Costmap 수신 - 크기: {msg.metadata.size_x}x{msg.metadata.size_y}, 해상도: {msg.metadata.resolution}')

        if self.current_state != NavigatorState.NAVIGATING:
            self.get_logger().debug(f'현재 상태: {self.current_state.value} - 장애물 감지 건너뜀')
            return

        if hasattr(self, 'current_waypoint_index') and hasattr(self, 'current_waypoint_poses'):
            if self.current_waypoint_index < len(self.current_waypoint_poses):
                current_target = self.current_waypoint_poses[self.current_waypoint_index]
                
                # 디버그: 현재 목표 웨이포인트 정보
                self.get_logger().debug(f'현재 목표 웨이포인트 {self.current_waypoint_index}: ({current_target.pose.position.x:.2f}, {current_target.pose.position.y:.2f})')
                
                is_blocked = self.is_waypoint_blocked(current_target, msg)
                self.get_logger().debug(f'웨이포인트 {self.current_waypoint_index} 차단 여부: {is_blocked}')
                
                if is_blocked:
                    # 중요: 콜백에서 바로 처리하지 말고 타이머로 지연 처리
                    if not self._replanning:
                        self._replanning = True
                        self.get_logger().warn(f'🚨 웨이포인트 {self.current_waypoint_index}에서 장애물 감지! 웨이포인트 재계획 시작!')
                        # 즉시 처리하도록 타이머 설정 (Nav2 우회로 생성 방지)
                        self.create_timer(0.05, self.handle_dynamic_obstacle_delayed)
                    else:
                        self.get_logger().debug('이미 재계획 중이므로 추가 감지 무시')
            else:
                self.get_logger().debug(f'웨이포인트 인덱스 범위 초과: {self.current_waypoint_index} >= {len(self.current_waypoint_poses)}')
        else:
            self.get_logger().debug('웨이포인트 정보 없음 - 장애물 감지 건너뜀')

    def is_waypoint_blocked(self, waypoint_pose, costmap_msg):
        """특정 웨이포인트가 장애물로 막혔는지 확인합니다.
        
        판단 기준: 웨이포인트가 완전히 막혔으면 해당 웨이포인트를 제외한 새로운 경로 계산
        """
        try:
            width = costmap_msg.metadata.size_x
            height = costmap_msg.metadata.size_y
            resolution = costmap_msg.metadata.resolution
            origin_x = costmap_msg.metadata.origin.position.x
            origin_y = costmap_msg.metadata.origin.position.y

            # 웨이포인트의 맵 좌표를 costmap 인덱스로 변환
            wx = int((waypoint_pose.pose.position.x - origin_x) / resolution)
            wy = int((waypoint_pose.pose.position.y - origin_y) / resolution)

            # 경계 확인
            if wx < 0 or wx >= width or wy < 0 or wy >= height:
                return False

            # 웨이포인트 중심에서 로봇 크기만큼 검사
            robot_radius_cells = max(3, int(0.35 / resolution))  # 로봇 반지름 35cm
            
            # 심각한 장애물 감지 (웨이포인트 제외 기준)
            critical_blocked_cells = 0  # 80+: 웨이포인트 사용 불가
            total_cells = 0

            for dx in range(-robot_radius_cells, robot_radius_cells + 1):
                for dy in range(-robot_radius_cells, robot_radius_cells + 1):
                    # 원형 영역만 검사
                    if dx*dx + dy*dy > robot_radius_cells*robot_radius_cells:
                        continue
                        
                    x = wx + dx
                    y = wy + dy
                    if 0 <= x < width and 0 <= y < height:
                        index = y * width + x
                        total_cells += 1
                        if index < len(costmap_msg.data):
                            cost_value = costmap_msg.data[index]
                            # 80 이상이면 웨이포인트 사용 불가
                            if cost_value >= 80:
                                critical_blocked_cells += 1

            # 비율 계산
            blocked_ratio = critical_blocked_cells / total_cells if total_cells > 0 else 0
            
            # 웨이포인트 완전 차단 판정: 20% 이상 막히면 해당 웨이포인트 제외
            is_blocked = blocked_ratio >= 0.2
            
            if is_blocked:
                self.get_logger().warn(f'🚨 웨이포인트 완전 차단 감지! 차단률: {blocked_ratio:.2%} - 웨이포인트 그래프에서 제외')
                
            return is_blocked
            
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
            
            # 헤딩이 계산된 새로운 웨이포인트 리스트 생성
            waypoint_poses = self.create_waypoint_poses_with_heading(path_wp_names)
            
            # 새로운 주행 시작
            self.current_waypoint_names = path_wp_names
            self.current_waypoint_poses = waypoint_poses
            self.current_waypoint_index = 0
            self.start_navigation(waypoint_poses)
            
        except Exception as e:
            self.get_logger().error(f'재계획 중 오류: {e}')
            self.current_state = NavigatorState.ERROR

    def find_path_astar_with_blocked(self, start_wp_name, goal_wp_name):
        """막힌 웨이포인트를 제외하고 직선 경로 우선으로 A* 알고리즘 경로 탐색"""
        open_set = []
        heapq.heappush(open_set, (0, start_wp_name))
        came_from = {}
        g_score = {name: float('inf') for name in self.waypoints}
        g_score[start_wp_name] = 0
        f_score = {name: float('inf') for name in self.waypoints}
        goal_pos = self.waypoints[goal_wp_name]['position']
        start_pos = self.waypoints[start_wp_name]['position']
        
        # 시작점에서 목표점까지의 직선 거리 계산
        direct_distance = math.sqrt((goal_pos['x'] - start_pos['x'])**2 + (goal_pos['y'] - start_pos['y'])**2)
        f_score[start_wp_name] = direct_distance
        
        self.get_logger().info(f'🎯 A* 경로 탐색 시작: {start_wp_name} -> {goal_wp_name} (직선거리: {direct_distance:.2f}m)')
        self.get_logger().info(f'🚫 제외할 웨이포인트: {list(self.blocked_waypoints)}')
        
        while open_set:
            _, current_name = heapq.heappop(open_set)
            
            if current_name == goal_wp_name:
                # 경로 재구성
                path = []
                while current_name in came_from:
                    path.append(current_name)
                    current_name = came_from[current_name]
                path.append(start_wp_name)
                final_path = path[::-1]
                
                # 경로 분석 로그
                total_distance = sum(
                    math.sqrt((self.waypoints[final_path[i+1]]['position']['x'] - self.waypoints[final_path[i]]['position']['x'])**2 +
                             (self.waypoints[final_path[i+1]]['position']['y'] - self.waypoints[final_path[i]]['position']['y'])**2)
                    for i in range(len(final_path)-1)
                )
                efficiency = (direct_distance / total_distance * 100) if total_distance > 0 else 0
                
                self.get_logger().info(f'✅ 최적 경로 발견: {final_path}')
                self.get_logger().info(f'📊 경로 효율성: 직선({direct_distance:.2f}m) vs 실제({total_distance:.2f}m) = {efficiency:.1f}%')
                
                return final_path
            
            current_pos = self.waypoints[current_name]['position']
            neighbors = self.waypoints[current_name].get('neighbors', [])
            
            # 이웃 노드들을 직선 거리 기준으로 정렬 (가까운 순서대로 우선 탐색)
            def neighbor_distance_to_goal(neighbor_name):
                neighbor_pos = self.waypoints[neighbor_name]['position']
                return math.sqrt((goal_pos['x'] - neighbor_pos['x'])**2 + (goal_pos['y'] - neighbor_pos['y'])**2)
            
            neighbors.sort(key=neighbor_distance_to_goal)
            
            for neighbor_name in neighbors:
                # 막힌 웨이포인트는 완전히 건너뛰기
                if neighbor_name in self.blocked_waypoints:
                    self.get_logger().debug(f'🚫 막힌 웨이포인트 건너뛰기: {neighbor_name}')
                    continue
                    
                neighbor_pos = self.waypoints[neighbor_name]['position']
                
                # 기본 거리 비용 계산
                edge_distance = math.sqrt(
                    (neighbor_pos['x'] - current_pos['x'])**2 + 
                    (neighbor_pos['y'] - current_pos['y'])**2
                )
                
                # 직선 경로 우선 보너스: 목표 방향과 일치할수록 비용 감소
                goal_direction = math.atan2(goal_pos['y'] - current_pos['y'], goal_pos['x'] - current_pos['x'])
                move_direction = math.atan2(neighbor_pos['y'] - current_pos['y'], neighbor_pos['x'] - current_pos['x'])
                direction_diff = abs(goal_direction - move_direction)
                if direction_diff > math.pi:
                    direction_diff = 2 * math.pi - direction_diff
                
                # 방향이 목표와 일치할수록 비용 감소 (최대 20% 할인)
                direction_bonus = 1.0 - (0.2 * (1.0 - direction_diff / math.pi))
                
                tentative_g_score = g_score[current_name] + (edge_distance * direction_bonus)
                
                if tentative_g_score < g_score[neighbor_name]:
                    came_from[neighbor_name] = current_name
                    g_score[neighbor_name] = tentative_g_score
                    
                    # 휴리스틱: 목표까지의 직선 거리
                    h_score = math.sqrt(
                        (goal_pos['x'] - neighbor_pos['x'])**2 + 
                        (goal_pos['y'] - neighbor_pos['y'])**2
                    )
                    f_score[neighbor_name] = tentative_g_score + h_score
                    
                    if neighbor_name not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor_name], neighbor_name))
                        self.get_logger().debug(f'🔍 탐색 추가: {neighbor_name} (f_score: {f_score[neighbor_name]:.2f})')
        
        self.get_logger().error(f'❌ {start_wp_name}에서 {goal_wp_name}까지 경로를 찾을 수 없습니다!')
        return None

    # --- 서비스 콜백 함수들 (우리 노드의 새로운 '메인 진입점') ---
    def set_goal_callback(self, request, response):
        """[API] 외부로부터 목표 지점 요청을 받으면 이 함수가 실행됩니다."""
        # 취소 중인 경우 목표를 대기열에 저장
        if self.is_canceling:
            self.get_logger().info(f"취소 진행 중 - 새 목표를 대기열에 저장: ({request.x:.2f}, {request.y:.2f})")
            self.pending_goal = request
            response.success = True
            response.message = "취소 완료 후 처리 예정"
            return response
            
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

        return self._process_goal_request(request, response)
    
    def _process_goal_request(self, request, response):
        """목표 요청을 실제로 처리하는 내부 함수"""
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
        
        # 취소 진행 상태로 설정
        self.is_canceling = True
        
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
        
        # 0.5초 후 취소 완료 처리 (Nav2 취소 처리 시간 확보)
        self.create_timer(0.5, self.complete_cancellation, once=True)
        
        response.success = True
        response.message = "주행 취소 요청을 보냈습니다."
        return response

    def complete_cancellation(self):
        """취소 완료 후 대기 중인 목표가 있으면 처리"""
        self.is_canceling = False
        self.get_logger().info("주행 취소 완료!")
        
        # 대기 중인 목표가 있으면 처리
        if self.pending_goal is not None:
            self.get_logger().info("대기 중인 목표를 처리합니다...")
            
            # 더미 response 객체 생성 (실제로는 사용되지 않음)
            from libo_interfaces.srv import SetGoal
            response = SetGoal.Response()
            
            result = self._process_goal_request(self.pending_goal, response)
            
            if result.success:
                self.get_logger().info("대기 중인 목표 처리 완료!")
            else:
                self.get_logger().error(f"대기 중인 목표 처리 실패: {result.message}")
            
            self.pending_goal = None

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
        
        # 헤딩이 계산된 웨이포인트 리스트 생성
        waypoint_poses = self.create_waypoint_poses_with_heading(path_wp_names)
        
        # 웨이포인트 이름 저장 (재계획에 필요)
        self.current_waypoint_names = path_wp_names
        
        self.start_navigation(waypoint_poses)
        return True

    def start_navigation(self, waypoint_poses):
        """웨이포인트를 단계별로 주행합니다 (FollowWaypoints 대신 개별 NavigateToPose 사용)."""
        self.get_logger().info(f"{len(waypoint_poses)}개의 지점으로 단계별 주행을 시작합니다...")
        self.current_state = NavigatorState.NAVIGATING
        
        # 웨이포인트 추적을 위한 변수 설정
        self.current_waypoint_poses = waypoint_poses
        self.current_waypoint_index = 0
        
        # 첫 번째 웨이포인트로 이동 시작
        self.navigate_to_current_waypoint()

    def navigate_to_current_waypoint(self):
        """현재 인덱스의 웨이포인트로 이동합니다."""
        if self.current_waypoint_index >= len(self.current_waypoint_poses):
            # 모든 웨이포인트 완료
            self.get_logger().info('🎉 모든 웨이포인트 주행 완료!')
            self.current_state = NavigatorState.IDLE
            self.blocked_waypoints.clear()
            self.notify_navigation_done("SUCCEEDED")
            return
        
        target = self.current_waypoint_poses[self.current_waypoint_index]
        wp_name = self.current_waypoint_names[self.current_waypoint_index] if self.current_waypoint_index < len(self.current_waypoint_names) else f"WP_{self.current_waypoint_index}"
        
        self.get_logger().info(f'🚀 웨이포인트 {self.current_waypoint_index + 1}/{len(self.current_waypoint_poses)} ({wp_name})로 이동 시작')
        self.get_logger().info(f'   목표: ({target.pose.position.x:.2f}, {target.pose.position.y:.2f})')
        
        # 🔥 핵심 해결책: 중간 웨이포인트가 있으면 더 가까운 목표로 설정
        if self.robot_current_pose:
            current_x = self.robot_current_pose.position.x
            current_y = self.robot_current_pose.position.y
            target_x = target.pose.position.x
            target_y = target.pose.position.y
            
            # 현재 위치에서 목표까지 거리 계산
            distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            # ETA 모니터링 시작 준비
            self.navigation_start_time = time.time()
            self.initial_eta_estimate = distance / 0.5  # 가정: 평균 속도 0.5m/s
            self.initial_direct_distance = distance  # 초기 직선 거리 저장
            self.eta_monitoring_active = True
            
            self.get_logger().info(f'📊 경로 모니터링 시작 - 직선거리: {distance:.2f}m, 우회감지 임계값: 1.5배({distance * 1.5:.2f}m)')
            
            # 거리가 2m 이상이면 중간 지점 생성 (Nav2 우회 방지)
            if distance > 2.0:
                # 현재 위치에서 목표 방향으로 1.5m 지점을 중간 목표로 설정
                direction_x = (target_x - current_x) / distance
                direction_y = (target_y - current_y) / distance
                
                intermediate_target = PoseStamped()
                intermediate_target.header.frame_id = 'map'
                intermediate_target.header.stamp = self.get_clock().now().to_msg()
                intermediate_target.pose.position.x = current_x + direction_x * 1.5
                intermediate_target.pose.position.y = current_y + direction_y * 1.5
                intermediate_target.pose.position.z = 0.0
                
                # 목표 방향으로 헤딩 설정
                yaw = math.atan2(direction_y, direction_x)
                qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
                intermediate_target.pose.orientation.x = qx
                intermediate_target.pose.orientation.y = qy
                intermediate_target.pose.orientation.z = qz
                intermediate_target.pose.orientation.w = qw
                
                self.get_logger().info(f'📍 거리 {distance:.2f}m > 2m: 중간 목표 설정 ({intermediate_target.pose.position.x:.2f}, {intermediate_target.pose.position.y:.2f})')
                
                # 중간 목표로 Nav2 실행
                self.navigator.goToPose(intermediate_target)
            else:
                # 거리가 가까우면 원래 목표로 직접 이동
                self.get_logger().info(f'📍 거리 {distance:.2f}m <= 2m: 직접 이동')
                self.navigator.goToPose(target)
        else:
            # 로봇 위치를 모르면 원래 목표로 이동
            self.navigator.goToPose(target)
        
        # 상태 확인 타이머 시작 (ETA 모니터링 포함)
        self.status_check_timer = self.create_timer(0.5, self.check_single_navigation_status)

    def check_single_navigation_status(self):
        """단일 웨이포인트 주행 상태를 확인하고 우회로를 모니터링합니다."""
        # 우회로 감지는 이제 path_callback에서 처리됨
        # 여기서는 추가적인 ETA 기반 모니터링만 수행
        if self.eta_monitoring_active and self.navigation_start_time:
            elapsed_time = time.time() - self.navigation_start_time
            
            # 최소 모니터링 시간 경과 후부터 감지 시작
            if elapsed_time >= self.min_monitoring_time:
                current_eta = self.get_current_navigation_eta()
                
                if current_eta and self.initial_eta_estimate:
                    eta_ratio = current_eta / self.initial_eta_estimate
                    
                    self.get_logger().debug(f'📊 ETA 백업 모니터링 - 초기: {self.initial_eta_estimate:.1f}s, 현재: {current_eta:.1f}s, 비율: {eta_ratio:.2f}')
                    
                    # 백업 ETA 급증 감지 (경로 기반 감지가 실패한 경우를 위한 보완)
                    if eta_ratio >= self.eta_spike_threshold:
                        self.get_logger().warn(f'🚨 백업 ETA 급증 감지! {self.initial_eta_estimate:.1f}s -> {current_eta:.1f}s (x{eta_ratio:.1f})')
                        
                        # ETA 모니터링 중단
                        self.eta_monitoring_active = False
                        
                        # 즉시 Nav2 중단하고 웨이포인트 재최적화
                        self.handle_eta_spike_detection()
                        return
        
        # 기존 네비게이션 완료 확인
        if not self.navigator.isTaskComplete():
            return
        
        # ETA 모니터링 종료
        self.eta_monitoring_active = False
        
        # 타이머 정리
        if self.status_check_timer is not None:
            self.destroy_timer(self.status_check_timer)
            self.status_check_timer = None
            
        result = self.navigator.getResult()
        wp_name = self.current_waypoint_names[self.current_waypoint_index] if self.current_waypoint_index < len(self.current_waypoint_names) else f"WP_{self.current_waypoint_index}"
        
        if result == TaskResult.SUCCEEDED:
            # 목표 웨이포인트에 도달했는지 확인
            target = self.current_waypoint_poses[self.current_waypoint_index]
            current_x = self.robot_current_pose.position.x if self.robot_current_pose else 0
            current_y = self.robot_current_pose.position.y if self.robot_current_pose else 0
            target_x = target.pose.position.x
            target_y = target.pose.position.y
            
            distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            # 목표 웨이포인트에 충분히 가까우면 다음 웨이포인트로
            if distance_to_target <= 0.5:  # 50cm 이내
                self.get_logger().info(f'✅ 웨이포인트 {wp_name} 도달 완료! (거리: {distance_to_target:.2f}m)')
                # 다음 웨이포인트로 진행
                self.current_waypoint_index += 1
                self.navigate_to_current_waypoint()
            else:
                # 아직 목표 웨이포인트에 도달하지 않았으면 계속 이동
                self.get_logger().info(f'🔄 중간 목표 도달, 웨이포인트 {wp_name}로 계속 이동 (남은 거리: {distance_to_target:.2f}m)')
                self.navigate_to_current_waypoint()
            
        elif result == TaskResult.CANCELED:
            if not self.is_canceling:
                self.get_logger().warn(f'❌ 웨이포인트 {wp_name} 주행이 취소됨')
                # 재계획 로직 실행
                self.handle_dynamic_obstacle_delayed()
            else:
                self.get_logger().info('외부 요청에 의한 주행 취소')
                self.current_state = NavigatorState.IDLE
                
        else:  # FAILED
            self.get_logger().warn(f'❌ 웨이포인트 {wp_name} 주행 실패!')
            # 실패한 웨이포인트를 블랙리스트에 추가하고 재계획
            self.blocked_waypoints.add(wp_name)
            self.get_logger().warn(f'⛔ 웨이포인트 {wp_name}을 블랙리스트에 추가')
            self.handle_dynamic_obstacle_delayed()

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

    def get_current_navigation_eta(self):
        """현재 네비게이션의 예상 도착 시간을 계산합니다."""
        try:
            if not self.robot_current_pose:
                return None
            
            # 목표 웨이포인트까지의 남은 거리 계산
            target = self.current_waypoint_poses[self.current_waypoint_index]
            current_x = self.robot_current_pose.position.x
            current_y = self.robot_current_pose.position.y
            target_x = target.pose.position.x
            target_y = target.pose.position.y
            
            remaining_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            # 평균 속도로 ETA 계산 (Nav2가 우회로를 생성하면 거리가 급증함)
            estimated_speed = 0.5  # m/s
            eta = remaining_distance / estimated_speed
            
            return eta
            
        except Exception as e:
            self.get_logger().debug(f'ETA 계산 중 오류: {e}')
            return None

    def handle_eta_spike_detection(self):
        """우회로 감지 시 Nav2 중단하고 웨이포인트 재최적화를 실행합니다."""
        try:
            self.get_logger().warn('🛑 Nav2 우회로 감지! 즉시 중단하고 웨이포인트 재최적화 시작!')
            
            # 1. Nav2 즉시 중단
            self.navigator.cancelTask()
            
            # 2. 타이머 정리
            if self.status_check_timer is not None:
                self.destroy_timer(self.status_check_timer)
                self.status_check_timer = None
            
            # 3. 현재 웨이포인트를 문제 있는 것으로 간주하고 제외
            if hasattr(self, 'current_waypoint_names') and hasattr(self, 'current_waypoint_index'):
                if self.current_waypoint_index < len(self.current_waypoint_names):
                    problematic_wp = self.current_waypoint_names[self.current_waypoint_index]
                    self.blocked_waypoints.add(problematic_wp)
                    self.get_logger().warn(f'⛔ 우회로 발생 웨이포인트 {problematic_wp}를 제외 리스트에 추가')
            
            # 4. 0.1초 후 웨이포인트 재최적화 실행 (Nav2 취소 처리 시간 확보)
            self.current_state = NavigatorState.IDLE
            self._replanning = True
            # once=True 인자 대신 타이머 콜백 내에서 직접 destroy_timer 호출
            def eta_replan_callback():
                self.execute_eta_based_replan()
                # 타이머 객체를 직접 파괴
                if hasattr(self, 'eta_replan_timer') and self.eta_replan_timer is not None:
                    self.destroy_timer(self.eta_replan_timer)
                    self.eta_replan_timer = None
            self.eta_replan_timer = self.create_timer(0.1, eta_replan_callback)
            
        except Exception as e:
            self.get_logger().error(f'우회로 감지 처리 중 오류: {e}')
            self.current_state = NavigatorState.ERROR

    def execute_eta_based_replan(self):
        """우회로 감지 후 웨이포인트 재계획을 실행합니다."""
        try:
            self.get_logger().info('🔄 우회로 감지 기반 웨이포인트 재최적화 실행 중...')
            
            # 현재 위치에서 목표까지 새로운 웨이포인트 경로 계산
            start_wp = self.get_closest_waypoint(self.robot_current_pose)
            goal_wp = self.get_closest_waypoint(self.current_goal_pose)
            
            if not start_wp or not goal_wp:
                self.get_logger().error("우회로 감지 기반 재계획을 위한 시작/목표 웨이포인트를 찾을 수 없습니다.")
                self.current_state = NavigatorState.ERROR
                self._replanning = False
                return
            
            # 문제가 있던 웨이포인트를 제외한 새로운 경로 계산
            path_wp_names = self.find_path_astar_with_blocked(start_wp, goal_wp)
            
            if not path_wp_names:
                self.get_logger().error("❌ 문제 웨이포인트를 제외한 대체 경로를 찾을 수 없습니다!")
                self.current_state = NavigatorState.ERROR
                self._replanning = False
                return
            
            self.get_logger().info(f"✅ 우회로 감지 기반 새로운 웨이포인트 경로: {path_wp_names}")
            self.get_logger().info(f"🚫 제외된 문제 웨이포인트: {list(self.blocked_waypoints)}")
            
            # 새로운 웨이포인트 리스트로 헤딩 계산
            waypoint_poses = self.create_waypoint_poses_with_heading(path_wp_names)
            
            # 새로운 웨이포인트 경로로 주행 재시작
            self.current_waypoint_names = path_wp_names
            self.current_waypoint_poses = waypoint_poses
            self.current_waypoint_index = 0
            
            self.get_logger().info(f'🚀 우회로 감지 기반 재최적화 경로로 주행 재시작! ({len(waypoint_poses)}개 지점)')
            self.start_navigation(waypoint_poses)
            
            self._replanning = False
            
        except Exception as e:
            self.get_logger().error(f'우회로 감지 기반 재계획 실행 중 오류: {e}')
            self.current_state = NavigatorState.ERROR
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

    def calculate_heading_to_next_waypoint(self, current_wp_name, next_wp_name):
        """현재 웨이포인트에서 다음 웨이포인트로의 헤딩을 계산합니다."""
        if not current_wp_name or not next_wp_name:
            return [0, 0, 0, 1]  # 기본 방향 (0라디안)
        
        try:
            current_pos = self.waypoints[current_wp_name]['position']
            next_pos = self.waypoints[next_wp_name]['position']
            
            # 다음 웨이포인트로의 방향 계산
            dx = next_pos['x'] - current_pos['x']
            dy = next_pos['y'] - current_pos['y']
            
            # atan2로 헤딩 각도 계산 (라디안)
            yaw = math.atan2(dy, dx)
            
            # 쿼터니언으로 변환
            qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
            
            self.get_logger().debug(f'헤딩 계산: {current_wp_name} -> {next_wp_name}, yaw: {yaw:.3f}rad ({math.degrees(yaw):.1f}도)')
            
            return [qx, qy, qz, qw]
            
        except Exception as e:
            self.get_logger().warn(f'헤딩 계산 중 오류: {e}')
            return [0, 0, 0, 1]

    def create_waypoint_poses_with_heading(self, path_wp_names):
        """웨이포인트 경로에서 각 지점의 헤딩을 계산하여 PoseStamped 리스트를 생성합니다."""
        waypoint_poses = []
        
        for i, name in enumerate(path_wp_names):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = self.waypoints[name]['position']['x']
            pose.pose.position.y = self.waypoints[name]['position']['y']
            pose.pose.position.z = 0.0
            
            # 헤딩 계산
            if i < len(path_wp_names) - 1:
                # 다음 웨이포인트가 있으면 그 방향으로 헤딩 설정
                next_name = path_wp_names[i + 1]
                qx, qy, qz, qw = self.calculate_heading_to_next_waypoint(name, next_name)
            else:
                # 마지막 웨이포인트는 이전 헤딩 유지 또는 기본값
                if i > 0:
                    prev_name = path_wp_names[i - 1]
                    qx, qy, qz, qw = self.calculate_heading_to_next_waypoint(prev_name, name)
                else:
                    qx, qy, qz, qw = [0, 0, 0, 1]  # 기본 방향
            
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            waypoint_poses.append(pose)
            
            self.get_logger().debug(f'웨이포인트 {i}: {name} -> 헤딩: ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})')
        
        return waypoint_poses

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
        """직선 경로 우선 A* 알고리즘으로 최적 경로를 찾습니다."""
        open_set = []
        heapq.heappush(open_set, (0, start_wp_name))
        came_from = {}
        g_score = {name: float('inf') for name in self.waypoints}
        g_score[start_wp_name] = 0
        f_score = {name: float('inf') for name in self.waypoints}
        goal_pos = self.waypoints[goal_wp_name]['position']
        start_pos = self.waypoints[start_wp_name]['position']
        
        # 시작점에서 목표점까지의 직선 거리
        direct_distance = math.sqrt((goal_pos['x'] - start_pos['x'])**2 + (goal_pos['y'] - start_pos['y'])**2)
        f_score[start_wp_name] = direct_distance
        
        self.get_logger().info(f'🎯 직선 우선 A* 탐색: {start_wp_name} -> {goal_wp_name} (직선거리: {direct_distance:.2f}m)')
        
        while open_set:
            _, current_name = heapq.heappop(open_set)
            
            if current_name == goal_wp_name:
                # 경로 재구성
                path = []
                while current_name in came_from:
                    path.append(current_name)
                    current_name = came_from[current_name]
                path.append(start_wp_name)
                final_path = path[::-1]
                
                # 경로 효율성 계산
                total_distance = sum(
                    math.sqrt((self.waypoints[final_path[i+1]]['position']['x'] - self.waypoints[final_path[i]]['position']['x'])**2 +
                             (self.waypoints[final_path[i+1]]['position']['y'] - self.waypoints[final_path[i]]['position']['y'])**2)
                    for i in range(len(final_path)-1)
                )
                efficiency = (direct_distance / total_distance * 100) if total_distance > 0 else 0
                
                self.get_logger().info(f'✅ 최적 경로: {final_path}')
                self.get_logger().info(f'📊 효율성: {efficiency:.1f}% (직선: {direct_distance:.2f}m, 실제: {total_distance:.2f}m)')
                
                return final_path
            
            current_pos = self.waypoints[current_name]['position']
            neighbors = self.waypoints[current_name].get('neighbors', [])
            
            # 이웃 노드들을 목표까지의 직선 거리로 정렬 (가까운 것부터 우선 탐색)
            def neighbor_distance_to_goal(neighbor_name):
                neighbor_pos = self.waypoints[neighbor_name]['position']
                return math.sqrt((goal_pos['x'] - neighbor_pos['x'])**2 + (goal_pos['y'] - neighbor_pos['y'])**2)
            
            neighbors.sort(key=neighbor_distance_to_goal)
            
            for neighbor_name in neighbors:
                neighbor_pos = self.waypoints[neighbor_name]['position']
                
                # 기본 이동 거리 비용
                edge_distance = math.sqrt(
                    (neighbor_pos['x'] - current_pos['x'])**2 + 
                    (neighbor_pos['y'] - current_pos['y'])**2
                )
                
                # 직선 경로 우선 보너스 계산
                goal_direction = math.atan2(goal_pos['y'] - current_pos['y'], goal_pos['x'] - current_pos['x'])
                move_direction = math.atan2(neighbor_pos['y'] - current_pos['y'], neighbor_pos['x'] - current_pos['x'])
                direction_diff = abs(goal_direction - move_direction)
                if direction_diff > math.pi:
                    direction_diff = 2 * math.pi - direction_diff
                
                # 목표 방향과 일치할수록 비용 감소 (최대 25% 할인)
                direction_bonus = 1.0 - (0.25 * (1.0 - direction_diff / math.pi))
                
                # Costmap 위험도도 고려 (있다면)
                if self.current_costmap is not None:
                    danger_penalty = self.sample_costmap_danger(current_pos, neighbor_pos)
                    total_cost = (edge_distance * direction_bonus) + (danger_penalty * 0.1)
                else:
                    total_cost = edge_distance * direction_bonus
                
                tentative_g_score = g_score[current_name] + total_cost
                
                if tentative_g_score < g_score[neighbor_name]:
                    came_from[neighbor_name] = current_name
                    g_score[neighbor_name] = tentative_g_score
                    
                    # 휴리스틱: 목표까지의 직선 거리
                    h_score = math.sqrt(
                        (goal_pos['x'] - neighbor_pos['x'])**2 + 
                        (goal_pos['y'] - neighbor_pos['y'])**2
                    )
                    f_score[neighbor_name] = tentative_g_score + h_score
                    
                    if neighbor_name not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor_name], neighbor_name))
        
        self.get_logger().error(f'❌ 경로를 찾을 수 없습니다: {start_wp_name} -> {goal_wp_name}')
        return None

    def handle_dynamic_obstacle_delayed(self):
        """콜백 블로킹을 피하기 위한 지연된 장애물 처리 - 웨이포인트 그래프 재계산"""
        try:
            self.get_logger().warn('🚨 막힌 웨이포인트 감지! Nav2 글로벌 플래너 중단하고 새로운 웨이포인트 경로 계산 시작!')
            
            # 1. 현재 네비게이션 즉시 취소 (Nav2 글로벌 플래너 중단)
            self.navigator.cancelTask()
            
            # 2. 타이머 정리
            if self.status_check_timer is not None:
                self.destroy_timer(self.status_check_timer)
                self.status_check_timer = None
            
            # 3. 상태를 즉시 변경하여 추가 감지 방지
            self.current_state = NavigatorState.IDLE
            
            # 4. 막힌 웨이포인트를 웨이포인트 그래프에서 완전 제외
            if hasattr(self, 'current_waypoint_names') and hasattr(self, 'current_waypoint_index'):
                if self.current_waypoint_index < len(self.current_waypoint_names):
                    blocked_wp = self.current_waypoint_names[self.current_waypoint_index]
                    self.blocked_waypoints.add(blocked_wp)
                    self.get_logger().warn(f'⛔ 웨이포인트 {blocked_wp}를 웨이포인트 그래프에서 완전 제외!')
            
            # 5. 즉시 웨이포인트 기반 재계획 실행 (Nav2 우회로 생성 방지)
            self.create_timer(0.0, self.execute_waypoint_replan, once=True)
            
        except Exception as e:
            self.get_logger().error(f'웨이포인트 재계획 처리 중 오류: {e}')
            self.current_state = NavigatorState.ERROR
            self._replanning = False

    def execute_waypoint_replan(self):
        """웨이포인트 그래프에서 막힌 노드를 제외한 새로운 경로 계산 및 실행"""
        try:
            self.get_logger().info('🔄 웨이포인트 그래프 기반 새 경로 계산 중...')
            
            # 현재 위치에서 목표까지 새로운 웨이포인트 경로 계산
            start_wp = self.get_closest_waypoint(self.robot_current_pose)
            goal_wp = self.get_closest_waypoint(self.current_goal_pose)
            
            if not start_wp or not goal_wp:
                self.get_logger().error("웨이포인트 재계획을 위한 시작/목표 웨이포인트를 찾을 수 없습니다.")
                self.current_state = NavigatorState.ERROR
                self._replanning = False
                return
            
            # 막힌 웨이포인트를 완전히 제외한 A* 경로 계산
            path_wp_names = self.find_path_astar_with_blocked(start_wp, goal_wp)
            
            if not path_wp_names:
                self.get_logger().error("❌ 막힌 웨이포인트를 제외한 대체 경로를 찾을 수 없습니다!")
                self.current_state = NavigatorState.ERROR
                self._replanning = False
                return
            
            self.get_logger().info(f"✅ 새로운 웨이포인트 경로 계산 완료: {path_wp_names}")
            self.get_logger().info(f"🚫 제외된 웨이포인트: {list(self.blocked_waypoints)}")
            
            # 새로운 웨이포인트 리스트로 헤딩 계산
            waypoint_poses = self.create_waypoint_poses_with_heading(path_wp_names)
            
            # 새로운 웨이포인트 경로로 주행 시작 (Nav2는 이 웨이포인트들만 사용)
            self.current_waypoint_names = path_wp_names
            self.current_waypoint_poses = waypoint_poses
            self.current_waypoint_index = 0
            
            self.get_logger().info(f'🚀 새로운 웨이포인트 경로로 주행 재시작! ({len(waypoint_poses)}개 지점)')
            self.start_navigation(waypoint_poses)
            
            self._replanning = False
            
        except Exception as e:
            self.get_logger().error(f'웨이포인트 재계획 실행 중 오류: {e}')
            self.current_state = NavigatorState.ERROR
            self._replanning = False

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
