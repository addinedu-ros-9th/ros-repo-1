#!/usr/bin/env python3
import rclpy
import yaml
import math
import heapq
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory

class OptimalWaypointNavigator(Node):
    def __init__(self):
        super().__init__('optimal_waypoint_navigator')
        self.waypoints = {}
        self.navigator = BasicNavigator()
        self.robot_current_pose = None
        self.initial_pose_received = False

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
            amcl_qos_profile)
        
        self.get_logger().info("Optimal Waypoint Navigator 시작됨. 초기 위치 설정을 기다립니다...")
        self.timer = self.create_timer(1.0, self.main_logic)

    def amcl_pose_callback(self, msg):
        # amcl이 발행하는 초기 (0,0) 위치는 부정확하므로 무시하는 로직
        # covariance[0]은 x위치의 불확실성을 의미, 이 값이 크면 아직 위치가 부정확하다는 뜻
        if msg.pose.covariance[0] > 0.1:
            self.get_logger().warn('AMCL 위치의 불확실성이 너무 높습니다. 무시합니다.', throttle_duration_sec=5)
            return
            
        self.robot_current_pose = msg.pose.pose
        if not self.initial_pose_received:
            self.get_logger().info('AMCL로부터 유효한 첫 위치 정보를 받았습니다! 초기 위치가 설정되었습니다.')
            self.initial_pose_received = True

    def main_logic(self):
        if not self.initial_pose_received:
            self.get_logger().warn('초기 위치가 설정되기를 기다리는 중...', throttle_duration_sec=5)
            return
        self.destroy_timer(self.timer)
        self.run_navigation()

    def load_waypoints(self):
        share_dir = get_package_share_directory('libo_waypoint_runner')
        waypoint_file_path = f'{share_dir}/config/waypoints.yaml'
        self.get_logger().info(f"웨이포인트 파일 로딩: {waypoint_file_path}")
        try:
            with open(waypoint_file_path, 'r') as file:
                data = yaml.safe_load(file)
                # 사용자님의 YAML 파일 구조에 맞춰, 'waypoints:' 키 없이 바로 데이터를 읽어옵니다.
                self.waypoints = data
                if not self.waypoints:
                    self.get_logger().error("YAML 파일은 비어있지 않지만, 웨이포인트 데이터를 로드하지 못했습니다!")
                    return False
                self.get_logger().info(f"성공적으로 {len(self.waypoints)}개의 웨이포인트를 로드했습니다.")
                return True
        except Exception as e:
            self.get_logger().error(f"웨이포인트 파일 로딩 실패: {e}")
            return False

    def get_closest_waypoint(self, pose):
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

    def run_navigation(self):
        if not self.load_waypoints():
            rclpy.shutdown()
            return

        self.navigator.waitUntilNav2Active()
        
        self.get_logger().info(f"로봇의 현재 위치: ({self.robot_current_pose.position.x:.2f}, {self.robot_current_pose.position.y:.2f})")

        goal_pose_test = PoseStamped().pose
        # goal_pose_test.position.x = 8.98
        # goal_pose_test.position.y = -0.16
        goal_pose_test.position.x = 8.97
        goal_pose_test.position.y = 1.5

        start_wp = self.get_closest_waypoint(self.robot_current_pose)
        goal_wp = self.get_closest_waypoint(goal_pose_test)
        
        if not start_wp or not goal_wp:
            self.get_logger().error(f"시작({start_wp}) 또는 목표({goal_wp}) 웨이포인트를 찾을 수 없습니다. 프로그램을 종료합니다.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"경로 탐색 시작: {start_wp} -> {goal_wp}")
        path_wp_names = self.find_path_astar(start_wp, goal_wp)

        if not path_wp_names:
            self.get_logger().error("최적 경로를 찾지 못했습니다!")
            rclpy.shutdown()
            return

        self.get_logger().info(f"계산된 최적 경로: {path_wp_names}")
        waypoint_poses = []
        for name in path_wp_names:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = self.waypoints[name]['position']['x']
            pose.pose.position.y = self.waypoints[name]['position']['y']
            pose.pose.orientation.w = 1.0
            waypoint_poses.append(pose)
        
        self.navigator.followWaypoints(waypoint_poses)
        
        # --- rclpy.spin_once() 문법 오류를 최종 수정합니다 ---
        while not self.navigator.isTaskComplete():
             feedback = self.navigator.getFeedback()
             if feedback:
                 self.get_logger().info(f'현재 웨이포인트 {feedback.current_waypoint + 1}/{len(waypoint_poses)} 로 이동 중...')
             # self는 현재 노드 객체를 의미합니다.
             rclpy.spin_once(self, timeout_sec=1.0)
        # ---
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('성공! 모든 웨이포인트에 도달했습니다.')
        else:
            self.get_logger().info(f'주행 실패! 최종 상태: {result.value}')
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OptimalWaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# import yaml
# import math
# import heapq
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
# from ament_index_python.packages import get_package_share_directory

# class OptimalWaypointNavigator(Node):
#     def __init__(self):
#         super().__init__('optimal_waypoint_navigator')
#         self.waypoints = {}
#         self.navigator = BasicNavigator()
#         self.robot_current_pose = None
#         self.initial_pose_received = False

#         amcl_qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         self.create_subscription(
#             PoseWithCovarianceStamped,
#             '/amcl_pose',
#             self.amcl_pose_callback,
#             amcl_qos_profile)
        
#         self.get_logger().info("Optimal Waypoint Navigator 시작됨. 초기 위치 설정을 기다립니다...")
#         self.timer = self.create_timer(1.0, self.main_logic)

#     def amcl_pose_callback(self, msg):
#         self.robot_current_pose = msg.pose.pose
#         if not self.initial_pose_received:
#             self.get_logger().info('AMCL로부터 첫 위치 정보를 받았습니다! 초기 위치가 설정되었습니다.')
#             self.initial_pose_received = True

#     def main_logic(self):
#         if not self.initial_pose_received:
#             self.get_logger().warn('초기 위치가 설정되기를 기다리는 중...', throttle_duration_sec=5)
#             return
#         self.destroy_timer(self.timer)
#         self.run_navigation()

#     def load_waypoints(self):
#         share_dir = get_package_share_directory('libo_waypoint_runner')
#         waypoint_file_path = f'{share_dir}/config/waypoints.yaml'
#         self.get_logger().info(f"웨이포인트 파일 로딩: {waypoint_file_path}")
#         try:
#             with open(waypoint_file_path, 'r') as file:
#                 data = yaml.safe_load(file)
#                 # --- YAML 파일 구조에 맞게 수정된 부분 ---
#                 # 이제 'waypoints:' 키 없이 바로 데이터를 읽어옵니다.
#                 self.waypoints = data
#                 if not self.waypoints:
#                     self.get_logger().error("YAML 파일은 비어있지 않지만, 웨이포인트 데이터를 로드하지 못했습니다!")
#                     return False
#                 self.get_logger().info(f"성공적으로 {len(self.waypoints)}개의 웨이포인트를 로드했습니다.")
#                 return True
#         except Exception as e:
#             self.get_logger().error(f"웨이포인트 파일 로딩 실패: {e}")
#             return False

#     def get_closest_waypoint(self, pose):
#         min_dist = float('inf')
#         closest_wp_name = None
#         if not self.waypoints:
#             return None
#         for name, data in self.waypoints.items():
#             dist = math.sqrt((pose.position.x - data['position']['x'])**2 +
#                              (pose.position.y - data['position']['y'])**2)
#             if dist < min_dist:
#                 min_dist = dist
#                 closest_wp_name = name
#         return closest_wp_name

#     def find_path_astar(self, start_wp_name, goal_wp_name):
#         open_set = []
#         heapq.heappush(open_set, (0, start_wp_name))
#         came_from = {}
#         g_score = {name: float('inf') for name in self.waypoints}
#         g_score[start_wp_name] = 0
#         f_score = {name: float('inf') for name in self.waypoints}
#         goal_pos = self.waypoints[goal_wp_name]['position']
#         start_pos = self.waypoints[start_wp_name]['position']
#         f_score[start_wp_name] = math.sqrt((goal_pos['x'] - start_pos['x'])**2 + (goal_pos['y'] - start_pos['y'])**2)
#         while open_set:
#             _, current_name = heapq.heappop(open_set)
#             if current_name == goal_wp_name:
#                 path = []
#                 while current_name in came_from:
#                     path.append(current_name)
#                     current_name = came_from[current_name]
#                 path.append(start_wp_name)
#                 return path[::-1]
#             current_pos = self.waypoints[current_name]['position']
#             for neighbor_name in self.waypoints[current_name].get('neighbors', []):
#                 neighbor_pos = self.waypoints[neighbor_name]['position']
#                 tentative_g_score = g_score[current_name] + math.sqrt((neighbor_pos['x'] - current_pos['x'])**2 + (neighbor_pos['y'] - current_pos['y'])**2)
#                 if tentative_g_score < g_score[neighbor_name]:
#                     came_from[neighbor_name] = current_name
#                     g_score[neighbor_name] = tentative_g_score
#                     h_score = math.sqrt((goal_pos['x'] - neighbor_pos['x'])**2 + (goal_pos['y'] - neighbor_pos['y'])**2)
#                     f_score[neighbor_name] = tentative_g_score + h_score
#                     if neighbor_name not in [i[1] for i in open_set]:
#                         heapq.heappush(open_set, (f_score[neighbor_name], neighbor_name))
#         return None

#     def run_navigation(self):
#         if not self.load_waypoints():
#             rclpy.shutdown()
#             return

#         self.navigator.waitUntilNav2Active()
        
#         self.get_logger().info(f"로봇의 현재 위치: ({self.robot_current_pose.position.x:.2f}, {self.robot_current_pose.position.y:.2f})")

#         goal_pose_test = PoseStamped().pose
#         goal_pose_test.position.x = 8.98
#         goal_pose_test.position.y = -0.16

#         start_wp = self.get_closest_waypoint(self.robot_current_pose)
#         goal_wp = self.get_closest_waypoint(goal_pose_test)
        
#         if not start_wp or not goal_wp:
#             self.get_logger().error(f"시작({start_wp}) 또는 목표({goal_wp}) 웨이포인트를 찾을 수 없습니다. 프로그램을 종료합니다.")
#             rclpy.shutdown()
#             return

#         self.get_logger().info(f"경로 탐색 시작: {start_wp} -> {goal_wp}")
#         path_wp_names = self.find_path_astar(start_wp, goal_wp)

#         if not path_wp_names:
#             self.get_logger().error("최적 경로를 찾지 못했습니다!")
#             rclpy.shutdown()
#             return

#         self.get_logger().info(f"계산된 최적 경로: {path_wp_names}")
#         waypoint_poses = []
#         for name in path_wp_names:
#             pose = PoseStamped()
#             pose.header.frame_id = 'map'
#             pose.header.stamp = self.navigator.get_clock().now().to_msg()
#             pose.pose.position.x = self.waypoints[name]['position']['x']
#             pose.pose.position.y = self.waypoints[name]['position']['y']
#             pose.pose.orientation.w = 1.0
#             waypoint_poses.append(pose)
        
#         self.navigator.followWaypoints(waypoint_poses)
        
#         while not self.navigator.isTaskComplete():
#              feedback = self.navigator.getFeedback()
#              if feedback:
#                  self.get_logger().info(f'현재 웨이포인트 {feedback.current_waypoint + 1}/{len(waypoint_poses)} 로 이동 중...')
#              rclpy.spin_once(timeout_sec=1.0)
        
#         result = self.navigator.getResult()
#         if result == TaskResult.SUCCEEDED:
#             self.get_logger().info('성공! 모든 웨이포인트에 도달했습니다.')
#         else:
#             self.get_logger().info(f'주행 실패! 최종 상태: {result.value}')
        
#         rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = OptimalWaypointNavigator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

# if __name__ == '__main__':
#     main()