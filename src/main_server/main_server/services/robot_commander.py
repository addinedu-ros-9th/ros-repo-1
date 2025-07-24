#!/usr/bin/env python3

import rclpy # ROS 2 파이썬 클라이언트 라이브러리
from rclpy.node import Node # ROS 2 노드 클래스
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped # 메시지 타입 임포트
from rclpy.duration import Duration # 시간 관련 클래스
import math  # 수학 함수 사용을 위한 모듈
import tf_transformations  # 오일러 → 쿼터니언 변환용 모듈

class RobotCommander(Node): # Node 클래스를 상속받아서 우리만의 노드 클래스를 만들어
    def __init__(self):
        super().__init__('robot_commander') # 'robot_commander'라는 이름으로 노드를 초기화하고, 부모 클래스의 생성자도 호출해
        self.get_logger().info('🤖 로봇 커맨더 노드가 시작되었습니다.') # 노드가 시작되면 로그를 남겨
        self.current_pose = None # 로봇의 현재 위치를 저장할 변수

        self.nav = BasicNavigator()

        # Nav2 랑 연결되는거 기다리기~ 다 돼면 알림!
        self.get_logger().info('Nav2 스택 활성화를 기다립니다...')
        self.nav.waitUntilNav2Active()
        self.get_logger().info('✅ Nav2 스택이 활성화되었습니다!')

        
        # self._set_initial_pose() # 노드가 시작될 때 초기 위치를 설정해줌.


        # AMCL로부터 로봇의 현재 위치를 받아오는 subscriber 생성
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10
        )

    def go_to_pose(self, x, y, yaw_degrees): # 로봇을 움직이게 하는 코드!
        """지정한 좌표로 로봇을 이동시키고, 완료될 때까지 결과를 모니터링하는 함수."""
        self.get_logger().info(f"목표 지점으로 이동 시작: X={x}, Y={y}, Yaw={yaw_degrees}")

        # --- 1. 목표 지점(PoseStamped) 생성 ---
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        
        q = self.get_quaternion_from_yaw(yaw_degrees)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        # --- 2. 네비게이션 명령 전송 ---
        self.nav.goToPose(goal_pose)

        # --- 3. 피드백 모니터링 루프 ---
        while not self.nav.isTaskComplete(): # 만약 아직 안끝났다면
            # 이동하는 동안 다른 콜백(예: amcl_callback)이 처리되도록 spin_once를 호출해.
            # 이렇게 해야 로봇이 움직이는 중에도 현재 위치를 계속 업데이트할 수 있어.
            rclpy.spin_once(self, timeout_sec=0.1)

            feedback = self.nav.getFeedback() # 피드백 ㄱㄱ
            if feedback:
                self.get_logger().info('남은 거리: {:.2f} 미터.'.format(feedback.distance_remaining))

                # 60초 이상 걸리면 타임아웃으로 간주하고 작업 취소
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                    self.get_logger().warn('네비게이션 타임아웃! 작업을 취소합니다.')
                    self.nav.cancelTask()

        # --- 4. 최종 결과 확인 ---
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✅ 목표 지점 도착 성공!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('⚠️ 목표가 취소되었습니다.')
        elif result == TaskResult.FAILED:
            self.get_logger().error('❌ 목표 지점 도착 실패!')

    def _set_initial_pose(self): #로봇의 시작 위치를 설정해줌~
        """로봇의 초기 위치(initial pose)를 설정하는 함수. RViz2에서 2D Pose Estimate를 클릭하는 것과 같다."""
        self.get_logger().info('로봇의 초기 위치를 설정합니다...')

        initial_yaw = 0.0  # 초기 yaw 각도 설정 (단위: 도)
        q = self.get_quaternion_from_yaw(initial_yaw)  # yaw 각도를 쿼터니언으로 변환

        initial_pose = PoseStamped()  # PoseStamped 메시지 생성
        initial_pose.header.frame_id = 'map'  # 참조 좌표계는 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()  # 현재 시간
        initial_pose.pose.position.x = 0.0  # 초기 X 좌표
        initial_pose.pose.position.y = 0.0  # 초기 Y 좌표
        initial_pose.pose.position.z = 0.0  # 초기 Z 좌표
        initial_pose.pose.orientation.x = q[0]  # 쿼터니언 X
        initial_pose.pose.orientation.y = q[1]  # 쿼터니언 Y
        initial_pose.pose.orientation.z = q[2]  # 쿼터니언 Z
        initial_pose.pose.orientation.w = q[3]  # 쿼터니언 W

        self.nav.setInitialPose(initial_pose)  # 내비게이터에 초기 위치 설정
        self.get_logger().info('✅ 초기 위치 설정 완료!')

    def get_quaternion_from_yaw(self, yaw_degrees):  # yaw 각도로부터 쿼터니언 생성
        yaw_radians = math.radians(yaw_degrees)  # 도(degree)를 라디안(radian)으로 변환
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw_radians)  # Z축 회전만 적용
        return quaternion  # 쿼터니언 반환

    def amcl_callback(self, msg): # 실시간 로봇 위치 보기
        """/amcl_pose 토픽을 구독하여 로봇의 현재 위치를 업데이트하는 콜백 함수"""
        self.current_pose = msg.pose.pose
        
        # 아래 주석을 풀면 로봇의 현재 위치가 계속 출력돼. 디버깅할 때 유용해.
        # self.get_logger().info(
        #     f"로봇 현재 위치: x={self.current_pose.position.x:.2f}, y={self.current_pose.position.y:.2f}"
        # )

def main(args=None):
    rclpy.init(args=args) # ROS 2 시스템을 초기화해
    
    robot_commander_node = RobotCommander() # 우리가 만든 RobotCommander 클래스의 인스턴스를 생성해

    # --- 테스트를 위해 go_to_pose 함수를 직접 호출 ---
    # 이 노드를 실행하면 로봇이 (0.0, 1.0) 좌표로 이동을 시작할 거야.
    # 나중에는 이 부분을 서비스나 액션 콜백 안에서 호출하게 될 거야.
    robot_commander_node.go_to_pose(x=0.0, y=-1.0, yaw_degrees=0.0)
    # ---------------------------------------------
    
    try:
        rclpy.spin(robot_commander_node) # 노드가 종료될 때까지 (Ctrl+C) 계속 실행하면서 콜백을 처리해
    except KeyboardInterrupt:
        robot_commander_node.get_logger().info('사용자에 의해 노드가 종료됩니다.') # Ctrl+C로 종료될 때 메시지를 남겨
    finally:
        # 노드와 rclpy 리소스를 깔끔하게 정리해주는 부분
        robot_commander_node.destroy_node() # 노드를 파괴해서 리소스를 반환해
        if rclpy.ok(): # rclpy가 아직 실행 중이라면
            rclpy.shutdown() # ROS 2 시스템을 완전히 종료해

if __name__ == '__main__': # 이 스크립트가 직접 실행될 때만 main 함수를 호출해
    main()