#!/usr/bin/env python3


import rclpy # ROS 2 파이썬 클라이언트 라이브러리
from rclpy.node import Node # ROS 2 노드 클래스
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math  # 수학 함수 사용을 위한 모듈
import tf_transformations  # 오일러 → 쿼터니언 변환용 모듈




class RobotCommander(Node): # Node 클래스를 상속받아서 우리만의 노드 클래스를 만들어
    def __init__(self):
        super().__init__('robot_commander') # 'robot_commander'라는 이름으로 노드를 초기화하고, 부모 클래스의 생성자도 호출해
        self.get_logger().info('🤖 로봇 커맨더 노드가 시작되었습니다.') # 노드가 시작되면 로그를 남겨

        self.nav = BasicNavigator()

        # Nav2 랑 연결되는거 기다리기~ 다 돼면 알림!
        self.get_logger().info('Nav2 스택 활성화를 기다립니다...')
        self.nav.waitUntilNav2Active()
        self.get_logger().info('✅ Nav2 스택이 활성화되었습니다!')

        # 노드가 시작될 때 초기 위치를 설정해줌.
        # self._set_initial_pose()


    def _set_initial_pose(self):
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


def main(args=None):
    rclpy.init(args=args) # ROS 2 시스템을 초기화해
    
    robot_commander_node = RobotCommander() # 우리가 만든 RobotCommander 클래스의 인스턴스를 생성해
    
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