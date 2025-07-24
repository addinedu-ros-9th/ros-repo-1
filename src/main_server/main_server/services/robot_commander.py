#!/usr/bin/env python3

import rclpy # ROS 2 파이썬 클라이언트 라이브러리
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped # 메시지 타입 임포트
from rclpy.duration import Duration # 시간 관련 클래스
from libo_interfaces.srv import Navigate # 우리가 만든 서비스 임포트
import math  # 수학 함수 사용을 위한 모듈
import tf_transformations  # 오일러 → 쿼터니언 변환용 모듈
import threading # 스레딩 라이브러리 임포트
import time # time.sleep()을 위한 임포트

class RobotCommander(BasicNavigator): # Node 대신 BasicNavigator를 직접 상속받아
    def __init__(self):
        # 'robot_commander'라는 이름으로 BasicNavigator 노드를 초기화
        super().__init__(node_name='robot_commander')
        self.get_logger().info('🤖 로봇 커맨더 노드가 시작되었습니다.') # 노드가 시작되면 로그를 남겨
        self.current_pose = None # 로봇의 현재 위치를 저장할 변수
        self.is_navigating = False # 현재 로봇이 이동 중인지 상태를 나타내는 플래그

        # Nav2 랑 연결되는거 기다리기~ 다 돼면 알림!
        self.get_logger().info('Nav2 스택 활성화를 기다립니다...')
        self.waitUntilNav2Active() # self.nav. 대신 self. 사용
        self.get_logger().info('✅ Nav2 스택이 활성화되었습니다!')

        
        # self._set_initial_pose() # 노드가 시작될 때 초기 위치를 설정해줌.


        # AMCL로부터 로봇의 현재 위치를 받아오는 subscriber 생성
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10
        )
        
        # 'navigate' 라는 이름으로 Navigate 서비스를 생성
        self.srv = self.create_service(Navigate, 'navigate', self.navigate_callback)

    def go_to_pose(self, x, y, yaw_degrees): # 로봇을 움직이게 하는 코드!
        """지정한 좌표로 로봇을 이동시키고, 완료될 때까지 결과를 모니터링하는 함수."""
        self.is_navigating = True # 네비게이션 시작 플래그 설정
        self.get_logger().info(f"목표 지점으로 이동 시작: X={x}, Y={y}, Yaw={yaw_degrees}")

        # --- 1. 목표 지점(PoseStamped) 생성 ---
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg() # self.nav. 대신 self. 사용
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        q = self.get_quaternion_from_yaw(yaw_degrees)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        # --- 2. 네비게이션 명령 전송 ---
        self.goToPose(goal_pose) # self.nav. 대신 self. 사용

        # --- 3. 피드백 모니터링 루프 ---
        while not self.isTaskComplete(): # self.nav. 대신 self. 사용
            # 이 while 루프는 네비게이션이 끝날 때까지 계속 실행됩니다.
            # time.sleep(0.1)이 없으면, 이 루프는 CPU 코어 하나를 100% 점유하는 '무한 폭주 루프'가 됩니다.
            # 0.1초 동안 잠시 멈춤으로써 CPU 사용량을 크게 줄이고,
            # 1초에 약 10번 정도 로봇의 상태를 확인하는 적절한 주기를 만듭니다.
            time.sleep(0.1) # 0.1초 대기

            feedback = self.getFeedback() # self.nav. 대신 self. 사용
            if feedback:
                self.get_logger().info('남은 거리: {:.2f} 미터.'.format(feedback.distance_remaining))

                # 60초 이상 걸리면 타임아웃으로 간주하고 작업 취소
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                    self.get_logger().warn('네비게이션 타임아웃! 작업을 취소합니다.')
                    self.cancelTask() # self.nav. 대신 self. 사용

        # --- 4. 최종 결과 확인 ---
        result = self.getResult() # self.nav. 대신 self. 사용
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('✅ 목표 지점 도착 성공!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('⚠️ 목표가 취소되었습니다.')
        elif result == TaskResult.FAILED:
            self.get_logger().error('❌ 목표 지점 도착 실패!')
        
        self.is_navigating = False # 네비게이션 종료 플래그 설정

    def navigate_callback(self, request, response): # 명령 srv 메세지 받으면 실행
        """'navigate' 서비스 요청을 받았을 때 실행되는 콜백 함수"""
        # 이미 다른 네비게이션 작업이 진행 중인 경우, 새로운 요청을 거부합니다.
        if self.is_navigating:
            self.get_logger().warn('이미 다른 네비게이션 작업을 수행중입니다. 새 요청을 거부합니다.')
            response.success = False
            response.message = "해당 로봇은 현재 다른 작업으로 바쁩니다."
            return response

        self.get_logger().info(f"서비스 요청 받음: X={request.x}, Y={request.y}, Yaw={request.yaw}")

        """요청 예시: ros2 run admin debug_tool 1.5 -1.0 90.0 """

        # go_to_pose 함수는 완료될 때까지 시간이 걸리므로(blocking),
        # 서비스 콜백이 멈추지 않도록 별도의 스레드에서 실행합니다.
        nav_thread = threading.Thread(target=self.go_to_pose, args=(request.x, request.y, request.yaw))
        nav_thread.start()

        # 클라이언트에게는 일단 명령을 잘 받았다고 즉시 응답합니다.
        response.success = True
        response.message = "Navigation 명령 잘 받았고 시작됨!"
        return response

    def _set_initial_pose(self): #로봇의 시작 위치를 설정해줌~
        """로봇의 초기 위치(initial pose)를 설정하는 함수. RViz2에서 2D Pose Estimate를 클릭하는 것과 같다."""
        self.get_logger().info('로봇의 초기 위치를 설정합니다...')

        initial_yaw = 0.0  # 초기 yaw 각도 설정 (단위: 도)
        q = self.get_quaternion_from_yaw(initial_yaw)  # yaw 각도를 쿼터니언으로 변환

        initial_pose = PoseStamped()  # PoseStamped 메시지 생성
        initial_pose.header.frame_id = 'map'  # 참조 좌표계는 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()  # 현재 시간
        initial_pose.pose.position.x = 0.0  # 초기 X 좌표
        initial_pose.pose.position.y = 0.0  # 초기 Y 좌표
        initial_pose.pose.position.z = 0.0  # 초기 Z 좌표
        initial_pose.pose.orientation.x = q[0]  # 쿼터니언 X
        initial_pose.pose.orientation.y = q[1]  # 쿼터니언 Y
        initial_pose.pose.orientation.z = q[2]  # 쿼터니언 Z
        initial_pose.pose.orientation.w = q[3]  # 쿼터니언 W

        self.setInitialPose(initial_pose)  # 내비게이터에 초기 위치 설정
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

    try:
        rclpy.spin(robot_commander_node) # 노드가 종료될 때까지 (Ctrl+C) 계속 실행하면서 콜백을 처리해
    except KeyboardInterrupt:
        robot_commander_node.get_logger().info('사용자에 의해 노드가 종료됩니다.') # Ctrl+C로 종료될 때 메시지를 남겨
    finally:
        # 프로그램 종료 시 Nav2 시스템도 깔끔하게 종료
        robot_commander_node.lifecycleShutdown()
        # 노드와 rclpy 리소스를 깔끔하게 정리해주는 부분
        # robot_commander_node.destroy_node() # BasicNavigator가 처리하므로 중복 호출 필요 없음
        if rclpy.ok(): # rclpy가 아직 실행 중이라면
            rclpy.shutdown() # ROS 2 시스템을 완전히 종료해

if __name__ == '__main__': # 이 스크립트가 직접 실행될 때만 main 함수를 호출해
    main()