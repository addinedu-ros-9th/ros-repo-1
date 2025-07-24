import rclpy # ROS 2 파이썬 클라이언트 라이브러리 임포트
from rclpy.node import Node # 노드 클래스 임포트
from rclpy.action import ActionClient # 액션 클라이언트 임포트
from geometry_msgs.msg import PoseStamped # 목표 위치 메시지 타입 임포트
from nav2_msgs.action import NavigateToPose # NavigateToPose 액션 타입 임포트
import time # 시간 관련 라이브러리 임포트
from transforms3d.euler import euler2quat # yaw (각도)를 쿼터니언으로 변환하기 위한 라이브러리 (pip install transforms3d 필요)

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator_client') # 'simple_navigator_client'라는 이름의 ROS 2 노드 생성
        
        # NavigateToPose 액션 서버에 연결할 액션 클라이언트 생성
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose') # 액션 서버의 이름은 'navigate_to_pose'

        self.get_logger().info('네비게이션 액션 서버를 기다리는 중...') # 정보 메시지 출력
        self.action_client.wait_for_server() # 액션 서버가 활성화될 때까지 기다려

        self.get_logger().info('네비게이션 액션 서버 연결 성공!') # 연결 성공 메시지 출력

    def send_goal(self, x, y, yaw_degrees):
        goal_msg = NavigateToPose.Goal() # NavigateToPose 액션의 Goal 메시지 객체 생성

        # 목표 위치 (PoseStamped) 설정
        goal_msg.pose.header.frame_id = 'map' # 목표 좌표계는 'map' 프레임
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg() # 현재 시간 스탬프 설정

        goal_msg.pose.pose.position.x = float(x) # 목표 X 좌표
        goal_msg.pose.pose.position.y = float(y) # 목표 Y 좌표
        goal_msg.pose.pose.position.z = 0.0 # Z 좌표는 2D 내비게이션에서는 보통 0

        # yaw 각도(도 단위)를 쿼터니언으로 변환
        # 로봇의 방향을 나타내며, Nav2에서 필수적인 정보
        quat = euler2quat(0, 0, float(yaw_degrees) * (3.141592653589793 / 180.0))
        goal_msg.pose.pose.orientation.x = quat[1]
        goal_msg.pose.pose.orientation.y = quat[2]
        goal_msg.pose.pose.orientation.z = quat[3]
        goal_msg.pose.pose.orientation.w = quat[0]

        self.get_logger().info(f'목표를 보냅니다: X={x}, Y={y}, Yaw={yaw_degrees}도') # 목표 전송 메시지 출력
        
        self._send_goal_future = self.action_client.send_goal_async(goal_msg) # 목표를 액션 서버로 전송하고 응답을 비동기적으로 기다려
        
        self._send_goal_future.add_done_callback(self.goal_response_callback) # 목표 전송 결과를 기다리는 콜백 함수 등록

    def goal_response_callback(self, future):
        goal_handle = future.result() # 목표 핸들 결과 가져오기
        if not goal_handle.accepted: # 목표가 액션 서버에 의해 수락되지 않았다면
            self.get_logger().info('목표가 거부되었습니다!') # 거부 메시지 출력
            return

        self.get_logger().info('목표가 수락되었습니다. 피드백을 기다리는 중...') # 수락 메시지 출력
        # 목표가 수락되었다면, 결과 수신을 위한 퓨처(future)를 생성
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback) # 결과 콜백 함수 등록

    def get_result_callback(self, future):
        result = future.result().result # 최종 결과 가져오기
        status = future.result().status # 결과 상태 가져오기
        
        # 최종 결과에 따라 메시지 출력
        if status == ActionClient.GoalStatus.SUCCEEDED:
            self.get_logger().info('목표 달성 성공!') # 성공 메시지 출력
        else:
            self.get_logger().info(f'목표 달성 실패: {status}') # 실패 메시지 출력

def main(args=None):
    rclpy.init(args=args) # ROS 2 시스템 초기화
    navigator = SimpleNavigator() # SimpleNavigator 객체 생성

    # 예시 목표: (X=1.0, Y=0.0) 위치로 0도 방향을 보며 이동
    # 이 값을 원하는 목표 좌표로 변경해봐
    navigator.send_goal(x=1.0, y=0.0, yaw_degrees=0.0)

    # ROS 이벤트 루프를 실행하여 콜백 함수들이 호출될 수 있도록 함
    # 액션이 완료되거나 Ctrl+C로 종료할 때까지 spin
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('사용자에 의해 노드 종료')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()