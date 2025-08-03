import rclpy
from rclpy.node import Node
from libo_interfaces.msg import DetectionTimer  # 감지 실패 시간을 전달하는 커스텀 메시지 타입
from libo_interfaces.srv import ActivateDetector, DeactivateDetector  # 감지 활성화/비활성화 서비스
import socket
import json
import threading

class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_bridge_node')

        # 노드 파라미터 선언 및 초기화
        self.declare_parameter('udp_port', 7008)  # 수신할 UDP 포트
        self.declare_parameter('robot_id', 'libo_a')  # 로봇 식별자

        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # 감지 실패 시간을 퍼블리시할 퍼블리셔 생성
        self.pub = self.create_publisher(DetectionTimer, 'detection_timer', 10)

        # 감지 활성화/비활성화 서비스 서버 생성
        self.activate_service = self.create_service(ActivateDetector, 'activate_detector', self.activate_callback)
        self.deactivate_service = self.create_service(DeactivateDetector, 'deactivate_detector', self.deactivate_callback)

        # UDP 소켓 생성 및 바인딩
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.get_logger().info(f"📱 UDP 수신 대기: 포트 {self.udp_port}")

        # 감지 활성화 여부 상태 변수
        self.tracking_active = False

        # 수신 스레드 시작
        threading.Thread(target=self.udp_listener, daemon=True).start()

    def activate_callback(self, request, response):
        """
        감지 활성화 서비스 콜백 함수
        """
        self.get_logger().info(f"▶️ 감지 활성화 요청 수신 from {request.robot_id}")
        self.tracking_active = True
        response.success = True
        response.message = "Tracking activated."
        return response

    def deactivate_callback(self, request, response):
        """
        감지 비활성화 서비스 콜백 함수
        """
        self.get_logger().info(f"⏹️ 감지 비활성화 요청 수신 from {request.robot_id}")
        self.tracking_active = False
        response.success = True
        response.message = "Tracking deactivated."
        return response

    def udp_listener(self):
        """
        UDP 메시지를 수신하고 감지 실패 시간을 처리하는 루프
        """
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
                message = json.loads(data.decode())
                if self.tracking_active:
                    lost_time = message.get("lost_time", 0)
                    # 감지가 계속되고 있으면 0초로 간주해 전송, 그렇지 않으면 실제 시간 전송
                    self.publish_fail_timer(int(lost_time))
            except Exception as e:
                self.get_logger().error(f"❗ 데이터 수신 실패: {e}")

    def publish_fail_timer(self, seconds):
        """
        감지 실패 시간 퍼블리시
        감지가 정상 작동 중이면 0, 실패 중이면 초 단위를 문자열로 전송
        """
        msg = DetectionTimer()
        msg.robot_id = self.robot_id
        msg.command = str(seconds)  # 문자열 형식으로 전송
        self.pub.publish(msg)
        self.get_logger().info(f"🚨 감지 실패 경고 발행: {seconds}초")


def main(args=None):
    rclpy.init(args=args)
    node = ROS2BridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()
