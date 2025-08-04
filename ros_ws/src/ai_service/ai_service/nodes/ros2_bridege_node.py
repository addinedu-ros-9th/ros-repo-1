import rclpy
from rclpy.node import Node
from libo_interfaces.msg import DetectionTimer
# 새로 추가할 서비스 임포트
from libo_interfaces.srv import ActivateDetector, DeactivateDetector, SetTarget, ClearTarget
import socket
import json
import threading

class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_bridge_node')

        # 노드 파라미터 선언 및 초기화
        self.declare_parameter('udp_listen_port', 7008)  # 수신 포트
        self.declare_parameter('robot_id', 'libo_a')
        # 명령을 보낼 추적 스크립트의 주소와 포트
        self.declare_parameter('detector_ip', '127.0.0.1')
        self.declare_parameter('detector_cmd_port', 7009)

        self.udp_listen_port = self.get_parameter('udp_listen_port').get_parameter_value().integer_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.detector_ip = self.get_parameter('detector_ip').get_parameter_value().string_value
        self.detector_cmd_port = self.get_parameter('detector_cmd_port').get_parameter_value().integer_value

        # 퍼블리셔, 서비스 서버 생성
        self.pub = self.create_publisher(DetectionTimer, 'detection_timer', 10)
        self.activate_service = self.create_service(ActivateDetector, 'activate_detector', self.activate_callback)
        self.deactivate_service = self.create_service(DeactivateDetector, 'deactivate_detector', self.deactivate_callback)
        # 타겟 지정/해제 서비스 서버 추가
        self.set_target_service = self.create_service(SetTarget, 'set_target', self.set_target_callback)
        self.clear_target_service = self.create_service(ClearTarget, 'clear_target', self.clear_target_callback)

        # UDP 수신 소켓
        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.bind(('0.0.0.0', self.udp_listen_port))
        self.get_logger().info(f"📱 UDP 수신 대기: 포트 {self.udp_listen_port}")

        # UDP 명령 전송용 소켓
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.tracking_active = False
        threading.Thread(target=self.udp_listener, daemon=True).start()

    def set_target_callback(self, request, response):
        """타겟 지정 서비스 콜백"""
        self.get_logger().info(f"🎯 타겟 지정 요청: ID {request.target_id}")
        try:
            # 추적 스크립트로 명령 전송
            command = {'command': 'set_target', 'target_id': request.target_id}
            self.cmd_sock.sendto(json.dumps(command).encode(), (self.detector_ip, self.detector_cmd_port))
            response.success = True
            response.message = f"Set target command sent for ID {request.target_id}."
        except Exception as e:
            self.get_logger().error(f"❗ 타겟 지정 명령 전송 실패: {e}")
            response.success = False
            response.message = "Failed to send command."
        return response

    def clear_target_callback(self, request, response):
        """타겟 해제 서비스 콜백"""
        self.get_logger().info("🗑️ 타겟 해제 요청")
        try:
            command = {'command': 'clear_target'}
            self.cmd_sock.sendto(json.dumps(command).encode(), (self.detector_ip, self.detector_cmd_port))
            response.success = True
            response.message = "Clear target command sent."
        except Exception as e:
            self.get_logger().error(f"❗ 타겟 해제 명령 전송 실패: {e}")
            response.success = False
            response.message = "Failed to send command."
        return response

    def activate_callback(self, request, response):
        self.get_logger().info(f"▶️ 감지 활성화 요청 수신 from {request.robot_id}")
        self.tracking_active = True
        response.success = True
        response.message = "Tracking activated."
        return response

    def deactivate_callback(self, request, response):
        self.get_logger().info(f"⏹️ 감지 비활성화 요청 수신 from {request.robot_id}")
        self.tracking_active = False
        response.success = True
        response.message = "Tracking deactivated."
        return response

    def udp_listener(self):
        while True:
            try:
                data, _ = self.listen_sock.recvfrom(65536)
                message = json.loads(data.decode())
                if self.tracking_active:
                    lost_time = message.get("lost_time", 0)
                    self.publish_fail_timer(int(lost_time))
            except Exception as e:
                self.get_logger().error(f"❗ 데이터 수신 실패: {e}")

    def publish_fail_timer(self, seconds):
        msg = DetectionTimer()
        msg.robot_id = self.robot_id
        msg.command = str(seconds)
        self.pub.publish(msg)
        self.get_logger().info(f"🚨 감지 실패 경고 발행: {seconds}초")


def main(args=None):
    rclpy.init(args=args)
    node = ROS2BridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()