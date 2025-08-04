# ROS2BridgeNode.py

import rclpy
from rclpy.node import Node
from libo_interfaces.msg import DetectionTimer
from libo_interfaces.srv import ActivateDetector, DeactivateDetector
import socket
import json
import threading

class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_bridge_node')

        # 파라미터 선언
        self.declare_parameter('udp_listen_port', 7008)
        self.declare_parameter('robot_id', 'libo_a')
        self.declare_parameter('detector_ip', '127.0.0.1')
        self.declare_parameter('detector_cmd_port', 7009)

        # 파라미터 가져오기
        self.udp_listen_port = self.get_parameter('udp_listen_port').get_parameter_value().integer_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.detector_ip = self.get_parameter('detector_ip').get_parameter_value().string_value
        self.detector_cmd_port = self.get_parameter('detector_cmd_port').get_parameter_value().integer_value

        # 퍼블리셔 및 서비스 서버
        self.pub = self.create_publisher(DetectionTimer, 'detection_timer', 10)
        self.activate_service = self.create_service(ActivateDetector, 'activate_detector', self.activate_callback)
        self.deactivate_service = self.create_service(DeactivateDetector, 'deactivate_detector', self.deactivate_callback)

        # 소켓 설정
        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.bind(('0.0.0.0', self.udp_listen_port))
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"📱 UDP 수신 대기: 포트 {self.udp_listen_port}")

        self.tracking_active = False
        threading.Thread(target=self.udp_listener, daemon=True).start()

    def activate_callback(self, request, response):
        """감지 활성화 및 중앙 타겟 찾기 명령 전송"""
        self.get_logger().info(f"▶️ 감지 활성화 요청. 추적기에게 중앙 타겟 찾기를 명령합니다.")
        self.tracking_active = True
        
        try:
            # "중앙 타겟 찾기" 명령 전송
            command = {'command': 'activate_and_find_center'}
            self.cmd_sock.sendto(json.dumps(command).encode(), (self.detector_ip, self.detector_cmd_port))
            response.success = True
            response.message = "Activation command sent. Tracker will find the center target."
            self.get_logger().info("✅ 활성화 및 중앙 타겟 찾기 명령 전송 완료")
        except Exception as e:
            self.get_logger().error(f"❗ 명령 전송 실패: {e}")
            response.success = False
            response.message = "Failed to send command."
            
        return response

    def deactivate_callback(self, request, response):
        """감지 비활성화 및 타겟 해제"""
        self.get_logger().info(f"⏹️ 감지 비활성화 및 타겟 해제 요청 from {request.robot_id}")
        self.tracking_active = False
        
        try:
            command = {'command': 'clear_target'}
            self.cmd_sock.sendto(json.dumps(command).encode(), (self.detector_ip, self.detector_cmd_port))
            response.success = True
            response.message = "Tracking deactivated and target cleared."
            self.get_logger().info("✅ 비활성화 및 타겟 해제 명령 전송 완료")
        except Exception as e:
            self.get_logger().error(f"❗ 명령 전송 실패: {e}")
            response.success = False
            response.message = "Failed to send command."

        return response

    def udp_listener(self):
        while True:
            try:
                data, _ = self.listen_sock.recvfrom(65536)
                if self.tracking_active:
                    message = json.loads(data.decode())
                    lost_time = message.get("lost_time", 0)
                    self.publish_fail_timer(int(lost_time))
            except Exception as e:
                self.get_logger().error(f"❗ 데이터 수신 실패: {e}")

    def publish_fail_timer(self, seconds):
        msg = DetectionTimer()
        msg.robot_id = self.robot_id
        msg.command = str(seconds)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ROS2BridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()