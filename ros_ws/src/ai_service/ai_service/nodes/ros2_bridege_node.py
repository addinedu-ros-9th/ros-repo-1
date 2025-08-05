# ROS2BridgeNode.py

import rclpy
from rclpy.node import Node
from libo_interfaces.msg import DetectionTimer, VoiceCommand # VoiceCommand 메시지 임포트
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
        self.detection_timer_pub = self.create_publisher(DetectionTimer, 'detection_timer', 10)
        # VoiceCommand 퍼블리셔 추가
        self.voice_command_pub = self.create_publisher(VoiceCommand, 'voice_command', 10)
        
        self.activate_service = self.create_service(ActivateDetector, 'activate_detector', self.activate_callback)
        self.deactivate_service = self.create_service(DeactivateDetector, 'deactivate_detector', self.deactivate_callback)

        # 소켓 설정
        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.bind(('0.0.0.0', self.udp_listen_port))
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"📱 UDP 수신 대기: 포트 {self.udp_listen_port}")

        self.tracking_active = False
        # 5초 이상 사람 없음 메시지를 보냈는지 추적하는 상태 변수
        self.no_person_message_sent = False
        
        threading.Thread(target=self.udp_listener, daemon=True).start()

    def activate_callback(self, request, response):
        """감지 활성화 및 중앙 타겟 찾기 명령 전송"""
        self.get_logger().info(f"▶️ 감지 활성화 요청. 추적기에게 중앙 타겟 찾기를 명령합니다.")
        self.tracking_active = True
        self.no_person_message_sent = False # 활성화 시 상태 초기화
        
        try:
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
                    
                    # 1. 기존 DetectionTimer 메시지 발행
                    self.publish_detection_timer(int(lost_time))
                    
                    # 2. 새로운 VoiceCommand 로직
                    self.handle_voice_command(int(lost_time))

            except Exception as e:
                self.get_logger().error(f"❗ 데이터 수신 실패: {e}")

    def publish_detection_timer(self, seconds):
        """DetectionTimer 메시지를 발행하는 함수"""
        msg = DetectionTimer()
        msg.robot_id = self.robot_id
        msg.command = str(seconds)
        self.detection_timer_pub.publish(msg)

    def publish_voice_command(self, category, action):
        """VoiceCommand 메시지를 발행하는 함수"""
        msg = VoiceCommand()
        msg.robot_id = self.robot_id
        msg.category = category
        msg.action = action
        self.voice_command_pub.publish(msg)
        self.get_logger().info(f"📢 음성 명령 발행: category='{category}', action='{action}'")

    def handle_voice_command(self, lost_time_seconds):
        """감지 상태에 따라 음성 명령을 처리하는 함수"""
        # 조건 1: 감지 실패 시간이 5초 이상이고, 아직 메시지를 보내지 않았을 때
        if lost_time_seconds >= 5 and not self.no_person_message_sent:
            self.get_logger().warn("감지 실패 5초 이상! 'no_person_5s' 메시지를 보냅니다.")
            self.publish_voice_command("assist", "no_person_5s")
            self.no_person_message_sent = True # 메시지를 보냈다고 상태 변경

        # 조건 2: 사람이 감지되었고, 이전에 '감지 실패' 상태였을 때
        elif lost_time_seconds < 5 and self.no_person_message_sent:
            self.get_logger().info("사람 재감지! 'person_detected' 메시지를 보냅니다.")
            self.publish_voice_command("assist", "person_detected")
            self.no_person_message_sent = False # 상태 초기화

def main(args=None):
    rclpy.init(args=args)
    node = ROS2BridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        node.get_logger().info('Closing sockets and shutting down node...')
        node.listen_sock.close()
        node.cmd_sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()