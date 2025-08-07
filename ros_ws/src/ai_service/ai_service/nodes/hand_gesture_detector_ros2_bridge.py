#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import json
import threading
import time
from libo_interfaces.msg import GestureResult

# 상수 정의
HAND_GESTURE_BRIDGE = 7023  # 손 제스처 브릿지 포트 (수신)

class HandGestureROS2Bridge(Node):
    """
    손 제스처 감지기(hand_gesture_detector.py)에서 UDP로 받은 제스처 데이터를
    ROS2 토픽으로 발행하는 브릿지 노드
    """
    def __init__(self):
        super().__init__('hand_gesture_ros2_bridge')
        
        # 로거 설정
        self.get_logger().info('손 제스처 ROS2 브릿지 노드 초기화 중...')
        
        # 제스처 결과 발행자
        self.gesture_publisher = self.create_publisher(
            GestureResult,
            '/gesture_result',
            10
        )
        
        # UDP 수신 소켓 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', HAND_GESTURE_BRIDGE))
        self.sock.settimeout(0.5)  # 0.5초 타임아웃 설정 (종료 처리용)
        
        # 스레드 실행 상태
        self.is_running = True
        
        # 수신 스레드 시작
        self.receive_thread = threading.Thread(target=self.receive_and_publish)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        self.get_logger().info(f'손 제스처 ROS2 브릿지 노드 초기화 완료! (포트: {HAND_GESTURE_BRIDGE})')
        
    def receive_and_publish(self):
        """UDP 패킷을 수신하여 ROS2 토픽으로 발행하는 스레드"""
        self.get_logger().info('UDP 패킷 수신 시작')
        
        buffer_size = 1024  # UDP 수신 버퍼 크기
        
        while self.is_running:
            try:
                # UDP 패킷 수신
                data, addr = self.sock.recvfrom(buffer_size)
                
                # JSON 데이터 파싱
                gesture_data = json.loads(data.decode('utf-8'))
                
                # 필수 필드 확인
                if 'robot_id' in gesture_data and 'gesture' in gesture_data:
                    # GestureResult 메시지 생성
                    msg = GestureResult()
                    msg.robot_id = gesture_data['robot_id']
                    msg.gesture = gesture_data['gesture']
                    
                    # ROS2 토픽 발행
                    self.gesture_publisher.publish(msg)
                    self.get_logger().info(f'제스처 발행: {msg.gesture} (로봇: {msg.robot_id})')
                else:
                    self.get_logger().warning('수신된 제스처 데이터 형식이 잘못되었습니다.')
                    
            except socket.timeout:
                # 타임아웃은 무시 (스레드 종료 확인용)
                pass
            except Exception as e:
                self.get_logger().error(f'패킷 수신/처리 오류: {str(e)}')
                time.sleep(0.1)
    
    def destroy_node(self):
        """노드 종료 시 정리 작업"""
        self.get_logger().info('손 제스처 ROS2 브릿지 노드 종료 중...')
        self.is_running = False
        
        if self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
            
        self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureROS2Bridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
