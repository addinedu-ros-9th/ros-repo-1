#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class LEDStatusPublisher(Node):
    def __init__(self):
        super().__init__('led_status_publisher')
        
        # LED 상태 발행자 생성
        self.publisher = self.create_publisher(String, 'led_status', 10)
        
        # 타이머 생성 (2초마다 상태 변경)
        self.timer = self.create_timer(2.0, self.publish_status)
        
        # 상태 리스트
        self.statuses = ["기쁨", "슬픔", "화남"]
        self.current_index = 0
        
        self.get_logger().info('LED Status Publisher started')
    
    def publish_status(self):
        # 현재 상태 가져오기
        current_status = self.statuses[self.current_index]
        
        # 메시지 생성 및 발행
        msg = String()
        msg.data = current_status
        self.publisher.publish(msg)
        
        self.get_logger().info(f'Published status: {current_status}')
        
        # 다음 상태로 인덱스 증가
        self.current_index = (self.current_index + 1) % len(self.statuses)

def main(args=None):
    rclpy.init(args=args)
    
    publisher = LEDStatusPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 