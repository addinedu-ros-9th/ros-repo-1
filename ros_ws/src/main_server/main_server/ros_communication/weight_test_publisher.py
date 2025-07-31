#!/usr/bin/env python3
"""
무게 센서 테스트 퍼블리셔 노드
ESP32 없이도 무게 데이터를 시뮬레이션하여 테스트할 수 있습니다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import time


class WeightTestPublisher(Node):
    """테스트용 무게 데이터 퍼블리셔"""
    
    def __init__(self):
        super().__init__('weight_test_publisher')
        
        # 무게 데이터 퍼블리셔 생성
        self.weight_publisher = self.create_publisher(
            Float32,
            'weight_data',
            10
        )
        
        # 타이머 생성 (1초마다 퍼블리시)
        self.timer = self.create_timer(1.0, self.publish_weight)
        
        # 테스트 데이터 변수
        self.base_weight = 0.0
        self.weight_variation = 0.0
        self.test_mode = 'idle'  # idle, object, heavy
        
        self.get_logger().info('🧪 무게 센서 테스트 퍼블리셔 시작')
        self.get_logger().info('📡 /weight_data 토픽으로 테스트 데이터 퍼블리시')
        self.get_logger().info('🌐 ROS Domain ID: 26')
        self.get_logger().info('💡 사용법:')
        self.get_logger().info('   - 기본: 0g 근처에서 작은 변동')
        self.get_logger().info('   - 물체 시뮬레이션: 100-500g')
        self.get_logger().info('   - 무거운 물체: 1000-2000g')
        
    def publish_weight(self):
        """무게 데이터 퍼블리시"""
        weight_msg = Float32()
        
        # 테스트 모드에 따른 무게 생성
        if self.test_mode == 'idle':
            # 기본 모드: 0g 근처에서 작은 변동
            weight = self.base_weight + random.uniform(-2.0, 2.0)
        elif self.test_mode == 'object':
            # 물체 시뮬레이션: 100-500g
            weight = random.uniform(100.0, 500.0)
        elif self.test_mode == 'heavy':
            # 무거운 물체: 1000-2000g
            weight = random.uniform(1000.0, 2000.0)
        else:
            weight = 0.0
        
        weight_msg.data = weight
        
        # 퍼블리시
        self.weight_publisher.publish(weight_msg)
        
        # 로그 출력
        if abs(weight) < 0.1:
            weight_str = "0.0 g"
        else:
            weight_str = f"{weight:.1f} g"
        
        self.get_logger().info(f'📊 테스트 무게: {weight_str} (모드: {self.test_mode})')
    
    def set_test_mode(self, mode):
        """테스트 모드 설정"""
        if mode in ['idle', 'object', 'heavy']:
            self.test_mode = mode
            self.get_logger().info(f'🔄 테스트 모드 변경: {mode}')
        else:
            self.get_logger().warn(f'❌ 잘못된 테스트 모드: {mode}')
    
    def set_base_weight(self, weight):
        """기본 무게 설정"""
        self.base_weight = weight
        self.get_logger().info(f'⚖️  기본 무게 설정: {weight:.1f} g')


def main(args=None):
    rclpy.init(args=args)
    
    weight_publisher = WeightTestPublisher()
    
    try:
        rclpy.spin(weight_publisher)
    except KeyboardInterrupt:
        weight_publisher.get_logger().info('🛑 무게 센서 테스트 퍼블리셔 종료')
    finally:
        weight_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 