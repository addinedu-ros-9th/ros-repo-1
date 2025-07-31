#!/usr/bin/env python3
"""
무게 센서 데이터 수신 노드
ESP32에서 퍼블리시하는 무게 데이터를 수신하고 처리합니다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from datetime import datetime


class WeightSubscriber(Node):
    """무게 데이터를 수신하는 노드"""
    
    def __init__(self):
        super().__init__('weight_subscriber')
        
        # 무게 데이터 수신을 위한 서브스크라이버 생성
        self.weight_subscription = self.create_subscription(
            Float32,
            'weight_data',
            self.weight_callback,
            10
        )
        
        # 무게 데이터 저장 변수
        self.current_weight = 0.0
        self.last_weight_update = None
        self.weight_history = []
        self.max_history_size = 100
        
        # 통계 변수
        self.min_weight = float('inf')
        self.max_weight = float('-inf')
        self.total_weight = 0.0
        self.weight_count = 0
        
        self.get_logger().info('🔍 무게 센서 데이터 수신 노드 시작')
        self.get_logger().info('📡 /weight_data 토픽을 구독합니다')
        self.get_logger().info('🌐 ROS Domain ID: 26')
        
        # 주기적으로 상태 출력
        self.timer = self.create_timer(5.0, self.status_callback)
        
    def weight_callback(self, msg):
        """무게 데이터 수신 콜백 함수"""
        weight = msg.data
        timestamp = datetime.now()
        
        # 현재 무게 업데이트
        self.current_weight = weight
        self.last_weight_update = timestamp
        
        # 무게 히스토리에 추가
        weight_data = {
            'weight': weight,
            'timestamp': timestamp
        }
        self.weight_history.append(weight_data)
        
        # 히스토리 크기 제한
        if len(self.weight_history) > self.max_history_size:
            self.weight_history.pop(0)
        
        # 통계 업데이트
        if weight > 0.1:  # 0.1g 이상일 때만 통계에 포함
            self.min_weight = min(self.min_weight, weight)
            self.max_weight = max(self.max_weight, weight)
            self.total_weight += weight
            self.weight_count += 1
        
        # 로그 출력
        if abs(weight) < 0.1:
            weight_str = "0.0 g"
        else:
            weight_str = f"{weight:.1f} g"
        
        self.get_logger().info(f'📊 무게: {weight_str}')
        
        # 무게가 일정 값 이상일 때 경고
        if weight > 1000.0:  # 1kg 이상
            self.get_logger().warn(f'⚠️  무거운 물체 감지: {weight:.1f} g')
        elif weight > 100.0:  # 100g 이상
            self.get_logger().info(f'📦 물체 감지: {weight:.1f} g')
    
    def status_callback(self):
        """주기적 상태 출력"""
        if self.last_weight_update:
            time_diff = (datetime.now() - self.last_weight_update).total_seconds()
            
            if time_diff > 10.0:  # 10초 이상 데이터가 없으면
                self.get_logger().warn(f'⚠️  무게 센서 연결 끊김 (마지막 업데이트: {time_diff:.1f}초 전)')
            else:
                self.get_logger().info(f'✅ 무게 센서 정상 작동 (마지막 업데이트: {time_diff:.1f}초 전)')
        
        # 통계 정보 출력
        if self.weight_count > 0:
            avg_weight = self.total_weight / self.weight_count
            self.get_logger().info(f'📈 통계 - 최소: {self.min_weight:.1f}g, 최대: {self.max_weight:.1f}g, 평균: {avg_weight:.1f}g')
    
    def get_current_weight(self):
        """현재 무게 반환"""
        return self.current_weight
    
    def get_weight_history(self):
        """무게 히스토리 반환"""
        return self.weight_history.copy()
    
    def get_weight_statistics(self):
        """무게 통계 반환"""
        if self.weight_count == 0:
            return {
                'min': 0.0,
                'max': 0.0,
                'average': 0.0,
                'count': 0
            }
        
        avg_weight = self.total_weight / self.weight_count
        return {
            'min': self.min_weight,
            'max': self.max_weight,
            'average': avg_weight,
            'count': self.weight_count
        }
    
    def reset_statistics(self):
        """통계 초기화"""
        self.min_weight = float('inf')
        self.max_weight = float('-inf')
        self.total_weight = 0.0
        self.weight_count = 0
        self.weight_history.clear()
        self.get_logger().info('🔄 무게 통계가 초기화되었습니다.')


def main(args=None):
    rclpy.init(args=args)
    
    weight_subscriber = WeightSubscriber()
    
    try:
        rclpy.spin(weight_subscriber)
    except KeyboardInterrupt:
        weight_subscriber.get_logger().info('🛑 무게 센서 수신 노드 종료')
    finally:
        weight_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 