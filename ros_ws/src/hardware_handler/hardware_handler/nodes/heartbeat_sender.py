#!/usr/bin/env python3

import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드 클래스
from libo_interfaces.msg import Heartbeat  # Heartbeat 메시지
from rcl_interfaces.msg import ParameterDescriptor  # 파라미터 설명
import time  # 시간 관련 기능

class HeartbeatSender(Node):  # Heartbeat 발행 노드
    def __init__(self):  # 노드 초기화
        super().__init__('heartbeat_sender')  # 부모 클래스 초기화하고 노드 이름 설정
        
        # sender_id 파라미터 선언 (기본값: 'libo_a')
        self.declare_parameter(
            'sender_id', 
            'libo_a',  # 기본값
            ParameterDescriptor(description='발신자 ID (예: libo_a, libo_b, libo_c, admin_1, admin_2, admin_3, kiosk_1, kiosk_2, kiosk_3)')  # 파라미터 설명
        )
        
        # heartbeat 주기 파라미터 선언 (기본값: 1초)
        self.declare_parameter(
            'heartbeat_period', 
            1.0,  # 기본값 (초)
            ParameterDescriptor(description='Heartbeat 발행 주기 (초)')  # 파라미터 설명
        )
        
        # 파라미터 값 가져오기
        self.sender_id = self.get_parameter('sender_id').value  # sender_id 파라미터 값 가져오기
        self.heartbeat_period = self.get_parameter('heartbeat_period').value  # heartbeat_period 파라미터 값 가져오기
        
        # Heartbeat 토픽 발행자 생성
        self.heartbeat_publisher = self.create_publisher(
            Heartbeat,  # 메시지 타입
            'heartbeat',  # 토픽 이름
            # QoS 프로파일 설정 (구독자가 없어도 발행하도록)
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # 최선 노력 전송
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,  # 휘발성 (구독자가 없으면 메시지 손실)
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,  # 마지막 N개 메시지만 유지
                depth=10  # 큐 깊이
            )
        )
        
        # 타이머 생성 (주기적으로 Heartbeat 발행)
        self.timer = self.create_timer(
            self.heartbeat_period,  # 타이머 주기
            self.publish_heartbeat  # 타이머 콜백 함수
        )
        
        self.get_logger().info(f'💓 Heartbeat Sender 시작됨 - sender_id: {self.sender_id}, 주기: {self.heartbeat_period}초')  # 시작 로그
    
    def publish_heartbeat(self):  # Heartbeat 메시지 발행
        """주기적으로 Heartbeat 메시지를 발행"""
        try:
            # Heartbeat 메시지 생성
            heartbeat_msg = Heartbeat()  # Heartbeat 메시지 객체 생성
            heartbeat_msg.sender_id = self.sender_id  # 발신자 ID 설정
            
            # 메시지 발행
            self.heartbeat_publisher.publish(heartbeat_msg)  # 토픽으로 메시지 발행
            
            # 현재 시간을 읽기 쉬운 형태로 변환
            current_time = time.strftime('%H:%M:%S', time.localtime())  # 현재 시간 포맷 변환
            
            # 자세한 로그 출력 (info 레벨로 변경)
            self.get_logger().info(f'💓 Heartbeat 전송됨 | Sender: {self.sender_id} | Time: {current_time}')  # 상세 로그 출력
            
        except Exception as e:  # 예외 발생 시 처리
            self.get_logger().error(f'❌ Heartbeat 발행 중 오류: {e}')  # 에러 로그

def main(args=None):  # 메인 함수
    rclpy.init(args=args)  # ROS2 초기화
    
    heartbeat_sender = HeartbeatSender()  # HeartbeatSender 노드 생성
    
    try:
        rclpy.spin(heartbeat_sender)  # 노드 실행 (무한 루프)
    except KeyboardInterrupt:  # Ctrl+C로 종료 시
        pass
    finally:
        heartbeat_sender.destroy_node()  # 노드 정리
        rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':  # 이 파일이 직접 실행될 때만
    main()  # 메인 함수 호출 