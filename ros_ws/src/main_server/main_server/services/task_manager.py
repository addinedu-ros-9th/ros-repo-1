#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest
from libo_interfaces.msg import Heartbeat  # Heartbeat 메시지 추가
import time  # 시간 관련 기능
import uuid  # 고유 ID 생성

class Robot:  # 로봇 정보를 담는 클래스
    def __init__(self, robot_id):  # Robot 객체 초기화
        self.robot_id = robot_id  # 로봇 ID 저장
        self.last_heartbeat_time = time.time()  # 마지막 하트비트 수신 시간
        self.is_active = True  # 활성 상태 (기본값: 활성)
    
    def update_heartbeat(self):  # 하트비트 업데이트
        """하트비트를 받았을 때 호출되는 메서드"""
        self.last_heartbeat_time = time.time()  # 마지막 하트비트 시간 업데이트
        self.is_active = True  # 활성 상태로 설정

    def check_timeout(self, timeout_seconds=3):  # 타임아웃 체크
        """지정된 시간(기본 3초) 이내에 하트비트가 수신되었는지 확인하는 메서드"""
        current_time = time.time()  # 현재 시간을 가져옴
        time_since_last_heartbeat = current_time - self.last_heartbeat_time  # 마지막 하트비트를 받은 후 얼마나 시간이 지났는지 계산
        if time_since_last_heartbeat > timeout_seconds:  # 지정된 시간보다 오래되었다면
            self.is_active = False  # 로봇을 비활성 상태로 변경
        return self.is_active  # 현재 로봇의 활성 상태를 반환 (True 또는 False)

class Task:  # 작업 정보를 담는 클래스
    def __init__(self, robot_id, task_type, call_location, goal_location):  # Task 객체 초기화
        self.task_id = str(uuid.uuid4())[:8]  # 고유한 작업 ID 생성 (8자리)
        self.robot_id = robot_id  # 로봇 ID 저장
        self.task_type = task_type  # 작업 타입 저장
        self.call_location = call_location  # 호출지 위치 저장
        self.goal_location = goal_location  # 목적지 위치 저장
        self.start_time = time.time()  # 시작 시간 기록
        self.end_time = None  # 종료 시간 (아직 없음)
        self.status = "created"  # 작업 상태 (created, running, completed, failed)
    
    def get_info(self):  # 작업 정보 반환
        """작업의 현재 정보를 문자열로 반환"""
        return f"Task[{self.task_id}] - {self.robot_id} | {self.task_type} | {self.call_location} -> {self.goal_location} | Status: {self.status}"

class TaskManager(Node):
    def __init__(self):  # TaskManager 노드 초기화 및 서비스 서버 설정
        super().__init__('task_manager')
        
        # TaskRequest 서비스 서버 생성
        self.service = self.create_service(
            TaskRequest,
            'task_request',
            self.task_request_callback
        )
        
        # Heartbeat 토픽 구독자 생성
        self.heartbeat_subscription = self.create_subscription(
            Heartbeat,  # 메시지 타입
            'heartbeat',  # 토픽 이름
            self.heartbeat_callback,  # 콜백 함수
            # QoS 프로파일 설정 (Heartbeat Sender와 호환되도록)
            qos_profile=rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # 최선 노력 수신
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,  # 휘발성
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,  # 마지막 N개 메시지만 유지
                depth=10  # 큐 깊이
            )
        )
        
        # 작업 목록을 저장할 리스트
        self.tasks = []  # 생성된 작업들을 저장할 리스트
        
        # 로봇 목록을 저장할 딕셔너리 (robot_id를 키로 사용)
        self.robots = {}  # 로봇들을 저장할 딕셔너리
        
        # 로봇 상태 체크 타이머 (1초마다 실행)
        self.robot_check_timer = self.create_timer(1.0, self.check_robot_timeouts)  # 1초마다 로봇 타임아웃 체크
        
        self.get_logger().info('🎯 Task Manager 시작됨 - task_request 서비스 대기 중...')
        self.get_logger().info('💓 Heartbeat 구독 시작됨 - heartbeat 토픽 모니터링 중...')
    
    def check_robot_timeouts(self):  # 로봇 타임아웃 체크
        """1초마다 로봇 목록을 확인하여 타임아웃된 로봇을 목록에서 제거"""
        inactive_robots = []  # 비활성 로봇 ID를 저장할 리스트
        for robot_id, robot in self.robots.items():  # 현재 등록된 모든 로봇에 대해 반복
            if not robot.check_timeout():  # 로봇의 타임아웃 여부를 확인
                inactive_robots.append(robot_id)  # 타임아웃된 로봇을 리스트에 추가
        
        for robot_id in inactive_robots:  # 비활성 로봇 리스트에 있는 모든 로봇에 대해 반복
            del self.robots[robot_id]  # 로봇 목록에서 해당 로봇을 제거
            self.get_logger().info(f'🤖 로봇 <{robot_id}> 제거됨 (사유: Heartbeat 타임아웃)')  # 로봇 제거 로그 출력
    
    def heartbeat_callback(self, msg):  # Heartbeat 메시지 수신 콜백
        """Heartbeat 메시지를 받았을 때 호출되는 콜백 함수"""
        try:
            # sender_id가 로봇인 경우에만 Robot 객체를 생성하거나 업데이트
            if msg.sender_id in self.robots:  # 이미 등록된 로봇이라면
                self.robots[msg.sender_id].update_heartbeat()  # 하트비트 시간만 갱신해줌
            else:  # 처음 보는 로봇이라면
                self.robots[msg.sender_id] = Robot(msg.sender_id)  # 새로운 로봇 객체를 생성해서 목록에 추가
                self.get_logger().info(f'🤖 새로운 로봇 <{msg.sender_id}> 감지됨')  # 새로운 로봇 감지 로그 출력
            
        except Exception as e:  # 예외 발생 시 처리
            self.get_logger().error(f'❌ Heartbeat 처리 중 오류: {e}')  # 에러 로그
    
    def task_request_callback(self, request, response):  # 키오스크로부터 받은 작업 요청을 처리
        """TaskRequest 서비스 콜백"""
        self.get_logger().info(f'📥 Task Request 받음!')
        self.get_logger().info(f'   - Robot ID: {request.robot_id}')
        self.get_logger().info(f'   - Task Type: {request.task_type}')
        self.get_logger().info(f'   - Call Location: {request.call_location}')
        self.get_logger().info(f'   - Goal Location: {request.goal_location}')
        
        # 새로운 Task 객체 생성
        new_task = Task(request.robot_id, request.task_type, request.call_location, request.goal_location)  # Task 객체 생성
        self.tasks.append(new_task)  # 작업 목록에 추가
        
        self.get_logger().info(f'✅ 새로운 작업 생성됨: {new_task.get_info()}')  # 생성된 작업 정보 출력
        
        # 응답 설정
        response.success = True
        response.message = f"Task request 잘 받았음! Task ID: {new_task.task_id}"
        
        self.get_logger().info(f'✅ Task Request 처리 완료: {response.message}')
        
        return response

def main(args=None):  # ROS2 노드 실행 및 종료 처리
    rclpy.init(args=args)
    
    task_manager = TaskManager()
    
    try:
        rclpy.spin(task_manager)
    except KeyboardInterrupt:
        pass
    finally:
        task_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
