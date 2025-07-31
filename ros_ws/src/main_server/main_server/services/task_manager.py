#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest
from libo_interfaces.msg import Heartbeat  # Heartbeat 메시지 추가
import time  # 시간 관련 기능
import uuid  # 고유 ID 생성
import json  # JSON 파일 저장용
import hashlib  # 파일 해시 계산용 (변경사항 체크용)

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
    
    def to_dict(self):  # Task 객체를 딕셔너리로 변환 (JSON 저장용)
        """Task 객체를 딕셔너리로 변환"""
        return {
            'task_id': self.task_id,
            'robot_id': self.robot_id,
            'task_type': self.task_type,
            'call_location': self.call_location,
            'goal_location': self.goal_location,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'status': self.status
        }

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
        
        # 작업 목록 저장 파일 경로
        self.tasks_file = "/tmp/current_tasks.json"  # 임시 파일에 저장
        
        # Heartbeat 로그 저장 파일 경로
        self.heartbeat_log_file = "/tmp/heartbeat_log.json"  # Heartbeat 로그 파일
        
        # Heartbeat 로그 리스트
        self.heartbeat_logs = []  # Heartbeat 로그를 저장할 리스트
        
        # 파일 해시 저장 (변경사항 체크용)
        self.last_heartbeat_hash = None  # 마지막 Heartbeat 로그 해시
        
        self.get_logger().info('🎯 Task Manager 시작됨 - task_request 서비스 대기 중...')
        self.get_logger().info('💓 Heartbeat 구독 시작됨 - heartbeat 토픽 모니터링 중...')
    
    def calculate_heartbeat_hash(self):  # Heartbeat 로그 해시 계산
        """현재 Heartbeat 로그의 해시를 계산"""
        heartbeat_json = json.dumps(self.heartbeat_logs, sort_keys=True)  # JSON 문자열로 변환
        return hashlib.md5(heartbeat_json.encode()).hexdigest()  # MD5 해시 계산
    
    def heartbeat_callback(self, msg):  # Heartbeat 메시지 수신 콜백
        """Heartbeat 메시지를 받았을 때 호출되는 콜백 함수"""
        try:
            # 현재 시간을 읽기 쉬운 형태로 변환
            current_time = time.strftime('%H:%M:%S', time.localtime())  # 현재 시간 포맷 변환
            
            # Heartbeat 로그 정보 생성
            heartbeat_info = {
                'sender_id': msg.sender_id,  # 발신자 ID
                'timestamp': time.time(),  # 현재 시간을 타임스탬프로 사용 (단순화)
                'received_time': time.time(),  # 수신 시간
                'received_time_str': current_time  # 수신 시간 문자열
            }
            
            # Heartbeat 로그 리스트에 추가 (최근 100개만 유지)
            self.heartbeat_logs.append(heartbeat_info)  # 로그 추가
            if len(self.heartbeat_logs) > 100:  # 100개 초과하면
                self.heartbeat_logs = self.heartbeat_logs[-100:]  # 최근 100개만 유지
            
            # 새로운 Heartbeat을 받았을 때만 파일에 저장
            current_hash = self.calculate_heartbeat_hash()  # 현재 해시 계산
            if current_hash != self.last_heartbeat_hash:  # 해시가 다르면 (새로운 데이터가 있으면)
                self.save_heartbeat_logs()  # 로그 파일 저장
                self.last_heartbeat_hash = current_hash  # 해시 업데이트
            
            # 터미널에 로그 출력
            self.get_logger().info(f'💓 Heartbeat 수신 | Sender: {msg.sender_id} | Time: {current_time} | Timestamp: {msg.timestamp.sec}.{msg.timestamp.nanosec}')  # Heartbeat 수신 로그
            
        except Exception as e:  # 예외 발생 시 처리
            self.get_logger().error(f'❌ Heartbeat 처리 중 오류: {e}')  # 에러 로그
    
    def save_heartbeat_logs(self):  # Heartbeat 로그를 파일에 저장
        """Heartbeat 로그를 JSON 파일에 저장"""
        try:
            with open(self.heartbeat_log_file, 'w') as f:  # 파일 쓰기 모드로 열기
                json.dump(self.heartbeat_logs, f, indent=2)  # JSON 형태로 저장 (들여쓰기 2칸)
        except Exception as e:
            self.get_logger().error(f'❌ Heartbeat 로그 저장 실패: {e}')  # 저장 실패 시 에러 로그
    
    def save_tasks_to_file(self):  # 작업 목록을 파일에 저장
        """현재 작업 목록을 JSON 파일에 저장"""
        try:
            tasks_data = [task.to_dict() for task in self.tasks]  # 모든 Task를 딕셔너리로 변환
            with open(self.tasks_file, 'w') as f:  # 파일 쓰기 모드로 열기
                json.dump(tasks_data, f, indent=2)  # JSON 형태로 저장 (들여쓰기 2칸)
        except Exception as e:
            self.get_logger().error(f'❌ 작업 목록 저장 실패: {e}')  # 저장 실패 시 에러 로그
    
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
        
        # 작업 목록을 파일에 저장
        self.save_tasks_to_file()  # 파일에 현재 작업 목록 저장
        
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
