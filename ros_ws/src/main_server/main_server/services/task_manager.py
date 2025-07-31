#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest
import time  # 시간 관련 기능
import uuid  # 고유 ID 생성
import json  # JSON 파일 저장용

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
        
        # 작업 목록을 저장할 리스트
        self.tasks = []  # 생성된 작업들을 저장할 리스트
        
        # 작업 목록 저장 파일 경로
        self.tasks_file = "/tmp/current_tasks.json"  # 임시 파일에 저장
        
        self.get_logger().info('🎯 Task Manager 시작됨 - task_request 서비스 대기 중...')
    
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
        
        # Callback시 자동 응답 내용
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
