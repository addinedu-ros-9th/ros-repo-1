#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest
from PyQt5.QtCore import QThread, pyqtSignal
import threading

class TaskRequestClient(QThread):
    """작업 요청 ROS2 서비스 클라이언트 (에스코팅 전용)"""
    
    # 요청 완료 시그널
    task_request_completed = pyqtSignal(bool, str)  # success, message
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.client = None
        self.request_data = None
        self._lock = threading.Lock()
        self._is_cleaning_up = False
        
    def init_ros(self):
        """ROS2 초기화"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = Node('task_request_client')
            self.client = self.node.create_client(TaskRequest, '/task_request')
            
            # 서비스 서버 대기
            timeout_count = 0
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('📡 task_request 서비스 대기 중...')
                timeout_count += 1
                if timeout_count > 10:  # 10초 대기 후 포기
                    print("❌ task_request 서비스 서버를 찾을 수 없습니다.")
                    return False
                
            print("✅ TaskRequestClient ROS2 초기화 완료")
            return True
            
        except Exception as e:
            print(f"❌ TaskRequestClient ROS2 초기화 실패: {e}")
            return False
    
    def request_escort_task(self, robot_id, call_location, goal_location):
        """
        에스코팅 작업 요청
        
        Args:
            robot_id (str): 로봇 ID (예: "libo_a")
            call_location (str): 호출지 위치 (예: "E9" - 키오스크)
            goal_location (str): 목적지 위치 (예: "D5" - 책 위치)
        
        Returns:
            bool: 요청 시작 성공 여부
        """
        try:
            # 요청 데이터 저장
            self.request_data = {
                'robot_id': robot_id,
                'task_type': 'escort',  # 에스코팅 고정
                'call_location': call_location,
                'goal_location': goal_location
            }
            
            print(f"🚀 에스코팅 요청 준비: {self.request_data}")
            
            # QThread로 비동기 요청 시작
            self.start()
            return True
            
        except Exception as e:
            print(f"❌ 에스코팅 작업 요청 준비 중 오류: {e}")
            self.task_request_completed.emit(False, f"요청 준비 중 오류: {str(e)}")
            return False
    
    def run(self):
        """QThread 실행 (백그라운드에서 ROS2 서비스 호출)"""
        try:
            # ROS2 초기화
            if self.node is None:
                if not self.init_ros():
                    self.task_request_completed.emit(False, "ROS2 초기화 실패")
                    return
            
            if self.node is None or self.client is None:
                self.task_request_completed.emit(False, "ROS2 클라이언트 초기화 실패")
                return
            
            # 서비스 요청 생성
            request = TaskRequest.Request()
            request.robot_id = self.request_data['robot_id']
            request.task_type = self.request_data['task_type']
            request.call_location = self.request_data['call_location']
            request.goal_location = self.request_data['goal_location']
            
            print(f"📤 TaskRequest 서비스 호출:")
            print(f"  - robot_id: {request.robot_id}")
            print(f"  - task_type: {request.task_type}")
            print(f"  - call_location: {request.call_location}")
            print(f"  - goal_location: {request.goal_location}")
            
            # 서비스 호출
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            
            if future.result() is not None:
                response = future.result()
                
                print(f"📥 TaskRequest 응답 수신:")
                print(f"  - success: {response.success}")
                print(f"  - message: {response.message}")
                
                # 결과 시그널 발생
                self.task_request_completed.emit(response.success, response.message)
                
            else:
                print("❌ TaskRequest 서비스 호출 실패")
                self.task_request_completed.emit(False, "서비스 호출 실패")
                
        except Exception as e:
            print(f"❌ TaskRequest 서비스 호출 중 오류: {e}")
            self.task_request_completed.emit(False, f"서비스 호출 오류: {str(e)}")
    
    def cleanup(self):
        """리소스 정리"""
        self._is_cleaning_up = True
        
        # 스레드가 실행 중이면 종료 대기
        if self.isRunning():
            self.quit()
            self.wait(2000)  # 2초 대기
        
        try:
            if self.node:
                try:
                    if hasattr(self.node, 'get_name') and self.node.get_name():
                        self.node.destroy_node()
                except Exception as e:
                    print(f"⚠️ task_request_client node 정리 중 오류: {e}")
                finally:
                    self.node = None
        except Exception as e:
            print(f"⚠️ task_request_client cleanup 중 오류: {e}")
        
        try:
            if self.client:
                self.client.destroy()
                self.client = None
        except Exception as e:
            print(f"⚠️ task_request_client client 정리 중 오류: {e}")
        
        print("✅ TaskRequestClient 리소스 정리 완료")