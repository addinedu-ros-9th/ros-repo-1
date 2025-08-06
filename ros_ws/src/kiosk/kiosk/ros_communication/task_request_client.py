#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest
from PyQt5.QtCore import QThread, pyqtSignal
import threading
import time
import uuid

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
        self._node_initialized = False
        self._node_name = f'task_request_client_{uuid.uuid4().hex[:8]}'  # 고유한 노드 이름
        
    def init_ros(self):
        """ROS2 초기화"""
        try:
            with self._lock:
                if self._is_cleaning_up:
                    return False
                
                # 이미 초기화되어 있으면 성공 반환
                if self._node_initialized and self.node is not None:
                    print("✅ TaskRequestClient 이미 초기화됨")
                    return True
                
                # 기존 노드가 있지만 초기화되지 않은 경우 정리
                if self.node is not None and not self._node_initialized:
                    try:
                        if self.client:
                            self.client.destroy()
                            self.client = None
                        self.node.destroy_node()
                        self.node = None
                    except Exception as e:
                        print(f"⚠️ 기존 노드 정리 중 오류: {e}")
                
                if not rclpy.ok():
                    rclpy.init()
                
                if self.node is None:
                    # 고유한 노드 이름으로 생성
                    self.node = Node(self._node_name)
                    self.client = self.node.create_client(TaskRequest, '/task_request')
                
                # 서비스 서버 대기 (타임아웃 설정)
                timeout_count = 0
                max_timeout = 10  # 최대 10초 대기
                
                while not self.client.wait_for_service(timeout_sec=1.0):
                    if self._is_cleaning_up:
                        return False
                    
                    self.node.get_logger().info('📡 task_request 서비스 대기 중...')
                    timeout_count += 1
                    
                    if timeout_count >= max_timeout:
                        print("❌ task_request 서비스 서버를 찾을 수 없습니다.")
                        return False
                
                self._node_initialized = True
                print(f"✅ TaskRequestClient ROS2 초기화 완료 (노드: {self._node_name})")
                return True
                
        except Exception as e:
            print(f"❌ TaskRequestClient ROS2 초기화 실패: {e}")
            # 초기화 실패 시 상태 초기화
            self._node_initialized = False
            self.node = None
            self.client = None
            return False
    
    def send_task_request(self, robot_id, task_type, call_location, goal_location):
        """
        일반적인 작업 요청 (팔로우, 에스코팅 등)
        
        Args:
            robot_id (str): 로봇 ID (예: "libo_a")
            task_type (str): 작업 타입 (예: "follow", "escort")
            call_location (str): 호출지 위치 (예: "E9" - 키오스크)
            goal_location (str): 목적지 위치 (예: "D5" - 책 위치)
        
        Returns:
            bool: 요청 시작 성공 여부
        """
        try:
            # 요청 데이터 저장
            self.request_data = {
                'robot_id': robot_id,
                'task_type': task_type,
                'call_location': call_location,
                'goal_location': goal_location
            }
            
            print(f"🚀 작업 요청 준비: {self.request_data}")
            
            # QThread로 비동기 요청 시작
            self.start()
            return True
            
        except Exception as e:
            print(f"❌ 작업 요청 준비 중 오류: {e}")
            self.task_request_completed.emit(False, f"요청 준비 중 오류: {str(e)}")
            return False
    
    def request_escort_task(self, robot_id, call_location, goal_location):
        """
        에스코팅 작업 요청 (기존 호환성 유지)
        
        Args:
            robot_id (str): 로봇 ID (예: "libo_a")
            call_location (str): 호출지 위치 (예: "E9" - 키오스크)
            goal_location (str): 목적지 위치 (예: "D5" - 책 위치)
        
        Returns:
            bool: 요청 시작 성공 여부
        """
        return self.send_task_request(robot_id, 'escort', call_location, goal_location)
    
    def run(self):
        """QThread 실행 (백그라운드에서 ROS2 서비스 호출)"""
        try:
            # ROS2 초기화
            if not self._node_initialized:
                if not self.init_ros():
                    self.task_request_completed.emit(False, "ROS2 초기화 실패")
                    return
            
            if self.node is None or self.client is None:
                self.task_request_completed.emit(False, "ROS2 클라이언트 초기화 실패")
                return
            
            # 요청 데이터 확인
            if self.request_data is None:
                self.task_request_completed.emit(False, "요청 데이터가 없습니다")
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
            
            # 서비스 호출 (타임아웃 설정)
            future = self.client.call_async(request)
            
            # 안전한 타임아웃 처리
            timeout_seconds = 30.0  # 30초 타임아웃
            start_time = time.time()
            
            while not future.done():
                if time.time() - start_time > timeout_seconds:
                    print("❌ TaskRequest 서비스 호출 타임아웃")
                    self.task_request_completed.emit(False, "서비스 호출 타임아웃")
                    return
                
                # 짧은 간격으로 스핀
                try:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                except Exception as spin_error:
                    print(f"⚠️ spin_once 오류 (무시): {spin_error}")
                    time.sleep(0.1)  # 잠시 대기
                    continue
            
            if future.done():
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
            else:
                print("❌ TaskRequest 서비스 호출 타임아웃")
                self.task_request_completed.emit(False, "서비스 호출 타임아웃")
                
        except Exception as e:
            print(f"❌ TaskRequest 서비스 호출 중 오류: {e}")
            self.task_request_completed.emit(False, f"서비스 호출 오류: {str(e)}")
    
    def cleanup(self):
        """리소스 정리"""
        with self._lock:
            self._is_cleaning_up = True
        
        # 스레드가 실행 중이면 종료 대기
        if self.isRunning():
            self.quit()
            self.wait(3000)  # 3초 대기
        
        try:
            if self.client:
                self.client.destroy()
                self.client = None
        except Exception as e:
            print(f"⚠️ task_request_client client 정리 중 오류: {e}")
        
        try:
            if self.node:
                try:
                    if hasattr(self.node, 'get_name') and self.node.get_name():
                        self.node.destroy_node()
                except Exception as e:
                    print(f"⚠️ task_request_client node 정리 중 오류: {e}")
                finally:
                    self.node = None
                    self._node_initialized = False
                    # 새로운 고유한 노드 이름 생성
                    self._node_name = f'task_request_client_{uuid.uuid4().hex[:8]}'
        except Exception as e:
            print(f"⚠️ task_request_client cleanup 중 오류: {e}")
        
        print("✅ TaskRequestClient 리소스 정리 완료")