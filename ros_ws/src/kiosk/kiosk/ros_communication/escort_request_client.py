#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest  # EscortRequest에서 TaskRequest로 변경
from PyQt5.QtCore import QObject, pyqtSignal

class TaskRequestClient(QObject):  # 클래스 이름 변경
    """작업 요청 ROS2 서비스 클라이언트"""  # 주석 변경
    
    # 시그널 정의
    task_request_completed = pyqtSignal(bool, str)  # success, message로 변경
    
    def __init__(self):  # TaskRequestClient 초기화 및 서비스 클라이언트 설정
        super().__init__()
        
        # ROS2 노드 초기화
        try:
            rclpy.init()
            self.node = Node('task_request_client')  # 노드 이름 변경
            self.client = self.node.create_client(TaskRequest, '/task_request')  # 서비스 이름 변경
            
            print("✅ TaskRequestClient 초기화 완료")  # 로그 메시지 변경
            
        except Exception as e:
            print(f"❌ TaskRequestClient 초기화 실패: {e}")  # 로그 메시지 변경
            self.node = None
            self.client = None
    
    def request_task(self, robot_id, task_type, call_location, goal_location):  # 메서드 이름과 파라미터 변경
        """
        작업 요청
        
        Args:
            robot_id (str): 로봇 ID (예: "libo_a", "libo_b")
            task_type (str): 작업 타입 (예: "escort", "assist", "delivery")
            call_location (str): 호출지 waypoint (예: "A2", "D3")
            goal_location (str): 목적지 waypoint (예: "A1", "E2")
        
        Returns:
            bool: 요청 성공 여부
        """
        if not self.client:
            print("❌ ROS2 클라이언트가 초기화되지 않았습니다.")
            self.task_request_completed.emit(False, "ROS2 클라이언트 초기화 실패")  # 시그널 변경
            return False
        
        try:
            # 서비스 요청 데이터 생성
            request = TaskRequest.Request()  # TaskRequest로 변경
            request.robot_id = robot_id
            request.task_type = task_type
            request.call_location = call_location
            request.goal_location = goal_location
            
            print(f"🚀 작업 요청 전송: robot_id={robot_id}, task_type={task_type}, call_location={call_location}, goal_location={goal_location}")  # 로그 메시지 변경
            
            # 서비스 호출
            future = self.client.call_async(request)
            
            # 응답 처리
            future.add_done_callback(self._handle_response)
            
            return True
            
        except Exception as e:
            print(f"❌ 작업 요청 중 오류: {e}")  # 로그 메시지 변경
            self.task_request_completed.emit(False, f"요청 중 오류: {str(e)}")  # 시그널 변경
            return False
    
    def _handle_response(self, future):  # 서비스 응답 처리
        """서비스 응답 처리"""
        try:
            response = future.result()
            
            success = response.success
            message = response.message
            
            print(f"📡 작업 요청 응답: success={success}, message={message}")  # 로그 메시지 변경
            
            # 시그널 발생
            self.task_request_completed.emit(success, message)  # 시그널 변경
            
        except Exception as e:
            print(f"❌ 응답 처리 중 오류: {e}")
            self.task_request_completed.emit(False, f"응답 처리 오류: {str(e)}")  # 시그널 변경
    
    def cleanup(self):  # 리소스 정리
        """리소스 정리"""
        try:
            if self.node:
                # 노드가 유효한지 확인
                try:
                    if hasattr(self.node, 'get_name') and self.node.get_name():
                        self.node.destroy_node()
                except Exception as e:
                    print(f"⚠️ task_request_client node 정리 중 오류: {e}")  # 로그 메시지 변경
                finally:
                    self.node = None
        except Exception as e:
            print(f"⚠️ task_request_client cleanup 중 오류: {e}")  # 로그 메시지 변경
        
        try:
            if self.client:
                self.client.destroy()
                self.client = None
        except Exception as e:
            print(f"⚠️ task_request_client client 정리 중 오류: {e}")  # 로그 메시지 변경
        
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"⚠️ rclpy shutdown 중 오류: {e}")
        
        print("✅ TaskRequestClient 리소스 정리 완료")  # 로그 메시지 변경

# 테스트용 코드
if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    client = TaskRequestClient()  # 클래스 이름 변경
    
    def on_response(success, message):  # 콜백 함수 변경
        print(f"응답: success={success}, message={message}")
        app.quit()
    
    client.task_request_completed.connect(on_response)  # 시그널 변경
    
    # 테스트 요청
    client.request_task("libo_a", "escort", "A2", "D3")  # 메서드와 파라미터 변경
    
    sys.exit(app.exec_()) 