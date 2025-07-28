#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import EscortRequest
from PyQt5.QtCore import QObject, pyqtSignal

class EscortRequestClient(QObject):
    """에스코팅 요청 ROS2 서비스 클라이언트"""
    
    # 시그널 정의
    escort_request_completed = pyqtSignal(bool, str, str)  # success, message, escort_id
    
    def __init__(self):
        super().__init__()
        
        # ROS2 노드 초기화
        try:
            rclpy.init()
            self.node = Node('escort_request_client')
            self.client = self.node.create_client(EscortRequest, '/escort_request')
            
            print("✅ EscortRequestClient 초기화 완료")
            
        except Exception as e:
            print(f"❌ EscortRequestClient 초기화 실패: {e}")
            self.node = None
            self.client = None
    
    def request_escort(self, robot_id, book_title, book_location):
        """
        에스코팅 요청
        
        Args:
            robot_id (str): 로봇 ID (예: "robot_01")
            book_title (str): 도서 제목
            book_location (str): 도서 위치 코드 (예: "D3", "D5")
        
        Returns:
            bool: 요청 성공 여부
        """
        if not self.client:
            print("❌ ROS2 클라이언트가 초기화되지 않았습니다.")
            self.escort_request_completed.emit(False, "ROS2 클라이언트 초기화 실패", "")
            return False
        
        try:
            # 서비스 요청 데이터 생성
            request = EscortRequest.Request()
            request.robot_id = robot_id
            request.book_title = book_title
            request.book_location = book_location
            
            print(f"🚀 에스코팅 요청 전송: robot_id={robot_id}, book_title={book_title}, book_location={book_location}")
            
            # 서비스 호출
            future = self.client.call_async(request)
            
            # 응답 처리
            future.add_done_callback(self._handle_response)
            
            return True
            
        except Exception as e:
            print(f"❌ 에스코팅 요청 중 오류: {e}")
            self.escort_request_completed.emit(False, f"요청 중 오류: {str(e)}", "")
            return False
    
    def _handle_response(self, future):
        """서비스 응답 처리"""
        try:
            response = future.result()
            
            success = response.success
            message = response.message
            escort_id = response.escort_id
            
            print(f"📡 에스코팅 요청 응답: success={success}, message={message}, escort_id={escort_id}")
            
            # 시그널 발생
            self.escort_request_completed.emit(success, message, escort_id)
            
        except Exception as e:
            print(f"❌ 응답 처리 중 오류: {e}")
            self.escort_request_completed.emit(False, f"응답 처리 오류: {str(e)}", "")
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.node:
                # 노드가 유효한지 확인
                try:
                    if hasattr(self.node, 'get_name') and self.node.get_name():
                        self.node.destroy_node()
                except Exception as e:
                    print(f"⚠️ escort_client node 정리 중 오류: {e}")
                finally:
                    self.node = None
        except Exception as e:
            print(f"⚠️ escort_client cleanup 중 오류: {e}")
        
        try:
            if self.client:
                self.client.destroy()
                self.client = None
        except Exception as e:
            print(f"⚠️ escort_client client 정리 중 오류: {e}")
        
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"⚠️ rclpy shutdown 중 오류: {e}")
        
        print("✅ EscortRequestClient 리소스 정리 완료")

# 테스트용 코드
if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    client = EscortRequestClient()
    
    def on_response(success, message, escort_id):
        print(f"응답: success={success}, message={message}, escort_id={escort_id}")
        app.quit()
    
    client.escort_request_completed.connect(on_response)
    
    # 테스트 요청
    client.request_escort("robot_01", "밑바닥부터 시작하는 딥러닝 1", "D3")
    
    sys.exit(app.exec_()) 