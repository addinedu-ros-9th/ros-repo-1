#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import rclpy
from rclpy.node import Node

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # ROS2 노드 초기화
        if not rclpy.ok():
            rclpy.init()
        self.ros_node = Node('kiosk_main_window')
        
        self.book_search_widget = None  # 책 검색 위젯 참조
        self.qr_check_client = None  # QR 체크 클라이언트
        self.task_request_client = None  # 태스크 요청 클라이언트
        self.admin_authenticated = False  # 관리자 인증 상태
        self.call_robot_timer = None  # Call Robot 버튼 타이머
        
        self.init_ui()
        self.setup_connections()
        self.init_ros_clients()
        
        print("✅ MainWindow 초기화 완료")
    
    def init_ui(self):
        """메인 UI 파일 로드"""
        # UI 파일 경로 - ROS2 패키지 설치 경로에서 찾기
        try:
            # 먼저 현재 디렉토리 기준으로 시도
            ui_file = os.path.join(os.path.dirname(__file__), '..', 'ui_files', 'main_window_kiosk.ui')
            if not os.path.exists(ui_file):
                # ROS2 패키지 설치 경로에서 찾기
                import ament_index_python
                ui_file = os.path.join(ament_index_python.get_package_share_directory('kiosk'), 'ui_files', 'main_window_kiosk.ui')
            uic.loadUi(ui_file, self)
        except Exception as e:
            print(f"UI 파일 로드 실패: {e}")
            print(f"시도한 경로: {ui_file}")
            raise
        
        self.setWindowTitle("LIBO BOOK STORE")
        
        # Call Robot 버튼 초기 상태 설정 (숨김)
        self.call_manager.setVisible(False)
        
        # 윈도우 크기 설정 (새로운 디자인에 맞게)
        self.resize(1200, 800)
        
        # 윈도우를 항상 최상위에 유지
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        
        print("✅ 메인 윈도우 UI 로드 완료")
    
    def setup_connections(self):
        """시그널-슬롯 연결"""
        # 기존 연결 해제 후 다시 연결 (중복 방지)
        try:
            self.book_search.clicked.disconnect()
            self.call_manager.clicked.disconnect()
            self.payment.clicked.disconnect()
            self.book_corner.clicked.disconnect()
            self.qr_scan_button.clicked.disconnect()
        except:
            pass
        
        self.book_search.clicked.connect(self.on_book_search_clicked)
        self.call_manager.clicked.connect(self.on_call_robot_clicked)
        self.payment.clicked.connect(self.on_payment_clicked)
        self.book_corner.clicked.connect(self.on_book_corner_clicked)
        self.qr_scan_button.clicked.connect(self.on_qr_scan_clicked)
        
        print("✅ 메인 윈도우 시그널-슬롯 연결 완료")
    
    def init_ros_clients(self):
        """ROS2 클라이언트 초기화"""
        try:
            # QR 체크 클라이언트 초기화 (pyzbar 의존성 확인)
            try:
                from kiosk.ros_communication.kiosk_qr_check_client import KioskQRCheckClient
                self.qr_check_client = KioskQRCheckClient(self.ros_node)
                self.qr_check_client.qr_check_completed.connect(self.on_qr_check_completed)
                print("✅ QR 스캔 클라이언트 초기화 완료")
            except ImportError as e:
                print(f"⚠️ QR 스캔 기능 비활성화: {e}")
                print("💡 QR 스캔을 사용하려면 'pip install pyzbar opencv-python'을 실행하세요")
                self.qr_check_client = None
            
            # 태스크 요청 클라이언트 초기화
            from kiosk.ros_communication.task_request_client import TaskRequestClient
            self.task_request_client = TaskRequestClient()
            self.task_request_client.task_request_completed.connect(self.on_task_request_response)
            
            print("✅ ROS2 클라이언트 초기화 완료")
            
        except Exception as e:
            print(f"❌ ROS2 클라이언트 초기화 실패: {e}")
    
    def on_qr_scan_clicked(self):
        """QR 스캔 버튼 클릭"""
        print("🔍 QR 스캔 시작")
        
        if self.qr_check_client:
            self.qr_check_client.start_qr_scan()
        else:
            QMessageBox.warning(self, "QR 스캔 기능 비활성화", 
                              "QR 스캔 기능을 사용할 수 없습니다.\n\n"
                              "QR 스캔을 사용하려면 다음 명령을 실행하세요:\n"
                              "pip install pyzbar opencv-python")
    
    def on_qr_check_completed(self, success, message):
        """QR 체크 완료 처리"""
        if success:
            print(f"✅ QR 인증 성공: {message}")
            self.admin_authenticated = True
            
            # Call Robot 버튼 표시
            self.show_call_robot_button()
            
            QMessageBox.information(self, "QR 인증 성공", 
                                  f"관리자 인증이 완료되었습니다.\n{message}\n\nCall Robot 버튼이 5초간 표시됩니다.")
        else:
            print(f"❌ QR 인증 실패: {message}")
            self.admin_authenticated = False
            QMessageBox.warning(self, "QR 인증 실패", f"QR 인증에 실패했습니다.\n{message}")
    
    def show_call_robot_button(self):
        """Call Robot 버튼 표시 (5초간)"""
        self.call_manager.setVisible(True)
        
        # 5초 후 버튼 숨기기
        if self.call_robot_timer:
            self.call_robot_timer.stop()
        
        self.call_robot_timer = QTimer()
        self.call_robot_timer.timeout.connect(self.hide_call_robot_button)
        self.call_robot_timer.start(5000)  # 5초
        
        print("🤖 Call Robot 버튼 표시 (5초)")
    
    def hide_call_robot_button(self):
        """Call Robot 버튼 숨기기"""
        self.call_manager.setVisible(False)
        self.admin_authenticated = False
        
        if self.call_robot_timer:
            self.call_robot_timer.stop()
        
        print("🤖 Call Robot 버튼 숨김")
    
    def on_call_robot_clicked(self):
        """Call Robot 버튼 클릭 (관리자용 로봇 호출)"""
        if not self.admin_authenticated:
            QMessageBox.warning(self, "인증 필요", "먼저 QR 인증을 완료해주세요.")
            return
        
        print("🤖 관리자용 로봇 호출 요청")
        
        try:
            # TaskRequest.srv 파라미터 준비
            robot_id = ""  # task_manager에서 자동 선택
            task_type = "assist"
            call_location = "E9"  # 키오스크 위치 (kiosk_1: 8.98, -0.16)
            goal_location = ""  # 어시스트 임무는 목적지 없음
            
            print(f"📍 TaskRequest 파라미터:")
            print(f"   robot_id: '{robot_id}' (task_manager에서 자동 선택)")
            print(f"   task_type: {task_type}")
            print(f"   call_location: {call_location} (키오스크)")
            print(f"   goal_location: '{goal_location}' (어시스트는 목적지 없음)")
            
            # Main Server의 task_manager.py로 TaskRequest 서비스 호출
            self.task_request_client.send_task_request(robot_id, task_type, call_location, goal_location)
            
            # 버튼 비활성화
            self.call_manager.setEnabled(False)
            self.call_manager.setText("요청 중...")
            
        except Exception as e:
            print(f"❌ 로봇 호출 요청 중 오류: {e}")
            QMessageBox.warning(self, "오류", f"로봇 호출 요청 중 오류가 발생했습니다: {str(e)}")
            
            # 버튼 재활성화
            self.call_manager.setEnabled(True)
            self.call_manager.setText("🤖 Call Robot")
    
    def on_task_request_response(self, success, message):
        """태스크 요청 응답 처리"""
        # 버튼 재활성화
        self.call_manager.setEnabled(True)
        self.call_manager.setText("🤖 Call Robot")
        
        if success:
            print(f"✅ 로봇 호출 성공: {message}")
            QMessageBox.information(self, "로봇 호출 성공", 
                                  f"로봇 호출이 성공했습니다.\n{message}\n\n로봇이 키오스크로 이동 중입니다.")
            
            # Call Robot 버튼 숨기기
            self.hide_call_robot_button()
        else:
            print(f"❌ 로봇 호출 실패: {message}")
            QMessageBox.warning(self, "로봇 호출 실패", 
                              f"로봇 호출에 실패했습니다.\n{message}")

    def center_window(self):
        """윈도우를 화면 중앙에 위치시키기"""
        # 화면의 사용 가능한 영역 가져오기
        screen = QApplication.desktop().screenGeometry()
        
        # 윈도우의 크기 가져오기 (새로운 크기)
        window_width = 1200
        window_height = 800
        
        # 중앙 좌표 계산
        center_x = (screen.width() - window_width) // 2
        center_y = (screen.height() - window_height) // 2
        
        # 윈도우 크기와 위치를 설정
        self.setGeometry(center_x, center_y, window_width, window_height)
        
        print(f"✅ 윈도우 중앙 정렬 완료: ({center_x}, {center_y})")
        print(f"화면 크기: {screen.width()}x{screen.height()}, 윈도우 크기: {window_width}x{window_height}")
        print(f"실제 위치: {self.pos().x()}, {self.pos().y()}")
    
    def force_center_window(self):
        """강제로 윈도우를 화면 중앙에 위치시키기"""
        # 화면의 사용 가능한 영역 가져오기
        screen = QApplication.desktop().screenGeometry()
        
        # 윈도우 크기 설정 (새로운 디자인 크기)
        window_width = 1200
        window_height = 800
        
        # 중앙 좌표 계산
        center_x = (screen.width() - window_width) // 2
        center_y = (screen.height() - window_height) // 2
        
        # 윈도우 크기와 위치를 강제로 설정
        self.setGeometry(center_x, center_y, window_width, window_height)
        
        print(f"🔧 강제 윈도우 중앙 정렬: ({center_x}, {center_y})")
        print(f"화면 크기: {screen.width()}x{screen.height()}")
        print(f"설정된 위치: {self.pos().x()}, {self.pos().y()}")
    
    def showEvent(self, event):
        """윈도우가 표시될 때마다 중앙 정렬"""
        super().showEvent(event)
        # 윈도우가 완전히 표시된 후 중앙 정렬
        QTimer.singleShot(50, self.force_center_window)
    
    def on_book_search_clicked(self):
        """Book Search 버튼 클릭"""
        print("📚 Book Search 화면으로 전환")
        
        # BookSearchWidget 임포트 및 생성
        from kiosk.ui.book_search_widget import BookSearchWidget
        
        if self.book_search_widget is None:
            self.book_search_widget = BookSearchWidget()
            # 홈 버튼 시그널 연결 (한 번만)
            self.book_search_widget.home_requested.connect(self.show_main_window)
        else:
            # 기존 위젯이 있으면 초기화
            self.book_search_widget.reset_widget()
        
        # 현재 윈도우 숨기고 책 검색 윈도우 표시
        self.hide()
        self.book_search_widget.show()
    
    def on_payment_clicked(self):
        """Payment 버튼 클릭"""
        print("💳 결제 화면으로 전환")
        # TODO: 결제 위젯으로 화면 전환
        QMessageBox.information(self, "결제", "결제 기능은 준비 중입니다.")
    
    def on_book_corner_clicked(self):
        """Book Corner 버튼 클릭"""
        print("📚 Book Corner 화면으로 전환")
        
        # BookCornerWidget 임포트 및 생성
        from kiosk.ui.book_corner_widget import BookCornerWidget
        
        if not hasattr(self, 'book_corner_widget') or self.book_corner_widget is None:
            # ROS2 노드 초기화 확인
            import rclpy
            if not rclpy.ok():
                rclpy.init()
            
            self.book_corner_widget = BookCornerWidget()
            # 홈 버튼 시그널 연결 (한 번만)
            self.book_corner_widget.home_requested.connect(self.show_main_window)
            
            # ROS2 노드 스핀을 위한 타이머 설정
            self.ros_timer = QTimer()
            self.ros_timer.timeout.connect(self.spin_ros_node)
            self.ros_timer.start(100)  # 100ms마다 ROS2 노드 스핀
        else:
            # 기존 위젯이 있으면 초기화
            self.book_corner_widget.reset_widget()
        
        # 현재 윈도우 숨기고 Book Corner 윈도우 표시
        self.hide()
        self.book_corner_widget.show()
    
    def spin_ros_node(self):
        """ROS2 노드 스핀 처리"""
        import rclpy
        
        if hasattr(self, 'book_corner_widget') and self.book_corner_widget:
            rclpy.spin_once(self.book_corner_widget, timeout_sec=0.0)
        
        # 메인 윈도우의 ROS2 노드도 스핀
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)
    
    def show_main_window(self):
        """메인 윈도우로 돌아오기"""
        print("🏠 메인 화면으로 돌아옴")
        
        # 책 검색 윈도우 숨기고 메인 윈도우 표시
        if self.book_search_widget:
            self.book_search_widget.hide()
        self.show()
        
        # 메인 윈도우 강제 중앙 정렬
        self.force_center_window()
    
    def closeEvent(self, event):
        """윈도우 종료 시 리소스 정리"""
        if self.qr_check_client:
            self.qr_check_client.cleanup()
        
        # ROS2 노드 정리
        if hasattr(self, 'ros_node'):
            self.ros_node.destroy_node()
        
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    
    # 애플리케이션 전체 스타일 설정
    app.setStyle('Fusion')
    
    window = MainWindow()
    window.show()
    
    # 윈도우가 완전히 로드된 후 강제 중앙 정렬
    QTimer.singleShot(300, window.force_center_window)
    
    # ROS2 노드 스핀을 위한 타이머 설정
    ros_timer = QTimer()
    ros_timer.timeout.connect(window.spin_ros_node)
    ros_timer.start(100)  # 100ms마다 ROS2 노드 스핀
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()