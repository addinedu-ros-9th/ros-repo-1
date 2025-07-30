#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.book_search_widget = None  # 책 검색 위젯 참조
        self.init_ui()
        self.setup_connections()
    
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
        
        self.setWindowTitle("LIBO KIOSK")
        
        print("✅ 메인 윈도우 UI 로드 완료")
    
    def setup_connections(self):
        """시그널-슬롯 연결"""
        # 기존 연결 해제 후 다시 연결 (중복 방지)
        try:
            self.book_search.clicked.disconnect()
            self.call_manager.clicked.disconnect()
            self.payment.clicked.disconnect()
            self.book_corner.clicked.disconnect()
        except:
            pass
        
        self.book_search.clicked.connect(self.on_book_search_clicked)
        self.call_manager.clicked.connect(self.on_call_manager_clicked)
        self.payment.clicked.connect(self.on_payment_clicked)
        self.book_corner.clicked.connect(self.on_book_corner_clicked)
        
        print("✅ 메인 윈도우 시그널-슬롯 연결 완료")

    def center_window(self):
        """윈도우를 화면 중앙에 위치시키기"""
        # 화면의 사용 가능한 영역 가져오기
        screen = QApplication.desktop().screenGeometry()
        
        # 윈도우의 크기 가져오기 (실제 크기)
        window_size = self.size()
        
        # 중앙 좌표 계산 (화면 중앙 - 윈도우 크기의 절반)
        center_x = (screen.width() - window_size.width()) // 2
        center_y = (screen.height() - window_size.height()) // 2
        
        # 윈도우 위치 설정 (강제로 설정)
        self.move(center_x, center_y)
        
        # 위치가 제대로 설정되었는지 확인
        current_pos = self.pos()
        if current_pos.x() != center_x or current_pos.y() != center_y:
            # 다시 한 번 강제로 설정
            self.setGeometry(center_x, center_y, window_size.width(), window_size.height())
        
        print(f"✅ 윈도우 중앙 정렬 완료: ({center_x}, {center_y})")
        print(f"화면 크기: {screen.width()}x{screen.height()}, 윈도우 크기: {window_size.width()}x{window_size.height()}")
        print(f"실제 위치: {self.pos().x()}, {self.pos().y()}")
    
    def force_center_window(self):
        """강제로 윈도우를 화면 중앙에 위치시키기"""
        # 화면의 사용 가능한 영역 가져오기
        screen = QApplication.desktop().screenGeometry()
        
        # 윈도우 크기 설정 (UI 파일의 크기)
        window_width = 1101
        window_height = 646
        
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
    
    def on_call_manager_clicked(self):
        """Call Manager 버튼 클릭"""
        print("📞 관리자 호출")
        # TODO: ROS2로 관리자 호출 신호 전송
        QMessageBox.information(self, "관리자 호출", "관리자를 호출했습니다.\n잠시만 기다려주세요.")
    
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
        if hasattr(self, 'book_corner_widget') and self.book_corner_widget:
            import rclpy
            rclpy.spin_once(self.book_corner_widget, timeout_sec=0.0)
    
    def show_main_window(self):
        """메인 윈도우로 돌아오기"""
        print("🏠 메인 화면으로 돌아옴")
        
        # 책 검색 윈도우 숨기고 메인 윈도우 표시
        if self.book_search_widget:
            self.book_search_widget.hide()
        self.show()
        
        # 메인 윈도우 강제 중앙 정렬
        self.force_center_window()

def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    
    # 윈도우가 완전히 로드된 후 강제 중앙 정렬
    QTimer.singleShot(300, window.force_center_window)
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()