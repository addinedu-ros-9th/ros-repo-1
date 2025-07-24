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
        # UI 파일 경로 
        ui_file = os.path.join(os.path.dirname(__file__), 'kiosk', 'ui_files', 'main_window_kiosk.ui')
        uic.loadUi(ui_file, self)
        
        self.setWindowTitle("LIBO KIOSK")
        print("✅ 메인 윈도우 UI 로드 완료")
    
    def setup_connections(self):
        """시그널-슬롯 연결"""
        # 기존 연결 해제 후 다시 연결 (중복 방지)
        try:
            self.book_search.clicked.disconnect()
            self.call_manager.clicked.disconnect()
            self.payment.clicked.disconnect()
        except:
            pass
        
        self.book_search.clicked.connect(self.on_book_search_clicked)
        self.call_manager.clicked.connect(self.on_call_manager_clicked)
        self.payment.clicked.connect(self.on_payment_clicked)
        
        print("✅ 메인 윈도우 시그널-슬롯 연결 완료")

    
    def on_book_search_clicked(self):
        """Book Search 버튼 클릭"""
        print("📚 Book Search 화면으로 전환")
        
        # BookSearchWidget 임포트 및 생성
        from kiosk.ui.book_search_widget import BookSearchWidget
        
        if self.book_search_widget is None:
            self.book_search_widget = BookSearchWidget()
            # 홈 버튼 시그널 연결 (한 번만)
            self.book_search_widget.home_requested.connect(self.show_main_window)
        
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
    
    def show_main_window(self):
        """메인 윈도우로 돌아오기"""
        print("🏠 메인 화면으로 돌아옴")
        
        # 책 검색 윈도우 숨기고 메인 윈도우 표시
        if self.book_search_widget:
            self.book_search_widget.hide()
        self.show()

def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()