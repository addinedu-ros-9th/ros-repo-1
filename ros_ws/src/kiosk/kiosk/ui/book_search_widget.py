#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

# ROS2 클라이언트 import 추가
from kiosk.ros_communication.book_search_client import BookSearchClient

class BookSearchWidget(QWidget):
    # 홈 버튼 클릭 시그널 정의
    home_requested = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        
        # ROS2 클라이언트 초기화
        self.search_client = BookSearchClient()
        self.search_client.search_completed.connect(self.on_search_results)
        
        self.init_ui()
        self.setup_connections()
    
    def init_ui(self):
        """UI 파일 로드"""
        # UI 파일 경로 
        ui_file = os.path.join(os.path.dirname(__file__), '..', 'ui_files', 'bookSearch.ui')
        uic.loadUi(ui_file, self)
        
        # 윈도우 설정
        self.setWindowTitle("LIBO Book Search")
        
        # 리보 상태 초기화
        self.update_libo_status("대기중", "#2ecc71")
        
        print("✅ BookSearch UI 로드 완료")
    
    def setup_connections(self):
        """시그널-슬롯 연결"""
        self.searchButton.clicked.connect(self.on_search_clicked)
        self.homeButton.clicked.connect(self.on_home_clicked)
        self.robotCallButton.clicked.connect(self.on_robot_call_clicked)
        self.orderButton.clicked.connect(self.on_order_clicked)
        
        # 엔터키로도 검색 가능
        self.searchLineEdit.returnPressed.connect(self.on_search_clicked)
        
        print("✅ 시그널-슬롯 연결 완료")
    
    def update_libo_status(self, status_text, color):
        """리보 상태 업데이트"""
        self.liboStatusLabel.setText(status_text)
        self.liboStatusLabel.setStyleSheet(f"color: {color}; border: none;")
    
    def on_search_clicked(self):
        """검색 버튼 클릭"""
        search_text = self.searchLineEdit.text().strip()
        
        if not search_text:
            QMessageBox.warning(self, "검색 오류", "검색어를 입력해주세요.")
            return
        
        print(f"🔍 검색어: {search_text}")
        
        # 검색 타입 확인
        if self.titleRadioButton.isChecked():
            search_type = "title"
        elif self.authorRadioButton.isChecked():
            search_type = "author"
        elif self.publisherRadioButton.isChecked():
            search_type = "publisher"
        elif self.categoryRadioButton.isChecked():
            search_type = "category_name"
        else:
            search_type = "title"
        
        print(f"🔍 검색 타입: {search_type}")
        
        # 검색 중 상태 표시
        self.searchButton.setText("검색중...")
        self.searchButton.setEnabled(False)
        self.update_libo_status("검색중", "#f39c12")
        
        # ROS2 서비스 호출
        self.search_client.search_books(search_text, search_type)

    def on_search_results(self, success, message, books):
        """검색 결과 처리"""
        # UI 상태 복원
        self.searchButton.setText("검색")
        self.searchButton.setEnabled(True)
        self.update_libo_status("대기중", "#2ecc71")
        
        if success:
            print(f"✅ 검색 성공: {len(books)}권 발견")
            self.display_search_results(books)
        else:
            print(f"❌ 검색 실패: {message}")
            QMessageBox.warning(self, "검색 실패", message)

    def display_search_results(self, books):
        """검색 결과를 UI에 표시"""
        # 기존 결과 초기화
        layout = self.bookListWidget.layout()
        if layout is None:
            layout = QVBoxLayout(self.bookListWidget)
        
        # 기존 위젯들 제거
        for i in reversed(range(layout.count())):
            child = layout.itemAt(i)
            if child.widget():
                child.widget().setParent(None)
        
        # 검색 결과가 없으면 메시지 표시
        if not books:
            no_result_label = QLabel("검색 결과가 없습니다.")
            no_result_label.setAlignment(Qt.AlignCenter)
            no_result_label.setStyleSheet("color: #7f8c8d; font-size: 16px; padding: 20px;")
            layout.addWidget(no_result_label)
            return
        
        # 검색 결과 표시
        for book in books:
            book_widget = self.create_book_item_widget(book)
            layout.addWidget(book_widget)
        
        # 스페이서 추가
        layout.addStretch()

    def create_book_item_widget(self, book):
        """개별 책 아이템 위젯 생성 (간소화 버전)"""
        widget = QWidget()
        widget.setFixedHeight(150)  # 높이 줄임
        widget.setStyleSheet("""
            QWidget {
                border: 1px solid #bdc3c7;
                border-radius: 8px;
                background-color: white;
                margin: 5px;
            }
            QWidget:hover {
                background-color: #ecf0f1;
                border-color: #3498db;
            }
        """)
        
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # 책 표지
        cover_label = QLabel("📖")
        cover_label.setFixedSize(80, 90)
        cover_label.setAlignment(Qt.AlignCenter)
        cover_label.setStyleSheet("""
            background-color: #f8f9fa; 
            border: 1px solid #dee2e6;
            border-radius: 4px;
            font-size: 28px;
        """)
        
        # 책 정보 (제목, 저자, 가격만)
        info_widget = QWidget()
        info_layout = QVBoxLayout(info_widget)
        info_layout.setSpacing(8)
        info_layout.setContentsMargins(0, 0, 0, 0)
        
        # 제목
        title_label = QLabel(f"📚 {book['title']}")
        title_label.setFont(QFont("Arial", 15, QFont.Bold))
        title_label.setStyleSheet("color: #2c3e50;")
        title_label.setWordWrap(True)  # 긴 제목 줄바꿈
        
        # 저자
        author_label = QLabel(f"✍️ {book['author']}")
        author_label.setFont(QFont("Arial", 12))
        author_label.setStyleSheet("color: #34495e;")
        
        # 가격
        price_label = QLabel(f"💰 {int(book['price']):,}원")
        price_label.setFont(QFont("Arial", 13, QFont.Bold))
        price_label.setStyleSheet("color: #27ae60;")
        
        # 재고 상태 표시 (간단한 아이콘으로)
        stock_icon = "✅" if book['stock_quantity'] > 0 else "❌"
        stock_status_label = QLabel(stock_icon)
        stock_status_label.setFont(QFont("Arial", 16))
        stock_status_label.setAlignment(Qt.AlignRight)
        
        # 레이아웃에 추가
        info_layout.addWidget(title_label)
        info_layout.addWidget(author_label)
        info_layout.addWidget(price_label)
        info_layout.addStretch()  # 여백 추가
        
        # 상단 레이아웃 (재고 상태 아이콘 우측 상단에)
        top_layout = QHBoxLayout()
        top_layout.addWidget(info_widget)
        top_layout.addWidget(stock_status_label, alignment=Qt.AlignTop)
        
        layout.addWidget(cover_label)
        layout.addLayout(top_layout)
        
        # 클릭 이벤트
        widget.mousePressEvent = lambda event: self.on_book_item_clicked(book)
        
        return widget

    def on_book_item_clicked(self, book):
        """책 아이템 클릭"""
        print(f"📖 선택된 책: {book['title']} ({book['location']}구역)")
        
        # 선택된 책 정보 표시
        QMessageBox.information(
            self, 
            "선택된 도서", 
            f"제목: {book['title']}\n"
            f"저자: {book['author']}\n"
            f"위치: {book['location']}구역\n"
            f"재고: {book['stock_quantity']}권\n\n"
            f"지도에서 위치를 확인하세요!"
        )
        
        # TODO: 지도에서 해당 위치 표시 (빨간색 깜빡임)
    
    def on_home_clicked(self):
        """홈 버튼 클릭"""
        print("🏠 홈으로 이동 요청")
        
        # ROS2 리소스 정리
        if hasattr(self, 'search_client'):
            self.search_client.cleanup()
        
        self.home_requested.emit()  # 시그널 발생
    
    def on_robot_call_clicked(self):
        """리보 호출 버튼 클릭"""
        print("🤖 리보 호출 요청")
        
        # 리보 상태 업데이트
        self.update_libo_status("호출중", "#f39c12")
        
        # TODO: ROS2로 리보 호출 신호 전송
        QMessageBox.information(self, "리보 호출", "리보를 호출했습니다.\n잠시만 기다려주세요.")
        
        # 3초 후 대기 상태로 복귀 (임시)
        QTimer.singleShot(3000, lambda: self.update_libo_status("대기중", "#2ecc71"))
    
    def on_order_clicked(self):
        """주문 문의 버튼 클릭"""
        print("📞 주문 문의")
        # TODO: 관리자에게 알림 전송
        QMessageBox.information(self, "주문 문의", "주문 문의가 접수되었습니다.\n관리자가 곧 연락드리겠습니다.")

    def closeEvent(self, event):
        """윈도우 종료 시 리소스 정리"""
        if hasattr(self, 'search_client'):
            self.search_client.cleanup()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = BookSearchWidget()
    widget.show()
    sys.exit(app.exec_())



