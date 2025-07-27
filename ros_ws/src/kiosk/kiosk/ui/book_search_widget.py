#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import requests
from urllib.parse import urlparse

# ROS2 클라이언트 import 추가
from kiosk.ros_communication.book_search_client import BookSearchClient
from kiosk.ros_communication.escort_request_client import EscortRequestClient
# 책 상세 정보 팝업창 import 추가
from kiosk.ui.book_detail_popup import BookDetailPopup

class BookSearchWidget(QWidget):
    # 홈 버튼 클릭 시그널 정의
    home_requested = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        
        # ROS2 클라이언트 초기화
        self.search_client = BookSearchClient()
        self.search_client.search_completed.connect(self.on_search_results)
        
        # 에스코팅 요청 클라이언트 초기화
        self.escort_client = EscortRequestClient()
        self.escort_client.escort_request_completed.connect(self.on_escort_response)
        
        self.init_ui()
        self.setup_connections()
    
    def init_ui(self):
        """UI 파일 로드"""
        # UI 파일 경로 
        ui_file = os.path.join(os.path.dirname(__file__), '..', 'ui_files', 'bookSearch.ui')
        uic.loadUi(ui_file, self)
        
        # 윈도우 설정
        self.setWindowTitle("LIBO Book Search")
        
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
        
        # ROS2 서비스 호출
        self.search_client.search_books(search_text, search_type)

    def on_search_results(self, success, message, books):
        """검색 결과 처리"""
        # UI 상태 복원
        self.searchButton.setText("검색")
        self.searchButton.setEnabled(True)
        
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

    def load_image_from_url(self, url):
        """URL에서 이미지 로드"""
        if not url:
            return None
        
        try:
            response = requests.get(url, timeout=10)
            response.raise_for_status()
            
            # QPixmap으로 이미지 로드
            pixmap = QPixmap()
            pixmap.loadFromData(response.content)
            
            return pixmap
        except Exception as e:
            print(f"❌ 이미지 로드 실패: {url} - {e}")
            return None

    def create_book_item_widget(self, book):
        """개별 책 아이템 위젯 생성 (이미지 표시 개선)"""
        widget = QWidget()
        widget.setFixedHeight(250)  # 높이 증가
        widget.setStyleSheet("""
            QWidget {
                border: 1px solid #bdc3c7;
                border-radius: 8px;
                background-color: white;
                margin: 8px;
            }
            QWidget:hover {
                background-color: #ecf0f1;
                border-color: #3498db;
            }
        """)
        
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(20, 20, 20, 20)  # 여백 증가
        layout.setSpacing(25)  # 간격 증가
        
        # 책 표지 이미지
        cover_label = QLabel()
        cover_label.setFixedSize(120, 150)  # 크기 증가
        cover_label.setAlignment(Qt.AlignCenter)
        cover_label.setStyleSheet("""
            background-color: #f8f9fa; 
            border: 1px solid #dee2e6;
            border-radius: 6px;
        """)
        
        # 이미지 URL에서 로드
        cover_url = book.get('cover_image_url', '')
        if cover_url:
            pixmap = self.load_image_from_url(cover_url)
            if pixmap:
                # 이미지 크기 조정
                scaled_pixmap = pixmap.scaled(110, 140, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                cover_label.setPixmap(scaled_pixmap)
            else:
                # 이미지 로드 실패 시 기본 아이콘
                cover_label.setText("📖")
                cover_label.setStyleSheet("""
                    background-color: #f8f9fa; 
                    border: 1px solid #dee2e6;
                    border-radius: 6px;
                    font-size: 32px;
                """)
        else:
            # URL이 없으면 기본 아이콘
            cover_label.setText("📖")
            cover_label.setStyleSheet("""
                background-color: #f8f9fa; 
                border: 1px solid #dee2e6;
                border-radius: 6px;
                font-size: 32px;
            """)
        
        # 책 정보 (더 넓은 공간 활용)
        info_widget = QWidget()
        info_widget.setMinimumWidth(800)  # 최소 너비 설정
        info_layout = QVBoxLayout(info_widget)
        info_layout.setSpacing(12)  # 간격 증가
        info_layout.setContentsMargins(0, 0, 0, 0)
        
        # 제목 (더 큰 폰트, 줄바꿈 허용)
        title_label = QLabel(f"📚 {book['title']}")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setStyleSheet("color: #2c3e50; line-height: 1.2;")
        title_label.setWordWrap(True)  # 긴 제목 줄바꿈
        title_label.setMinimumHeight(50)  # 최소 높이 설정
        title_label.setMinimumWidth(600)  # 최소 너비 설정
        
        # 저자
        author_label = QLabel(f"✍️ {book['author']}")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        author_label.setStyleSheet("color: #34495e;")
        title_label.setMinimumHeight(50)  # 최소 높이 설정
        title_label.setMinimumWidth(600)  # 최소 너비 설정
        
        # 출판사
        publisher_label = QLabel(f"🏢 {book['publisher']}")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        publisher_label.setStyleSheet("color: #7f8c8d;")
        title_label.setMinimumHeight(50)  # 최소 높이 설정
        title_label.setMinimumWidth(600)  # 최소 너비 설정
        
        # 가격
        price_label = QLabel(f"💰 {int(book['price']):,}원")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        price_label.setStyleSheet("color: #27ae60;")
        title_label.setMinimumHeight(50)  # 최소 높이 설정
        title_label.setMinimumWidth(600)  # 최소 너비 설정

        # 위치
        location_label = QLabel(f"📍 {book['location']}구역")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        location_label.setStyleSheet("color: #e67e22;")
        title_label.setMinimumHeight(50)  # 최소 높이 설정
        title_label.setMinimumWidth(600)  # 최소 너비 설정
        
        # 재고 상태 표시
        stock_icon = "✅" if book['stock_quantity'] > 0 else "❌"
        stock_status_label = QLabel(f"{stock_icon} {book['stock_quantity']}권")
        stock_status_label.setFont(QFont("Arial", 13, QFont.Bold))
        stock_status_label.setAlignment(Qt.AlignRight)
        stock_status_label.setStyleSheet("color: #27ae60;" if book['stock_quantity'] > 0 else "color: #e74c3c;")
        
        # 레이아웃에 추가
        info_layout.addWidget(title_label)
        info_layout.addWidget(author_label)
        info_layout.addWidget(publisher_label)
        info_layout.addWidget(price_label)
        info_layout.addWidget(location_label)
        info_layout.addStretch()
        
        # 상단 레이아웃 (재고 상태 아이콘 우측 상단에)
        top_layout = QHBoxLayout()
        top_layout.addWidget(info_widget, 1)  # stretch factor 1
        top_layout.addWidget(stock_status_label, 0, alignment=Qt.AlignTop | Qt.AlignRight)
        
        layout.addWidget(cover_label)
        layout.addLayout(top_layout, 1)  # stretch factor 1로 더 많은 공간 할당
        
        # 클릭 이벤트
        widget.mousePressEvent = lambda event: self.on_book_item_clicked(book)
        
        return widget

    def on_book_item_clicked(self, book):
        """책 아이템 클릭"""
        print(f"📖 선택된 책: {book['title']} ({book['location']}구역)")
        
        # 책 상세 정보 팝업창 표시
        try:
            popup = BookDetailPopup(book, self)
            popup.escort_requested.connect(self.on_escort_requested)
            popup.exec_()
        except Exception as e:
            print(f"❌ 팝업창 표시 중 오류: {e}")
            QMessageBox.warning(self, "오류", f"팝업창을 표시할 수 없습니다: {str(e)}")
    
    def on_escort_requested(self, escort_data):
        """에스코팅 요청 처리"""
        try:
            print(f"🚀 에스코팅 요청 수신: {escort_data}")
            
            # ROS2 서비스를 통해 에스코팅 요청
            robot_id = escort_data.get('robot_id', 'robot_01')
            book_title = escort_data.get('book_title', '')
            book_location = escort_data.get('book_location', '')
            
            success = self.escort_client.request_escort(robot_id, book_title, book_location)
            
            if not success:
                QMessageBox.warning(self, "오류", "에스코팅 요청을 전송할 수 없습니다.")
            
        except Exception as e:
            print(f"❌ 에스코팅 요청 처리 중 오류: {e}")
            QMessageBox.warning(self, "오류", f"에스코팅 요청 처리 중 오류가 발생했습니다: {str(e)}")
    
    def on_escort_response(self, success, message, escort_id):
        """에스코팅 요청 응답 처리"""
        try:
            if success:
                QMessageBox.information(
                    self,
                    "에스코팅 요청",
                    f"리보를 호출했습니다.\n"
                    f"에스코팅 ID: {escort_id}\n"
                    f"메시지: {message}\n\n"
                    f"잠시만 기다려주세요."
                )
                print(f"✅ 에스코팅 요청 성공: escort_id={escort_id}")
            else:
                QMessageBox.warning(
                    self,
                    "에스코팅 요청 실패",
                    f"에스코팅 요청이 실패했습니다.\n"
                    f"오류: {message}"
                )
                print(f"❌ 에스코팅 요청 실패: {message}")
                
        except Exception as e:
            print(f"❌ 에스코팅 응답 처리 중 오류: {e}")
    
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
        
        # TODO: ROS2로 리보 호출 신호 전송
        QMessageBox.information(self, "리보 호출", "리보를 호출했습니다.\n잠시만 기다려주세요.")
    
    def on_order_clicked(self):
        """주문 문의 버튼 클릭"""
        print("📞 주문 문의")
        # TODO: 관리자에게 알림 전송
        QMessageBox.information(self, "주문 문의", "주문 문의가 접수되었습니다.\n관리자가 곧 연락드리겠습니다.")

    def closeEvent(self, event):
        """윈도우 종료 시 리소스 정리"""
        if hasattr(self, 'search_client'):
            self.search_client.cleanup()
        if hasattr(self, 'escort_client'):
            self.escort_client.cleanup()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = BookSearchWidget()
    widget.show()
    sys.exit(app.exec_())



