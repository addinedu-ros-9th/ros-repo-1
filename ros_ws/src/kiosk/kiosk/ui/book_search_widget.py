#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QScrollArea
from PyQt5 import uic
import requests
from urllib.parse import urlparse

# ROS2 클라이언트 import 추가
from kiosk.ros_communication.book_search_client import BookSearchClient
from kiosk.ros_communication.task_request_client import TaskRequestClient
# 책 상세 정보 팝업창 import 추가
from kiosk.ui.book_detail_popup import BookDetailPopup

class BookSearchWidget(QWidget):
    # 홈 버튼 클릭 시그널 정의
    home_requested = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        
        # ROS2 클라이언트 초기화 (지연 초기화)
        self.search_client = None
        self.task_request_client = None
        
        self.init_ui()
        self.setup_connections()
        
        # 초기화 시 빈 화면으로 시작
        self.show_empty_state()
        
        # 초기 버튼 텍스트 설정
        self.update_search_button_text()
        
        print("✅ BookSearchWidget 초기화 완료")
    
    def init_ui(self):
        """UI 파일 로드"""
        # UI 파일 경로 - ROS2 패키지 설치 경로에서 찾기
        try:
            # 먼저 현재 디렉토리 기준으로 시도
            ui_file = os.path.join(os.path.dirname(__file__), '..', 'ui_files', 'bookSearch.ui')
            if not os.path.exists(ui_file):
                # ROS2 패키지 설치 경로에서 찾기
                import ament_index_python
                ui_file = os.path.join(ament_index_python.get_package_share_directory('kiosk'), 'ui_files', 'bookSearch.ui')
            uic.loadUi(ui_file, self)
        except Exception as e:
            print(f"UI 파일 로드 실패: {e}")
            print(f"시도한 경로: {ui_file}")
            raise
        
        # 윈도우 설정
        self.setWindowTitle("LIBO Book Search")
        
        print("✅ BookSearch UI 로드 완료")
    
    def setup_connections(self):
        """시그널-슬롯 연결"""
        self.searchButton.clicked.connect(self.on_search_clicked)
        self.homeButton.clicked.connect(self.on_home_clicked)
        self.orderButton.clicked.connect(self.on_order_clicked)
        
        # 엔터키로도 검색 가능
        self.searchLineEdit.returnPressed.connect(self.on_search_clicked)
        
        # 검색어 변경 시 버튼 텍스트 업데이트
        self.searchLineEdit.textChanged.connect(self.update_search_button_text)
        
        # 검색 라인 에디터에서 엔터키 이벤트 차단
        self.searchLineEdit.installEventFilter(self)
        
        print("✅ 시그널-슬롯 연결 완료")
    
    def show_empty_state(self):
        """빈 상태 표시 (초기 화면)"""
        layout = self.bookListWidget.layout()
        if layout is None:
            layout = QVBoxLayout(self.bookListWidget)
        
        # 기존 위젯들 완전히 제거
        self.clear_book_list_widget()
        
        # 빈 상태 메시지 표시
        empty_label = QLabel("🔍 검색어를 입력하고 검색 버튼을 눌러주세요.")
        empty_label.setAlignment(Qt.AlignCenter)
        empty_label.setStyleSheet("""
            color: #7f8c8d; 
            font-size: 16px; 
            padding: 40px; 
            line-height: 1.5;
            background-color: #f8f9fa;
            border-radius: 10px;
            margin: 20px;
        """)
        layout.addWidget(empty_label)
        
        print("✅ 빈 상태 표시 완료")
    
    def on_search_clicked(self):
        """검색 버튼 클릭"""
        search_text = self.searchLineEdit.text().strip()
        
        # 검색어가 없으면 검색어 입력 요청 팝업창 표시
        if not search_text:
            print("🔍 검색어 없음 - 입력 요청 팝업창 표시")
            self.show_search_input_request_popup()
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
        
        # 검색 시작 시 스크롤을 맨 위로 이동
        self.scroll_to_top()
        
        # 검색 중 상태 표시
        self.searchButton.setText("검색중...")
        self.searchButton.setEnabled(False)
        
        # ROS2 서비스 호출
        try:
            # 클라이언트 지연 초기화
            if self.search_client is None:
                self.search_client = BookSearchClient()
                self.search_client.search_completed.connect(self.on_search_results)
            
            print(f"🚀 검색 요청 전송: {search_text} ({search_type})")
            self.search_client.search_books(search_text, search_type)
        except Exception as e:
            print(f"❌ 검색 요청 중 오류: {e}")
            self.searchButton.setText("검색")
            self.searchButton.setEnabled(True)
            QMessageBox.warning(self, "검색 오류", f"검색 요청 중 오류가 발생했습니다: {str(e)}")
    
    def reset_widget(self):
        """위젯 초기화"""
        try:
            # 검색 라인 에디터 초기화
            self.searchLineEdit.clear()
            
            # 검색 버튼 초기화
            self.searchButton.setText("검색")
            self.searchButton.setEnabled(True)
            
            # 검색 결과 리스트 초기화
            self.clear_book_list_widget()
            
            # 기본 검색 타입 설정 (제목)
            self.titleRadioButton.setChecked(True)
            
            # 빈 상태로 표시
            self.show_empty_state()
            
            print("✅ 위젯 초기화 완료")
            
        except Exception as e:
            print(f"❌ 위젯 초기화 중 오류: {e}")

    def on_search_results(self, success, message, books):
        """검색 결과 처리"""
        print(f"📥 검색 결과 수신: success={success}, message={message}, books_count={len(books) if books else 0}")
        
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
        # 기존 결과 완전 초기화
        layout = self.bookListWidget.layout()
        if layout is None:
            layout = QVBoxLayout(self.bookListWidget)
        
        # 기존 위젯들 완전히 제거
        self.clear_book_list_widget()
        
        # 검색어 확인
        search_text = self.searchLineEdit.text().strip()
        
        # 검색 결과가 없으면 메시지 표시
        if not books:
            no_result_label = QLabel(f"'{search_text}'에 대한 검색 결과가 없습니다.\n다른 검색어를 시도해보세요.")
            no_result_label.setAlignment(Qt.AlignCenter)
            no_result_label.setStyleSheet("color: #7f8c8d; font-size: 16px; padding: 20px; line-height: 1.5;")
            layout.addWidget(no_result_label)
            return
        
        # 결과 개수 표시
        result_count_label = QLabel(f"🔍 '{search_text}' 검색 결과: {len(books)}권")
        
        result_count_label.setAlignment(Qt.AlignCenter)
        result_count_label.setStyleSheet("color: #2c3e50; font-size: 14px; font-weight: bold; padding: 10px; background-color: #ecf0f1; border-radius: 5px; margin: 5px;")
        layout.addWidget(result_count_label)
        
        # 검색 결과 표시
        for book in books:
            book_widget = self.create_book_item_widget(book)
            layout.addWidget(book_widget)
        
        # 스페이서 추가
        layout.addStretch()
        
        # 스크롤을 맨 위로 이동
        self.scroll_to_top()

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
        """개별 책 아이템 위젯 생성"""
        widget = QWidget()
        widget.setFixedHeight(250)
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
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(20)
        
        # 책 표지 이미지
        cover_label = QLabel()
        cover_label.setFixedSize(150, 200)
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
                scaled_pixmap = pixmap.scaled(150, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                cover_label.setPixmap(scaled_pixmap)
            else:
                cover_label.setText("📖")
                cover_label.setStyleSheet("""
                    background-color: #f8f9fa; 
                    border: 1px solid #dee2e6;
                    border-radius: 6px;
                    font-size: 48px;
                """)
        else:
            cover_label.setText("📖")
            cover_label.setStyleSheet("""
                background-color: #f8f9fa; 
                border: 1px solid #dee2e6;
                border-radius: 6px;
                font-size: 48px;
            """)
        
        # 책 정보
        info_widget = QWidget()
        info_widget.setMinimumWidth(800)
        info_layout = QVBoxLayout(info_widget)
        info_layout.setSpacing(0)
        info_layout.setContentsMargins(0, 0, 0, 0)
        
        # 제목
        title_label = QLabel(f"📚 {book['title']}")
        title_label.setFont(QFont("Arial", 16, QFont.Bold))
        title_label.setStyleSheet("color: #2c3e50; line-height: 1.2;")
        title_label.setWordWrap(True)
        title_label.setMinimumHeight(60)
        title_label.setMinimumWidth(600)
        
        # 저자
        author_label = QLabel(f"✍️ {book['author']}")
        author_label.setFont(QFont("Arial", 14))
        author_label.setStyleSheet("color: #34495e;")
        author_label.setMinimumHeight(35)
        author_label.setMinimumWidth(600)
        
        # 출판사
        publisher_label = QLabel(f"🏢 {book['publisher']}")
        publisher_label.setFont(QFont("Arial", 14))
        publisher_label.setStyleSheet("color: #7f8c8d;")
        publisher_label.setMinimumHeight(35)
        publisher_label.setMinimumWidth(600)
        
        # 가격
        price_label = QLabel(f"💰 {int(book['price']):,}원")
        price_label.setFont(QFont("Arial", 14))
        price_label.setStyleSheet("color: #27ae60;")
        price_label.setMinimumHeight(35)
        price_label.setMinimumWidth(600)

        # 위치 - location 또는 location_id 처리
        location = book.get('location', book.get('location_id', 'Unknown'))
        location_label = QLabel(f"📍 {location}구역")
        location_label.setFont(QFont("Arial", 14))
        location_label.setStyleSheet("color: #e67e22;")
        location_label.setMinimumHeight(35)
        location_label.setMinimumWidth(600)
        
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
        top_layout.addWidget(info_widget, 1)
        top_layout.addWidget(stock_status_label, 0, alignment=Qt.AlignTop | Qt.AlignRight)
        
        layout.addWidget(cover_label)
        layout.addLayout(top_layout, 1)
        
        # 클릭 이벤트
        widget.mousePressEvent = lambda event: self.on_book_item_clicked(book)
        
        return widget

    def clear_book_list_widget(self):
        """책 리스트 위젯 완전 초기화"""
        layout = self.bookListWidget.layout()
        if layout:
            # 모든 위젯 제거
            while layout.count():
                child = layout.takeAt(0)
                if child.widget():
                    child.widget().deleteLater()
                elif child.layout():
                    self.clear_layout(child.layout())
    
    def clear_layout(self, layout):
        """레이아웃 내의 모든 위젯 재귀적으로 제거"""
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
            elif child.layout():
                self.clear_layout(child.layout())
    
    def scroll_to_top(self):
        """스크롤을 맨 위로 이동"""
        try:
            # 스크롤 영역 찾기
            scroll_area = self.bookListWidget.parent()
            while scroll_area and not isinstance(scroll_area, QScrollArea):
                scroll_area = scroll_area.parent()
            
            if scroll_area:
                scroll_area.verticalScrollBar().setValue(0)
                print("✅ 스크롤을 맨 위로 이동")
        except Exception as e:
            print(f"⚠️ 스크롤 이동 중 오류: {e}")
    
    def eventFilter(self, obj, event):
        """이벤트 필터 - 엔터키 스크롤 방지"""
        if obj == self.searchLineEdit and event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_Return or event.key() == Qt.Key_Enter:
                # 엔터키 이벤트를 처리하고 다른 위젯으로 전파하지 않음
                self.on_search_clicked()
                return True
        return super().eventFilter(obj, event)

    def on_book_item_clicked(self, book):
        """책 아이템 클릭"""
        print(f"📖 선택된 책: {book['title']} ({book.get('location', book.get('location_id', 'Unknown'))}구역)")
        
        # 책 상세 정보 팝업창 표시
        self.show_book_detail_popup(book)
    
    def on_escort_requested(self, escort_data):
        """에스코팅 요청 처리 - DB 테이블 구조에 맞게 완벽 매핑"""
        try:
            print(f"🚀 에스코팅 요청 수신: {escort_data}")
            
            # DB location 테이블 기준 위치 매핑
            book_location_id = escort_data.get('book_location', 'D5')
            
            # book.location_id → location.id 매핑
            location_mapping = {
                'D5': 'D5',  # computer 구역 (2.92, 0.98)
                'D7': 'D7',  # language 구역 (5.74, 1.18)  
                'C8': 'C8',  # novel 구역 (7.53, 2.36)
            }
            
            # TaskRequest.srv 파라미터 준비
            robot_id = escort_data.get('robot_id', '')  # escort_data에서 로봇 ID 가져오기 (기본값: 빈 문자열)
            call_location = "E9"  # 키오스크 위치 (kiosk_1: 8.98, -0.16)
            goal_location = location_mapping.get(book_location_id, "D5")  # 책 위치
            
            print(f"📍 TaskRequest 파라미터:")
            print(f"   robot_id: '{robot_id}' (task_manager에서 자동 선택)")
            print(f"   task_type: escort")
            print(f"   call_location: {call_location} (키오스크)")
            print(f"   goal_location: {goal_location} (책 위치)")
            
            # 클라이언트 지연 초기화
            if self.task_request_client is None:
                self.task_request_client = TaskRequestClient()
                self.task_request_client.task_request_completed.connect(self.on_task_request_response)
            
            # Main Server의 task_manager.py로 TaskRequest 서비스 호출
            success = self.task_request_client.request_escort_task(
                robot_id=robot_id,
                call_location=call_location, 
                goal_location=goal_location
            )
            
            if not success:
                QMessageBox.warning(self, "서비스 오류", 
                                  "TaskRequest 서비스를 호출할 수 없습니다.\n"
                                  "main_server가 실행 중인지 확인해주세요.")
            
        except Exception as e:
            print(f"❌ 에스코팅 요청 처리 중 오류: {e}")
            QMessageBox.warning(self, "요청 오류", 
                              f"에스코팅 요청 중 오류가 발생했습니다:\n{str(e)}")

    def on_task_request_response(self, success, message):
        """TaskRequest 서비스 응답 처리"""
        try:
            if success:
                print(f"✅ TaskRequest 성공: {message}")
                
                # 성공 메시지 팝업창 표시 (카운트다운 포함)
                self.show_success_popup_with_countdown(message)
                
            else:
                QMessageBox.warning(
                    self,
                    "❌ 에스코팅 요청 실패",
                    f"에스코팅 요청이 실패했습니다.\n\n"
                    f"🔍 실패 원인: {message}\n\n"
                    f"💡 해결방법:\n"
                    f"• main_server가 실행 중인지 확인\n"
                    f"• 로봇이 사용 가능한지 확인\n"
                    f"• 네트워크 연결 상태 확인\n\n"
                    f"다시 시도해보세요."
                )
                print(f"❌ TaskRequest 실패: {message}")
                
                # 실패 시 현재 팝업창의 버튼만 재활성화
                self.reset_popup_button()
                
        except Exception as e:
            print(f"❌ TaskRequest 응답 처리 중 오류: {e}")
    
    def show_success_popup_with_countdown(self, message):
        """성공 메시지 팝업창 표시 (카운트다운 포함)"""
        try:
            # 성공 메시지 팝업창 생성
            success_dialog = QDialog(self)
            success_dialog.setWindowTitle("🤖 에스코팅 요청 완료")
            success_dialog.setModal(True)
            success_dialog.setFixedSize(500, 300)
            
            # 레이아웃 설정
            layout = QVBoxLayout(success_dialog)
            
            # 성공 메시지 라벨
            message_label = QLabel(
                f"리보 에스코팅 요청이 성공적으로 접수되었습니다!\n\n"
                f"리보가 키오스크로 이동 후 \n"
                f"선택하신 책 위치로 안내할 예정입니다."
            )
            message_label.setAlignment(Qt.AlignCenter)
            message_label.setStyleSheet("""
                QLabel {
                    font-size: 14px;
                    padding: 20px;
                    line-height: 1.5;
                }
            """)
            layout.addWidget(message_label)
            
            # 카운트다운 라벨
            countdown_label = QLabel("5초 후 메인화면으로 돌아갑니다.")
            countdown_label.setAlignment(Qt.AlignCenter)
            countdown_label.setStyleSheet("""
                QLabel {
                    font-size: 16px;
                    font-weight: bold;
                    color: #27ae60;
                    padding: 10px;
                }
            """)
            layout.addWidget(countdown_label)
            
            # 확인 버튼
            ok_button = QPushButton("확인")
            ok_button.setStyleSheet("""
                QPushButton {
                    font-size: 14px;
                    padding: 10px 20px;
                    background-color: #3498db;
                    color: white;
                    border: none;
                    border-radius: 5px;
                }
                QPushButton:hover {
                    background-color: #2980b9;
                }
            """)
            ok_button.clicked.connect(success_dialog.accept)
            layout.addWidget(ok_button, alignment=Qt.AlignCenter)
            
            # 카운트다운 타이머 설정
            countdown_seconds = 5
            countdown_timer = QTimer()
            
            def update_countdown():
                nonlocal countdown_seconds
                countdown_seconds -= 1
                if countdown_seconds > 0:
                    countdown_label.setText(f"{countdown_seconds}초 후 메인화면으로 돌아갑니다.")
                else:
                    countdown_timer.stop()
                    # 모든 팝업창 닫기
                    self.close_all_popups()
                    # 카운트다운 완료 시 바로 메인화면으로 이동
                    self.on_home_clicked()
            
            countdown_timer.timeout.connect(update_countdown)
            countdown_timer.start(1000)  # 1초마다 업데이트
            
            # 팝업창 표시
            success_dialog.exec_()
            
        except Exception as e:
            print(f"❌ 성공 팝업창 표시 중 오류: {e}")
            # 오류 시 기본 방식으로 처리
            QMessageBox.information(
                self,
                "🤖 에스코팅 요청 완료",
                f"리보 에스코팅 요청이 성공적으로 접수되었습니다!\n\n"
                f"선택하신 책 위치로 안내할 예정입니다.\n\n"
                f"5초 후 메인화면으로 돌아갑니다."
            )
            # 5초 후 메인화면으로 이동
            QTimer.singleShot(5000, self.on_home_clicked)
    
    def reset_popup_button(self):
        """팝업창의 에스코팅 버튼 초기화"""
        try:
            # 현재 활성 팝업창 찾기
            for widget in QApplication.topLevelWidgets():
                if isinstance(widget, QDialog) and widget.isVisible():
                    # BookDetailPopup 인스턴스인지 확인
                    if hasattr(widget, 'reset_escort_button'):
                        widget.reset_escort_button()
                        break
        except Exception as e:
            print(f"❌ 팝업창 버튼 초기화 중 오류: {e}")
    
    def close_all_popups(self):
        """모든 팝업창 닫기"""
        try:
            # 모든 활성 팝업창 찾아서 닫기
            for widget in QApplication.topLevelWidgets():
                if isinstance(widget, QDialog) and widget.isVisible():
                    widget.close()
                    print(f"✅ 팝업창 닫기: {widget.windowTitle()}")
        except Exception as e:
            print(f"❌ 팝업창 닫기 중 오류: {e}")
    
    def show_search_input_request_popup(self):
        """검색어 입력 요청 팝업창 표시"""
        try:
            QMessageBox.information(
                self,
                "🔍 검색어 입력",
                "검색어를 입력한 후 검색 버튼을 눌러주세요."
            )
        except Exception as e:
            print(f"❌ 검색어 입력 요청 팝업창 표시 중 오류: {e}")
    
    def update_search_button_text(self):
        """검색어 변경 시 버튼 텍스트 업데이트"""
        try:
            search_text = self.searchLineEdit.text().strip()
            if not search_text:
                self.searchButton.setText("검색")
            else:
                self.searchButton.setText("검색")
        except Exception as e:
            print(f"❌ 검색 버튼 텍스트 업데이트 중 오류: {e}")
    
    def show_book_detail_popup(self, book):
        """책 상세정보 팝업창 표시"""
        try:
            popup = BookDetailPopup(book, self)
            popup.escort_requested.connect(self.on_escort_requested)
            popup.exec_()
        except Exception as e:
            print(f"❌ 팝업창 표시 중 오류: {e}")
            QMessageBox.warning(self, "오류", f"팝업창을 표시할 수 없습니다: {str(e)}")
    
    def on_home_clicked(self):
        """홈 버튼 클릭 - 리소스 정리 포함"""
        print("🏠 홈으로 이동 요청")
        
        # 위젯 초기화
        self.reset_widget()
        
        # ROS2 클라이언트 안전 정리
        self.cleanup_ros_clients()
        
        self.home_requested.emit()
    
    def cleanup_ros_clients(self):
        """ROS2 클라이언트 안전 정리"""
        try:
            if self.search_client:
                if self.search_client.isRunning():
                    self.search_client.quit()
                    self.search_client.wait(3000)  # 3초 대기
                self.search_client.cleanup()
                self.search_client = None
                print("✅ search_client 정리 완료")
        except Exception as e:
            print(f"⚠️ search_client 정리 중 오류: {e}")
        
        try:
            if self.task_request_client:
                if self.task_request_client.isRunning():
                    self.task_request_client.quit()
                    self.task_request_client.wait(3000)  # 3초 대기
                self.task_request_client.cleanup()
                self.task_request_client = None
                print("✅ task_request_client 정리 완료")
        except Exception as e:
            print(f"⚠️ task_request_client 정리 중 오류: {e}")
    
    def on_order_clicked(self):
        """주문 문의 버튼 클릭"""
        print("📞 주문 문의")
        QMessageBox.information(
            self, 
            "주문 문의", 
            "📞 주문 문의가 접수되었습니다!\n\n"
            "관리자가 곧 연락드리겠습니다.\n"
            "감사합니다."
        )

    def closeEvent(self, event):
        """윈도우 종료 시 리소스 정리"""
        self.cleanup_ros_clients()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = BookSearchWidget()
    widget.show()
    sys.exit(app.exec_())