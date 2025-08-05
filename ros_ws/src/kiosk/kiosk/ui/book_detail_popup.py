#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import math

class BookDetailPopup(QDialog):
    """책 상세 정보 팝업창"""
    
    # 에스코팅 요청 시그널
    escort_requested = pyqtSignal(dict)
    
    def __init__(self, book_info, parent=None):
        super().__init__(parent)
        
        self.book_info = book_info
        self.map_image = None
        self.book_location = book_info.get('location', 'D5')
        self.is_resizing = False  # 리사이징 중복 방지 플래그
        self.resize_timer = None  # 리사이즈 타이머
        
        # 위치별 좌표 매핑 (waypoint.png 이미지 기준 - 778x416 크기)
        self.location_coordinates = {
            'D5': (308, 526),   
            'D7': (649, 527),  
            'C8': (999, 507),
        }
        
        self.init_ui()
        self.setup_connections()
        
        # 지도 로드 및 윈도우 크기 조정을 위한 타이머 설정
        QTimer.singleShot(100, self.load_map_and_adjust_size)
        
        self.display_book_info()
    
    def init_ui(self):
        """UI 초기화"""
        # UI 파일 로드 - ROS2 패키지 설치 경로에서 찾기
        try:
            # 먼저 현재 디렉토리 기준으로 시도
            ui_file = os.path.join(os.path.dirname(__file__), '..', 'ui_files', 'book_detail_popup.ui')
            if not os.path.exists(ui_file):
                # ROS2 패키지 설치 경로에서 찾기
                import ament_index_python
                ui_file = os.path.join(ament_index_python.get_package_share_directory('kiosk'), 'ui_files', 'book_detail_popup.ui')
            uic.loadUi(ui_file, self)
        except Exception as e:
            print(f"UI 파일 로드 실패: {e}")
            print(f"시도한 경로: {ui_file}")
            raise
        
        # 윈도우 설정
        self.setWindowTitle("도서 상세 정보")
        self.setModal(True)
        
        print("✅ BookDetailPopup UI 로드 완료")
    
    def setup_connections(self):
        """시그널-슬롯 연결"""
        self.escortRequestButton.clicked.connect(self.on_escort_request_clicked)
        self.closeButton.clicked.connect(self.close)
        
        print("✅ BookDetailPopup 시그널-슬롯 연결 완료")
    
    def load_map_and_adjust_size(self):
        """지도 이미지 로드 및 윈도우 크기 조정"""
        try:
            # 지도 이미지 파일 경로 (waypoint.png 사용)
            map_file = '/home/robolee/dev_ws/ros-repo-1/ros_ws/src/waypoint3.png'
            
            if not os.path.exists(map_file):
                print(f"❌ 지도 이미지 파일을 찾을 수 없습니다: {map_file}")
                self.mapLabel.setText("지도 이미지를 찾을 수 없습니다.")
                return
            
            # 지도 이미지 로드
            self.map_image = QPixmap(map_file)
            if self.map_image.isNull():
                print("❌ 지도 이미지 로드 실패")
                self.mapLabel.setText("지도 이미지 로드 실패")
                return
            
            # 지도에 빨간색 원 그리기
            self.draw_location_marker()
            
            # 윈도우 크기 조정
            self.adjust_window_size()
            
            print(f"✅ 지도 이미지 로드 완료: {map_file}")
            
        except Exception as e:
            print(f"❌ 지도 로드 중 오류: {e}")
            self.mapLabel.setText(f"지도 로드 오류: {str(e)}")
    
    def adjust_window_size(self):
        """윈도우 크기 조정"""
        try:
            # 지도 이미지 크기 가져오기
            if self.map_image:
                map_width = self.map_image.width()
                map_height = self.map_image.height()
                
                # 지도 비율 계산
                map_ratio = map_width / map_height
                
                # 지도 라벨 크기 설정 (지도 크기 유지하면서 조정)
                map_label_width = max(600, int(400 * map_ratio))  # 지도 크기 확대
                map_label_height = max(700, int(500 / map_ratio))  # 지도 높이 확대
                
                # 지도 라벨 크기 조정
                self.mapLabel.setMinimumSize(map_label_width, map_label_height)
                self.mapLabel.setMaximumSize(map_label_width, map_label_height)
                
                # 전체 윈도우 크기 조정
                total_width = map_label_width + 100  # 지도 + 정보 패널 (패널 크기 최소화)
                total_height = max(map_label_height + 120, 900)  # 전체 창 높이 확대
                
                self.resize(total_width, total_height)
                
                # 지도 다시 그리기
                self.draw_location_marker()
                
                print(f"✅ 윈도우 크기 조정 완료: {total_width}x{total_height}")
                
        except Exception as e:
            print(f"❌ 윈도우 크기 조정 중 오류: {e}")
    
    def load_map_with_location(self):
        """지도 이미지 로드 및 위치 표시 (기존 메서드 - 호환성 유지)"""
        self.load_map_and_adjust_size()
    
    def draw_location_marker(self):
        """지도에 위치 마커 그리기"""
        if self.map_image is None:
            return
        
        # 지도 라벨 크기 가져오기 (패딩 고려)
        label_width = self.mapLabel.width()
        label_height = self.mapLabel.height()
        
        # 패딩을 고려한 실제 사용 가능한 크기 계산
        padding = 20  # 패딩 값
        available_width = label_width - (padding * 2)
        available_height = label_height - (padding * 2)
        
        # 지도 이미지를 사용 가능한 크기에 맞게 조정
        scaled_map = self.map_image.scaled(
            available_width, 
            available_height, 
            Qt.KeepAspectRatio, 
            Qt.SmoothTransformation
        )
        
        # 원본 이미지와 조정된 이미지의 비율 계산
        original_width = self.map_image.width()
        original_height = self.map_image.height()
        scaled_width = scaled_map.width()
        scaled_height = scaled_map.height()
        
        scale_x = scaled_width / original_width
        scale_y = scaled_height / original_height
        
        # 빨간색 원 그리기
        painter = QPainter(scaled_map)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 빨간색 원 설정
        pen = QPen(QColor(255, 0, 0), 4)  # 빨간색, 두께 4
        painter.setPen(pen)
        
        brush = QBrush(QColor(255, 0, 0, 100))  # 반투명 빨간색
        painter.setBrush(brush)
        
        # 위치 좌표 가져오기 및 비례 조정
        location = self.book_location
        x, y = 0, 0  # 기본값 초기화
        
        if location in self.location_coordinates:
            original_x, original_y = self.location_coordinates[location]
            
            # 좌표를 조정된 크기에 맞게 변환
            x = int(original_x * scale_x)
            y = int(original_y * scale_y)
            
            # 원 그리기 (크기도 비례 조정)
            circle_size = max(10, int(20 * min(scale_x, scale_y)))
            painter.drawEllipse(x - circle_size//2, y - circle_size//2, circle_size, circle_size)
            
            # 위치 텍스트 표시
            painter.setPen(QPen(QColor(255, 0, 0), 2))
            font_size = max(8, int(12 * min(scale_x, scale_y)))
            painter.setFont(QFont("Arial", font_size, QFont.Bold))
            painter.drawText(x + circle_size//2 + 5, y + 5, f"{location}구역")
        else:
            print(f"⚠️ 위치 '{location}'에 대한 좌표가 정의되지 않았습니다.")
        
        painter.end()
        
        # 지도 라벨에 가운데 정렬로 표시
        self.mapLabel.setAlignment(Qt.AlignCenter)
        self.mapLabel.setPixmap(scaled_map)
        
        # 위치 정보 업데이트
        self.locationInfoLabel.setText(f"위치: {location}구역")
        
        print(f"✅ 위치 마커 그리기 완료: {location}구역 (조정된 좌표: {x}, {y})")
    
    def display_book_info(self):
        """책 정보 표시"""
        try:
            # 제목
            title = self.book_info.get('title', '제목 없음')
            self.bookTitleLabel.setText(title)
            
            # 책 표지 이미지 로드 및 표시
            self.load_book_cover()
            
            # 저자
            author = self.book_info.get('author', '저자 정보 없음')
            self.bookAuthorLabel.setText(f"저자: {author}")
            
            # 출판사
            publisher = self.book_info.get('publisher', '출판사 정보 없음')
            self.bookPublisherLabel.setText(f"출판사: {publisher}")
            
            # 카테고리
            category = self.book_info.get('category_name', '카테고리 정보 없음')
            self.bookCategoryLabel.setText(f"카테고리: {category}")
            
            # 가격
            price = self.book_info.get('price', 0)
            self.bookPriceLabel.setText(f"가격: {int(price):,}원")
            
            # 재고
            stock = self.book_info.get('stock_quantity', 0)
            stock_text = f"재고: {stock}권"
            if stock > 0:
                self.bookStockLabel.setStyleSheet("""
                    QLabel {
                        color: #27ae60;
                        padding: 5px;
                        font-weight: bold;
                    }
                """)
            else:
                self.bookStockLabel.setStyleSheet("""
                    QLabel {
                        color: #e74c3c;
                        padding: 5px;
                        font-weight: bold;
                    }
                """)
            self.bookStockLabel.setText(stock_text)
            
            # ISBN
            isbn = self.book_info.get('isbn', 'ISBN 정보 없음')
            self.bookIsbnLabel.setText(f"ISBN: {isbn}")
            
            print("✅ 책 정보 표시 완료")
            
        except Exception as e:
            print(f"❌ 책 정보 표시 중 오류: {e}")
    
    def load_book_cover(self):
        """책 표지 이미지 로드 및 표시"""
        try:
            cover_url = self.book_info.get('cover_image_url', '')
            
            if not cover_url:
                # 표지 URL이 없으면 기본 아이콘 표시
                self.bookCoverLabel.setText("📖")
                self.bookCoverLabel.setStyleSheet("""
                    QLabel {
                        border: 2px solid #bdc3c7;
                        border-radius: 8px;
                        background-color: #f8f9fa;
                        padding: 10px;
                        font-size: 48px;
                        color: #7f8c8d;
                    }
                """)
                return
            
            # URL에서 이미지 로드
            import requests
            response = requests.get(cover_url, timeout=10)
            response.raise_for_status()
            
            # QPixmap으로 이미지 로드
            pixmap = QPixmap()
            pixmap.loadFromData(response.content)
            
            if pixmap.isNull():
                print(f"❌ 이미지 로드 실패: {cover_url}")
                self.bookCoverLabel.setText("📖")
                self.bookCoverLabel.setStyleSheet("""
                    QLabel {
                        border: 2px solid #bdc3c7;
                        border-radius: 8px;
                        background-color: #f8f9fa;
                        padding: 10px;
                        font-size: 48px;
                        color: #7f8c8d;
                    }
                """)
                return
            
            # 이미지 크기 조정 (최대 200x280)
            scaled_pixmap = pixmap.scaled(200, 280, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            # 표지 이미지 표시
            self.bookCoverLabel.setPixmap(scaled_pixmap)
            self.bookCoverLabel.setStyleSheet("""
                QLabel {
                    border: 2px solid #bdc3c7;
                    border-radius: 8px;
                    background-color: white;
                    padding: 5px;
                }
            """)
            
            print(f"✅ 책 표지 이미지 로드 완료: {cover_url}")
            
        except Exception as e:
            print(f"❌ 책 표지 이미지 로드 중 오류: {e}")
            # 오류 시 기본 아이콘 표시
            self.bookCoverLabel.setText("📖")
            self.bookCoverLabel.setStyleSheet("""
                QLabel {
                    border: 2px solid #bdc3c7;
                    border-radius: 8px;
                    background-color: #f8f9fa;
                    padding: 5px;
                    font-size: 48px;
                    color: #7f8c8d;
                }
            """)
    
    def on_escort_request_clicked(self):
        """에스코팅 요청 버튼 클릭"""
        try:
            print(f"🚀 에스코팅 요청: {self.book_info['title']}")
            
            # 에스코팅 요청 데이터 준비 (로봇 ID는 task_manager에서 자동 선택)
            escort_data = {
                'robot_id': '',  # 빈 문자열로 전송하여 task_manager에서 자동 선택
                'book_title': self.book_info['title'],
                'book_location': self.book_location,
                'book_info': self.book_info
            }
            
            # 시그널 발생
            self.escort_requested.emit(escort_data)
            
            # 버튼 비활성화
            self.escortRequestButton.setEnabled(False)
            self.escortRequestButton.setText("요청 중...")
            
            print("✅ 에스코팅 요청 시그널 발생")
            
        except Exception as e:
            print(f"❌ 에스코팅 요청 중 오류: {e}")
            QMessageBox.warning(self, "오류", f"에스코팅 요청 중 오류가 발생했습니다: {str(e)}")
    
    def reset_escort_button(self):
        """에스코팅 요청 버튼 초기화"""
        self.escortRequestButton.setEnabled(True)
        self.escortRequestButton.setText("에스코팅 요청")
    
    def resizeEvent(self, event):
        """윈도우 크기 변경 시 지도 다시 그리기 (디바운싱 적용)"""
        super().resizeEvent(event)
        
        # 기존 타이머가 있으면 취소
        if self.resize_timer:
            self.resize_timer.stop()
        
        # 새로운 타이머 시작 (300ms 후에 실행)
        self.resize_timer = QTimer()
        self.resize_timer.setSingleShot(True)
        self.resize_timer.timeout.connect(self._delayed_redraw)
        self.resize_timer.start(300)
    
    def _delayed_redraw(self):
        """지연된 지도 다시 그리기"""
        if self.map_image and not self.is_resizing:
            self.is_resizing = True
            self.draw_location_marker()
            self.is_resizing = False

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # 테스트용 책 정보
    test_book = {
        'title': '밑바닥부터 시작하는 딥러닝 1',
        'author': '사이토 고키',
        'publisher': '한빛미디어',
        'category_name': '컴퓨터',
        'price': 32000,
        'stock_quantity': 2,
        'isbn': '9788968481475',
        'location': 'D'
    }
    
    popup = BookDetailPopup(test_book)
    popup.show()
    
    sys.exit(app.exec_()) 