#!/home/robolee/venv/jazzy/bin/python3

# 🔧 Qt 플러그인 충돌 해결을 위한 환경변수 설정 (OpenCV vs PyQt5)
import os
os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)  # OpenCV Qt 경로 제거
os.environ['QT_QPA_PLATFORM'] = 'xcb'  # 명시적으로 xcb 사용
os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':0')  # PyQt와 OpenCV 충돌 방지

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QDialog, QVBoxLayout, QHBoxLayout, 
                             QLabel, QLineEdit, QPushButton, QListWidget, 
                             QMessageBox, QListWidgetItem, QSpinBox, QWidget)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QUrl, QTimer
from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import uic
import pymysql
import requests
import json
import cv2
import numpy as np
from imutils.video import VideoStream
from pyzbar import pyzbar
import time
# main_server 패키지의 모듈들을 import
from main_server.database.aladin_api_client import AladinAPIClient
from main_server.utils.book_data_parser import BookDataParser
from main_server.database.db_manager import DatabaseManager

class CameraWindow(QWidget):
    """카메라 화면을 표시하는 팝업 창"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("바코드 스캔")
        self.setGeometry(100, 100, 640, 480)
        
        # 레이아웃 설정
        layout = QVBoxLayout()
        
        # 카메라 화면을 표시할 라벨
        self.camera_label = QLabel()
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(600, 400)
        self.camera_label.setStyleSheet("border: 2px solid #ccc; background-color: black;")
        layout.addWidget(self.camera_label)
        
        # 상태 메시지
        self.status_label = QLabel("카메라를 초기화하는 중...")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
        # 중지 버튼
        self.stop_button = QPushButton("스캔 중지")
        self.stop_button.clicked.connect(self.close)
        layout.addWidget(self.stop_button)
        
        self.setLayout(layout)
    
    def update_frame(self, frame):
        """카메라 프레임 업데이트"""
        # OpenCV BGR을 RGB로 변환
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # numpy 배열을 QImage로 변환
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        # QPixmap으로 변환하여 표시
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(600, 400, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.camera_label.setPixmap(scaled_pixmap)
    
    def update_status(self, message):
        """상태 메시지 업데이트"""
        self.status_label.setText(message)

class BarcodeScannerThread(QThread):
    """바코드 스캔 스레드"""
    barcode_detected = pyqtSignal(str)
    frame_ready = pyqtSignal(np.ndarray)
    status_update = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.running = False
        self.vs = None
        self.found_barcodes = set()
    
    def run(self):
        """스캔 실행 - Qt 충돌 방지"""
        try:
            # 🔧 Qt 충돌 방지를 위한 환경변수 설정
            os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
            
            self.vs = VideoStream(src=0).start()
            time.sleep(2.0)
            self.running = True
            self.status_update.emit("카메라가 준비되었습니다. 바코드를 비춰주세요.")
            
            while self.running:
                frame = self.vs.read()
                if frame is None:
                    continue
                    
                frame = cv2.resize(frame, (640, 480))
                
                # 바코드 디코딩
                barcodes = pyzbar.decode(frame)
                
                for barcode in barcodes:
                    (x, y, w, h) = barcode.rect
                    # 바코드 영역에 사각형 그리기
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    
                    barcode_data = barcode.data.decode("utf-8")
                    text = str(barcode_data)
                    
                    # 바코드 번호 표시
                    cv2.putText(frame, text, (x, y - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    # 새로운 바코드인 경우 신호 발생
                    if barcode_data not in self.found_barcodes:
                        self.found_barcodes.add(barcode_data)
                        self.barcode_detected.emit(barcode_data)
                        self.status_update.emit(f"바코드 감지: {barcode_data}")
                
                # 프레임을 메인 스레드로 전송
                self.frame_ready.emit(frame)
                
                # 짧은 지연
                time.sleep(0.1)
                
        except Exception as e:
            print(f"바코드 스캔 오류: {e}")
            self.status_update.emit(f"오류 발생: {str(e)}")
        finally:
            if self.vs:
                self.vs.stop()
    
    def stop(self):
        """스캔 중지"""
        self.running = False
        if self.vs:
            self.vs.stop()

class StockGUI(Node):
    def __init__(self):
        super().__init__('stock_gui')
        
        # UI 초기화
        self.app = QApplication(sys.argv)
        self.dialog = QDialog()
        # UI 파일 경로 수정 (설치된 패키지 경로 사용)
        ui_file_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'share', 'stock', 'ui', 'stock_ui.ui')
        uic.loadUi(ui_file_path, self.dialog)
        
        # 컴포넌트 초기화
        self.aladin_client = AladinAPIClient("ttbleeshun08062356001")
        self.parser = BookDataParser()
        self.db_manager = DatabaseManager()
        
        # 현재 선택된 책 정보
        self.current_book_info = None
        
        # 바코드 스캔 관련
        self.barcode_scanner = None
        self.scanning = False
        self.camera_window = None
        
        # UI 연결
        self.setup_ui_connections()
        
        # 위치 매핑 업데이트 (요청사항에 따라)
        self.parser.location_mapping = {
            '컴퓨터': 'D5',
            '언어': 'D7', 
            '소설': 'C8'
        }
        
        self.get_logger().info('📚 Stock GUI 시작됨')
    
    def setup_ui_connections(self):
        """UI 이벤트 연결"""
        # 검색 버튼
        self.dialog.searchButton.clicked.connect(self.search_book)
        
        # 입고 버튼
        self.dialog.stockInButton.clicked.connect(self.stock_in_book)
        
        # ISBN 입력 필드에서 Enter 키
        self.dialog.isbnInput.returnPressed.connect(self.search_book)
        
        # 리스트 선택 이벤트
        self.dialog.bookList.itemClicked.connect(self.on_book_selected)
        
        # 바코드 스캔 버튼 (UI에 추가 필요)
        if hasattr(self.dialog, 'barcodeScanButton'):
            self.dialog.barcodeScanButton.clicked.connect(self.toggle_barcode_scan)
    
    def toggle_barcode_scan(self):
        """바코드 스캔 토글"""
        if not self.scanning:
            self.start_barcode_scan()
        else:
            self.stop_barcode_scan()
    
    def start_barcode_scan(self):
        """바코드 스캔 시작"""
        if self.barcode_scanner is None:
            self.barcode_scanner = BarcodeScannerThread()
            self.barcode_scanner.barcode_detected.connect(self.on_barcode_detected)
            self.barcode_scanner.frame_ready.connect(self.update_camera_frame)
            self.barcode_scanner.status_update.connect(self.update_camera_status)
        
        # 카메라 창 생성 및 표시
        self.camera_window = CameraWindow()
        self.camera_window.show()
        
        self.barcode_scanner.start()
        self.scanning = True
        
        if hasattr(self.dialog, 'barcodeScanButton'):
            self.dialog.barcodeScanButton.setText("스캔 중지")
        
        self.update_status("📷 바코드 스캔 중... (바코드를 카메라에 비춰주세요)")
    
    def stop_barcode_scan(self):
        """바코드 스캔 중지"""
        if self.barcode_scanner:
            self.barcode_scanner.stop()
            self.barcode_scanner.wait()
            self.barcode_scanner = None
        
        if self.camera_window:
            self.camera_window.close()
            self.camera_window = None
        
        self.scanning = False
        
        if hasattr(self.dialog, 'barcodeScanButton'):
            self.dialog.barcodeScanButton.setText("바코드 스캔")
        
        self.update_status("📷 바코드 스캔이 중지되었습니다.")
    
    def on_barcode_detected(self, barcode_data):
        """바코드 감지 처리"""
        self.update_status(f"📷 바코드 감지: {barcode_data}")
        
        # ISBN 입력 필드에 바코드 데이터 설정
        self.dialog.isbnInput.setText(barcode_data)
        
        # 자동으로 검색 실행
        self.search_book()
        
        # 스캔 중지
        self.stop_barcode_scan()
    
    def update_camera_frame(self, frame):
        """카메라 프레임 업데이트"""
        if self.camera_window:
            self.camera_window.update_frame(frame)
    
    def update_camera_status(self, message):
        """카메라 창 상태 업데이트"""
        if self.camera_window:
            self.camera_window.update_status(message)
    
    def search_book(self):
        """ISBN으로 책 검색"""
        isbn = self.dialog.isbnInput.text().strip()
        
        if not isbn:
            QMessageBox.warning(self.dialog, "경고", "ISBN을 입력해주세요.")
            return
        
        self.update_status("🔍 도서 검색 중...")
        self.dialog.searchButton.setEnabled(False)
        
        # 검색 스레드 시작
        self.search_thread = SearchThread(self.aladin_client, isbn)
        self.search_thread.finished.connect(self.on_search_finished)
        self.search_thread.start()
    
    def on_search_finished(self, results):
        """검색 완료 처리"""
        self.dialog.searchButton.setEnabled(True)
        self.dialog.bookList.clear()
        
        if not results:
            self.update_status("❌ 검색 결과가 없습니다.")
            self.dialog.bookInfoLabel.setText("검색 결과가 없습니다.")
            # 기본 이미지 표시
            self.dialog.bookCoverLabel.setText("표지 이미지")
            self.dialog.bookCoverLabel.setPixmap(QPixmap())
            # 입고 관련 UI 비활성화
            self.dialog.stockInButton.setEnabled(False)
            self.dialog.quantitySpinBox.setEnabled(False)
            return
        
        # 검색 결과를 리스트에 표시
        for book in results:
            item = QListWidgetItem(f"{book['title']} - {book['author']}")
            item.setData(Qt.UserRole, book)
            self.dialog.bookList.addItem(item)
        
        self.update_status(f"✅ {len(results)}권의 도서를 찾았습니다.")
        self.dialog.bookInfoLabel.setText("리스트에서 도서를 선택하세요.")
        # 기본 이미지 표시
        self.dialog.bookCoverLabel.setText("표지 이미지")
        self.dialog.bookCoverLabel.setPixmap(QPixmap())
        # 입고 관련 UI 비활성화
        self.dialog.stockInButton.setEnabled(False)
        self.dialog.quantitySpinBox.setEnabled(False)
    
    def on_book_selected(self, item):
        """도서 선택 처리"""
        book_info = item.data(Qt.UserRole)
        self.current_book_info = book_info
        
        # 도서 정보 표시
        info_text = f"""
        도서 정보:
        제목: {book_info.get('title', 'N/A')}
        저자: {book_info.get('author', 'N/A')}
        출판사: {book_info.get('publisher', 'N/A')}
        가격: {book_info.get('priceSales', 0):,}원
        ISBN: {book_info.get('isbn13', book_info.get('isbn', 'N/A'))}
        """
        
        self.dialog.bookInfoLabel.setText(info_text)
        
        # 도서 표지 이미지 로드
        cover_url = book_info.get('cover', '')
        self.load_book_cover(cover_url)
        
        # 입고 관련 UI 활성화
        self.dialog.stockInButton.setEnabled(True)
        self.dialog.quantitySpinBox.setEnabled(True)
        self.update_status("✅ 도서가 선택되었습니다. 수량을 설정하고 입고 버튼을 클릭하세요.")
    
    def stock_in_book(self):
        """도서 입고 처리"""
        if not self.current_book_info:
            QMessageBox.warning(self.dialog, "경고", "입고할 도서를 선택해주세요.")
            return
        
        # 수량 가져오기
        quantity = self.dialog.quantitySpinBox.value()
        
        # 사용자 확인
        reply = QMessageBox.question(
            self.dialog, 
            "입고 확인", 
            f"'{self.current_book_info.get('title', 'N/A')}' 도서를 {quantity}권 입고하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        self.update_status(f"💾 도서 {quantity}권 입고 중...")
        self.dialog.stockInButton.setEnabled(False)
        
        # 입고 스레드 시작 (수량 정보 포함)
        self.stock_thread = StockInThread(self.parser, self.db_manager, self.current_book_info, quantity)
        self.stock_thread.finished.connect(self.on_stock_in_finished)
        self.stock_thread.start()
    
    def on_stock_in_finished(self, success, message):
        """입고 완료 처리"""
        self.dialog.stockInButton.setEnabled(True)
        
        if success:
            self.update_status("✅ 도서 입고 완료!")
            QMessageBox.information(self.dialog, "성공", message)
            
            # 입력 필드 초기화
            self.dialog.isbnInput.clear()
            self.dialog.bookList.clear()
            self.dialog.bookInfoLabel.setText("도서 정보가 여기에 표시됩니다.")
            self.dialog.bookCoverLabel.setText("표지 이미지")
            self.dialog.bookCoverLabel.setPixmap(QPixmap())
            # 입고 관련 UI 초기화
            self.dialog.stockInButton.setEnabled(False)
            self.dialog.quantitySpinBox.setEnabled(False)
            self.dialog.quantitySpinBox.setValue(1)
            self.current_book_info = None
        else:
            self.update_status("❌ 도서 입고 실패")
            QMessageBox.critical(self.dialog, "오류", message)
    
    def load_book_cover(self, cover_url: str):
        """도서 표지 이미지 로드"""
        if not cover_url:
            # 기본 이미지 표시
            self.dialog.bookCoverLabel.setText("표지 없음")
            return
        
        try:
            # 이미지 다운로드
            response = requests.get(cover_url, timeout=10)
            if response.status_code == 200:
                # QImage로 변환
                image = QImage()
                image.loadFromData(response.content)
                
                # QPixmap으로 변환하여 표시
                pixmap = QPixmap.fromImage(image)
                
                # 크기 조정 (150x200 비율 유지)
                scaled_pixmap = pixmap.scaled(150, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                
                self.dialog.bookCoverLabel.setPixmap(scaled_pixmap)
                self.dialog.bookCoverLabel.setAlignment(Qt.AlignCenter)
            else:
                self.dialog.bookCoverLabel.setText("이미지 로드 실패")
        except Exception as e:
            print(f"이미지 로드 오류: {e}")
            self.dialog.bookCoverLabel.setText("이미지 로드 실패")
    
    def update_status(self, message):
        """상태 메시지 업데이트"""
        self.dialog.statusLabel.setText(f"상태: {message}")
        self.get_logger().info(message)
    
    def run(self):
        """GUI 실행"""
        self.dialog.show()
        return self.app.exec_()

class SearchThread(QThread):
    """검색 스레드"""
    finished = pyqtSignal(list)
    
    def __init__(self, aladin_client, isbn):
        super().__init__()
        self.aladin_client = aladin_client
        self.isbn = isbn
    
    def run(self):
        try:
            # ISBN으로 도서 검색
            book_info = self.aladin_client.search_specific_book_by_isbn(self.isbn)
            
            if book_info:
                self.finished.emit([book_info])
            else:
                self.finished.emit([])
        except Exception as e:
            print(f"검색 오류: {e}")
            self.finished.emit([])

class StockInThread(QThread):
    """입고 스레드"""
    finished = pyqtSignal(bool, str)
    
    def __init__(self, parser, db_manager, book_info, quantity=1):
        super().__init__()
        self.parser = parser
        self.db_manager = db_manager
        self.book_info = book_info
        self.quantity = quantity
    
    def run(self):
        try:
            # 도서 정보 파싱
            parsed_book = self.parser.parse_aladin_response(self.book_info)
            
            if not parsed_book:
                self.finished.emit(False, "도서 정보 파싱에 실패했습니다.")
                return
            
            # 수량 설정
            parsed_book['stock_quantity'] = self.quantity
            
            # 데이터 유효성 검증
            if not self.parser.validate_parsed_data(parsed_book):
                self.finished.emit(False, "파싱된 데이터가 유효하지 않습니다.")
                return
            
            # 데이터베이스에 등록
            success = self.db_manager.register_book(parsed_book)
            
            if success:
                message = f"'{parsed_book['title']}' 도서가 {self.quantity}권 성공적으로 입고되었습니다."
                self.finished.emit(True, message)
            else:
                self.finished.emit(False, "데이터베이스 등록에 실패했습니다.")
                
        except Exception as e:
            self.finished.emit(False, f"입고 중 오류 발생: {str(e)}")

def main(args=None):
    """메인 함수 - Qt 충돌 해결"""
    print("🚀 Stock GUI 시작... (Qt 플러그인 충돌 해결)")
    
    # 🔧 Qt 플러그인 충돌 해결을 위한 환경변수 설정
    os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)  # OpenCV Qt 경로 제거
    os.environ['QT_QPA_PLATFORM'] = 'xcb'  # 명시적으로 xcb 사용
    os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':0')  # PyQt와 OpenCV 충돌 방지
    
    # 🔧 ROS2 환경 확인
    ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
    ros_version = os.environ.get('ROS_VERSION', 'unknown')
    print(f"🔧 ROS2 환경: {ros_distro} {ros_version}")
    
    # 🔧 Python 경로 설정
    current_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))  # ros_ws/src
    if src_dir not in sys.path:
        sys.path.insert(0, src_dir)
    
    try:
        # ROS2 초기화 (PyQt 앱 생성 전에!)
        rclpy.init(args=args)
        print("✅ ROS2 초기화 완료")
        
        stock_gui = StockGUI()
        print("✅ Stock GUI 생성 완료")
        
        exit_code = stock_gui.run()
        return exit_code
    except KeyboardInterrupt:
        print("⚠️ 사용자에 의해 중단됨")
        pass
    except Exception as e:
        print(f"❌ GUI 실행 중 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'stock_gui' in locals():
            stock_gui.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("🔧 리소스 정리 완료")

if __name__ == '__main__':
    main() 