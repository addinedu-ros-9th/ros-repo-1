#!/usr/bin/env python3

import sys
import os
import time
import cv2
import numpy as np
from typing import Dict, List, Optional
from threading import Thread
import requests

# PyQt5 imports
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

# 바코드 스캔을 위한 라이브러리
from pyzbar import pyzbar
from imutils.video import VideoStream
import imutils

# ROS2 imports 
import rclpy
from rclpy.node import Node

# 프로젝트 imports (결제 시스템용)
from main_server.database.db_manager import DatabaseManager

class CameraWindow(QWidget):
    """카메라 화면을 표시하는 팝업 창 - Stock GUI와 동일한 방식"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("📱 바코드 스캔 - Payment")
        self.setGeometry(100, 100, 640, 480)
        
        # 레이아웃 설정
        layout = QVBoxLayout()
        
        # 카메라 화면을 표시할 라벨
        self.camera_label = QLabel()
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(600, 400)
        self.camera_label.setStyleSheet("""
            QLabel {
                border: 2px solid #71866a; 
                background-color: black;
                border-radius: 10px;
            }
        """)
        layout.addWidget(self.camera_label)
        
        # 상태 메시지
        self.status_label = QLabel("카메라를 초기화하는 중...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("""
            QLabel {
                font-size: 14px;
                color: #2c3e50;
                padding: 10px;
                font-weight: bold;
            }
        """)
        layout.addWidget(self.status_label)
        
        # 중지 버튼
        self.stop_button = QPushButton("❌ 스캔 중지")
        self.stop_button.setStyleSheet("""
            QPushButton {
                background: #dc3545;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 10px 20px;
                font-weight: bold;
                font-size: 14px;
            }
            QPushButton:hover {
                background: #c82333;
            }
        """)
        self.stop_button.clicked.connect(self.close)
        layout.addWidget(self.stop_button)
        
        self.setLayout(layout)
        
        # 창 스타일 설정
        self.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #f8f9fa, stop:1 #e9ecef);
            }
        """)
    
    def update_frame(self, frame):
        """카메라 프레임 업데이트 - Stock GUI와 동일한 방식"""
        try:
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
        except Exception as e:
            print(f"❌ 프레임 업데이트 오류: {e}")
    
    def update_status(self, message):
        """상태 메시지 업데이트"""
        self.status_label.setText(message)

class BarcodeScannerThread(QThread):
    """바코드 스캔 스레드 - Stock GUI와 동일한 방식으로 수정"""
    barcode_detected = pyqtSignal(str)
    frame_ready = pyqtSignal(np.ndarray)
    status_update = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.running = False
        self.vs = None
        self.found_barcodes = set()
        self.last_scan_time = 0
        self.scan_cooldown = 3.0  # 3초 쿨다운
    
    def run(self):
        """스캔 실행 - VideoStream 사용 (Stock GUI 방식)"""
        try:
            # 🔧 Qt 충돌 방지를 위한 환경변수 설정
            import os
            os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
            
            # VideoStream 초기화 (Stock GUI와 동일)
            self.vs = VideoStream(src=0).start()
            time.sleep(2.0)
            self.running = True
            self.status_update.emit("카메라가 준비되었습니다. 📱 바코드를 비춰주세요!")
            
            while self.running:
                frame = self.vs.read()
                if frame is None:
                    continue
                    
                frame = cv2.resize(frame, (640, 480))
                
                # 바코드 디코딩
                barcodes = pyzbar.decode(frame)
                
                for barcode in barcodes:
                    # 쿨다운 체크
                    current_time = time.time()
                    if current_time - self.last_scan_time < self.scan_cooldown:
                        continue
                    
                    (x, y, w, h) = barcode.rect
                    # 바코드 영역에 사각형 그리기 (녹색으로 변경)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    barcode_data = barcode.data.decode("utf-8")
                    text = str(barcode_data)
                    
                    # 바코드 번호 표시 (녹색으로 변경)
                    cv2.putText(frame, text, (x, y - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # 새로운 바코드인 경우 신호 발생
                    if barcode_data not in self.found_barcodes:
                        self.found_barcodes.add(barcode_data)
                        self.barcode_detected.emit(barcode_data)
                        self.status_update.emit(f"✅ 바코드 감지: {barcode_data}")
                        self.last_scan_time = current_time
                
                # Payment 라벨 추가
                cv2.putText(frame, "Payment System - Barcode Scanner", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, "Detected books will be added to cart", (10, 460),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                # 프레임을 메인 스레드로 전송 (OpenCV 윈도우 사용 안함!)
                self.frame_ready.emit(frame)
                
                # 짧은 지연
                time.sleep(0.1)
                
        except Exception as e:
            print(f"❌ 바코드 스캔 오류: {e}")
            self.status_update.emit(f"❌ 오류 발생: {str(e)}")
        finally:
            if self.vs:
                self.vs.stop()
    
    def stop(self):
        """스캔 중지 - Stock GUI와 동일한 방식"""
        self.running = False
        if self.vs:
            self.vs.stop()

class CartItem:
    """장바구니 아이템 클래스"""
    def __init__(self, book_info: dict, quantity: int = 1):
        self.book_info = book_info
        self.quantity = quantity
        self.title = book_info.get('title', 'N/A')
        # DB에서 가격 가져오기
        self.price = int(book_info.get('price', 0))
        # DB에서 ISBN 가져오기
        self.isbn = book_info.get('isbn', '')
        self.total_price = self.price * self.quantity
    
    def update_quantity(self, quantity: int):
        """수량 업데이트"""
        self.quantity = quantity
        self.total_price = self.price * self.quantity

class PaymentGUI(QObject):
    """Payment GUI 메인 클래스 - 바코드 스캔 방식 개선"""
    
    
    # 싱글톤 패턴으로 중복 실행 방지
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        # 이미 초기화된 경우 중복 초기화 방지
        if hasattr(self, '_initialized'):
            return
        self._initialized = True
        
        # 🔧 Qt 환경변수 설정 (OpenCV 충돌 방지)
        import os
        os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
        
        # 🔧 QObject 초기화
        super().__init__()
        
        # 🔧 ROS2 노드 별도 생성 (상속 대신 컴포지션 사용)
        self.ros_node = Node('payment_gui')
        
        # 🔧 PyQt 애플리케이션 초기화 - 기존 인스턴스 확인
        self.app = QApplication.instance()
        if self.app is None:
            self.app = QApplication(sys.argv)
            print("✅ 새로운 PyQt 애플리케이션 생성")
        else:
            print("✅ 기존 PyQt 애플리케이션 사용")
        
        # UI 로드 - 경로 안전성 개선
        ui_file = None
        possible_paths = [
            # 현재 스크립트와 같은 디렉토리의 ui_files
            os.path.join(os.path.dirname(__file__), 'ui_files', 'payment_window.ui'),
            # 상위 디렉토리의 ui_files
            os.path.join(os.path.dirname(__file__), '..', 'ui_files', 'payment_window.ui'),
            # 패키지 루트의 ui_files
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ui_files', 'payment_window.ui'),
        ]
        
        for path in possible_paths:
            abs_path = os.path.abspath(path)
            if os.path.exists(abs_path):
                ui_file = abs_path
                print(f"✅ UI 파일 발견: {abs_path}")
                break
        
        # share 디렉토리에서도 찾기 (설치된 위치)
        if ui_file is None:
            try:
                import ament_index_python
                share_dir = ament_index_python.get_package_share_directory('kiosk')
                share_ui_file = os.path.join(share_dir, 'ui_files', 'payment_window.ui')
                if os.path.exists(share_ui_file):
                    ui_file = share_ui_file
                    print(f"✅ UI 파일 발견 (share): {share_ui_file}")
            except Exception as e:
                print(f"⚠️ ament_index_python 사용 불가: {e}")
        
        # UI 파일이 없으면 간단한 기본 창 생성
        if ui_file is None or not os.path.exists(ui_file):
            print("⚠️ UI 파일을 찾을 수 없어 기본 창을 생성합니다.")
            self.dialog = self.create_default_dialog()
        else:
            try:
                self.dialog = uic.loadUi(ui_file)
                print(f"✅ UI 로드 성공: {ui_file}")
            except Exception as e:
                print(f"❌ UI 로드 실패: {e}")
                print("⚠️ 기본 창을 생성합니다.")
                self.dialog = self.create_default_dialog()
        
        # 데이터베이스 초기화 (결제 시스템용)
        self.db_manager = DatabaseManager()
        
        # 바코드 스캐너 초기화 (Stock GUI 방식으로 변경)
        self.barcode_scanner = None
        self.scanning = False
        self.camera_window = None
        
        # 장바구니 관리
        self.cart = []
        self.total_amount = 0
        
        # 현재 상태
        self.current_book_info = None
        
        # ROS2 스핀을 위한 타이머 설정 (스레드 충돌 방지)
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros_node)
        self.ros_timer.start(100)  # 100ms 간격
        
        # UI 초기화
        self.setup_ui()
        self.connect_signals()
        
        print("✅ Payment GUI 초기화 완료 (바코드 스캔 방식 개선)")
    
    def create_default_dialog(self):
        """UI 파일이 없을 때 기본 다이얼로그 생성"""
        dialog = QDialog()
        dialog.setWindowTitle("LIBO - Payment System 💳")
        dialog.setFixedSize(1000, 700)
        
        # 메인 레이아웃
        main_layout = QHBoxLayout(dialog)
        
        # 왼쪽 패널 (검색 및 결과)
        left_panel = QVBoxLayout()
        
        # 검색 영역
        search_group = QGroupBox("📚 도서 검색")
        search_layout = QVBoxLayout(search_group)
        
        # 검색 입력
        input_layout = QHBoxLayout()
        self.book_title_input = QLineEdit()
        self.book_title_input.setPlaceholderText("도서명 또는 ISBN을 입력하세요...")
        self.search_button = QPushButton("🔍 검색")
        self.qr_scan_button = QPushButton("📱 Barcode Scan")
        
        input_layout.addWidget(self.book_title_input)
        input_layout.addWidget(self.search_button)
        input_layout.addWidget(self.qr_scan_button)
        search_layout.addLayout(input_layout)
        
        # 검색 결과 리스트
        self.search_results_list = QListWidget()
        self.search_results_list.setMaximumHeight(200)
        search_layout.addWidget(QLabel("검색 결과:"))
        search_layout.addWidget(self.search_results_list)
        
        # 수량 선택 및 추가
        add_layout = QHBoxLayout()
        add_layout.addWidget(QLabel("수량:"))
        self.quantity_spinbox = QSpinBox()
        self.quantity_spinbox.setMinimum(1)
        self.quantity_spinbox.setMaximum(10)
        self.quantity_spinbox.setValue(1)
        self.add_to_cart_button = QPushButton("🛒 장바구니 추가")
        self.add_to_cart_button.setEnabled(False)
        
        add_layout.addWidget(self.quantity_spinbox)
        add_layout.addWidget(self.add_to_cart_button)
        search_layout.addLayout(add_layout)
        
        left_panel.addWidget(search_group)
        
        # 오른쪽 패널 (장바구니 및 결제)
        right_panel = QVBoxLayout()
        
        # 장바구니 영역
        cart_group = QGroupBox("🛒 장바구니")
        cart_layout = QVBoxLayout(cart_group)
        
        # 장바구니 테이블
        self.cart_table = QTableWidget(0, 4)
        self.cart_table.setHorizontalHeaderLabels(["Book", "Qty", "Price", "Total"])
        self.cart_table.setColumnWidth(0, 180)
        self.cart_table.setColumnWidth(1, 50)
        self.cart_table.setColumnWidth(2, 80)
        self.cart_table.setColumnWidth(3, 80)
        cart_layout.addWidget(self.cart_table)
        
        # 장바구니 버튼들
        cart_buttons = QHBoxLayout()
        self.remove_item_button = QPushButton("❌ 선택 제거")
        self.remove_item_button.setEnabled(False)
        self.clear_cart_button = QPushButton("🧹 전체 비우기")
        
        cart_buttons.addWidget(self.remove_item_button)
        cart_buttons.addWidget(self.clear_cart_button)
        cart_layout.addLayout(cart_buttons)
        
        right_panel.addWidget(cart_group)
        
        # 결제 영역
        payment_group = QGroupBox("💳 결제")
        payment_layout = QVBoxLayout(payment_group)
        
        self.total_label = QLabel("Total: ₩0")
        self.total_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #dc3545;")
        self.total_label.setAlignment(Qt.AlignCenter)
        
        self.payment_button = QPushButton("💳 Pay Now")
        self.payment_button.setEnabled(False)
        self.payment_button.setStyleSheet("""
            QPushButton {
                background: #28a745;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 15px;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: #34ce57;
            }
            QPushButton:disabled {
                background: #6c757d;
            }
        """)
        
        self.back_button = QPushButton("🏠 메인으로")
        self.back_button.setStyleSheet("""
            QPushButton {
                background: #6c757d;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 10px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: #7d8489;
            }
        """)
        
        payment_layout.addWidget(self.total_label)
        payment_layout.addWidget(self.payment_button)
        payment_layout.addWidget(self.back_button)
        
        right_panel.addWidget(payment_group)
        
        # 레이아웃 조합
        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 1)
        
        # 다이얼로그에 속성으로 위젯들 설정 (uic.loadUi와 동일하게)
        dialog.book_title_input = self.book_title_input
        dialog.search_button = self.search_button
        dialog.qr_scan_button = self.qr_scan_button
        dialog.search_results_list = self.search_results_list
        dialog.quantity_spinbox = self.quantity_spinbox
        dialog.add_to_cart_button = self.add_to_cart_button
        dialog.cart_table = self.cart_table
        dialog.remove_item_button = self.remove_item_button
        dialog.clear_cart_button = self.clear_cart_button
        dialog.total_label = self.total_label
        dialog.payment_button = self.payment_button
        dialog.back_button = self.back_button
        
        return dialog
    
    def spin_ros_node(self):
        """안전한 ROS2 노드 스핀 처리"""
        try:
            if rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        except Exception as e:
            # RCL 오류 무시 (프로그램 종료 시 정상적인 오류)
            if "not valid" not in str(e):
                print(f"❌ ROS2 스핀 오류: {e}")
    
    def start_barcode_scan(self):
        """바코드 스캔 시작 - Stock GUI 방식으로 개선"""
        if self.scanning:
            return
        
        # 바코드 스캐너 스레드 생성 (Stock GUI 방식)
        if self.barcode_scanner is None:
            self.barcode_scanner = BarcodeScannerThread()
            self.barcode_scanner.barcode_detected.connect(self.on_barcode_detected)
            self.barcode_scanner.frame_ready.connect(self.update_camera_frame)
            self.barcode_scanner.status_update.connect(self.update_camera_status)
        
        # 카메라 창 생성 및 표시 (Stock GUI와 동일)
        self.camera_window = CameraWindow()
        self.camera_window.show()
        
        # 스캔 시작
        self.barcode_scanner.start()
        self.scanning = True
        
        # UI 업데이트
        if hasattr(self.dialog, 'qr_scan_button'):
            self.dialog.qr_scan_button.setText("📱 스캔 중지")
            self.dialog.qr_scan_button.setEnabled(True)
        
        print("📱 바코드 스캔을 시작합니다 (Stock GUI 방식)")
    
    def stop_barcode_scan(self):
        """바코드 스캔 중지 - Stock GUI 방식으로 개선"""
        if self.barcode_scanner:
            self.barcode_scanner.stop()
            self.barcode_scanner.wait()
            self.barcode_scanner = None
        
        if self.camera_window:
            self.camera_window.close()
            self.camera_window = None
        
        self.scanning = False
        
        # UI 업데이트
        if hasattr(self.dialog, 'qr_scan_button'):
            self.dialog.qr_scan_button.setText("📱 Barcode Scan")
        
        print("📱 바코드 스캔이 중지되었습니다")
    
    def toggle_barcode_scan(self):
        """바코드 스캔 토글 - Stock GUI와 동일한 방식"""
        if not self.scanning:
            self.start_barcode_scan()
        else:
            self.stop_barcode_scan()
    
    @pyqtSlot(str)
    def on_barcode_detected(self, barcode_data: str):
        """바코드 감지 처리 - Stock GUI 방식으로 개선"""
        print(f"📱 바코드 감지됨: {barcode_data}")
        
        # ISBN 입력 필드에 바코드 데이터 설정
        if hasattr(self.dialog, 'book_title_input'):
            self.dialog.book_title_input.setText(barcode_data)
        
        # 자동으로 검색 실행
        self.search_book_by_isbn(barcode_data)
        
        # 스캔 자동 중지
        self.stop_barcode_scan()
    
    def update_camera_frame(self, frame):
        """카메라 프레임 업데이트 - Stock GUI와 동일"""
        if self.camera_window:
            self.camera_window.update_frame(frame)
    
    def update_camera_status(self, message):
        """카메라 창 상태 업데이트 - Stock GUI와 동일"""
        if self.camera_window:
            self.camera_window.update_status(message)
    
    def search_book(self):
        """도서 검색 (제목으로)"""
        query = self.dialog.book_title_input.text().strip()
        if not query:
            QMessageBox.warning(self.dialog, "경고", "검색할 도서명을 입력해주세요.")
            return
        
        # ISBN인지 제목인지 판단
        if query.isdigit() and len(query) in [10, 13]:
            self.search_book_by_isbn(query)
        else:
            self.search_book_by_title(query)
    
    def search_book_by_isbn(self, isbn: str):
        """ISBN으로 도서 검색 (DB에서만)"""
        if hasattr(self.dialog, 'search_button'):
            self.dialog.search_button.setEnabled(False)
        if hasattr(self.dialog, 'search_results_list'):
            self.dialog.search_results_list.clear()
        
        print(f"🔍 ISBN으로 검색 중: {isbn}")
        
        try:
            # DB에서 검색
            db_result = self.db_manager.get_book_by_isbn(isbn)
            if db_result:
                print("✅ DB에서 도서 발견")
                self.display_search_results([db_result], from_db=True)
            else:
                print("❌ DB에서 도서를 찾을 수 없습니다.")
                QMessageBox.information(self.dialog, "검색 결과", "해당 ISBN의 도서를 찾을 수 없습니다.")
        except Exception as e:
            print(f"❌ 도서 검색 중 오류: {e}")
            QMessageBox.critical(self.dialog, "오류", f"도서 검색 중 오류가 발생했습니다: {str(e)}")
        finally:
            if hasattr(self.dialog, 'search_button'):
                self.dialog.search_button.setEnabled(True)
    
    def search_book_by_title(self, title: str):
        """제목으로 도서 검색 (DB에서)"""
        try:
            books = self.db_manager.search_books(title, 'title')
            self.display_search_results(books, from_db=True)
            print(f"🔍 제목으로 검색: {title}")
        except Exception as e:
            print(f"❌ 도서 검색 중 오류: {e}")
            QMessageBox.critical(self.dialog, "오류", f"도서 검색 중 오류가 발생했습니다: {str(e)}")
    
    def display_search_results(self, results: list, from_db: bool = False):
        """검색 결과 표시 (DB 결과만)"""
        if not hasattr(self.dialog, 'search_results_list'):
            print("⚠️ search_results_list가 UI에 없습니다.")
            return
        
        self.dialog.search_results_list.clear()
        
        if not results:
            print("❌ 검색 결과가 없습니다.")
            QMessageBox.information(self.dialog, "검색 결과", "검색 결과가 없습니다.")
            return
        
        for book in results:
            # DB 결과 표시
            title = book.get('title', 'N/A')
            author = book.get('author', 'N/A')
            price = book.get('price', 0)
            stock = book.get('stock_quantity', 0)
            
            # 재고 부족 확인
            if stock <= 0:
                item_text = f"❌ {title} - {author} (₩{price:,}) [재고 없음]"
            else:
                item_text = f"✅ {title} - {author} (₩{price:,}) [재고: {stock}]"
            
            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, book)
            
            # 재고 없는 경우 비활성화 표시
            if book.get('stock_quantity', 0) <= 0:
                item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
                
            self.dialog.search_results_list.addItem(item)
        
        print(f"✅ {len(results)}권의 도서를 찾았습니다.")
    
    @pyqtSlot(QListWidgetItem)
    def on_book_selected(self, item: QListWidgetItem):
        """도서 선택 처리"""
        self.current_book_info = item.data(Qt.UserRole)
        book_title = self.current_book_info.get('title', 'N/A')
        
        # 재고 확인 (DB 결과인 경우)
        if 'stock_quantity' in self.current_book_info:
            stock = self.current_book_info.get('stock_quantity', 0)
            if stock <= 0:
                QMessageBox.warning(self.dialog, "재고 부족", f"'{book_title}'은(는) 현재 재고가 없습니다.")
                if hasattr(self.dialog, 'add_to_cart_button'):
                    self.dialog.add_to_cart_button.setEnabled(False)
                return
        
        print(f"📚 도서 선택됨: {book_title}")
        
        # 추가 버튼 활성화
        if hasattr(self.dialog, 'add_to_cart_button'):
            self.dialog.add_to_cart_button.setEnabled(True)
    
    def add_to_cart(self):
        """장바구니에 추가"""
        if not self.current_book_info:
            QMessageBox.warning(self.dialog, "경고", "추가할 도서를 선택해주세요.")
            return
        
        quantity = 1
        if hasattr(self.dialog, 'quantity_spinbox'):
            quantity = self.dialog.quantity_spinbox.value()
        
        # ISBN 가져오기 개선 - DB와 API 모두 지원
        isbn = self.current_book_info.get('isbn', self.current_book_info.get('isbn13', ''))
        
        # 재고 확인 (DB 결과인 경우)
        if 'stock_quantity' in self.current_book_info:
            available_stock = self.current_book_info.get('stock_quantity', 0)
            if available_stock < quantity:
                QMessageBox.warning(
                    self.dialog, 
                    "재고 부족", 
                    f"현재 재고: {available_stock}권\n요청 수량: {quantity}권\n\n재고가 부족합니다."
                )
                return
        
        # 중복 확인
        for cart_item in self.cart:
            if cart_item.isbn == isbn:
                new_quantity = cart_item.quantity + quantity
                
                # 재고 재확인
                if 'stock_quantity' in self.current_book_info:
                    available_stock = self.current_book_info.get('stock_quantity', 0)
                    if available_stock < new_quantity:
                        QMessageBox.warning(
                            self.dialog, 
                            "재고 부족", 
                            f"현재 재고: {available_stock}권\n장바구니 + 추가 수량: {new_quantity}권\n\n재고가 부족합니다."
                        )
                        return
                
                cart_item.update_quantity(new_quantity)
                self.update_cart_display()
                print(f"📚 기존 아이템 수량 증가: {cart_item.title}")
                return
        
        # 새 아이템 추가
        cart_item = CartItem(self.current_book_info, quantity)
        self.cart.append(cart_item)
        
        self.update_cart_display()
        print(f"🛒 장바구니에 추가됨: {cart_item.title} x{quantity}")
        
        # 성공 메시지
        QMessageBox.information(
            self.dialog, 
            "추가 완료", 
            f"'{cart_item.title}'이(가)\n장바구니에 추가되었습니다! 🛒"
        )
    
    def update_cart_display(self):
        """장바구니 표시 업데이트"""
        if not hasattr(self.dialog, 'cart_table'):
            print("⚠️ cart_table이 UI에 없습니다.")
            return
        
        self.dialog.cart_table.setRowCount(len(self.cart))
        
        self.total_amount = 0
        
        for i, cart_item in enumerate(self.cart):
            # 도서명
            title_text = cart_item.title[:25] + "..." if len(cart_item.title) > 25 else cart_item.title
            title_item = QTableWidgetItem(title_text)
            self.dialog.cart_table.setItem(i, 0, title_item)
            
            # 수량
            qty_item = QTableWidgetItem(str(cart_item.quantity))
            qty_item.setTextAlignment(Qt.AlignCenter)
            self.dialog.cart_table.setItem(i, 1, qty_item)
            
            # 단가
            price_item = QTableWidgetItem(f"₩{cart_item.price:,}")
            price_item.setTextAlignment(Qt.AlignRight)
            self.dialog.cart_table.setItem(i, 2, price_item)
            
            # 합계
            total_item = QTableWidgetItem(f"₩{cart_item.total_price:,}")
            total_item.setTextAlignment(Qt.AlignRight)
            self.dialog.cart_table.setItem(i, 3, total_item)
            
            self.total_amount += cart_item.total_price
        
        self.update_total_display()
        self.update_payment_button_state()
    
    def remove_selected_item(self):
        """선택된 아이템 제거"""
        if not hasattr(self.dialog, 'cart_table'):
            return
        
        current_row = self.dialog.cart_table.currentRow()
        if current_row >= 0 and current_row < len(self.cart):
            removed_item = self.cart.pop(current_row)
            self.update_cart_display()
            print(f"🗑️ 아이템 제거됨: {removed_item.title}")
    
    def clear_cart(self):
        """장바구니 비우기"""
        if not self.cart:
            return
        
        reply = QMessageBox.question(
            self.dialog, 
            "장바구니 비우기", 
            "모든 아이템을 제거하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.cart.clear()
            self.update_cart_display()
            print("🧹 장바구니를 비웠습니다.")
    
    def on_cart_selection_changed(self):
        """장바구니 선택 변경 처리"""
        if hasattr(self.dialog, 'cart_table') and hasattr(self.dialog, 'remove_item_button'):
            has_selection = self.dialog.cart_table.currentRow() >= 0
            self.dialog.remove_item_button.setEnabled(has_selection)
    
    def process_payment(self):
        """결제 처리"""
        if not self.cart:
            QMessageBox.warning(self.dialog, "경고", "결제할 상품이 없습니다.")
            return
        
        # 결제 전 재고 재확인
        if not self.validate_cart_stock():
            return
        
        # RFID 카드 인증 요청
        self.request_rfid_authentication()
    
    def validate_cart_stock(self) -> bool:
        """장바구니 아이템들의 재고 재확인"""
        try:
            for cart_item in self.cart:
                isbn = cart_item.isbn
                
                # DB에서 현재 재고 확인
                current_book = self.db_manager.get_book_by_isbn(isbn)
                if not current_book:
                    QMessageBox.warning(
                        self.dialog, 
                        "재고 확인 오류",
                        f"'{cart_item.title}'의 재고를 확인할 수 없습니다."
                    )
                    return False
                
                current_stock = current_book.get('stock_quantity', 0)
                if current_stock < cart_item.quantity:
                    QMessageBox.warning(
                        self.dialog,
                        "재고 부족",
                        f"'{cart_item.title}'\n\n" +
                        f"현재 재고: {current_stock}권\n" +
                        f"주문 수량: {cart_item.quantity}권\n\n" +
                        "재고가 부족하여 결제할 수 없습니다."
                    )
                    return False
            
            return True
            
        except Exception as e:
            print(f"❌ 재고 확인 중 오류: {e}")
            QMessageBox.critical(self.dialog, "오류", "재고 확인 중 오류가 발생했습니다.")
            return False
    
    def request_rfid_authentication(self):
        """RFID 카드 인증 요청"""
        # RFID 인증 다이얼로그 생성
        rfid_dialog = QDialog(self.dialog)
        rfid_dialog.setWindowTitle("💳 RFID 카드 결제")
        rfid_dialog.setFixedSize(450, 350)
        rfid_dialog.setModal(True)
        
        layout = QVBoxLayout(rfid_dialog)
        layout.setSpacing(20)
        layout.setContentsMargins(30, 30, 30, 30)
        
        # 제목
        title_label = QLabel("💳 RFID 카드를 리더기에 대주세요")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("""
            QLabel {
                font-size: 18px;
                font-weight: bold;
                color: #2c3e50;
                padding: 10px;
            }
        """)
        layout.addWidget(title_label)
        
        # 결제 정보 표시
        payment_info = QLabel(f"""
💰 결제 금액: ₩{self.total_amount:,}
📚 구매 도서: {len(self.cart)}권

RFID 카드를 리더기에 터치해주세요
        """)
        payment_info.setAlignment(Qt.AlignCenter)
        payment_info.setStyleSheet("""
            QLabel {
                font-size: 14px;
                color: #2c3e50;
                background: white;
                padding: 20px;
                border-radius: 10px;
                border: 2px solid #71866a;
            }
        """)
        layout.addWidget(payment_info)
        
        # RFID 상태 표시
        status_label = QLabel("🔄 RFID 카드를 기다리는 중...")
        status_label.setAlignment(Qt.AlignCenter)
        status_label.setStyleSheet("""
            QLabel {
                font-size: 16px;
                font-weight: bold;
                color: #007bff;
                padding: 10px;
            }
        """)
        layout.addWidget(status_label)
        
        # 버튼들
        button_layout = QHBoxLayout()
        
        cancel_btn = QPushButton("❌ 취소")
        cancel_btn.setStyleSheet("""
            QPushButton {
                background: #6c757d;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 10px 20px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: #7d8489;
            }
        """)
        cancel_btn.clicked.connect(rfid_dialog.reject)
        
        # 임시 결제 완료 버튼 (실제로는 RFID 리더기에서 자동 처리)
        simulate_btn = QPushButton("💳 결제 완료 (임시)")
        simulate_btn.setStyleSheet("""
            QPushButton {
                background: #28a745;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 10px 20px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: #34ce57;
            }
        """)
        
        def simulate_rfid_success():
            status_label.setText("✅ RFID 카드 인식 완료!")
            status_label.setStyleSheet("""
                QLabel {
                    font-size: 16px;
                    font-weight: bold;
                    color: #28a745;
                    padding: 10px;
                }
            """)
            QTimer.singleShot(1000, rfid_dialog.accept)  # 1초 후 자동 닫기
        
        simulate_btn.clicked.connect(simulate_rfid_success)
        
        button_layout.addWidget(cancel_btn)
        button_layout.addWidget(simulate_btn)
        layout.addLayout(button_layout)
        
        # 다이얼로그 실행
        if rfid_dialog.exec_() == QDialog.Accepted:
            # RFID 인증 성공 시 결제 진행
            print("✅ RFID 카드 결제 성공")
            self.simulate_payment_success()
        else:
            print("❌ RFID 카드 결제가 취소되었습니다.")
    
    def simulate_payment_success(self):
        """결제 성공 시뮬레이션 (임시)"""
        # 재고 감소 처리
        self.update_stock_quantities()
        
        # 성공 팝업 표시
        self.show_payment_success_popup()
    
    def update_stock_quantities(self):
        """데이터베이스 재고 수량 감소"""
        try:
            for cart_item in self.cart:
                isbn = cart_item.isbn
                quantity = cart_item.quantity
                
                # DB에서 현재 재고 확인 후 감소
                success = self.decrease_book_stock(isbn, quantity)
                if success:
                    print(f"✅ 재고 감소: {cart_item.title} (-{quantity}권)")
                else:
                    print(f"❌ 재고 감소 실패: {cart_item.title}")
                    
        except Exception as e:
            print(f"❌ 재고 감소 중 오류: {e}")
    
    def decrease_book_stock(self, isbn: str, quantity: int) -> bool:
        """데이터베이스 재고 감소"""
        if not self.db_manager.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return False
        
        try:
            import pymysql
            
            with self.db_manager.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                # 트랜잭션 시작
                cursor.execute("START TRANSACTION")
                
                # 현재 재고 확인
                cursor.execute("SELECT id, stock_quantity FROM book WHERE isbn = %s", (isbn,))
                book = cursor.fetchone()
                
                if not book:
                    print(f"❌ 도서를 찾을 수 없습니다: ISBN {isbn}")
                    cursor.execute("ROLLBACK")
                    return False
                
                current_stock = book['stock_quantity'] 
                
                # 재고 부족 확인
                if current_stock < quantity:
                    print(f"❌ 재고 부족: 현재 {current_stock}권, 요청 {quantity}권")
                    cursor.execute("ROLLBACK")
                    return False
                
                # 재고 감소
                new_stock = current_stock - quantity
                update_sql = "UPDATE book SET stock_quantity = %s WHERE isbn = %s"
                cursor.execute(update_sql, (new_stock, isbn))
                
                # 트랜잭션 커밋
                cursor.execute("COMMIT")
                
                print(f"✅ 재고 감소 성공: ISBN {isbn}")
                print(f"   기존 재고: {current_stock}권 → 새로운 재고: {new_stock}권")
                
                return True
                
        except Exception as e:
            print(f"❌ 재고 감소 실패: {e}")
            # 에러 시 롤백
            try:
                cursor.execute("ROLLBACK")
            except:
                pass
            return False
    
    def show_payment_success_popup(self):
        """결제 완료 팝업 + 5초 카운트다운"""
        try:
            # 커스텀 다이얼로그 생성
            success_dialog = QDialog(self.dialog)
            success_dialog.setWindowTitle("💳 결제 완료")
            success_dialog.setFixedSize(500, 300)
            success_dialog.setWindowFlags(Qt.Dialog | Qt.WindowTitleHint)
            
            # 다이얼로그 스타일 설정 (메인 키오스크와 동일한 색상)
            success_dialog.setStyleSheet("""
                QDialog {
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                        stop:0 #f8f9fa, stop:1 #e9ecef);
                    border-radius: 15px;
                }
                QLabel {
                    color: #2c3e50;
                    font-weight: bold;
                }
            """)
            
            # 레이아웃 생성
            layout = QVBoxLayout(success_dialog)
            layout.setSpacing(20)
            layout.setContentsMargins(30, 30, 30, 30)
            
            # 성공 아이콘 + 제목
            title_label = QLabel("🎉 결제가 완료되었습니다!")
            title_label.setAlignment(Qt.AlignCenter)
            title_label.setStyleSheet("""
                QLabel {
                    font-size: 20px;
                    font-weight: bold;
                    color: #28a745;
                    padding: 10px;
                }
            """)
            layout.addWidget(title_label)
            
            # 결제 정보
            info_text = f"""
📚 구매 도서: {len(self.cart)}권
💰 총 결제 금액: ₩{self.total_amount:,}

이용해 주셔서 감사합니다! 📖
            """
            
            info_label = QLabel(info_text)
            info_label.setAlignment(Qt.AlignCenter)
            info_label.setStyleSheet("""
                QLabel {
                    font-size: 14px;
                    color: #2c3e50;
                    background: white;
                    padding: 20px;
                    border-radius: 10px;
                    border: 2px solid #71866a;
                }
            """)
            layout.addWidget(info_label)
            
            # 카운트다운 라벨
            countdown_label = QLabel("5초 후 메인 화면으로 돌아갑니다...")
            countdown_label.setAlignment(Qt.AlignCenter)
            countdown_label.setStyleSheet("""
                QLabel {
                    font-size: 16px;
                    font-weight: bold;
                    color: #71866a;
                    padding: 10px;
                }
            """)
            layout.addWidget(countdown_label)
            
            # 다이얼로그 중앙 정렬
            success_dialog.move(
                self.dialog.x() + (self.dialog.width() - success_dialog.width()) // 2,
                self.dialog.y() + (self.dialog.height() - success_dialog.height()) // 2
            )
            
            # 카운트다운 타이머 설정
            countdown_seconds = [5]  # 리스트로 감싸서 참조 유지
            
            countdown_timer = QTimer()
            
            def update_countdown():
                if countdown_seconds[0] > 0:
                    countdown_label.setText(f"{countdown_seconds[0]}초 후 메인 화면으로 돌아갑니다...")
                    countdown_seconds[0] -= 1
                else:
                    countdown_timer.stop()
                    success_dialog.close()
                    # 모든 위젯창 닫고 메인 윈도우로 복귀
                    self.return_to_main_window()
            
            countdown_timer.timeout.connect(update_countdown)  
            countdown_timer.start(1000)  # 1초마다 업데이트
            
            # 팝업창 표시 (모달)
            success_dialog.exec_()
            
        except Exception as e:
            print(f"❌ 성공 팝업창 표시 중 오류: {e}")
            # 오류 시 기본 방식으로 처리
            QMessageBox.information(
                self.dialog,
                "💳 결제 완료",
                f"결제가 완료되었습니다!\n총 결제 금액: ₩{self.total_amount:,}\n\n5초 후 메인화면으로 돌아갑니다."
            )
            # 5초 후 메인 윈도우로 복귀
            QTimer.singleShot(5000, self.return_to_main_window)
    
    def return_to_main_window(self):
        """메인 윈도우로 복귀"""
        try:
            print("🏠 메인 윈도우로 복귀 시작...")
            
            # 1. 모든 팝업창 닫기
            self.close_all_popups()
            
            # 2. 바코드 스캔 중지
            if self.scanning:
                self.stop_barcode_scan()
            
            # 3. Payment 윈도우 닫기
            self.dialog.close()
            
            # 4. 메인 윈도우 표시 및 리프레시
            self.show_and_refresh_main_window()
            
            print("✅ 메인 윈도우 복귀 완료")
            
        except Exception as e:
            print(f"❌ 메인 윈도우 복귀 중 오류: {e}")
    
    def close_all_popups(self):
        """모든 팝업창 닫기"""
        try:
            for widget in QApplication.topLevelWidgets():
                if isinstance(widget, QDialog) and widget.isVisible():
                    widget.close()
                    print(f"✅ 팝업창 닫기: {widget.windowTitle()}")
        except Exception as e:
            print(f"❌ 팝업창 닫기 중 오류: {e}")
    
    def show_and_refresh_main_window(self):
        """메인 윈도우 표시 및 리프레시"""
        try:
            # 부모 윈도우 찾기 개선
            parent_window = None
            for widget in QApplication.topLevelWidgets():
                # 메인 윈도우 클래스명 확인
                if (hasattr(widget, 'objectName') and 
                    ('MainWindow' in str(type(widget)) or 'KioskGUI' in str(type(widget)))):
                    parent_window = widget
                    break
            
            if parent_window:
                # 메인 윈도우 표시
                parent_window.show()
                parent_window.raise_()
                parent_window.activateWindow()
                
                # 메인 윈도우 중앙 정렬 (기존 함수 활용)
                if hasattr(parent_window, 'force_center_window'):
                    parent_window.force_center_window()
                
                # 메인 윈도우 리프레시 (UI 상태 초기화)
                if hasattr(parent_window, 'refresh_main_window'):
                    parent_window.refresh_main_window()
                
                print("✅ 메인 윈도우 표시 및 리프레시 완료")
            else:
                print("⚠️ 메인 윈도우를 찾을 수 없습니다.")
                # 애플리케이션 종료
                self.app.quit()
                
        except Exception as e:
            print(f"❌ 메인 윈도우 표시 중 오류: {e}")
    
    def close_window(self):
        """창 닫기"""
        # 스캐닝 중이면 중지
        if self.scanning:
            self.stop_barcode_scan()
        
        # 메인 윈도우로 복귀
        self.show_and_refresh_main_window()
        
        # Payment 윈도우 닫기
        self.dialog.close()
        print("👋 Payment 창을 닫습니다.")
    
    def connect_signals(self):
        """안전한 시그널-슬롯 연결"""
        def safe_connect(widget_name, signal_name, slot_method, signal_args=None):
            """안전하게 시그널을 연결하는 헬퍼 함수"""
            try:
                if hasattr(self.dialog, widget_name):
                    widget = getattr(self.dialog, widget_name)
                    if hasattr(widget, signal_name):
                        signal = getattr(widget, signal_name)
                        if signal_args:
                            signal[signal_args].connect(slot_method)
                        else:
                            signal.connect(slot_method)
                        print(f"✅ 연결 성공: {widget_name}.{signal_name}")
                        return True
                    else:
                        print(f"⚠️ 시그널 없음: {widget_name}.{signal_name}")
                else:
                    print(f"⚠️ 위젯 없음: {widget_name}")
                return False
            except Exception as e:
                print(f"❌ 연결 실패 {widget_name}.{signal_name}: {e}")
                return False
        
        # 각 연결을 안전하게 시도
        connections = [
            # 기본 버튼들
            ('back_button', 'clicked', self.close_window),
            ('search_button', 'clicked', self.search_book), 
            ('qr_scan_button', 'clicked', self.toggle_barcode_scan),  # toggle로 변경
            ('add_to_cart_button', 'clicked', self.add_to_cart),
            ('remove_item_button', 'clicked', self.remove_selected_item),
            ('clear_cart_button', 'clicked', self.clear_cart),
            ('payment_button', 'clicked', self.process_payment),
            
            # 입력 필드
            ('book_title_input', 'returnPressed', self.search_book),
            
            # 리스트 및 테이블
            ('cart_table', 'itemSelectionChanged', self.on_cart_selection_changed),
        ]
        
        connected_count = 0
        for widget_name, signal_name, slot_method in connections:
            if safe_connect(widget_name, signal_name, slot_method):
                connected_count += 1
        
        # search_results_list 연결을 별도로 처리 (QListWidget 특성상)
        try:
            if hasattr(self.dialog, 'search_results_list'):
                self.dialog.search_results_list.itemClicked.connect(self.on_book_selected)
                print("✅ 연결 성공: search_results_list.itemClicked")
                connected_count += 1
            else:
                print("⚠️ 위젯 없음: search_results_list")
        except Exception as e:
            print(f"❌ 연결 실패 search_results_list.itemClicked: {e}")
        
        print(f"🔗 시그널-슬롯 연결 완료: {connected_count}개 연결됨")
        
        # 연결된 개수가 너무 적으면 경고
        if connected_count < 6:
            print("⚠️ 일부 UI 요소 연결이 실패했습니다.")
            print("💡 payment_window.ui 파일의 objectName들을 확인해주세요.")
    
    def setup_ui(self):
        """UI 초기 설정 - 안전성 개선"""
        # 윈도우 설정
        self.dialog.setWindowTitle("LIBO - Payment System 💳")
        
        # 필수 UI 요소 존재 확인
        ui_elements_status = {}
        required_elements = {
            'cart_table': '장바구니 테이블',
            'total_label': '총합 라벨', 
            'search_results_list': '검색 결과 리스트',
            'book_title_input': '도서 제목 입력',
            'quantity_spinbox': '수량 선택',
            'add_to_cart_button': '장바구니 추가 버튼',
            'payment_button': '결제 버튼',
            'qr_scan_button': '바코드 스캔 버튼'
        }
        
        for element_name, description in required_elements.items():
            if hasattr(self.dialog, element_name):
                ui_elements_status[element_name] = True
                print(f"✅ {description} 발견")
            else:
                ui_elements_status[element_name] = False
                print(f"❌ {description} 누락: {element_name}")
        
        # 장바구니 테이블 설정 (있는 경우에만)
        if ui_elements_status.get('cart_table', False):
            try:
                self.dialog.cart_table.setColumnWidth(0, 180)  # Book 
                self.dialog.cart_table.setColumnWidth(1, 50)   # Qty
                self.dialog.cart_table.setColumnWidth(2, 80)   # Price
                self.dialog.cart_table.setColumnWidth(3, 80)   # Total
                
                # 테이블 헤더 설정
                self.dialog.cart_table.setHorizontalHeaderLabels(["Book", "Qty", "Price", "Total"])
                print("✅ 장바구니 테이블 설정 완료")
            except Exception as e:
                print(f"❌ 장바구니 테이블 설정 실패: {e}")
        
        # 초기 상태 설정 (안전하게)
        try:
            self.update_total_display()
            self.update_payment_button_state()
        except Exception as e:
            print(f"⚠️ 초기 상태 설정 중 일부 오류: {e}")
        
        print("🎨 UI 설정 완료 (바코드 스캔 방식 개선)")
    
    def update_total_display(self):
        """총합 표시 업데이트 - 안전성 개선"""
        try:
            if hasattr(self.dialog, 'total_label'):
                self.dialog.total_label.setText(f"Total: ₩{self.total_amount:,}")
                
                # 총합이 있으면 강조 스타일 적용
                if self.total_amount > 0:
                    self.dialog.total_label.setStyleSheet("""
                        QLabel {
                            color: #dc3545;
                            background: transparent;
                            border: none;
                            font-weight: bold;
                            font-size: 18px;
                        }
                    """)
                else:
                    self.dialog.total_label.setStyleSheet("""
                        QLabel {
                            color: #2c3e50;
                            background: transparent;
                            border: none;
                            font-weight: bold;
                        }
                    """)
            else:
                print("⚠️ total_label이 UI에 없습니다.")
        except Exception as e:
            print(f"❌ 총합 표시 업데이트 오류: {e}")
    
    def update_payment_button_state(self):
        """결제 버튼 상태 업데이트 - 안전성 개선"""
        try:
            if hasattr(self.dialog, 'payment_button'):
                has_items = len(self.cart) > 0
                self.dialog.payment_button.setEnabled(has_items)
                
                if has_items:
                    self.dialog.payment_button.setText(f"💳 Pay Now (₩{self.total_amount:,})")
                else:
                    self.dialog.payment_button.setText("💳 Pay Now")
            else:
                print("⚠️ payment_button이 UI에 없습니다.")
        except Exception as e:
            print(f"❌ 결제 버튼 상태 업데이트 오류: {e}")
    
    def center_payment_window(self):
        """Payment 윈도우 중앙 정렬"""
        screen = QApplication.desktop().screenGeometry()
        
        window_width = 1000
        window_height = 700
        
        center_x = (screen.width() - window_width) // 2
        center_y = (screen.height() - window_height) // 2
        
        self.dialog.setGeometry(center_x, center_y, window_width, window_height)
        print(f"✅ Payment 윈도우 중앙 정렬: ({center_x}, {center_y})")
    
    def run(self):
        """GUI 실행 - 안전한 종료 처리"""
        try:
            self.dialog.show()
            
            # 윈도우 중앙 정렬
            self.center_payment_window()
            
            return self.app.exec_()
        except Exception as e:
            print(f"❌ GUI 실행 중 오류: {e}")
            return 1
        finally:
            # 안전한 리소스 정리
            self.cleanup_resources()
    
    def cleanup_resources(self):
        """리소스 정리"""
        try:
            # 바코드 스캐너 정리
            if self.scanning:
                self.stop_barcode_scan()
            
            # ROS2 노드 정리
            if hasattr(self, 'ros_node'):
                self.ros_node.destroy_node()
            
            # ROS2 타이머 정리
            if hasattr(self, 'ros_timer') and self.ros_timer.isActive():
                self.ros_timer.stop()
            
            print("✅ 리소스 정리 완료")
        except Exception as e:
            print(f"❌ 리소스 정리 중 오류: {e}")


def main(args=None):
    """메인 함수 - Qt 충돌 해결"""
    print("🚀 Payment GUI 시작... (바코드 스캔 방식 개선)")
    
    # 🔧 Qt 플러그인 충돌 해결을 위한 환경변수 설정
    import os
    os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)  # OpenCV Qt 경로 제거
    os.environ['QT_QPA_PLATFORM'] = 'xcb'  # 명시적으로 xcb 사용
    
    # 🔧 PyQt와 OpenCV 충돌 방지
    os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':0')
    
    try:
        # ROS2 초기화 (PyQt 앱 생성 전에!)
        rclpy.init(args=args)
        print("✅ ROS2 초기화 완료")
        
        # Payment GUI 생성 및 실행
        payment_gui = PaymentGUI()
        exit_code = payment_gui.run()
        
        return exit_code
        
    except KeyboardInterrupt:
        print("⚠️ 사용자가 프로그램을 중단했습니다.")
        return 0
    except Exception as e:
        print(f"❌ 프로그램 실행 중 오류: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # 안전한 종료 처리
        try:
            if 'payment_gui' in locals():
                payment_gui.cleanup_resources()
                print("✅ Payment GUI 리소스 정리 완료")
        except:
            pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
                print("✅ ROS2 종료 완료")
        except:
            pass


if __name__ == '__main__':
    sys.exit(main())