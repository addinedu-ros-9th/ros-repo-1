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
import imutils

# ROS2 imports 
import rclpy
from rclpy.node import Node

# 프로젝트 imports (기존 구조와 동일)
from main_server.database.db_manager import DatabaseManager
from main_server.api.aladin_client import AladinApiClient
from main_server.parser.book_parser import BookParser

class BarcodeScanner(QObject):
    """바코드 스캔 기능을 담당하는 클래스"""
    barcode_detected = pyqtSignal(str)  # 바코드 감지 시그널
    scanning_stopped = pyqtSignal()      # 스캔 중지 시그널
    
    def __init__(self):
        super().__init__()
        self.cap = None
        self.scanning = False
        self.last_scan_time = 0
        self.scan_cooldown = 3.0  # 3초 쿨다운
    
    def start_scanning(self):
        """바코드 스캔 시작"""
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("❌ 카메라를 열 수 없습니다.")
                return False
            
            self.scanning = True
            print("📱 바코드 스캔을 시작합니다. 'q'를 눌러 종료하세요.")
            
            while self.scanning:
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                # 프레임 리사이즈 (성능 최적화)
                frame = imutils.resize(frame, width=400)
                
                # 바코드 검출
                barcodes = pyzbar.decode(frame)
                
                for barcode in barcodes:
                    # 쿨다운 체크
                    current_time = time.time()
                    if current_time - self.last_scan_time < self.scan_cooldown:
                        continue
                    
                    # 바코드 데이터 추출
                    barcode_data = barcode.data.decode('utf-8')
                    barcode_type = barcode.type
                    
                    print(f"✅ 바코드 감지: {barcode_data} (타입: {barcode_type})")
                    
                    # 바코드 영역 표시
                    (x, y, w, h) = barcode.rect
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # 바코드 정보 표시
                    text = f"{barcode_data} ({barcode_type})"
                    cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # 시그널 발생
                    self.barcode_detected.emit(barcode_data)
                    self.last_scan_time = current_time
                
                # 화면 표시
                cv2.putText(frame, "Barcode Scanner - Press 'q' to quit", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow("Payment - Barcode Scanner", frame)
                
                # 'q' 키로 종료
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            return True
            
        except Exception as e:
            print(f"❌ 바코드 스캔 오류: {e}")
            return False
        finally:
            self.stop_scanning()
    
    def stop_scanning(self):
        """바코드 스캔 중지"""
        self.scanning = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        self.scanning_stopped.emit()
        print("📱 바코드 스캔을 중지했습니다.")

class BookSearchThread(QThread):
    """도서 검색 스레드 (Stock과 동일한 구조)"""
    finished = pyqtSignal(list)  # 검색 완료 시그널
    
    def __init__(self, aladin_client: AladinApiClient, isbn: str):
        super().__init__()
        self.aladin_client = aladin_client
        self.isbn = isbn
    
    def run(self):
        """검색 실행"""
        try:
            results = self.aladin_client.search_by_isbn(self.isbn)
            self.finished.emit(results if results else [])
        except Exception as e:
            print(f"❌ 도서 검색 오류: {e}")
            self.finished.emit([])

class CartItem:
    """장바구니 아이템 클래스"""
    def __init__(self, book_info: dict, quantity: int = 1):
        self.book_info = book_info
        self.quantity = quantity
        self.title = book_info.get('title', 'N/A')
        self.price = int(book_info.get('priceSales', book_info.get('price', 0)))
        self.isbn = book_info.get('isbn13', book_info.get('isbn', ''))
        self.total_price = self.price * self.quantity
    
    def update_quantity(self, quantity: int):
        """수량 업데이트"""
        self.quantity = quantity
        self.total_price = self.price * self.quantity

class PaymentGUI(Node):
    """Payment GUI 메인 클래스"""
    
    def __init__(self):
        super().__init__('payment_gui')
        
        # PyQt 애플리케이션 초기화
        self.app = QApplication(sys.argv)
        
        # UI 로드
        ui_file = os.path.join(os.path.dirname(__file__), 'ui_files', 'payment_window.ui')
        self.dialog = uic.loadUi(ui_file)
        
        # 데이터베이스 및 API 클라이언트 초기화 (기존 구조와 동일)
        self.db_manager = DatabaseManager()
        self.aladin_client = AladinApiClient()
        self.parser = BookParser()
        
        # 바코드 스캐너 초기화
        self.barcode_scanner = BarcodeScanner()
        self.scanner_thread = None
        
        # 장바구니 관리
        self.cart: List[CartItem] = []
        self.total_amount = 0
        
        # 현재 상태
        self.current_book_info = None
        self.search_thread = None
        self.scanning_active = False
        
        # UI 초기화
        self.setup_ui()
        self.connect_signals()
        
        print("✅ Payment GUI 초기화 완료")
    
    def setup_ui(self):
        """UI 초기 설정"""
        # 윈도우 설정
        self.dialog.setWindowTitle("LIBO - Payment System 💳")
        self.dialog.setWindowIcon(QIcon("📚"))
        
        # 장바구니 테이블 설정
        self.dialog.cart_table.setColumnWidth(0, 180)  # Book 
        self.dialog.cart_table.setColumnWidth(1, 50)   # Qty
        self.dialog.cart_table.setColumnWidth(2, 80)   # Price
        self.dialog.cart_table.setColumnWidth(3, 80)   # Total
        
        # 초기 상태 설정
        self.update_total_display()
        self.update_payment_button_state()
        
        print("🎨 UI 설정 완료")
    
    def connect_signals(self):
        """시그널-슬롯 연결"""
        # 버튼 시그널 연결
        self.dialog.back_button.clicked.connect(self.close_window)
        self.dialog.search_button.clicked.connect(self.search_book)
        self.dialog.qr_scan_button.clicked.connect(self.start_barcode_scan)
        self.dialog.add_to_cart_button.clicked.connect(self.add_to_cart)
        self.dialog.remove_item_button.clicked.connect(self.remove_selected_item)
        self.dialog.clear_cart_button.clicked.connect(self.clear_cart)
        self.dialog.payment_button.clicked.connect(self.process_payment)
        
        # 검색 결과 리스트 시그널 연결
        self.dialog.search_results_list.itemClicked.connect(self.on_book_selected)
        
        # 바코드 스캐너 시그널 연결
        self.barcode_scanner.barcode_detected.connect(self.on_barcode_detected)
        self.barcode_scanner.scanning_stopped.connect(self.on_scanning_stopped)
        
        # 장바구니 테이블 시그널 연결
        self.dialog.cart_table.itemSelectionChanged.connect(self.on_cart_selection_changed)
        
        print("🔗 시그널-슬롯 연결 완료")
    
    def start_barcode_scan(self):
        """바코드 스캔 시작"""
        if self.scanning_active:
            return
        
        self.scanning_active = True
        self.dialog.qr_scan_button.setText("📱 스캔 중...")
        self.dialog.qr_scan_button.setEnabled(False)
        
        # 메시지 표시
        QMessageBox.information(
            self.dialog, 
            "바코드 스캔", 
            "📱 바코드를 카메라에 대주세요!\n\n" +
            "• 도서의 바코드를 화면에 비춰주세요\n" +
            "• 바코드가 인식되면 자동으로 검색됩니다\n" +
            "• 'q' 키를 눌러 스캔을 종료할 수 있습니다"
        )
        
        # 바코드 스캔 스레드 시작
        self.scanner_thread = Thread(target=self.barcode_scanner.start_scanning)
        self.scanner_thread.start()
        
        print("📱 바코드 스캔을 시작합니다...")
    
    @pyqtSlot(str)
    def on_barcode_detected(self, barcode_data: str):
        """바코드 감지 처리"""
        print(f"📱 바코드 감지됨: {barcode_data}")
        
        # ISBN으로 자동 검색
        self.dialog.book_title_input.setText(barcode_data)
        self.search_book_by_isbn(barcode_data)
        
        # 스캔 자동 중지
        self.barcode_scanner.stop_scanning()
    
    @pyqtSlot()
    def on_scanning_stopped(self):
        """스캔 중지 처리"""
        self.scanning_active = False
        self.dialog.qr_scan_button.setText("📱 QR Scan")
        self.dialog.qr_scan_button.setEnabled(True)
        print("📱 바코드 스캔이 중지되었습니다.")
    
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
        """ISBN으로 도서 검색"""
        if self.search_thread and self.search_thread.isRunning():
            return
        
        self.dialog.search_button.setEnabled(False)
        self.dialog.search_results_list.clear()
        
        print(f"🔍 ISBN으로 검색 중: {isbn}")
        
        # 먼저 DB에서 검색
        db_result = self.db_manager.get_book_by_isbn(isbn)
        if db_result:
            print("✅ DB에서 도서 발견")
            self.display_search_results([db_result], from_db=True)
            self.dialog.search_button.setEnabled(True)
            return
        
        # DB에 없으면 API에서 검색
        print("🌐 API에서 검색 중...")
        self.search_thread = BookSearchThread(self.aladin_client, isbn)
        self.search_thread.finished.connect(self.on_search_finished)
        self.search_thread.start()
    
    def search_book_by_title(self, title: str):
        """제목으로 도서 검색 (DB에서)"""
        books = self.db_manager.search_books(title, 'title')
        self.display_search_results(books, from_db=True)
        print(f"🔍 제목으로 검색: {title}")
    
    @pyqtSlot(list)
    def on_search_finished(self, results: list):
        """검색 완료 처리"""
        self.dialog.search_button.setEnabled(True)
        self.display_search_results(results, from_db=False)
    
    def display_search_results(self, results: list, from_db: bool = False):
        """검색 결과 표시"""
        self.dialog.search_results_list.clear()
        
        if not results:
            print("❌ 검색 결과가 없습니다.")
            QMessageBox.information(self.dialog, "검색 결과", "검색 결과가 없습니다.")
            return
        
        for book in results:
            if from_db:
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
            else:
                # API 결과 표시  
                title = book.get('title', 'N/A')
                author = book.get('author', 'N/A')
                price = book.get('priceSales', 0)
                item_text = f"🌐 {title} - {author} (₩{price:,}) [온라인]"
            
            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, book)
            
            # 재고 없는 경우 비활성화 표시
            if from_db and book.get('stock_quantity', 0) <= 0:
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
                self.dialog.add_to_cart_button.setEnabled(False)
                return
        
        print(f"📚 도서 선택됨: {book_title}")
        
        # 추가 버튼 활성화
        self.dialog.add_to_cart_button.setEnabled(True)
    
    def add_to_cart(self):
        """장바구니에 추가"""
        if not self.current_book_info:
            QMessageBox.warning(self.dialog, "경고", "추가할 도서를 선택해주세요.")
            return
        
        quantity = self.dialog.quantity_spinbox.value()
        isbn = self.current_book_info.get('isbn13', self.current_book_info.get('isbn', ''))
        
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
    
    def update_total_display(self):
        """총합 표시 업데이트"""
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
    
    def update_payment_button_state(self):
        """결제 버튼 상태 업데이트"""
        has_items = len(self.cart) > 0
        self.dialog.payment_button.setEnabled(has_items)
        
        if has_items:
            self.dialog.payment_button.setText(f"💳 Pay Now (₩{self.total_amount:,})")
        else:
            self.dialog.payment_button.setText("💳 Pay Now")
    
    def remove_selected_item(self):
        """선택된 아이템 제거"""
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
        has_selection = self.dialog.cart_table.currentRow() >= 0
        self.dialog.remove_item_button.setEnabled(has_selection)
    
    def process_payment(self):
        """결제 처리"""
        if not self.cart:
            QMessageBox.warning(self.dialog, "경고", "결제할 상품이 없습니다.")
            return
        
        # 임시로 결제 완료 처리 (나중에 RFID로 대체)
        self.simulate_payment_success()
        
        print(f"💳 결제 완료! 총액: ₩{self.total_amount:,}")
    
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
        """데이터베이스 재고 감소 (db_manager.py 참고)"""
        if not self.db_manager.connection:
            print("❌ 데이터베이스 연결이 없습니다.")
            return False
        
        try:
            import pymysql
            
            with self.db_manager.connection.cursor(pymysql.cursors.DictCursor) as cursor:
                # 현재 재고 확인
                cursor.execute("SELECT id, stock_quantity FROM book WHERE isbn = %s", (isbn,))
                book = cursor.fetchone()
                
                if not book:
                    print(f"❌ 도서를 찾을 수 없습니다: ISBN {isbn}")
                    return False
                
                current_stock = book['stock_quantity']
                
                # 재고 부족 확인
                if current_stock < quantity:
                    print(f"❌ 재고 부족: 현재 {current_stock}권, 요청 {quantity}권")
                    return False
                
                # 재고 감소
                new_stock = current_stock - quantity
                update_sql = "UPDATE book SET stock_quantity = %s WHERE isbn = %s"
                cursor.execute(update_sql, (new_stock, isbn))
                
                print(f"✅ 재고 감소 성공: ISBN {isbn}")
                print(f"   기존 재고: {current_stock}권 → 새로운 재고: {new_stock}권")
                
                return True
                
        except Exception as e:
            print(f"❌ 재고 감소 실패: {e}")
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
            if self.scanning_active:
                self.barcode_scanner.stop_scanning()
            
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
            # PyQt5에서 부모 윈도우 찾기
            parent_window = None
            for widget in QApplication.topLevelWidgets():
                if hasattr(widget, 'objectName') and 'MainWindow' in str(type(widget)):
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
        if self.scanning_active:
            self.barcode_scanner.stop_scanning()
        
        # 메인 윈도우로 복귀
        self.show_and_refresh_main_window()
        
        # Payment 윈도우 닫기
        self.dialog.close()
        print("👋 Payment 창을 닫습니다.")
    
    def run(self):
        """GUI 실행"""
        self.dialog.show()
        
        # 윈도우 중앙 정렬
        self.center_payment_window()
        
        return self.app.exec_()
    
    def center_payment_window(self):
        """Payment 윈도우 중앙 정렬"""
        screen = QApplication.desktop().screenGeometry()
        
        window_width = 1000
        window_height = 700
        
        center_x = (screen.width() - window_width) // 2
        center_y = (screen.height() - window_height) // 2
        
        self.dialog.setGeometry(center_x, center_y, window_width, window_height)
        print(f"✅ Payment 윈도우 중앙 정렬: ({center_x}, {center_y})")

def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    payment_gui = PaymentGUI()
    
    try:
        exit_code = payment_gui.run()
        return exit_code
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"❌ GUI 실행 중 오류: {e}")
    finally:
        if 'payment_gui' in locals():
            payment_gui.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()