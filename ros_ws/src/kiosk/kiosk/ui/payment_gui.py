#!/usr/bin/env python3

# payment_gui.py 수정 - RFID 결제 통합
# 기존 코드에 RFID 관련 기능만 추가

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
try:
    from pyzbar import pyzbar
    print("✅ pyzbar 모듈 로드 성공")
except ImportError as e:
    print(f"❌ pyzbar 모듈 로드 실패: {e}")
    print("💡 sudo apt install python3-pyzbar를 실행하세요")
    sys.exit(1)

# ROS2 imports (RFID 토픽 구독용)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# 프로젝트 imports
try:
    from main_server.database.db_manager import DatabaseManager
    print("✅ DatabaseManager 모듈 로드 성공")
except ImportError as e:
    print(f"❌ DatabaseManager 모듈 로드 실패: {e}")
    print("💡 ROS2 워크스페이스가 빌드되었는지 확인하세요")
    sys.exit(1)

# ===== 기존 CameraWindow, BarcodeScannerThread, CartItem 클래스들은 동일 =====
# ... (기존 코드와 동일)

class PaymentGUI(QObject):
    """Payment GUI 메인 클래스 - RFID 결제 통합"""
    
    def __init__(self):
        super().__init__()
        
        # ROS2 노드 별도 생성 (RFID 토픽 구독용)
        self.ros_node = Node('payment_gui_rfid')
        
        # PyQt 애플리케이션 초기화
        self.app = QApplication.instance()
        if self.app is None:
            self.app = QApplication(sys.argv)
            print("✅ 새로운 PyQt 애플리케이션 생성")
        else:
            print("✅ 기존 PyQt 애플리케이션 사용")
        
        # UI 로드 (기존 코드와 동일)
        self.load_ui()
        
        # 데이터베이스 초기화
        self.db_manager = DatabaseManager()
        
        # 바코드 스캐너 초기화 (기존 코드와 동일)
        self.barcode_scanner = None
        self.scanning = False
        self.camera_window = None
        
        # 장바구니 관리 (기존 코드와 동일)
        self.cart = []
        self.total_amount = 0
        self.current_book_info = None
        
        # ===== RFID 관련 새로운 기능 =====
        self.setup_rfid_subscribers()
        self.rfid_payment_active = False
        
        # ROS2 스핀을 위한 타이머 설정
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros_node)
        self.ros_timer.start(100)  # 100ms 간격
        
        # UI 초기화 및 시그널 연결 (기존 코드와 동일)
        self.setup_ui()
        self.connect_signals()
        
        print("✅ Payment GUI + RFID 초기화 완료")
    
    def setup_rfid_subscribers(self):
        """RFID 관련 ROS2 구독자 설정"""
        try:
            # RFID 결제 데이터 구독
            self.rfid_subscription = self.ros_node.create_subscription(
                String,
                'rfid_payment',
                self.rfid_payment_callback,
                10
            )
            
            # RFID 결제 상태 구독 (선택사항)
            self.rfid_status_subscription = self.ros_node.create_subscription(
                Bool,
                'payment_status',
                self.rfid_status_callback,
                10
            )
            
            # 결제 결과 발행자 생성 (ESP32로 결과 전송)
            self.payment_result_publisher = self.ros_node.create_publisher(
                String,
                'payment_result',
                10
            )
            
            print("✅ RFID 구독자/발행자 설정 완료")
            print("📡 토픽 구독: /rfid_payment, /payment_status")
            print("📡 토픽 발행: /payment_result")
            
        except Exception as e:
            print(f"❌ RFID 구독자 설정 실패: {e}")
    
    def rfid_payment_callback(self, msg):
        """RFID 결제 데이터 수신 콜백"""
        try:
            rfid_data = msg.data
            print(f"📱 RFID 결제 데이터 수신: {rfid_data}")
            
            # 데이터 파싱 (예: "CARD_ID:A1B2C3D4,TIMESTAMP:12345")
            if "CARD_ID:" in rfid_data:
                card_id = rfid_data.split("CARD_ID:")[1].split(",")[0]
                print(f"💳 카드 ID: {card_id}")
                
                # 즉시 결제 처리
                QTimer.singleShot(100, self.process_rfid_payment)
            
        except Exception as e:
            print(f"❌ RFID 데이터 처리 오류: {e}")
            self.send_payment_result("FAILED:Data processing error")
    
    def rfid_status_callback(self, msg):
        """RFID 결제 상태 수신 콜백"""
        self.rfid_payment_active = msg.data
        if self.rfid_payment_active:
            print("⏳ RFID 결제 진행 중...")
        else:
            print("✅ RFID 결제 완료")
    
    def process_rfid_payment(self):
        """RFID 카드 감지 시 즉시 결제 처리"""
        print("🚀 RFID 즉시 결제 처리 시작!")
        
        # 장바구니가 비어있는지 확인
        if not self.cart:
            print("❌ 결제할 상품이 없습니다.")
            QMessageBox.warning(self.dialog, "결제 오류", "장바구니가 비어있습니다.\n먼저 상품을 추가해주세요.")
            self.send_payment_result("FAILED:Empty cart")
            return
        
        # 결제 전 재고 재확인
        if not self.validate_cart_stock():
            print("❌ 재고 부족으로 결제 실패")
            self.send_payment_result("FAILED:Insufficient stock")
            return
        
        try:
            # 즉시 재고 감소 처리
            if self.update_stock_quantities():
                print("✅ RFID 결제 성공!")
                
                # 성공 결과 ESP32로 전송
                success_msg = f"SUCCESS:Payment completed for {len(self.cart)} items, Total: ₩{self.total_amount:,}"
                self.send_payment_result(success_msg)
                
                # 성공 팝업 표시
                self.show_rfid_payment_success()
                
            else:
                print("❌ 재고 업데이트 실패")
                self.send_payment_result("FAILED:Stock update failed")
                
        except Exception as e:
            print(f"❌ RFID 결제 처리 중 오류: {e}")
            self.send_payment_result(f"FAILED:Processing error - {str(e)}")
    
    def send_payment_result(self, result_message: str):
        """결제 결과를 ESP32로 전송"""
        try:
            msg = String()
            msg.data = result_message
            self.payment_result_publisher.publish(msg)
            print(f"📡 결제 결과 전송: {result_message}")
        except Exception as e:
            print(f"❌ 결제 결과 전송 실패: {e}")
    
    def validate_cart_stock(self) -> bool:
        """장바구니 아이템들의 재고 재확인 (기존 코드와 동일)"""
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
    
    def update_stock_quantities(self) -> bool:
        """데이터베이스 재고 수량 감소 (기존 코드와 동일)"""
        try:
            all_success = True
            
            for cart_item in self.cart:
                isbn = cart_item.isbn
                quantity = cart_item.quantity
                
                # DB에서 현재 재고 확인 후 감소
                success = self.decrease_book_stock(isbn, quantity)
                if success:
                    print(f"✅ 재고 감소: {cart_item.title} (-{quantity}권)")
                else:
                    print(f"❌ 재고 감소 실패: {cart_item.title}")
                    all_success = False
            
            return all_success
            
        except Exception as e:
            print(f"❌ 재고 감소 중 오류: {e}")
            return False
    
    def decrease_book_stock(self, isbn: str, quantity: int) -> bool:
        """데이터베이스 재고 감소 (기존 코드와 동일)"""
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
    
    def show_rfid_payment_success(self):
        """RFID 결제 완료 팝업 (기존 success popup 수정)"""
        try:
            # 커스텀 다이얼로그 생성
            success_dialog = QDialog(self.dialog)
            success_dialog.setWindowTitle("💳 RFID 결제 완료")
            success_dialog.setFixedSize(600, 400)
            success_dialog.setWindowFlags(Qt.Dialog | Qt.WindowTitleHint)
            
            # 다이얼로그 스타일 설정
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
            layout.setSpacing(25)
            layout.setContentsMargins(40, 40, 40, 40)
            
            # 성공 아이콘 + 제목
            title_label = QLabel("🎉 RFID 결제가 완료되었습니다!")
            title_label.setAlignment(Qt.AlignCenter)
            title_label.setStyleSheet("""
                QLabel {
                    font-size: 24px;
                    font-weight: bold;
                    color: #28a745;
                    padding: 15px;
                }
            """)
            layout.addWidget(title_label)
            
            # 결제 정보 상세
            info_text = f"""
💳 결제 방식: RFID 카드 결제
📚 구매 도서: {len(self.cart)}권
💰 총 결제 금액: ₩{self.total_amount:,}

📦 재고가 자동으로 업데이트되었습니다.
이용해 주셔서 감사합니다! 📖
            """
            
            info_label = QLabel(info_text)
            info_label.setAlignment(Qt.AlignCenter)
            info_label.setStyleSheet("""
                QLabel {
                    font-size: 16px;
                    color: #2c3e50;
                    background: white;
                    padding: 25px;
                    border-radius: 12px;
                    border: 2px solid #71866a;
                    line-height: 1.6;
                }
            """)
            layout.addWidget(info_label)
            
            # 카운트다운 라벨
            countdown_label = QLabel("5초 후 메인 화면으로 돌아갑니다...")
            countdown_label.setAlignment(Qt.AlignCenter)
            countdown_label.setStyleSheet("""
                QLabel {
                    font-size: 18px;
                    font-weight: bold;
                    color: #71866a;
                    padding: 12px;
                }
            """)
            layout.addWidget(countdown_label)
            
            # 다이얼로그 중앙 정렬
            success_dialog.move(
                self.dialog.x() + (self.dialog.width() - success_dialog.width()) // 2,
                self.dialog.y() + (self.dialog.height() - success_dialog.height()) // 2
            )
            
            # 카운트다운 타이머 설정
            countdown_seconds = [5]
            
            countdown_timer = QTimer()
            
            def update_countdown():
                if countdown_seconds[0] > 0:
                    countdown_label.setText(f"{countdown_seconds[0]}초 후 메인 화면으로 돌아갑니다...")
                    countdown_seconds[0] -= 1
                else:
                    countdown_timer.stop()
                    success_dialog.close()
                    # 장바구니 초기화 후 메인 윈도우로 복귀
                    QTimer.singleShot(200, self.return_to_main_after_rfid_payment)
            
            countdown_timer.timeout.connect(update_countdown)  
            countdown_timer.start(1000)  # 1초마다 업데이트
            
            # 팝업창 표시 (모달)
            success_dialog.exec_()
            
        except Exception as e:
            print(f"❌ RFID 성공 팝업창 표시 중 오류: {e}")
            # 오류 시 기본 방식으로 처리
            QMessageBox.information(
                self.dialog,
                "💳 RFID 결제 완료",
                f"RFID 결제가 완료되었습니다!\n총 결제 금액: ₩{self.total_amount:,}\n\n5초 후 메인화면으로 돌아갑니다."
            )
            # 5초 후 메인 윈도우로 복귀
            QTimer.singleShot(5000, self.return_to_main_after_rfid_payment)
    
    def return_to_main_after_rfid_payment(self):
        """RFID 결제 후 메인 윈도우로 복귀"""
        try:
            print("🏠 RFID 결제 완료 - 메인 윈도우로 복귀")
            
            # 장바구니 초기화
            self.cart.clear()
            self.total_amount = 0
            self.update_cart_display()
            self.update_total_display()
            self.update_payment_button_state()
            
            # 검색 결과 초기화
            if hasattr(self.dialog, 'search_results_list'):
                self.dialog.search_results_list.clear()
            if hasattr(self.dialog, 'book_title_input'):
                self.dialog.book_title_input.clear()
            
            print("✅ 장바구니 및 UI 초기화 완료")
            
            # 메인 윈도우로 복귀
            self.return_to_main_window()
            
        except Exception as e:
            print(f"❌ RFID 결제 후 복귀 중 오류: {e}")
            # 오류 시에도 메인 윈도우로 복귀 시도
            self.return_to_main_window()
    
    def spin_ros_node(self):
        """안전한 ROS2 노드 스핀 처리"""
        try:
            if rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.0)
        except Exception as e:
            # RCL 오류 무시 (프로그램 종료 시 정상적인 오류)
            if "not valid" not in str(e):
                print(f"❌ ROS2 스핀 오류: {e}")
    
    # ===== 기존 함수들 (동일하게 유지) =====
    def load_ui(self):
        """UI 파일 로드 (기존 코드와 동일)"""
        # ... 기존 UI 로드 코드
        pass
    
    def setup_ui(self):
        """UI 초기 설정 (기존 코드와 동일)"""
        # ... 기존 UI 설정 코드
        pass
    
    def connect_signals(self):
        """시그널-슬롯 연결 (기존 코드와 동일)"""
        # ... 기존 시그널 연결 코드
        pass
    
    # ... 기타 기존 함수들 (search_book, add_to_cart, etc.)
    
    def cleanup_resources(self):
        """리소스 정리 (RFID 구독자 추가)"""
        try:
            # 바코드 스캐너 정리
            if self.scanning:
                self.stop_barcode_scan()
            
            # ROS2 구독자/발행자 정리
            if hasattr(self, 'rfid_subscription'):
                self.ros_node.destroy_subscription(self.rfid_subscription)
            if hasattr(self, 'rfid_status_subscription'):
                self.ros_node.destroy_subscription(self.rfid_status_subscription)
            if hasattr(self, 'payment_result_publisher'):
                self.ros_node.destroy_publisher(self.payment_result_publisher)
            
            # ROS2 노드 정리
            if hasattr(self, 'ros_node'):
                self.ros_node.destroy_node()
            
            # ROS2 타이머 정리
            if hasattr(self, 'ros_timer') and self.ros_timer.isActive():
                self.ros_timer.stop()
            
            print("✅ RFID 리소스 정리 완료")
        except Exception as e:
            print(f"❌ 리소스 정리 중 오류: {e}")


def main(args=None):
    """메인 함수 - RFID 통합"""
    print("🚀 Payment GUI + RFID 시작...")
    
    # Qt 환경변수 설정
    import os
    os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':0')
    
    # ROS2 환경 확인
    ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
    ros_version = os.environ.get('ROS_VERSION', 'unknown')
    print(f"🔧 ROS2 환경: {ros_distro} {ros_version}")
    
    try:
        # ROS2 초기화
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
                print("✅ Payment GUI + RFID 리소스 정리 완료")
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