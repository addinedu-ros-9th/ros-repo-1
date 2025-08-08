#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
# Explicit imports for linters/static checks
from PyQt5.QtWidgets import QMainWindow, QMessageBox, QInputDialog, QApplication
from PyQt5.QtCore import QTimer, QSettings, Qt
import rclpy
from rclpy.node import Node

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # ROS2 노드 초기화
        if not rclpy.ok():
            rclpy.init()
        self.ros_node = Node('kiosk_main_window')
        
        # Kiosk 위치 상태 (기본값 E9) - QSettings에서 복원
        self.kiosk_location_id = self._load_kiosk_location_from_settings()

        self.book_search_widget = None  # 책 검색 위젯 참조
        self.qr_check_client = None  # QR 체크 클라이언트
        self.task_request_client = None  # 태스크 요청 클라이언트
        self.admin_authenticated = False  # 관리자 인증 상태
        self.call_robot_timer = None  # Call Robot 버튼 타이머
        
        # 🔧 Payment GUI 중복 실행 방지 플래그
        self.payment_gui_running = False
        self.payment_process = None  # Payment GUI 프로세스 참조
        
        self.init_ui()
        self.setup_connections()
        self.init_ros_clients()
        
        print("✅ MainWindow 초기화 완료")
    
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
        
        self.setWindowTitle("LIBO BOOK STORE")
        
        # Call Robot 버튼 초기 상태 설정 (숨김)
        self.call_manager.setVisible(False)
        
        # Kiosk Settings 버튼 초기 상태 설정 (숨김)
        if hasattr(self, 'kiosk_settings'):
            self.kiosk_settings.setVisible(False)
        
        # 윈도우 크기 설정 (새로운 디자인에 맞게)
        self.resize(1200, 800)
        
        # 윈도우를 항상 최상위에 유지
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)

        # Kiosk 설정 버튼 라벨에 현재 위치 표시
        if hasattr(self, 'kiosk_settings'):
            self._update_kiosk_settings_button_label()
        
        print("✅ 메인 윈도우 UI 로드 완료")
    
    def setup_connections(self):
        """시그널-슬롯 연결"""
        # 기존 연결 해제 후 다시 연결 (중복 방지)
        try:
            self.book_search.clicked.disconnect()
            self.call_manager.clicked.disconnect()
            self.payment.clicked.disconnect()
            self.book_corner.clicked.disconnect()
            self.qr_scan_button.clicked.disconnect()
            if hasattr(self, 'kiosk_settings'):
                self.kiosk_settings.clicked.disconnect()
        except:
            pass
        
        self.book_search.clicked.connect(self.on_book_search_clicked)
        self.call_manager.clicked.connect(self.on_call_robot_clicked)
        self.payment.clicked.connect(self.on_payment_clicked)
        self.book_corner.clicked.connect(self.on_book_corner_clicked)
        self.qr_scan_button.clicked.connect(self.on_qr_scan_clicked)
        if hasattr(self, 'kiosk_settings'):
            self.kiosk_settings.clicked.connect(self.kiosk_location_setting)
        
        print("✅ 메인 윈도우 시그널-슬롯 연결 완료")
    
    def init_ros_clients(self):
        """ROS2 클라이언트 초기화"""
        try:
            # QR 체크 클라이언트 초기화 (pyzbar 의존성 확인)
            try:
                from kiosk.ros_communication.kiosk_qr_check_client import KioskQRCheckClient
                self.qr_check_client = KioskQRCheckClient(self.ros_node)
                self.qr_check_client.qr_check_completed.connect(self.on_qr_check_completed)
                print("✅ QR 스캔 클라이언트 초기화 완료")
            except ImportError as e:
                print(f"⚠️ QR 스캔 기능 비활성화: {e}")
                print("💡 QR 스캔을 사용하려면 'pip install pyzbar opencv-python'을 실행하세요")
                self.qr_check_client = None
            
            # 태스크 요청 클라이언트 초기화
            from kiosk.ros_communication.task_request_client import TaskRequestClient
            self.task_request_client = TaskRequestClient()
            self.task_request_client.task_request_completed.connect(self.on_task_request_response)
            
            print("✅ ROS2 클라이언트 초기화 완료")
            
        except Exception as e:
            print(f"❌ ROS2 클라이언트 초기화 실패: {e}")
    
    def on_qr_scan_clicked(self):
        """QR 스캔 버튼 클릭"""
        print("🔍 QR 스캔 시작")
        
        if self.qr_check_client:
            self.qr_check_client.start_qr_scan()
        else:
            QMessageBox.warning(self, "QR 스캔 기능 비활성화", 
                              "QR 스캔 기능을 사용할 수 없습니다.\n\n"
                              "QR 스캔을 사용하려면 다음 명령을 실행하세요:\n"
                              "pip install pyzbar opencv-python")
    
    def on_qr_check_completed(self, success, message):
        """QR 체크 완료 처리"""
        if success:
            print(f"✅ QR 인증 성공: {message}")
            self.admin_authenticated = True
            
            # Call Robot 버튼과 Kiosk Settings 버튼 표시
            self.show_call_robot_button()
            
            QMessageBox.information(self, "QR 인증 성공", 
                                  f"관리자 인증이 완료되었습니다.\n{message}\n\nCall Robot 버튼과 Kiosk Settings 버튼이 10초간 표시됩니다.")
        else:
            print(f"❌ QR 인증 실패: {message}")
            self.admin_authenticated = False
            QMessageBox.warning(self, "QR 인증 실패", f"QR 인증에 실패했습니다.\n{message}")
    
    def show_call_robot_button(self):
        """Call Robot 버튼 표시 (10초간)"""
        self.call_manager.setVisible(True)
        
        # Kiosk Settings 버튼도 표시
        if hasattr(self, 'kiosk_settings'):
            self.kiosk_settings.setVisible(True)
        
        # 10초 후 버튼 숨기기
        if self.call_robot_timer:
            self.call_robot_timer.stop()
        
        self.call_robot_timer = QTimer()
        self.call_robot_timer.timeout.connect(self.hide_call_robot_button)
        self.call_robot_timer.start(10000)  # 10초
        
        print("🤖 Call Robot 버튼 및 Kiosk Settings 버튼 표시")
    
    def hide_call_robot_button(self):
        """Call Robot 버튼 숨기기"""
        self.call_manager.setVisible(False)
        
        # Kiosk Settings 버튼도 숨기기
        if hasattr(self, 'kiosk_settings'):
            self.kiosk_settings.setVisible(False)
        
        self.admin_authenticated = False
        
        if self.call_robot_timer:
            self.call_robot_timer.stop()
        
        print("🤖 Call Robot 버튼 및 Kiosk Settings 버튼 숨김")
    
    def on_call_robot_clicked(self):
        """Call Robot 버튼 클릭 (관리자용 로봇 호출)"""
        if not self.admin_authenticated:
            QMessageBox.warning(self, "인증 필요", "먼저 QR 인증을 완료해주세요.")
            return
        
        print("🤖 관리자용 로봇 호출 요청")
        
        try:
            # TaskRequest.srv 파라미터 준비
            robot_id = ""  # task_manager에서 자동 선택
            task_type = "assist"
            call_location = getattr(self, 'kiosk_location_id', 'E9')  # 선택된 키오스크 위치
            goal_location = ""  # 어시스트 임무는 목적지 없음
            
            print(f"📍 TaskRequest 파라미터:")
            print(f"   robot_id: '{robot_id}' (task_manager에서 자동 선택)")
            print(f"   task_type: {task_type}")
            print(f"   call_location: {call_location} (키오스크)")
            print(f"   goal_location: '{goal_location}' (어시스트는 목적지 없음)")
            
            # Main Server의 task_manager.py로 TaskRequest 서비스 호출
            self.task_request_client.send_task_request(robot_id, task_type, call_location, goal_location)
            
            # 버튼 비활성화
            self.call_manager.setEnabled(False)
            self.call_manager.setText("요청 중...")
            
        except Exception as e:
            print(f"❌ 로봇 호출 요청 중 오류: {e}")
            QMessageBox.warning(self, "오류", f"로봇 호출 요청 중 오류가 발생했습니다: {str(e)}")
            
            # 버튼 재활성화
            self.call_manager.setEnabled(True)
            self.call_manager.setText("🤖 Call Robot")
    
    def on_task_request_response(self, success, message):
        """태스크 요청 응답 처리"""
        # 버튼 재활성화
        self.call_manager.setEnabled(True)
        self.call_manager.setText("🤖 Call Robot")
        
        if success:
            print(f"✅ 로봇 호출 성공: {message}")
            QMessageBox.information(self, "로봇 호출 성공", 
                                  f"로봇 호출이 성공했습니다.\n{message}\n\n로봇이 키오스크로 이동 중입니다.")
            
            # Call Robot 버튼 숨기기
            self.hide_call_robot_button()
        else:
            print(f"❌ 로봇 호출 실패: {message}")
            QMessageBox.warning(self, "로봇 호출 실패", 
                              f"로봇 호출에 실패했습니다.\n{message}")

    def center_window(self):
        """윈도우를 화면 중앙에 위치시키기"""
        # 화면의 사용 가능한 영역 가져오기
        screen = QApplication.desktop().screenGeometry()
        
        # 윈도우의 크기 가져오기 (새로운 디자인에 맞게)
        window_width = 1200
        window_height = 800
        
        # 중앙 좌표 계산
        center_x = (screen.width() - window_width) // 2
        center_y = (screen.height() - window_height) // 2
        
        # 윈도우 크기와 위치를 설정
        self.setGeometry(center_x, center_y, window_width, window_height)
        
        print(f"✅ 윈도우 중앙 정렬 완료: ({center_x}, {center_y})")
        print(f"화면 크기: {screen.width()}x{screen.height()}, 윈도우 크기: {window_width}x{window_height}")
        print(f"실제 위치: {self.pos().x()}, {self.pos().y()}")
    
    def force_center_window(self):
        """강제로 윈도우를 화면 중앙에 위치시키기"""
        # 화면의 사용 가능한 영역 가져오기
        screen = QApplication.desktop().screenGeometry()
        
        # 윈도우 크기 설정 (새로운 디자인 크기)
        window_width = 1200
        window_height = 800
        
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
            # 현재 설정된 키오스크 위치 주입
            if hasattr(self.book_search_widget, 'set_kiosk_location'):
                self.book_search_widget.set_kiosk_location(getattr(self, 'kiosk_location_id', 'E9'))
            # 홈 버튼 시그널 연결 (한 번만)
            self.book_search_widget.home_requested.connect(self.show_main_window)
        else:
            # 기존 위젯이 있으면 초기화
            self.book_search_widget.reset_widget()
            if hasattr(self.book_search_widget, 'set_kiosk_location'):
                self.book_search_widget.set_kiosk_location(getattr(self, 'kiosk_location_id', 'E9'))
        
        # 현재 윈도우 숨기고 책 검색 윈도우 표시
        self.hide()
        self.book_search_widget.show()
    
    def on_payment_clicked(self):
        """Payment 버튼 클릭"""
        print("💳 결제 화면으로 전환")
        
        # 🔧 기존 Payment GUI 프로세스 종료
        if self.payment_gui_running and hasattr(self, 'payment_process') and self.payment_process:
            try:
                print("🔄 기존 Payment GUI 프로세스 종료 중...")
                self.payment_process.terminate()
                self.payment_process.wait(timeout=3)  # 3초 대기
                print("✅ 기존 Payment GUI 프로세스 종료 완료")
            except subprocess.TimeoutExpired:
                print("⚠️ 프로세스 종료 시간 초과, 강제 종료")
                self.payment_process.kill()
            except Exception as e:
                print(f"❌ 프로세스 종료 중 오류: {e}")
            
            self.payment_gui_running = False
            self.payment_process = None
        
        # 🔧 시스템에서 실행 중인 다른 payment_gui 프로세스 확인 및 종료
        try:
            import subprocess
            import time
            
            # 모든 payment_gui.py 프로세스 찾기
            result = subprocess.run(['pgrep', '-f', 'payment_gui.py'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:  # 프로세스가 실행 중
                pids = result.stdout.strip().split('\n')
                print(f"🔄 발견된 Payment GUI 프로세스: {pids}")
                
                for pid in pids:
                    if pid and pid != str(os.getpid()):  # 현재 프로세스 제외
                        try:
                            print(f"🔄 Payment GUI 프로세스 종료 중: PID {pid}")
                            # 먼저 SIGTERM으로 정상 종료 시도
                            subprocess.run(['kill', '-TERM', pid], timeout=2)
                            time.sleep(0.5)  # 0.5초 대기
                            
                            # 프로세스가 여전히 실행 중인지 확인
                            try:
                                subprocess.run(['kill', '-0', pid], check=True, timeout=1)
                                print(f"⚠️ 프로세스 {pid}가 여전히 실행 중, 강제 종료")
                                subprocess.run(['kill', '-KILL', pid], timeout=2)
                            except subprocess.CalledProcessError:
                                print(f"✅ 프로세스 {pid} 정상 종료됨")
                            except subprocess.TimeoutExpired:
                                print(f"⚠️ 프로세스 {pid} 종료 시간 초과")
                        except Exception as e:
                            print(f"❌ 프로세스 {pid} 종료 중 오류: {e}")
                
                # 모든 프로세스가 종료될 때까지 잠시 대기
                time.sleep(1)
                
                # 다시 한 번 확인하여 모든 프로세스가 종료되었는지 확인
                result = subprocess.run(['pgrep', '-f', 'payment_gui.py'], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    remaining_pids = result.stdout.strip().split('\n')
                    print(f"⚠️ 여전히 실행 중인 프로세스: {remaining_pids}")
                else:
                    print("✅ 모든 Payment GUI 프로세스가 종료됨")
                    
        except Exception as e:
            print(f"❌ 시스템 프로세스 확인 중 오류: {e}")
        
        try:
            # 현재 메인 윈도우 숨기기
            self.hide()
            
            # Payment GUI 실행 (별도 프로세스로)
            import subprocess
            import sys
            
            # payment_gui.py 파일 경로 (kiosk 패키지 내)
            payment_script = os.path.join(os.path.dirname(__file__), 'payment_gui.py')
            
            if os.path.exists(payment_script):
                # 🔧 중복 실행 방지 플래그 설정
                self.payment_gui_running = True
                
                # 환경변수 설정
                env = os.environ.copy()
                
                # ROS2 환경 설정 - 현재 시스템 환경변수 사용
                ros_ws_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))  # ros_ws
                # env['ROS_DISTRO']와 env['ROS_VERSION']은 현재 시스템 환경에서 자동으로 설정됨
                
                # Python 경로 설정
                src_path = os.path.join(ros_ws_path, 'src')
                if src_path not in env.get('PYTHONPATH', ''):
                    env['PYTHONPATH'] = os.pathsep.join([
                        src_path,
                        env.get('PYTHONPATH', '')
                    ])
                
                # ROS2 setup.bash 소싱 효과를 위한 환경변수
                install_path = os.path.join(ros_ws_path, 'install')
                if os.path.exists(install_path):
                    env['AMENT_PREFIX_PATH'] = os.pathsep.join([
                        install_path,
                        env.get('AMENT_PREFIX_PATH', '')
                    ])
                
                print(f"🔧 Payment GUI 실행 환경:")
                print(f"   스크립트: {payment_script}")
                print(f"   Python 경로: {env.get('PYTHONPATH', '')[:100]}...")
                print(f"   ROS 환경: {env.get('ROS_DISTRO', 'N/A')} {env.get('ROS_VERSION', 'N/A')}")
                
                self.payment_process = subprocess.Popen(
                    [sys.executable, payment_script],
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    cwd=ros_ws_path  # 작업 디렉토리를 ros_ws로 설정
                )
                print("✅ Payment GUI 실행됨")
                
                # 🔧 프로세스 종료 감지를 위한 타이머 설정
                def check_payment_process():
                    if self.payment_process and self.payment_process.poll() is not None:  # 프로세스가 종료됨
                        self.payment_gui_running = False
                        self.payment_process = None
                        print("✅ Payment GUI 프로세스 종료됨")
                        # 메인 윈도우 다시 표시 및 리프레시
                        QTimer.singleShot(100, self.show_main_window_after_payment)
                    else:
                        # 프로세스가 아직 실행 중이면 다시 체크
                        QTimer.singleShot(1000, check_payment_process)
                
                # 1초마다 프로세스 상태 확인
                QTimer.singleShot(1000, check_payment_process)
                
            else:
                print(f"❌ Payment 스크립트를 찾을 수 없습니다: {payment_script}")
                QMessageBox.critical(self, "오류", "결제 시스템을 찾을 수 없습니다.")
                self.show()  # 메인 윈도우 다시 표시
            
        except Exception as e:
            print(f"❌ Payment 화면 전환 중 오류: {e}")
            QMessageBox.critical(self, "오류", f"결제 화면을 열 수 없습니다.\n{str(e)}")
            self.payment_gui_running = False  # 플래그 리셋
            self.payment_process = None
            self.show()  # 메인 윈도우 다시 표시
    
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
        # 현재 설정된 키오스크 위치 주입
        if hasattr(self.book_corner_widget, 'set_kiosk_location'):
            self.book_corner_widget.set_kiosk_location(getattr(self, 'kiosk_location_id', 'E9'))
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
        import rclpy
        
        if hasattr(self, 'book_corner_widget') and self.book_corner_widget:
            rclpy.spin_once(self.book_corner_widget, timeout_sec=0.0)
        
        # 메인 윈도우의 ROS2 노드도 스핀
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)
    
    def show_main_window(self):
        """메인 윈도우로 돌아오기"""
        print("🏠 메인 화면으로 돌아옴")
        
        # 책 검색 윈도우 숨기고 메인 윈도우 표시
        if self.book_search_widget:
            self.book_search_widget.hide()
        self.show()
        
        # 메인 윈도우 강제 중앙 정렬
        self.force_center_window()
    
    def show_main_window_after_payment(self):
        """결제 후 메인 윈도우 표시 및 리프레시"""
        print("🏠 결제 후 메인 윈도우 복귀")
        
        try:
            # 메인 윈도우 표시
            self.show()
            self.raise_()
            self.activateWindow()
            
            # 메인 윈도우 중앙 정렬
            self.force_center_window()
            
            # 메인 윈도우 리프레시 (UI 상태 초기화)
            self.refresh_main_window()
            
            print("✅ 결제 후 메인 윈도우 복귀 완료")
            
        except Exception as e:
            print(f"❌ 결제 후 메인 윈도우 복귀 중 오류: {e}")
            # 오류 시에도 기본 표시
            self.show()
            self.force_center_window()
    
    def closeEvent(self, event):
        """윈도우 종료 시 리소스 정리"""
        if self.qr_check_client:
            self.qr_check_client.cleanup()
        
        # ROS2 노드 정리
        if hasattr(self, 'ros_node'):
            self.ros_node.destroy_node()
        
        event.accept()



    # MainWindow 클래스에 추가할 리프레시 함수들

    def refresh_main_window(self):
        """메인 윈도우 전체 리프레시"""
        try:
            print("🔄 메인 윈도우 리프레시 시작...")
            
            # 1. UI 상태 초기화
            self.reset_ui_state()
            
            # 2. 관리자 인증 상태 초기화
            self.admin_authenticated = False
            
            # 3. Call Robot 버튼 숨기기
            self.hide_call_robot_button()
            
            # 4. 기존 위젯들 정리
            self.cleanup_child_widgets()
            
            # 5. 윈도우 중앙 정렬
            self.force_center_window()
            
            print("✅ 메인 윈도우 리프레시 완료")
            
        except Exception as e:
            print(f"❌ 메인 윈도우 리프레시 중 오류: {e}")

    def reset_ui_state(self):
        """UI 상태 초기화"""
        try:
            # 버튼 활성화 상태 복원
            self.book_search.setEnabled(True)
            self.book_corner.setEnabled(True)
            self.payment.setEnabled(True)
            self.qr_scan_button.setEnabled(True)
            
            # 스타일 복원 (필요시)
            self.book_search.setStyleSheet(self.book_search.styleSheet())
            self.book_corner.setStyleSheet(self.book_corner.styleSheet())
            self.payment.setStyleSheet(self.payment.styleSheet())
            
            print("✅ UI 상태 초기화 완료")
            
        except Exception as e:
            print(f"❌ UI 상태 초기화 중 오류: {e}")

    def cleanup_child_widgets(self):
        """자식 위젯들 정리"""
        try:
            # Book Search Widget 정리
            if hasattr(self, 'book_search_widget') and self.book_search_widget:
                if self.book_search_widget.isVisible():
                    self.book_search_widget.hide()
                # 위젯 리셋 (재사용을 위해 삭제하지 않음)
                if hasattr(self.book_search_widget, 'reset_widget'):
                    self.book_search_widget.reset_widget()
            
            # Book Corner Widget 정리
            if hasattr(self, 'book_corner_widget') and self.book_corner_widget:
                if self.book_corner_widget.isVisible():
                    self.book_corner_widget.hide()
                # 위젯 리셋
                if hasattr(self.book_corner_widget, 'reset_widget'):
                    self.book_corner_widget.reset_widget()
            
            # Payment Widget 정리 (있다면)
            if hasattr(self, 'payment_widget') and self.payment_widget:
                if self.payment_widget.isVisible():
                    self.payment_widget.hide()
            
            print("✅ 자식 위젯들 정리 완료")
            
        except Exception as e:
            print(f"❌ 자식 위젯 정리 중 오류: {e}")

    def hide_call_robot_button(self):
        """Call Robot 버튼 숨기기"""
        try:
            if hasattr(self, 'call_manager'):
                self.call_manager.setVisible(False)
                print("✅ Call Robot 버튼 숨김 처리 완료")
            
            # Kiosk Settings 버튼도 숨기기
            if hasattr(self, 'kiosk_settings'):
                self.kiosk_settings.setVisible(False)
                print("✅ Kiosk Settings 버튼 숨김 처리 완료")
        except Exception as e:
            print(f"❌ Call Robot 버튼 숨김 처리 중 오류: {e}")

    def kiosk_location_setting(self):
        """Kiosk 위치 설정 다이얼로그 (E9=kiosk_1, C3=kiosk_2)"""
        try:
            options = [
                "E9 (kiosk_1)",
                "C3 (kiosk_2)"
            ]
            current_display = f"{self.kiosk_location_id} (kiosk_1)" if self.kiosk_location_id == 'E9' else f"{self.kiosk_location_id} (kiosk_2)"
            item, ok = QInputDialog.getItem(self, "키오스크 위치 설정", "위치를 선택하세요:", options, 0, False)
            if ok and item:
                # 선택값에서 위치 ID 추출 (앞의 토큰)
                new_loc = item.split()[0]
                self.kiosk_location_id = new_loc
                # 저장 및 UI 반영
                self._save_kiosk_location_to_settings(new_loc)
                self._update_kiosk_settings_button_label()
                # 자식 위젯들에 반영
                if hasattr(self, 'book_search_widget') and self.book_search_widget:
                    if hasattr(self.book_search_widget, 'set_kiosk_location'):
                        self.book_search_widget.set_kiosk_location(new_loc)
                if hasattr(self, 'book_corner_widget') and self.book_corner_widget:
                    if hasattr(self.book_corner_widget, 'set_kiosk_location'):
                        self.book_corner_widget.set_kiosk_location(new_loc)
                QMessageBox.information(self, "설정 완료", f"키오스크 위치가 {new_loc}로 설정되었습니다.")
        except Exception as e:
            print(f"❌ Kiosk 위치 설정 중 오류: {e}")

    def _update_kiosk_settings_button_label(self):
        try:
            if hasattr(self, 'kiosk_settings'):
                self.kiosk_settings.setText(f" Kiosk Settings")
        except Exception as e:
            print(f"⚠️ Kiosk 설정 버튼 라벨 갱신 오류: {e}")

    def _load_kiosk_location_from_settings(self) -> str:
        try:
            settings = QSettings('LIBO', 'KioskApp')
            value = settings.value('kiosk/location_id', 'E9')
            # QSettings가 QVariant로 반환할 수 있으므로 str 변환 보장
            return str(value) if value else 'E9'
        except Exception:
            return 'E9'

    def _save_kiosk_location_to_settings(self, value: str) -> None:
        try:
            settings = QSettings('LIBO', 'KioskApp')
            settings.setValue('kiosk/location_id', value)
        except Exception as e:
            print(f"⚠️ Kiosk 위치 저장 오류: {e}")



def main(args=None):
    app = QApplication(sys.argv)
    
    # 애플리케이션 전체 스타일 설정
    app.setStyle('Fusion')
    
    window = MainWindow()
    window.show()
    
    # 윈도우가 완전히 로드된 후 강제 중앙 정렬
    QTimer.singleShot(300, window.force_center_window)
    
    # ROS2 노드 스핀을 위한 타이머 설정
    ros_timer = QTimer()
    ros_timer.timeout.connect(window.spin_ros_node)
    ros_timer.start(100)  # 100ms마다 ROS2 노드 스핀
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()