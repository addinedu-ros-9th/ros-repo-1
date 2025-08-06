#!/usr/bin/env python3

import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

# ROS2 관련 import 추가
import rclpy
from rclpy.node import Node
from libo_interfaces.msg import Waypoint
from std_msgs.msg import String

# TaskRequest 클라이언트 import 추가
from kiosk.ros_communication.task_request_client import TaskRequestClient

class BookCornerWidget(Node, QWidget): # Node를 QWidget 앞으로 이동
    # 홈 버튼 클릭 시그널 정의
    home_requested = pyqtSignal()
    
    def __init__(self):
        # QWidget과 Node의 초기화 함수를 각각 명시적으로 호출
        QWidget.__init__(self)
        Node.__init__(self, 'book_corner_widget')
        
        # TaskRequest 클라이언트 초기화
        self.task_request_client = TaskRequestClient()
        self.task_request_client.task_request_completed.connect(self.on_task_request_response)
        
        self.init_ui()
        self.setup_connections()
        self.setup_ros_communication()
        
        print("✅ BookCornerWidget 초기화 완료")
    
    def init_ui(self):
        """UI 파일 로드"""
        # UI 파일 경로 - ROS2 패키지 설치 경로에서 찾기
        try:
            # 먼저 현재 디렉토리 기준으로 시도
            ui_file = os.path.join(os.path.dirname(__file__), '..', 'ui_files', 'book_corner_widget.ui')
            if not os.path.exists(ui_file):
                # ROS2 패키지 설치 경로에서 찾기
                import ament_index_python
                ui_file = os.path.join(ament_index_python.get_package_share_directory('kiosk'), 'ui_files', 'book_corner_widget.ui')
            uic.loadUi(ui_file, self)
        except Exception as e:
            print(f"UI 파일 로드 실패: {e}")
            print(f"시도한 경로: {ui_file}")
            raise
        
        # 윈도우 설정
        self.setWindowTitle("LIBO Book Corner")
        
        # 창 크기 고정
        self.setFixedSize(1100, 900)
        
        # 지도 이미지 설정
        self.setup_map_image()
        
        print("✅ BookCorner UI 로드 완료")
    
    def setup_connections(self):
        """시그널-슬롯 연결"""
        self.homeButton.clicked.connect(self.on_home_clicked)
        
        # 지도 위에 코너 버튼들 생성
        self.create_corner_buttons()
        
        print("✅ 시그널-슬롯 연결 완료")
    
    def create_corner_buttons(self):
        """지도 위에 코너 버튼들 생성"""
        # 지도 라벨을 부모로 하는 버튼들 생성
        # 각 코너의 상대적 위치 (지도 내에서의 위치)
        corner_positions = {
            "컴퓨터": (300, 310),
            "언어": (490, 310),
            "소설": (680, 310)
        }
        
        # 각 코너별 버튼 크기 설정
        corner_sizes = {
            "컴퓨터": (90, 130),    # 가로 짧고 세로 긴
            "언어": (90, 130),      # 가로 짧고 세로 긴
            "소설": (130, 90)       # 가로 길고 세로 짧음
        }
        
        # 각 코너에 버튼 생성
        for corner_name, (x, y) in corner_positions.items():
            button = QPushButton(corner_name, self.mapLabel)
            button_width, button_height = corner_sizes[corner_name]
            button.setGeometry(x, y, button_width, button_height)
            button.setStyleSheet("""
                QPushButton {
                    background-color: #71866a;
                    color: white;
                    border: none;
                    border-radius: 10px;
                    font-size: 16px;
                    font-weight: bold;
                    padding: 10px;
                }
                QPushButton:hover {
                    background-color: #2980b9;
                }
                QPushButton:pressed {
                    background-color: #21618c;
                }
            """)
            
            # 버튼 클릭 이벤트 연결
            button.clicked.connect(lambda checked, name=corner_name: self.on_corner_button_clicked(name))
            
            print(f"✅ {corner_name} 코너 버튼 생성 완료 (위치: {x}, {y})")
    
    def setup_map_image(self):
        """지도 이미지 설정"""
        try:
            # waypoint4.png 이미지 로드
            image_path = '/home/robolee/dev_ws/ros-repo-1/ros_ws/src/waypoint4.png'
            
            if os.path.exists(image_path):
                pixmap = QPixmap(image_path)
                
                # 지도 크기 축소 (원본 크기의 80%로 축소)
                scaled_pixmap = pixmap.scaled(
                    int(pixmap.width() * 0.9), 
                    int(pixmap.height() * 0.7), 
                    Qt.KeepAspectRatio, 
                    Qt.SmoothTransformation
                )
                
                self.mapLabel.setPixmap(scaled_pixmap)
                self.mapLabel.setScaledContents(False)  # 비율 유지하면서 축소
                print(f"✅ 지도 이미지 설정 완료 (축소됨): {image_path}")
            else:
                print(f"❌ 지도 이미지를 찾을 수 없습니다: {image_path}")
                
        except Exception as e:
            print(f"❌ 지도 이미지 설정 실패: {e}")
    
    def setup_ros_communication(self):
        """ROS2 통신 설정"""
        # Waypoint 메시지 발행자 생성
        self.waypoint_publisher = self.create_publisher(
            Waypoint, 
            '/waypoint_goal', 
            10
        )
        
        # 상태 메시지 발행자 생성
        self.status_publisher = self.create_publisher(
            String, 
            '/robot_status', 
            10
        )
        
        # 코너별 waypoint 매핑
        self.corner_waypoints = {
            "컴퓨터": "D5",
            "언어": "D7", 
            "소설": "C8"
        }
        
        print("✅ ROS2 통신 설정 완료")
    
    def publish_waypoint(self, waypoint_id):
        """waypoint_id를 발행하여 로봇 이동 명령"""
        msg = Waypoint()
        msg.waypoint_id = waypoint_id
        self.waypoint_publisher.publish(msg)
        print(f"🎯 Waypoint 발행: {waypoint_id}")
    
    def publish_status_message(self, message):
        """상태 메시지 발행"""
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
        print(f"📢 상태 메시지 발행: {message}")
    
    def on_corner_button_clicked(self, corner_name):
        """코너 버튼 클릭 처리"""
        print(f"🎯 {corner_name} 코너 버튼 클릭됨")
        self.on_corner_selected(corner_name)
    
    def on_corner_selected(self, corner_name):
        """코너 선택 처리"""
        print(f"선택된 코너: {corner_name}")
        
        # 확인 팝업 표시
        reply = QMessageBox.question(
            self, 
            "에스코팅 요청", 
            f"{corner_name} 코너로 에스코팅을 요청하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            print(f"✅ {corner_name} 코너 에스코팅 요청 승인")
            # TaskRequest를 통한 에스코팅 요청
            self.request_escort_to_corner(corner_name)
        else:
            print("에스코팅 요청 취소")
    
    def request_escort_to_corner(self, corner_name):
        """코너로 에스코팅 요청 - TaskRequest 서비스 사용"""
        try:
            print(f"🚀 {corner_name} 코너 에스코팅 요청 시작")
            
            # 코너별 waypoint 매핑
            corner_waypoints = {
                "컴퓨터": "D5",
                "언어": "D7", 
                "소설": "C8"
            }
            
            # TaskRequest.srv 파라미터 준비
            robot_id = ""  # task_manager에서 자동 선택
            call_location = "E9"  # 키오스크 위치
            goal_location = corner_waypoints.get(corner_name, "D5")  # 코너 위치
            
            print(f"📍 TaskRequest 파라미터:")
            print(f"   robot_id: '{robot_id}' (task_manager에서 자동 선택)")
            print(f"   task_type: escort")
            print(f"   call_location: {call_location} (키오스크)")
            print(f"   goal_location: {goal_location} ({corner_name} 코너)")
            
            # TaskRequest 서비스 호출
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
                
                # 실패 시 위젯 리프레시
                self.refresh_widget()
                
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
                f"선택하신 코너로 안내할 예정입니다."
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
                f"선택하신 코너로 안내할 예정입니다.\n\n"
                f"5초 후 메인화면으로 돌아갑니다."
            )
            # 5초 후 메인화면으로 이동
            QTimer.singleShot(5000, self.on_home_clicked)
    
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
    
    def refresh_widget(self):
        """위젯 리프레시"""
        try:
            print("🔄 Book Corner 위젯 리프레시")
            
            # 지도 이미지 다시 로드
            self.setup_map_image()
            
            # 코너 버튼들 다시 생성
            self.create_corner_buttons()
            
            print("✅ 위젯 리프레시 완료")
            
        except Exception as e:
            print(f"❌ 위젯 리프레시 중 오류: {e}")
    
    def on_home_clicked(self):
        """홈 버튼 클릭"""
        print("🏠 홈으로 돌아가기")
        
        # TaskRequest 클라이언트 정리
        self.cleanup_task_request_client()
        
        self.hide()  # 현재 위젯 숨기기
        self.home_requested.emit()
    
    def cleanup_task_request_client(self):
        """TaskRequest 클라이언트 안전 정리"""
        try:
            if hasattr(self, 'task_request_client') and self.task_request_client:
                if self.task_request_client.isRunning():
                    self.task_request_client.quit()
                    self.task_request_client.wait(1000)
                # cleanup은 호출하지 않고 노드만 정리
                self.task_request_client.cleanup()
                print("✅ task_request_client 정리 완료")
        except Exception as e:
            print(f"⚠️ task_request_client 정리 중 오류: {e}")
    
    def reset_task_request_client(self):
        """TaskRequest 클라이언트 재초기화"""
        try:
            if hasattr(self, 'task_request_client') and self.task_request_client:
                # 기존 클라이언트 정리
                self.cleanup_task_request_client()
                
                # 새로운 클라이언트 생성
                self.task_request_client = TaskRequestClient()
                self.task_request_client.task_request_completed.connect(self.on_task_request_response)
                print("✅ task_request_client 재초기화 완료")
        except Exception as e:
            print(f"⚠️ task_request_client 재초기화 중 오류: {e}")
    
    def reset_widget(self):
        """위젯 초기화"""
        print("🔄 Book Corner 위젯 초기화")
        # 필요한 초기화 작업 수행
        self.setup_map_image()
        
        # TaskRequestClient 재초기화
        self.reset_task_request_client()
    
    def showEvent(self, event):
        """위젯이 표시될 때"""
        super().showEvent(event)
        # 윈도우 중앙 정렬
        self.center_window()
        
        # TaskRequestClient 초기화 상태 확인
        if hasattr(self, 'task_request_client') and self.task_request_client:
            if not self.task_request_client._node_initialized:
                print("🔄 TaskRequestClient 재초기화 필요")
                self.reset_task_request_client()
    
    def center_window(self):
        """윈도우를 화면 중앙에 위치시키기"""
        screen = QApplication.desktop().screenGeometry()
        
        # 고정된 창 크기 사용
        window_width = 1100
        window_height = 900
        
        center_x = (screen.width() - window_width) // 2
        center_y = (screen.height() - window_height) // 2
        
        self.move(center_x, center_y)
        print(f"✅ Book Corner 윈도우 중앙 정렬: ({center_x}, {center_y})")
        print(f"화면 크기: {screen.width()}x{screen.height()}, 창 크기: {window_width}x{window_height}")
    
    def closeEvent(self, event):
        """윈도우 종료 시 리소스 정리"""
        self.cleanup_task_request_client()
        event.accept()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = BookCornerWidget()
    window.show()
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("사용자에 의해 종료됩니다.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 