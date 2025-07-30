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

class BookCornerWidget(Node, QWidget): # Node를 QWidget 앞으로 이동
    # 홈 버튼 클릭 시그널 정의
    home_requested = pyqtSignal()
    
    def __init__(self):
        # QWidget과 Node의 초기화 함수를 각각 명시적으로 호출
        QWidget.__init__(self)
        Node.__init__(self, 'book_corner_widget')
        
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
        self.setFixedSize(1200, 900)
        
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
            "컴퓨터": (230, 330),
            "언어": (480, 330),
            "소설": (740, 330)
        }
        
        # 각 코너별 버튼 크기 설정
        corner_sizes = {
            "컴퓨터": (100, 200),    # 가로 짧고 세로 긴
            "언어": (100, 200),      # 가로 짧고 세로 긴
            "소설": (200, 100)       # 가로 길고 세로 짧음
        }
        
        # 각 코너에 버튼 생성
        for corner_name, (x, y) in corner_positions.items():
            button = QPushButton(corner_name, self.mapLabel)
            button_width, button_height = corner_sizes[corner_name]
            button.setGeometry(x, y, button_width, button_height)
            button.setStyleSheet("""
                QPushButton {
                    background-color: #3498db;
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
                self.mapLabel.setPixmap(pixmap)
                self.mapLabel.setScaledContents(True)
                print(f"✅ 지도 이미지 설정 완료: {image_path}")
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
    
    def start_escorting_scenario(self, corner_name):
        """에스코팅 시나리오 시작"""
        print(f"🚀 {corner_name} 코너 에스코팅 시나리오 시작")
        
        # 1단계: 로봇이 키오스크로 이동
        self.publish_status_message(f"로봇이 키오스크 위치로 와서 {corner_name} 코너로 에스코팅 하겠습니다.")
        self.publish_waypoint("E9")  # 키오스크로 이동
        
        # 2단계: 키오스크 도착 후 선택한 코너로 이동
        corner_waypoint = self.corner_waypoints.get(corner_name)
        if corner_waypoint:
            # 10초 후에 코너로 이동 (실제 로봇 이동 시간 고려)
            print(f"⏰ 10초 후 {corner_name} 코너로 이동 예정...")
            QTimer.singleShot(60000, lambda: self.move_to_corner(corner_name, corner_waypoint))
        else:
            print(f"❌ 알 수 없는 코너: {corner_name}")
    
    def move_to_corner(self, corner_name, waypoint_id):
        """선택한 코너로 이동"""
        print(f"🎯 {corner_name} 코너로 이동 시작")
        self.publish_waypoint(waypoint_id)
        
        # 3단계: 코너 도착 후 완료 메시지
        # 15초 후에 완료 메시지 (실제 로봇 이동 시간 고려)
        print(f"⏰ 15초 후 {corner_name} 코너 도착 예정...")
        QTimer.singleShot(60000, lambda: self.complete_escorting(corner_name))
    
    def complete_escorting(self, corner_name):
        """에스코팅 완료 처리"""
        print(f"✅ {corner_name} 코너 에스코팅 완료")
        self.publish_status_message(f"{corner_name} 코너 에스코팅이 완료되었습니다.")
        
        # 4단계: 베이스(충전소)로 복귀
        print("⏰ 5초 후 베이스로 복귀 예정...")
        QTimer.singleShot(60000, lambda: self.return_to_base())
    
    def return_to_base(self):
        """베이스(충전소)로 복귀"""
        print("🏠 베이스(충전소)로 복귀")
        self.publish_waypoint("E3")  # 베이스로 이동
        self.publish_status_message("베이스(충전소)로 복귀합니다.")

    
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
            # 에스코팅 시나리오 시작
            self.start_escorting_scenario(corner_name)
            
            QMessageBox.information(
                self, 
                "에스코팅 요청", 
                f"{corner_name} 코너로 에스코팅을 요청했습니다.\n로봇이 곧 도착할 예정입니다."
            )
        else:
            print("에스코팅 요청 취소")
    
    def on_home_clicked(self):
        """홈 버튼 클릭"""
        print("🏠 홈으로 돌아가기")
        self.hide()  # 현재 위젯 숨기기
        self.home_requested.emit()
    
    def reset_widget(self):
        """위젯 초기화"""
        print("🔄 Book Corner 위젯 초기화")
        # 필요한 초기화 작업 수행
        self.load_map_image()
    
    def showEvent(self, event):
        """위젯이 표시될 때"""
        super().showEvent(event)
        # 윈도우 중앙 정렬
        self.center_window()
    
    def center_window(self):
        """윈도우를 화면 중앙에 위치시키기"""
        screen = QApplication.desktop().screenGeometry()
        
        # 고정된 창 크기 사용
        window_width = 1200
        window_height = 900
        
        center_x = (screen.width() - window_width) // 2
        center_y = (screen.height() - window_height) // 2
        
        self.move(center_x, center_y)
        print(f"✅ Book Corner 윈도우 중앙 정렬: ({center_x}, {center_y})")
        print(f"화면 크기: {screen.width()}x{screen.height()}, 창 크기: {window_width}x{window_height}")

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