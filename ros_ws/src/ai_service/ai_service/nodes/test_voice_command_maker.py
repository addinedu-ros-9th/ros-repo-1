#!/usr/bin/env python3
# test_voice_command_maker.py
# VoiceCommand 메시지를 발행하기 위한 PyQt5 기반 테스트 도구

import sys
import os
from datetime import datetime
import pytz
import threading
import time

import rclpy
from rclpy.node import Node
from libo_interfaces.msg import VoiceCommand

from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QPushButton,
                            QVBoxLayout, QHBoxLayout, QWidget, QLabel,
                            QLineEdit, QGridLayout, QStatusBar, QGroupBox,
                            QScrollArea)
from PyQt5.QtCore import Qt, pyqtSignal, QSize, QTimer
from PyQt5.QtGui import QFont, QIcon


# 한국 시간 출력 함수
def get_kr_time():
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 밀리초 3자리까지 표시


class ROS2Publisher(Node):
    """VoiceCommand 메시지를 발행하는 ROS2 노드"""
    def __init__(self):
        super().__init__('voice_command_publisher')
        self.publisher = self.create_publisher(
            VoiceCommand,
            '/voice_command',
            10
        )
        self.get_logger().info('VoiceCommand 발행자 초기화 완료')
        
    def publish_command(self, robot_id, category, action):
        """VoiceCommand 메시지 발행"""
        msg = VoiceCommand()
        msg.robot_id = robot_id
        msg.category = category
        msg.action = action
        
        self.publisher.publish(msg)
        self.get_logger().info(f'메시지 발행: robot_id={robot_id}, category={category}, action={action}')
        return True


class VoiceCommandMaker(QMainWindow):
    """음성 명령을 생성하고 발행하는 GUI 애플리케이션"""
    status_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Voice Command Maker')
        self.setGeometry(100, 100, 800, 600)
        self.setMinimumSize(800, 600)
        
        # ROS2 초기화는 별도 스레드에서 수행
        self.ros2_thread = None
        self.ros2_node = None
        self.init_ros2()
        
        # GUI 초기화
        self.init_ui()
        
        # 상태 시그널 연결
        self.status_signal.connect(self.update_status)
        
    def init_ros2(self):
        """ROS2 노드 초기화 (별도 스레드에서 실행)"""
        self.ros2_shutdown = threading.Event()  # 종료 이벤트 추가
        
        def run_ros2():
            rclpy.init()
            self.ros2_node = ROS2Publisher()
            self.status_signal.emit("ROS2 노드 초기화 완료")
            
            # 종료 신호가 올 때까지 스핀 (종료 가능하게)
            while rclpy.ok() and not self.ros2_shutdown.is_set():
                rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
            
            # 정리
            if self.ros2_node and rclpy.ok():
                self.ros2_node.destroy_node()
            print("ROS2 스레드 종료됨")
            
        self.ros2_thread = threading.Thread(target=run_ros2, daemon=True)
        self.ros2_thread.start()
        
        # ROS2 노드 초기화 대기
        time.sleep(1)
        
    def init_ui(self):
        """GUI 요소 초기화"""
        # 중앙 위젯 설정
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 메인 레이아웃
        main_layout = QVBoxLayout(main_widget)
        
        # 로봇 ID 입력 섹션
        id_layout = QHBoxLayout()
        id_label = QLabel("로봇 ID:")
        self.id_input = QLineEdit("libo_a")  # 기본값
        id_layout.addWidget(id_label)
        id_layout.addWidget(self.id_input)
        id_layout.addStretch(1)
        
        # 탭 위젯 생성
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.North)
        self.tabs.setMovable(True)
        
        # 카테고리별 탭 추가
        self.add_category_tab("common", "일반")
        self.add_category_tab("escort", "안내")
        self.add_category_tab("delivery", "배달")
        self.add_category_tab("assist", "어시스트")
        
        # 상태 표시줄
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("준비됨")
        
        # 레이아웃에 위젯 추가
        main_layout.addLayout(id_layout)
        main_layout.addWidget(self.tabs)
        
    def add_category_tab(self, category, display_name):
        """카테고리별 탭 추가"""
        tab = QWidget()
        
        # 스크롤 영역 추가
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        
        # 명령 버튼 그룹
        if category == "common":
            self.add_command_group(scroll_layout, category, "전원/초기화", [
                ("power_on", "전원 켜기"),
                ("initialized", "초기화 완료"),
                ("charging", "충전 시작"),
                ("battery_sufficient", "배터리 충분"),
            ])
            
            self.add_command_group(scroll_layout, category, "이동", [
                ("depart_base", "베이스 출발"),
                ("obstacle_detected", "장애물 감지"),
                ("reroute", "경로 재설정"),
                ("return", "복귀"),
                ("arrived_base", "베이스 도착"),
            ])
            
        elif category == "escort":
            self.add_command_group(scroll_layout, category, "안내 시작", [
                ("depart_base", "베이스 출발"),
                ("arrived_kiosk", "키오스크 도착"),
            ])
            
            self.add_command_group(scroll_layout, category, "안내 중", [
                ("lost_user", "사용자 감지 실패"),
                ("user_reconnected", "사용자 재연결"),
            ])
            
            self.add_command_group(scroll_layout, category, "안내 종료", [
                ("arrived_destination", "목적지 도착"),
                ("return", "복귀"),
                ("arrived_base", "베이스 도착"),
            ])
            
        elif category == "delivery":
            self.add_command_group(scroll_layout, category, "배달 시작", [
                ("depart_base", "베이스 출발"),
                ("arrived_admin_desk", "관리자 데스크 도착"),
                ("receive_next_goal", "목적지 수신"),
            ])
            
            self.add_command_group(scroll_layout, category, "배달 중", [
                ("arrived_destination", "목적지 도착"),
                ("called_by_staff", "직원 호출"),
            ])
            
            self.add_command_group(scroll_layout, category, "배달 종료", [
                ("return", "복귀"),
                ("arrived_base", "베이스 도착"),
            ])
            
        elif category == "assist":
            self.add_command_group(scroll_layout, category, "어시스트 시작", [
                ("depart_base", "베이스 출발"),
                ("arrived_kiosk", "키오스크 도착"),
                ("qr_authenticated", "QR 인증 완료"),
            ])
            
            self.add_command_group(scroll_layout, category, "어시스트 중", [
                ("no_person_5s", "사람 감지 실패"),
                ("person_detected", "사람 감지 성공"),
                ("called_by_staff", "직원 호출"),
                ("pause", "일시정지"),
                ("resume", "재개"),
            ])
            
            self.add_command_group(scroll_layout, category, "어시스트 종료", [
                ("return", "복귀"),
                ("arrived_base", "베이스 도착"),
            ])
        
        # 스트레치 추가 (하단 여백)
        scroll_layout.addStretch(1)
        
        # 스크롤 영역 설정 완료
        scroll.setWidget(scroll_content)
        
        # 탭 레이아웃에 스크롤 영역 추가
        tab_layout = QVBoxLayout(tab)
        tab_layout.addWidget(scroll)
        
        # 탭 추가
        self.tabs.addTab(tab, display_name)
        
    def add_command_group(self, parent_layout, category, group_name, commands):
        """명령 버튼 그룹 추가"""
        group = QGroupBox(group_name)
        group_layout = QGridLayout(group)
        
        for row, (action, display_name) in enumerate(commands):
            btn = QPushButton(display_name)
            btn.setMinimumHeight(40)
            btn.clicked.connect(lambda checked, c=category, a=action: self.send_command(c, a))
            group_layout.addWidget(btn, row, 0)
        
        parent_layout.addWidget(group)
        
    def send_command(self, category, action):
        """명령 발행"""
        robot_id = self.id_input.text().strip()
        if not robot_id:
            robot_id = "libo_a"  # 기본값
        
        if not self.ros2_node:
            self.status_signal.emit("ROS2 노드가 초기화되지 않았습니다")
            return
        
        # 메시지 발행
        try:
            self.ros2_node.publish_command(robot_id, category, action)
            self.status_signal.emit(f"명령 발행 성공: [{robot_id}] {category}/{action}")
            
            # 콘솔에도 출력
            print(f"[{get_kr_time()}][TEST] VoiceCommand 발행: robot_id={robot_id}, "
                  f"category={category}, action={action}")
            
        except Exception as e:
            self.status_signal.emit(f"명령 발행 실패: {str(e)}")
            print(f"[{get_kr_time()}][ERROR] 명령 발행 실패: {str(e)}")
    
    def update_status(self, message):
        """상태 표시줄 업데이트"""
        self.status_bar.showMessage(f"{get_kr_time()} - {message}")
        
    def closeEvent(self, event):
        """애플리케이션 종료 시 정리"""
        print("종료 중...")
        
        # ROS2 종료 이벤트 설정
        if hasattr(self, 'ros2_shutdown'):
            self.ros2_shutdown.set()
        
        # 스레드가 끝날 때까지 최대 2초간 대기
        if hasattr(self, 'ros2_thread') and self.ros2_thread.is_alive():
            self.ros2_thread.join(timeout=2.0)
            
        # ROS2 종료
        try:
            rclpy.shutdown()
            print("ROS2 셧다운 완료")
        except Exception as e:
            print(f"ROS2 셧다운 중 오류: {e}")
            
        print("프로그램이 안전하게 종료되었습니다")
        event.accept()


def main():
    # Ctrl+C 시그널 핸들러 설정
    import signal
    
    app = QApplication(sys.argv)
    window = VoiceCommandMaker()
    window.show()
    
    # Ctrl+C 핸들러 등록
    def signal_handler(sig, frame):
        print("Ctrl+C 감지됨, 프로그램을 종료합니다...")
        window.close()
        app.quit()
    
    # SIGINT 시그널(Ctrl+C) 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    
    # 타이머를 사용해 시그널 처리를 할 수 있도록 주기적 이벤트 발생
    timer = QTimer()
    timer.timeout.connect(lambda: None)  # 더미 함수
    timer.start(100)  # 100ms마다 타이머 이벤트 발생
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n프로그램이 Ctrl+C로 종료되었습니다.")
    except Exception as e:
        print(f"예외 발생: {e}")
    finally:
        # 마지막 정리 - ROS2가 여전히 실행 중인 경우를 대비
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
        print("프로그램 종료됨")
