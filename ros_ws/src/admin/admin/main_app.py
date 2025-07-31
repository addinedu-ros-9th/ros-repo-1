#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory

from admin.tabs.task_request_tab import TaskRequestTab # 우리가 만든 TaskRequestTab을 임포트
from admin.tabs.heartbeat_monitor_tab import HeartbeatMonitorTab # 새로 만든 HeartbeatMonitorTab을 임포트
from std_msgs.msg import String  # 임시로 String 메시지 사용

class AdminWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ros_node = rclpy.create_node('admin_gui_node') # GUI 전체에서 사용할 ROS 노드 생성
        self.robot_status_text = "No robots detected..."  # 로봇 상태 텍스트
        self.init_ui() # UI 파일을 로드하고 초기화하는 함수를 호출
        self.init_tabs() # 탭들을 초기화하고 추가하는 함수를 호출
        self.init_robot_status_subscriber()  # String 구독자 초기화
        self.init_timer() # ROS 통신을 위한 타이머 시작

    def init_ui(self):
        package_share_dir = get_package_share_directory('admin') # 'admin' 패키지 공유 디렉토리 경로를 찾음
        ui_file = os.path.join(package_share_dir, 'ui', 'main_window.ui') # UI 파일의 전체 경로를 조합
        uic.loadUi(ui_file, self) # UI 파일을 불러와 현재 객체에 적용

    def init_tabs(self):
        # Task Request 탭 추가
        self.task_request_tab = TaskRequestTab(self.ros_node) # TaskRequestTab 객체를 생성
        self.tabWidget.addTab(self.task_request_tab, "🚀 Task Request 테스트") # 'tabWidget'에 새 탭을 추가

        # Heartbeat Monitor 탭 추가
        self.heartbeat_monitor_tab = HeartbeatMonitorTab(self.ros_node) # HeartbeatMonitorTab 객체를 생성하고 메인 노드를 전달
        self.tabWidget.addTab(self.heartbeat_monitor_tab, "💓 Heartbeat 모니터") # 'tabWidget'에 새 탭을 추가

    def init_robot_status_subscriber(self):  # String 구독자 초기화
        """robot_status 토픽을 구독해서 로봇 상태를 실시간 업데이트"""
        self.robot_status_subscription = self.ros_node.create_subscription(
            String,  # 메시지 타입
            'robot_status',  # 토픽 이름
            self.robot_status_callback,  # 콜백 함수
            10  # QoS depth
        )

    def robot_status_callback(self, msg):  # 로봇 상태 메시지 수신 콜백
        """String 메시지를 받았을 때 GUI 업데이트"""
        try:
            self.robot_status_text = msg.data  # 받은 문자열 저장
            self.update_robot_status_display()  # GUI 업데이트
            
        except Exception as e:
            print(f"로봇 상태 처리 중 오류: {e}")

    def update_robot_status_display(self):  # 로봇 상태 표시 업데이트
        """로봇 상태 위젯의 라벨들을 업데이트"""
        try:
            if "No active robots" in self.robot_status_text:
                self.robot_count_label.setText("Count: 0")  # 로봇 수 0
                self.robot_list_label.setText("No robots detected...")  # 로봇 없을 때
            else:
                # "Active robots: libo_a, libo_b" 형태에서 로봇 추출
                if "Active robots:" in self.robot_status_text:
                    robot_part = self.robot_status_text.split("Active robots: ")[1]
                    robot_list = [r.strip() for r in robot_part.split(",")]
                    robot_count = len(robot_list)
                    
                    self.robot_count_label.setText(f"Count: {robot_count}")  # 로봇 수 업데이트
                    
                    # 각 로봇을 새 줄로 표시
                    robot_display = "\n".join([f"{robot}: ✅ 🔋?" for robot in robot_list])
                    self.robot_list_label.setText(robot_display)  # 로봇 리스트 업데이트
                else:
                    self.robot_list_label.setText(self.robot_status_text)  # 원본 텍스트 그대로 표시
                
        except Exception as e:
            print(f"로봇 상태 표시 업데이트 중 오류: {e}")

    def init_timer(self):
        self.ros_timer = QTimer(self) # QTimer 객체 생성
        self.ros_timer.timeout.connect(self.spin_ros_nodes) # 타이머가 만료될 때마다 spin_ros_nodes 함수를 호출하도록 연결
        self.ros_timer.start(100) # 100ms (0.1초) 간격으로 타이머 시작

    def spin_ros_nodes(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0) # 메인 GUI의 ROS 노드를 스핀
        # task_request_tab에 client_node가 존재하면 그것도 스핀
        if hasattr(self, 'task_request_tab') and hasattr(self.task_request_tab, 'client_node'):
            rclpy.spin_once(self.task_request_tab.client_node, timeout_sec=0)
        
        # heartbeat_monitor_tab에 node가 존재하면 그것도 스핀
        if hasattr(self, 'heartbeat_monitor_tab') and hasattr(self.heartbeat_monitor_tab, 'node'):
            rclpy.spin_once(self.heartbeat_monitor_tab.node, timeout_sec=0)

    def closeEvent(self, event):
        self.task_request_tab.shutdown() # TaskRequest 탭의 정리 함수 호출
        self.heartbeat_monitor_tab.shutdown() # Heartbeat 탭의 정리 함수도 호출
        self.ros_node.destroy_node() # 메인 ROS 노드 종료
        rclpy.shutdown() # ROS2 시스템 전체 종료
        event.accept() # 창 닫기 이벤트 수락

def main(args=None):
    rclpy.init(args=args) # ROS2 시스템 초기화 (노드 생성 전에 한번만 호출)
    app = QApplication(sys.argv) # PyQt 어플리케이션 객체 생성
    window = AdminWindow() # 메인 윈도우 객체 생성
    window.show() # 윈도우를 화면에 표시
    sys.exit(app.exec_()) # 어플리케이션 이벤트 루프 시작 및 종료 코드 처리

if __name__ == '__main__':
    main() 