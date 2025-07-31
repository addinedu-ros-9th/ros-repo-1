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

class AdminWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ros_node = rclpy.create_node('admin_gui_node') # GUI 전체에서 사용할 ROS 노드 생성
        self.init_ui() # UI 파일을 로드하고 초기화하는 함수를 호출
        self.init_tabs() # 탭들을 초기화하고 추가하는 함수를 호출
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
        self.heartbeat_monitor_tab = HeartbeatMonitorTab() # HeartbeatMonitorTab 객체를 생성
        self.tabWidget.addTab(self.heartbeat_monitor_tab, "💓 Heartbeat 모니터") # 'tabWidget'에 새 탭을 추가

    def init_timer(self):
        self.ros_timer = QTimer(self) # QTimer 객체 생성
        self.ros_timer.timeout.connect(self.spin_ros_nodes) # 타이머가 만료될 때마다 spin_ros_nodes 함수를 호출하도록 연결
        self.ros_timer.start(100) # 100ms (0.1초) 간격으로 타이머 시작

    def spin_ros_nodes(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0) # 메인 GUI의 ROS 노드를 스핀
        # task_request_tab에 client_node가 존재하면 그것도 스핀
        if hasattr(self, 'task_request_tab') and hasattr(self.task_request_tab, 'client_node'):
            rclpy.spin_once(self.task_request_tab.client_node, timeout_sec=0)

    def closeEvent(self, event):
        self.task_request_tab.shutdown() # TaskRequest 탭의 정리 함수 호출
        # self.heartbeat_monitor_tab.shutdown() # 만약 Heartbeat 탭에 종료 시 처리할 내용이 있다면 추가
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