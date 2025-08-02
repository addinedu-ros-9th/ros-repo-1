#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QObject, Qt
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

# 새로운 탭 클래스들 import
from admin.tabs.main_control_tab import MainControlTab
from admin.tabs.ai_server_control_tab import AiServerControlTab

class AdminWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ros_node = rclpy.create_node('admin_gui_node')  # GUI 전체에서 사용할 ROS 노드 생성
        
        self.init_ui()
        self.init_tabs()
        self.init_timer()
        
        # 윈도우 설정
        self.setWindowTitle("LIBO Administrator System v2.0 - Tabbed")
        self.setMinimumSize(1320, 900)
        self.resize(1320, 900)

    def init_ui(self):
        """메인 윈도우 UI 초기화"""
        # main_window.ui 파일 로드
        ui_file_path = os.path.join(get_package_share_directory('admin'), 'ui', 'main_window.ui')
        uic.loadUi(ui_file_path, self)

    def init_tabs(self):
        """탭 초기화 및 추가"""
        # 메인 컨트롤 탭
        self.main_control_tab = MainControlTab(self.ros_node)
        self.tabWidget.widget(0).layout().addWidget(self.main_control_tab)
        
        # AI 서버 컨트롤 탭
        self.ai_server_control_tab = AiServerControlTab(self.ros_node)
        self.tabWidget.widget(1).layout().addWidget(self.ai_server_control_tab)
        
        # System Logs 탭은 나중에 구현 예정
        self.get_logger().info("✅ 탭 초기화 완료")

    def init_timer(self):
        """ROS 통신을 위한 타이머 시작"""
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros_nodes)
        self.ros_timer.start(10)  # 10ms마다 실행 (100Hz)
        
        self.get_logger().info("🔄 ROS 통신 타이머 시작됨")

    def spin_ros_nodes(self):
        """ROS 노드 스핀"""
        rclpy.spin_once(self.ros_node, timeout_sec=0.001)

    def get_logger(self):
        """ROS 로거 반환"""
        return self.ros_node.get_logger()

    def closeEvent(self, event):
        """윈도우 종료 시 처리"""
        self.get_logger().info("🛑 Admin GUI 종료 중...")
        
        # 탭들 정리
        if hasattr(self, 'main_control_tab'):
            self.main_control_tab.cleanup()
        if hasattr(self, 'ai_server_control_tab'):
            self.ai_server_control_tab.cleanup()
        
        # 타이머 정지
        if hasattr(self, 'ros_timer'):
            self.ros_timer.stop()
        
        # ROS 노드 정리
        if hasattr(self, 'ros_node'):
            self.ros_node.destroy_node()
        
        event.accept()

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    window = AdminWindow()
    window.show()
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 