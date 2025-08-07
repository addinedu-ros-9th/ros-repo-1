#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
import time
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit, QScrollArea
from PyQt5.QtCore import Qt, QTimer
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

# OverallStatus 메시지 import
from libo_interfaces.msg import OverallStatus

class SystemLogsTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node  # ROS 노드 저장
        
        # OverallStatus 관련 변수들
        self.overall_status_data = {}  # 로봇 상태 데이터 저장
        
        self.init_ui()  # UI 초기화
        self.init_ros_connections()  # ROS 연결 초기화
        self.init_timers()  # 타이머 초기화
    
    def init_ui(self):
        """UI 초기화"""
        try:
            # system_logs_tab.ui 파일 로드 시도
            ui_file_path = os.path.join(get_package_share_directory('admin'), 'ui', 'system_logs_tab.ui')
            uic.loadUi(ui_file_path, self)
            self.get_logger().info("✅ System Logs Tab UI 로드 완료")
            
        except Exception as e:
            # UI 파일이 없을 경우 기본 레이아웃 설정
            self.get_logger().warning(f"⚠️ UI 파일 로드 실패: {e}, 기본 레이아웃 사용")
            self.layout = QVBoxLayout()
            self.setLayout(self.layout)
            
            # 제목 라벨 추가
            title_label = QLabel("📋 System Logs - OverallStatus 모니터링")
            title_label.setAlignment(Qt.AlignCenter)
            title_label.setStyleSheet("font-size: 16px; font-weight: bold; margin: 10px;")
            self.layout.addWidget(title_label)
            
            # OverallStatus 표시용 텍스트 에디터 추가
            self.overall_status_text = QTextEdit()
            self.overall_status_text.setReadOnly(True)  # 읽기 전용으로 설정
            self.overall_status_text.setPlainText("📋 System Logs 시작 - OverallStatus 모니터링 대기 중...\n")
            self.overall_status_text.setStyleSheet("font-family: monospace; font-size: 12px;")
            self.layout.addWidget(self.overall_status_text)
            
            self.get_logger().info("✅ System Logs Tab 기본 UI 초기화 완료")
    
    def init_ros_connections(self):
        """ROS 연결 초기화"""
        try:
            # OverallStatus 구독자
            self.overall_status_subscription = self.ros_node.create_subscription(
                OverallStatus, 'robot_status', self.overall_status_callback, 10
            )
            self.get_logger().info("✅ OverallStatus 구독자 초기화 완료")
            
        except Exception as e:
            self.get_logger().error(f"❌ ROS 연결 초기화 중 오류: {e}")
    
    def init_timers(self):
        """타이머 초기화"""
        # OverallStatus 업데이트 타이머 (10초마다)
        self.overall_status_timer = QTimer()
        self.overall_status_timer.timeout.connect(self.update_overall_status_display)
        self.overall_status_timer.start(10000)  # 10초마다
        self.get_logger().info("✅ OverallStatus 업데이트 타이머 시작됨 (10초 간격)")
    
    def overall_status_callback(self, msg):
        """OverallStatus 메시지 수신"""
        robot_id = msg.robot_id
        self.overall_status_data[robot_id] = {
            'timestamp': msg.timestamp,
            'robot_state': msg.robot_state,
            'is_available': msg.is_available,
            'battery': msg.battery,
            'book_weight': msg.book_weight,
            'position_x': msg.position_x,
            'position_y': msg.position_y,
            'position_yaw': msg.position_yaw,
            'received_time': time.time()
        }
        self.get_logger().debug(f"📥 OverallStatus 수신: {robot_id}")
    
    def update_overall_status_display(self):
        """OverallStatus 표시 업데이트 (10초마다) - 한 줄씩 누적되는 로그 방식"""
        try:
            # overall_status_text가 존재하는지 확인
            if not hasattr(self, 'overall_status_text'):
                self.get_logger().error("❌ overall_status_text 위젯을 찾을 수 없음")
                return
            
            current_time = time.time()
            active_robots = []
            
            # 현재 시간 정보 (한 줄로)
            time_str = time.strftime('%Y-%m-%d %H:%M:%S')
            
            for robot_id, status in self.overall_status_data.items():
                # 30초 이상 업데이트가 없으면 제거
                if current_time - status['received_time'] > 30.0:
                    continue
                
                # 로봇 상태에 따라 사용 가능/불가 결정
                robot_state = status['robot_state']
                if robot_state in ['INIT', 'CHARGING']:
                    available_text = "🔴"
                elif status['is_available']:
                    available_text = "🟢"
                else:
                    available_text = "🔴"
                
                # 배터리 상태에 따른 이모지
                battery_level = status['battery']
                if battery_level > 50:
                    battery_emoji = "🔋"
                elif battery_level > 20:
                    battery_emoji = "🟡"
                else:
                    battery_emoji = "🔴"
                
                # 한 줄로 압축된 로봇 정보
                robot_info = f"[{time_str}] {available_text} {robot_id} | 상태:{status['robot_state']} | {battery_emoji}{status['battery']}% | 📦{status.get('book_weight', 0.0):.1f}kg | 📍({status.get('position_x', 0.0):.2f},{status.get('position_y', 0.0):.2f}) | 🧭{status.get('position_yaw', 0.0):.1f}°"
                active_robots.append(robot_info)
            
            if active_robots:
                # 각 로봇 정보를 한 줄씩 추가
                for robot_info in active_robots:
                    self.overall_status_text.append(robot_info)
                
                # 스크롤바를 맨 아래로 이동
                scrollbar = self.overall_status_text.verticalScrollBar()
                scrollbar.setValue(scrollbar.maximum())
                
                self.get_logger().debug(f"✅ OverallStatus 로그 추가: {len(active_robots)}개 로봇 정보")
            else:
                # 활성 로봇이 없을 때도 한 줄로 로그 추가
                no_robot_text = f"[{time_str}] ⚠️ 활성 로봇 없음"
                self.overall_status_text.append(no_robot_text)
                
                # 스크롤바를 맨 아래로 이동
                scrollbar = self.overall_status_text.verticalScrollBar()
                scrollbar.setValue(scrollbar.maximum())
                
                self.get_logger().debug("✅ OverallStatus 로그 추가: 활성 로봇 없음")
                
        except Exception as e:
            self.get_logger().error(f"❌ OverallStatus 업데이트 중 오류: {e}")
    
    def cleanup(self):
        """탭 정리 작업"""
        self.get_logger().info("🛑 System Logs Tab 정리 중...")
        
        # 타이머 정리
        if hasattr(self, 'overall_status_timer'):
            self.overall_status_timer.stop()
            self.get_logger().info("✅ OverallStatus 업데이트 타이머 정지됨")
    
    def get_logger(self):
        """ROS 로거 반환"""
        return self.ros_node.get_logger() 