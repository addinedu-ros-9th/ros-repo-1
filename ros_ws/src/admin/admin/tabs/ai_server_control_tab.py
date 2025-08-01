#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
import time
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit, QFrame
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QObject, Qt
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from libo_interfaces.srv import ActivateDetector, DeactivateDetector

class AiServerControlTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node
        self.detector_log = []
        self.server_active = False  # 서버 상태 (기본값: OFF)
        
        # ROS 서비스 서버들 (초기에는 None)
        self.activate_detector_service = None
        self.deactivate_detector_service = None
        
        self.init_ui()
        
        # 초기 로그 메시지
        self.log_detector_message("👁️ AI Server Detector Control 탭이 시작되었습니다.")
        self.log_detector_message("🔴 서버가 비활성화 상태입니다. 'Server ON' 버튼을 눌러 활성화하세요.")
    
    def init_ui(self):
        """UI 초기화 - ai_server_control_tab.ui 파일 로드"""
        ui_file_path = os.path.join(get_package_share_directory('admin'), 'ui', 'ai_server_control_tab.ui')
        uic.loadUi(ui_file_path, self)
        
        # 시그널 연결
        self.toggle_server_button.clicked.connect(self.toggle_server)
        self.clear_log_button.clicked.connect(self.clear_log)
        
        # 초기 버튼 상태 설정 (서버가 비활성화 상태이므로 OFF로 표시)
        self.toggle_server_button.setText("🔴 Server OFF")
        self.toggle_server_button.setStyleSheet("background-color: #e74c3c; color: white; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; font-size: 12px; min-height: 30px;")
    
    def toggle_server(self):
        """서버 ON/OFF 토글"""
        if self.server_active:
            # 서버 비활성화
            self.stop_server()
            self.toggle_server_button.setText("🔴 Server OFF")
            self.toggle_server_button.setStyleSheet("background-color: #e74c3c; color: white; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; font-size: 12px; min-height: 30px;")
            self.log_detector_message("🔴 서버가 비활성화되었습니다.")
        else:
            # 서버 활성화
            self.start_server()
            self.toggle_server_button.setText("🟢 Server ON")
            self.toggle_server_button.setStyleSheet("background-color: #27ae60; color: white; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; font-size: 12px; min-height: 30px;")
            self.log_detector_message("🟢 서버가 활성화되었습니다. TaskManager 요청을 받을 준비가 되었습니다.")
    
    def start_server(self):
        """ActivateDetector/DeactivateDetector 서비스 서버 시작"""
        try:
            # ActivateDetector 서비스 서버 생성
            self.activate_detector_service = self.ros_node.create_service(
                ActivateDetector,
                'activate_detector',
                self.activate_detector_service_callback
            )
            
            # DeactivateDetector 서비스 서버 생성
            self.deactivate_detector_service = self.ros_node.create_service(
                DeactivateDetector,
                'deactivate_detector',
                self.deactivate_detector_service_callback
            )
            
            self.server_active = True
            self.log_detector_message("✅ ActivateDetector/DeactivateDetector 서비스 서버가 시작되었습니다.")
            
        except Exception as e:
            self.log_detector_message(f"❌ 서버 시작 실패: {str(e)}")
            self.server_active = False
    
    def stop_server(self):
        """ActivateDetector/DeactivateDetector 서비스 서버 중지"""
        try:
            # 서비스 서버 제거
            if self.activate_detector_service:
                self.ros_node.destroy_service(self.activate_detector_service)
                self.activate_detector_service = None
            
            if self.deactivate_detector_service:
                self.ros_node.destroy_service(self.deactivate_detector_service)
                self.deactivate_detector_service = None
            
            self.server_active = False
            self.log_detector_message("🛑 ActivateDetector/DeactivateDetector 서비스 서버가 중지되었습니다.")
            
        except Exception as e:
            self.log_detector_message(f"❌ 서버 중지 실패: {str(e)}")
    
    def activate_detector_service_callback(self, request, response):
        """ActivateDetector 서비스 요청 처리 (TaskManager에서 호출)"""
        if not self.server_active:
            response.success = False
            response.message = "서버가 비활성화 상태입니다."
            return response
        
        robot_id = request.robot_id
        current_time = time.strftime('%H:%M:%S', time.localtime())
        
        self.log_detector_message(f"📤 ActivateDetector 요청 수신: {robot_id} at {current_time}")
        
        try:
            # 여기서 실제 감지기 활성화 로직 구현
            # 현재는 시뮬레이션으로 성공 응답
            response.success = True
            response.message = f"감지기 활성화 완료: {robot_id}"
            
            self.log_detector_message(f"✅ ActivateDetector 처리 완료: {robot_id}")
            self.log_detector_message(f"📤 응답 전송: SUCCESS - {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"감지기 활성화 실패: {str(e)}"
            
            self.log_detector_message(f"❌ ActivateDetector 처리 실패: {str(e)}")
            self.log_detector_message(f"📤 응답 전송: FAILED - {response.message}")
        
        return response
    
    def deactivate_detector_service_callback(self, request, response):
        """DeactivateDetector 서비스 요청 처리 (TaskManager에서 호출)"""
        if not self.server_active:
            response.success = False
            response.message = "서버가 비활성화 상태입니다."
            return response
        
        robot_id = request.robot_id
        current_time = time.strftime('%H:%M:%S', time.localtime())
        
        self.log_detector_message(f"📤 DeactivateDetector 요청 수신: {robot_id} at {current_time}")
        
        try:
            # 여기서 실제 감지기 비활성화 로직 구현
            # 현재는 시뮬레이션으로 성공 응답
            response.success = True
            response.message = f"감지기 비활성화 완료: {robot_id}"
            
            self.log_detector_message(f"✅ DeactivateDetector 처리 완료: {robot_id}")
            self.log_detector_message(f"📤 응답 전송: SUCCESS - {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"감지기 비활성화 실패: {str(e)}"
            
            self.log_detector_message(f"❌ DeactivateDetector 처리 실패: {str(e)}")
            self.log_detector_message(f"📤 응답 전송: FAILED - {response.message}")
        
        return response
    
    def log_detector_message(self, message):
        """Detector 로그 메시지 출력"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        # 로그 리스트에 추가
        self.detector_log.append(log_entry)
        
        # 최근 100개만 유지
        if len(self.detector_log) > 100:
            self.detector_log = self.detector_log[-100:]
        
        # UI 업데이트
        self.update_detector_log_display()
    
    def update_detector_log_display(self):
        """Detector 로그 표시 업데이트"""
        log_text = "\n".join(self.detector_log)
        self.detector_log_text.setPlainText(log_text)
        
        # 자동 스크롤
        cursor = self.detector_log_text.textCursor()
        cursor.movePosition(cursor.End)
        self.detector_log_text.setTextCursor(cursor)
    
    def clear_log(self):
        """로그 내용 지우기"""
        self.detector_log = []
        self.detector_log_text.clear()
        self.log_detector_message("🧹 로그가 지워졌습니다.")
    
    def get_log_count(self):
        """현재 로그 개수 반환"""
        return len(self.detector_log)
    
    def get_last_log_message(self):
        """마지막 로그 메시지 반환"""
        if self.detector_log:
            return self.detector_log[-1]
        return ""
    
    def cleanup(self):
        """탭 종료 시 정리"""
        if self.server_active:
            self.stop_server() 