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
        
        # ROS 클라이언트들
        self.activate_detector_client = self.ros_node.create_client(ActivateDetector, 'activate_detector')
        self.deactivate_detector_client = self.ros_node.create_client(DeactivateDetector, 'deactivate_detector')
        
        self.init_ui()
        self.init_connections()
    
    def init_ui(self):
        """UI 초기화 - ai_server_control_tab.ui 파일 로드"""
        ui_file_path = os.path.join(get_package_share_directory('admin'), 'ui', 'ai_server_control_tab.ui')
        uic.loadUi(ui_file_path, self)
        
        # 시그널 연결
        self.activate_detector_button.clicked.connect(self.activate_detector)
        self.deactivate_detector_button.clicked.connect(self.deactivate_detector)
        self.clear_log_button.clicked.connect(self.clear_log)
        
        # 초기 로그 메시지
        self.log_detector_message("👁️ AI Server Detector Control 탭이 시작되었습니다.")
        self.log_detector_message("로봇 ID를 입력하고 Activate/Deactivate 버튼을 사용하세요.")
    
    def init_connections(self):
        """서비스 연결 상태 확인"""
        # 서비스 연결 상태를 주기적으로 확인
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self.check_service_connections)
        self.connection_timer.start(5000)  # 5초마다 확인
        
        # 초기 연결 상태 확인
        self.check_service_connections()
    
    def check_service_connections(self):
        """서비스 연결 상태 확인 및 버튼 활성화/비활성화"""
        activate_available = self.activate_detector_client.wait_for_service(timeout_sec=0.1)
        deactivate_available = self.deactivate_detector_client.wait_for_service(timeout_sec=0.1)
        
        # ActivateDetector 서비스 상태
        if activate_available:
            self.activate_detector_button.setEnabled(True)
            self.activate_detector_button.setToolTip("ActivateDetector 서비스 사용 가능")
        else:
            self.activate_detector_button.setEnabled(False)
            self.activate_detector_button.setToolTip("ActivateDetector 서비스를 찾을 수 없음")
        
        # DeactivateDetector 서비스 상태
        if deactivate_available:
            self.deactivate_detector_button.setEnabled(True)
            self.deactivate_detector_button.setToolTip("DeactivateDetector 서비스 사용 가능")
        else:
            self.deactivate_detector_button.setEnabled(False)
            self.deactivate_detector_button.setToolTip("DeactivateDetector 서비스를 찾을 수 없음")
        
        # 연결 상태 로그 (변경사항이 있을 때만)
        if not hasattr(self, '_last_activate_status'):
            self._last_activate_status = None
            self._last_deactivate_status = None
        
        if self._last_activate_status != activate_available:
            if activate_available:
                self.log_detector_message("✅ ActivateDetector 서비스 연결됨")
            else:
                self.log_detector_message("❌ ActivateDetector 서비스 연결 끊어짐")
            self._last_activate_status = activate_available
        
        if self._last_deactivate_status != deactivate_available:
            if deactivate_available:
                self.log_detector_message("✅ DeactivateDetector 서비스 연결됨")
            else:
                self.log_detector_message("❌ DeactivateDetector 서비스 연결 끊어짐")
            self._last_deactivate_status = deactivate_available
    
    def activate_detector(self):
        """ActivateDetector 서비스 호출"""
        robot_id = self.detector_robot_id_edit.text().strip()
        
        if not robot_id:
            self.log_detector_message("❌ 로봇 ID를 입력해주세요.")
            return
        
        if not self.activate_detector_client.wait_for_service(timeout_sec=1.0):
            self.log_detector_message("❌ ActivateDetector 서비스를 찾을 수 없습니다.")
            return
        
        # 버튼 비활성화 (중복 클릭 방지)
        self.activate_detector_button.setEnabled(False)
        self.activate_detector_button.setText("⏳ Activating...")
        
        # 서비스 요청
        request = ActivateDetector.Request()
        request.robot_id = robot_id
        
        future = self.activate_detector_client.call_async(request)
        future.add_done_callback(self.activate_detector_callback)
        
        self.log_detector_message(f"📤 ActivateDetector 요청 전송: {robot_id}")
    
    def activate_detector_callback(self, future):
        """ActivateDetector 응답 처리"""
        try:
            response = future.result()
            if response.success:
                self.log_detector_message(f"✅ ActivateDetector 성공: {response.message}")
            else:
                self.log_detector_message(f"❌ ActivateDetector 실패: {response.message}")
        except Exception as e:
            self.log_detector_message(f"❌ ActivateDetector 오류: {str(e)}")
        finally:
            # 버튼 상태 복원
            self.activate_detector_button.setEnabled(True)
            self.activate_detector_button.setText("🟢 Activate Detector")
    
    def deactivate_detector(self):
        """DeactivateDetector 서비스 호출"""
        robot_id = self.detector_robot_id_edit.text().strip()
        
        if not robot_id:
            self.log_detector_message("❌ 로봇 ID를 입력해주세요.")
            return
        
        if not self.deactivate_detector_client.wait_for_service(timeout_sec=1.0):
            self.log_detector_message("❌ DeactivateDetector 서비스를 찾을 수 없습니다.")
            return
        
        # 버튼 비활성화 (중복 클릭 방지)
        self.deactivate_detector_button.setEnabled(False)
        self.deactivate_detector_button.setText("⏳ Deactivating...")
        
        # 서비스 요청
        request = DeactivateDetector.Request()
        request.robot_id = robot_id
        
        future = self.deactivate_detector_client.call_async(request)
        future.add_done_callback(self.deactivate_detector_callback)
        
        self.log_detector_message(f"📤 DeactivateDetector 요청 전송: {robot_id}")
    
    def deactivate_detector_callback(self, future):
        """DeactivateDetector 응답 처리"""
        try:
            response = future.result()
            if response.success:
                self.log_detector_message(f"✅ DeactivateDetector 성공: {response.message}")
            else:
                self.log_detector_message(f"❌ DeactivateDetector 실패: {response.message}")
        except Exception as e:
            self.log_detector_message(f"❌ DeactivateDetector 오류: {str(e)}")
        finally:
            # 버튼 상태 복원
            self.deactivate_detector_button.setEnabled(True)
            self.deactivate_detector_button.setText("🔴 Deactivate Detector")
    
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
        # 현재는 특별한 정리 작업이 없음
        pass 