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
from libo_interfaces.srv import ActivateQRScanner, DeactivateQRScanner  # QR Scanner 서비스 추가
from libo_interfaces.srv import EndTask  # EndTask 서비스 추가
from libo_interfaces.srv import RobotQRCheck  # RobotQRCheck 서비스 추가
from libo_interfaces.msg import DetectionTimer  # DetectionTimer 메시지 추가
from libo_interfaces.msg import VoiceCommand  # VoiceCommand 메시지 추가

class AiServerControlTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node
        self.detector_log = []
        self.detection_timer_log = []  # DetectionTimer 전용 로그
        self.server_active = False  # 서버 상태 (기본값: OFF)
        self.detection_timer_active = False  # DetectionTimer 발행 상태 (기본값: OFF)
        
        # VoiceCommand 구독 관련 변수들
        self.voice_command_log = []  # VoiceCommand 전용 로그
        self.voice_subscription_active = False  # VoiceCommand 구독 상태 (기본값: OFF)
        self.voice_command_subscription = None  # VoiceCommand 구독자
        
        # DetectionTimer 발행 관련 변수들
        self.detection_timer_publisher = None  # DetectionTimer 퍼블리셔
        self.detection_counter = 0  # DetectionTimer 카운터
        self.detection_timer = QTimer()  # DetectionTimer 발행 타이머
        self.detection_timer.timeout.connect(self.publish_detection_timer)
        
        # ROS 서비스 서버들 (초기에는 None)
        self.activate_detector_service = None
        self.deactivate_detector_service = None
        self.activate_qr_scanner_service = None  # QR Scanner 활성화 서비스
        self.deactivate_qr_scanner_service = None  # QR Scanner 비활성화 서비스
        
        # EndTask 서비스 클라이언트
        self.end_task_client = None
        self.end_task_robot_id = "libo_a"  # 기본 로봇 ID
        self.end_task_type = "assist"  # 기본 작업 타입
        
        # RobotQRCheck 서비스 클라이언트
        self.robot_qr_check_client = None
        self.robot_qr_check_robot_id = "libo_a"  # 기본 로봇 ID
        self.robot_qr_check_admin_name = "김민수"  # 기본 관리자 이름
        
        self.init_ui()
        
        # EndTask 클라이언트 초기화 (서버 활성화와 독립적)
        self.end_task_client = self.ros_node.create_client(EndTask, 'end_task')
        
        # RobotQRCheck 클라이언트 초기화 (서버 활성화와 독립적)
        self.robot_qr_check_client = self.ros_node.create_client(RobotQRCheck, 'robot_qr_check')
        
        # 초기 로그 메시지
        self.log_detector_message("👁️ Vision Manager Control 탭이 시작되었습니다.")
        self.log_detector_message("🔴 서버가 비활성화 상태입니다. 'Server ON' 버튼을 눌러 활성화하세요.")
        self.log_detection_timer_message("⏰ DetectionTimer Control이 시작되었습니다.")
        self.log_detection_timer_message("🔴 DetectionTimer가 비활성화 상태입니다.")
        self.log_voice_command_message("🗣️ Talker Manager Control이 시작되었습니다.")
        self.log_voice_command_message("🔴 VoiceCommand 구독이 비활성화 상태입니다.")
        self.log_voice_command_message("🏁 EndTask 기능이 준비되었습니다. (Vision Manager와 독립적)")
        self.log_detector_message("🔍 RobotQRCheck 기능이 준비되었습니다. (Vision Manager와 독립적)")
    
    def init_ui(self):
        """UI 초기화 - ai_server_control_tab.ui 파일 로드"""
        ui_file_path = os.path.join(get_package_share_directory('admin'), 'ui', 'ai_server_control_tab.ui')
        uic.loadUi(ui_file_path, self)
        
        # 시그널 연결
        self.toggle_server_button.clicked.connect(self.toggle_server)
        self.clear_log_button.clicked.connect(self.clear_log)
        self.send_robot_qr_check_button.clicked.connect(self.send_robot_qr_check)
        
        # DetectionTimer 관련 시그널 연결
        self.toggle_detection_timer_button.clicked.connect(self.toggle_detection_timer)
        self.clear_detection_log_button.clicked.connect(self.clear_detection_timer_log)
        
        # VoiceCommand 관련 시그널 연결
        self.toggle_voice_subscription_button.clicked.connect(self.toggle_voice_subscription)
        self.clear_voice_log_button.clicked.connect(self.clear_voice_command_log)
        
        # EndTask 관련 시그널 연결
        self.end_task_button.clicked.connect(self.send_end_task)
        
        # 초기 버튼 상태 설정 (서버가 비활성화 상태이므로 OFF로 표시)
        self.toggle_server_button.setText("🔴 Server OFF")
        self.toggle_server_button.setStyleSheet("background-color: #e74c3c; color: white; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; font-size: 12px; min-height: 30px;")
        
        # DetectionTimer 버튼 초기 상태
        self.toggle_detection_timer_button.setText("▶️ Start Timer")
        self.toggle_detection_timer_button.setStyleSheet("background-color: #27ae60; color: white; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; font-size: 12px; min-height: 30px;")
        
        # 카운터 표시 초기화
        self.counter_display.setText("0")
    
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
            
            # ActivateQRScanner 서비스 서버 생성
            self.activate_qr_scanner_service = self.ros_node.create_service(
                ActivateQRScanner,
                'activate_qr_scanner',
                self.activate_qr_scanner_service_callback
            )
            
            # DeactivateQRScanner 서비스 서버 생성
            self.deactivate_qr_scanner_service = self.ros_node.create_service(
                DeactivateQRScanner,
                'deactivate_qr_scanner',
                self.deactivate_qr_scanner_service_callback
            )
            
            # DetectionTimer 퍼블리셔 생성
            self.detection_timer_publisher = self.ros_node.create_publisher(
                DetectionTimer,
                'detection_timer',
                10
            )
            
            self.server_active = True
            
            # DetectionTimer 발행 시작 (1초마다)
            self.detection_counter = 0
            self.detection_timer.start(1000)  # 1초마다 발행
            
            self.log_detector_message("✅ ActivateDetector/DeactivateDetector 서비스 서버가 시작되었습니다.")
            self.log_detector_message("✅ ActivateQRScanner/DeactivateQRScanner 서비스 서버가 시작되었습니다.")
            self.log_detector_message("⏰ DetectionTimer 발행이 시작되었습니다. (1초마다)")
            
        except Exception as e:
            self.log_detector_message(f"❌ 서버 시작 실패: {str(e)}")
            self.server_active = False
    
    def stop_server(self):
        """ActivateDetector/DeactivateDetector 서비스 서버 중지"""
        try:
            # DetectionTimer 발행 중지
            if self.detection_timer.isActive():
                self.detection_timer.stop()
                self.detection_counter = 0
                self.log_detector_message("⏹️ DetectionTimer 발행이 중지되었습니다.")
            
            # DetectionTimer 퍼블리셔 제거
            if self.detection_timer_publisher:
                self.detection_timer_publisher = None
            
            # 서비스 서버 제거
            if self.activate_detector_service:
                self.ros_node.destroy_service(self.activate_detector_service)
                self.activate_detector_service = None
            
            if self.deactivate_detector_service:
                self.ros_node.destroy_service(self.deactivate_detector_service)
                self.deactivate_detector_service = None
            
            if self.activate_qr_scanner_service:
                self.ros_node.destroy_service(self.activate_qr_scanner_service)
                self.activate_qr_scanner_service = None
            
            if self.deactivate_qr_scanner_service:
                self.ros_node.destroy_service(self.deactivate_qr_scanner_service)
                self.deactivate_qr_scanner_service = None
            
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
    
    def activate_qr_scanner_service_callback(self, request, response):
        """ActivateQRScanner 서비스 요청 처리 (TaskManager에서 호출)"""
        if not self.server_active:
            response.success = False
            response.message = "서버가 비활성화 상태입니다."
            return response
        
        robot_id = request.robot_id
        current_time = time.strftime('%H:%M:%S', time.localtime())
        
        self.log_detector_message(f"📤 ActivateQRScanner 요청 수신: {robot_id} at {current_time}")
        
        try:
            # 여기서 실제 QR 스캐너 활성화 로직 구현
            # 현재는 시뮬레이션으로 성공 응답
            response.success = True
            response.message = f"QR 스캐너 활성화 완료: {robot_id}"
            
            self.log_detector_message(f"✅ ActivateQRScanner 처리 완료: {robot_id}")
            self.log_detector_message(f"📤 응답 전송: SUCCESS - {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"QR 스캐너 활성화 실패: {str(e)}"
            
            self.log_detector_message(f"❌ ActivateQRScanner 처리 실패: {str(e)}")
            self.log_detector_message(f"📤 응답 전송: FAILED - {response.message}")
        
        return response
    
    def deactivate_qr_scanner_service_callback(self, request, response):
        """DeactivateQRScanner 서비스 요청 처리 (TaskManager에서 호출)"""
        if not self.server_active:
            response.success = False
            response.message = "서버가 비활성화 상태입니다."
            return response
        
        robot_id = request.robot_id
        current_time = time.strftime('%H:%M:%S', time.localtime())
        
        self.log_detector_message(f"📤 DeactivateQRScanner 요청 수신: {robot_id} at {current_time}")
        
        try:
            # 여기서 실제 QR 스캐너 비활성화 로직 구현
            # 현재는 시뮬레이션으로 성공 응답
            response.success = True
            response.message = f"QR 스캐너 비활성화 완료: {robot_id}"
            
            self.log_detector_message(f"✅ DeactivateQRScanner 처리 완료: {robot_id}")
            self.log_detector_message(f"📤 응답 전송: SUCCESS - {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"QR 스캐너 비활성화 실패: {str(e)}"
            
            self.log_detector_message(f"❌ DeactivateQRScanner 처리 실패: {str(e)}")
            self.log_detector_message(f"📤 응답 전송: FAILED - {response.message}")
        
        return response
    
    def toggle_detection_timer(self):
        """DetectionTimer ON/OFF 토글"""
        if self.detection_timer_active:
            # DetectionTimer 비활성화
            self.stop_detection_timer()
            self.toggle_detection_timer_button.setText("▶️ Start Timer")
            self.toggle_detection_timer_button.setStyleSheet("background-color: #27ae60; color: white; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; font-size: 12px; min-height: 30px;")
            self.log_detection_timer_message("⏹️ DetectionTimer가 중지되었습니다.")
        else:
            # DetectionTimer 활성화
            self.start_detection_timer()
            self.toggle_detection_timer_button.setText("⏹️ Stop Timer")
            self.toggle_detection_timer_button.setStyleSheet("background-color: #e74c3c; color: white; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; font-size: 12px; min-height: 30px;")
            self.log_detection_timer_message("▶️ DetectionTimer가 시작되었습니다. (2초마다 발행)")
    
    def start_detection_timer(self):
        """DetectionTimer 발행 시작"""
        try:
            # DetectionTimer 퍼블리셔 생성 (아직 없다면)
            if not self.detection_timer_publisher:
                self.detection_timer_publisher = self.ros_node.create_publisher(
                    DetectionTimer,
                    'detection_timer',
                    10
                )
                self.log_detection_timer_message("✅ DetectionTimer 퍼블리셔가 생성되었습니다.")
            
            # 카운터 초기화
            self.detection_counter = 0
            self.counter_display.setText("0")
            
            # 타이머 시작 (1초마다)
            self.detection_timer.start(1000)
            self.detection_timer_active = True
            
            self.log_detection_timer_message("⏰ DetectionTimer 발행이 시작되었습니다. (1초마다)")
            
        except Exception as e:
            self.log_detection_timer_message(f"❌ DetectionTimer 시작 실패: {str(e)}")
            self.detection_timer_active = False
    
    def stop_detection_timer(self):
        """DetectionTimer 발행 중지"""
        try:
            # 타이머 중지
            if self.detection_timer.isActive():
                self.detection_timer.stop()
                self.detection_timer_active = False
                self.log_detection_timer_message("⏹️ DetectionTimer 발행이 중지되었습니다.")
            
            # 카운터 초기화
            self.detection_counter = 0
            self.counter_display.setText("0")
            
        except Exception as e:
            self.log_detection_timer_message(f"❌ DetectionTimer 중지 실패: {str(e)}")
    
    def clear_detection_timer_log(self):
        """DetectionTimer 로그 내용 지우기"""
        self.detection_timer_log = []
        self.detection_timer_log_text.clear()
        self.log_detection_timer_message("🧹 DetectionTimer 로그가 지워졌습니다.")
    
    def publish_detection_timer(self):
        """DetectionTimer 메시지 발행"""
        if not self.detection_timer_active or not self.detection_timer_publisher:
            return
        
        try:
            # 카운터 증가
            self.detection_counter += 1
            
            # UI 카운터 표시 업데이트
            self.counter_display.setText(str(self.detection_counter))
            
            # DetectionTimer 메시지 생성
            detection_msg = DetectionTimer()
            detection_msg.robot_id = self.robot_id_edit.text()  # UI에서 입력받은 로봇 ID
            detection_msg.command = str(self.detection_counter)  # 카운터를 문자열로 변환
            
            # 메시지 발행
            self.detection_timer_publisher.publish(detection_msg)
            
            # 로그 출력
            self.log_detection_timer_message(f"⏰ DetectionTimer 발행: robot_id={detection_msg.robot_id}, command={detection_msg.command}")
            
        except Exception as e:
            self.log_detection_timer_message(f"❌ DetectionTimer 발행 실패: {str(e)}")
    
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
    
    def send_robot_qr_check(self):
        """RobotQRCheck 서비스 요청 발행"""
        if not self.robot_qr_check_client:
            self.log_detector_message("❌ RobotQRCheck 클라이언트가 초기화되지 않았습니다.")
            return
        
        try:
            # UI에서 입력된 값 읽기
            robot_id = self.qr_robot_id_edit.text().strip()
            admin_name = self.qr_admin_name_edit.text().strip()
            
            # 입력값 검증
            if not robot_id:
                self.log_detector_message("❌ Robot ID를 입력해주세요.")
                return
            
            if not admin_name:
                self.log_detector_message("❌ Admin Name을 입력해주세요.")
                return
            
            # RobotQRCheck 서비스 요청 생성
            request = RobotQRCheck.Request()
            request.robot_id = robot_id
            request.admin_name = admin_name
            
            self.log_detector_message(f"📤 RobotQRCheck 요청 발행: robot_id={request.robot_id}, admin_name={request.admin_name}")
            
            # 비동기 서비스 호출
            future = self.robot_qr_check_client.call_async(request)
            future.add_done_callback(self.robot_qr_check_response_callback)
            
        except Exception as e:
            self.log_detector_message(f"❌ RobotQRCheck 요청 발행 실패: {str(e)}")
    
    def robot_qr_check_response_callback(self, future):
        """RobotQRCheck 서비스 응답 처리"""
        try:
            response = future.result()
            if response.success:
                self.log_detector_message(f"✅ RobotQRCheck 성공: {response.message}")
            else:
                self.log_detector_message(f"❌ RobotQRCheck 실패: {response.message}")
        except Exception as e:
            self.log_detector_message(f"❌ RobotQRCheck 응답 처리 중 오류: {str(e)}")
    
    def get_log_count(self):
        """현재 로그 개수 반환"""
        return len(self.detector_log)
    
    def get_last_log_message(self):
        """마지막 로그 메시지 반환"""
        if self.detector_log:
            return self.detector_log[-1]
        return ""
    
    def log_detection_timer_message(self, message):
        """DetectionTimer 전용 로그 메시지 출력"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        # 로그 리스트에 추가
        self.detection_timer_log.append(log_entry)
        
        # 최근 50개만 유지
        if len(self.detection_timer_log) > 50:
            self.detection_timer_log = self.detection_timer_log[-50:]
        
        # UI 업데이트
        self.update_detection_timer_log_display()
    
    def update_detection_timer_log_display(self):
        """DetectionTimer 로그 표시 업데이트"""
        log_text = "\n".join(self.detection_timer_log)
        self.detection_timer_log_text.setPlainText(log_text)
        
        # 자동 스크롤
        cursor = self.detection_timer_log_text.textCursor()
        cursor.movePosition(cursor.End)
        self.detection_timer_log_text.setTextCursor(cursor)
    
    def cleanup(self):
        """탭 종료 시 정리"""
        if self.server_active:
            self.stop_server()
        if self.detection_timer_active:
            self.stop_detection_timer()
        if self.voice_subscription_active:
            self.stop_voice_subscription()
    
    def toggle_voice_subscription(self):
        """VoiceCommand 구독 ON/OFF 토글"""
        if self.voice_subscription_active:
            # VoiceCommand 구독 비활성화
            self.stop_voice_subscription()
            self.toggle_voice_subscription_button.setText("🔴 구독 OFF")
            self.toggle_voice_subscription_button.setStyleSheet("background-color: #e74c3c; color: white; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; font-size: 12px; min-height: 30px;")
            self.subscription_status_display.setText("🔴 비활성화")
            self.subscription_status_display.setStyleSheet("font-weight: bold; color: #e74c3c;")
            self.log_voice_command_message("🔴 VoiceCommand 구독이 중지되었습니다.")
        else:
            # VoiceCommand 구독 활성화
            self.start_voice_subscription()
            self.toggle_voice_subscription_button.setText("🟢 구독 ON")
            self.toggle_voice_subscription_button.setStyleSheet("background-color: #27ae60; color: white; border: none; padding: 10px 15px; border-radius: 5px; font-weight: bold; font-size: 12px; min-height: 30px;")
            self.subscription_status_display.setText("🟢 활성화")
            self.subscription_status_display.setStyleSheet("font-weight: bold; color: #27ae60;")
            self.log_voice_command_message("🟢 VoiceCommand 구독이 시작되었습니다.")
    
    def start_voice_subscription(self):
        """VoiceCommand 구독 시작"""
        try:
            # VoiceCommand 구독자 생성
            self.voice_command_subscription = self.ros_node.create_subscription(
                VoiceCommand,
                'voice_command',
                self.voice_command_callback,
                10
            )
            
            self.voice_subscription_active = True
            self.log_voice_command_message("✅ VoiceCommand 구독자가 생성되었습니다.")
            self.log_voice_command_message("📡 TaskManager의 VoiceCommand 메시지를 모니터링 중...")
            
        except Exception as e:
            self.log_voice_command_message(f"❌ VoiceCommand 구독 시작 실패: {str(e)}")
            self.voice_subscription_active = False
    
    def stop_voice_subscription(self):
        """VoiceCommand 구독 중지"""
        try:
            # 구독자 제거
            if self.voice_command_subscription:
                self.ros_node.destroy_subscription(self.voice_command_subscription)
                self.voice_command_subscription = None
            
            self.voice_subscription_active = False
            self.log_voice_command_message("⏹️ VoiceCommand 구독이 중지되었습니다.")
            
        except Exception as e:
            self.log_voice_command_message(f"❌ VoiceCommand 구독 중지 실패: {str(e)}")
    
    def voice_command_callback(self, msg):
        """VoiceCommand 메시지 수신 콜백"""
        try:
            robot_id = msg.robot_id
            category = msg.category
            action = msg.action
            current_time = time.strftime('%H:%M:%S', time.localtime())
            
            # 로그 메시지 생성
            log_message = f"📥 VoiceCommand 수신: robot_id={robot_id}, category={category}, action={action} at {current_time}"
            self.log_voice_command_message(log_message)
            
            # 카테고리별 아이콘 추가
            category_icons = {
                "common": "🔧",
                "escort": "🚶", 
                "delivery": "📦",
                "assist": "🤝"
            }
            
            icon = category_icons.get(category, "❓")
            detail_message = f"{icon} {category.upper()}: {action}"
            self.log_voice_command_message(f"   → {detail_message}")
            
        except Exception as e:
            self.log_voice_command_message(f"❌ VoiceCommand 처리 중 오류: {str(e)}")
    
    def clear_voice_command_log(self):
        """VoiceCommand 로그 내용 지우기"""
        self.voice_command_log = []
        self.voice_command_log_text.clear()
        self.log_voice_command_message("🧹 VoiceCommand 로그가 지워졌습니다.")
    
    def send_end_task(self):
        """EndTask 서비스 요청 발행"""
        if not self.end_task_client:
            self.log_voice_command_message("❌ EndTask 클라이언트가 초기화되지 않았습니다.")
            return
        
        try:
            # UI에서 입력된 값 읽기
            robot_id = self.end_task_robot_id_edit.text().strip()
            task_type = self.end_task_type_edit.text().strip()
            
            # 입력값 검증
            if not robot_id:
                self.log_voice_command_message("❌ Robot ID를 입력해주세요.")
                return
            
            if not task_type:
                self.log_voice_command_message("❌ Task Type을 입력해주세요.")
                return
            
            # EndTask 서비스 요청 생성
            request = EndTask.Request()
            request.robot_id = robot_id
            request.task_type = task_type
            
            self.log_voice_command_message(f"📤 EndTask 요청 발행: robot_id={request.robot_id}, task_type={request.task_type}")
            
            # 비동기 서비스 호출
            future = self.end_task_client.call_async(request)
            future.add_done_callback(self.end_task_response_callback)
            
        except Exception as e:
            self.log_voice_command_message(f"❌ EndTask 요청 발행 실패: {str(e)}")
    
    def end_task_response_callback(self, future):
        """EndTask 서비스 응답 처리"""
        try:
            response = future.result()
            if response.success:
                self.log_voice_command_message(f"✅ EndTask 성공: {response.message}")
            else:
                self.log_voice_command_message(f"❌ EndTask 실패: {response.message}")
        except Exception as e:
            self.log_voice_command_message(f"❌ EndTask 응답 처리 중 오류: {str(e)}")
    
    def log_voice_command_message(self, message):
        """VoiceCommand 전용 로그 메시지 출력"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        # 로그 리스트에 추가
        self.voice_command_log.append(log_entry)
        
        # 최근 50개만 유지
        if len(self.voice_command_log) > 50:
            self.voice_command_log = self.voice_command_log[-50:]
        
        # UI 업데이트
        self.update_voice_command_log_display()
    
    def update_voice_command_log_display(self):
        """VoiceCommand 로그 표시 업데이트"""
        log_text = "\n".join(self.voice_command_log)
        self.voice_command_log_text.setPlainText(log_text)
        
        # 자동 스크롤
        cursor = self.voice_command_log_text.textCursor()
        cursor.movePosition(cursor.End)
        self.voice_command_log_text.setTextCursor(cursor) 