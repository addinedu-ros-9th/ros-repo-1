#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
import time  # 시간 추적용 추가
import random  # 랜덤 값 생성용 추가
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit, QTableWidget, QTableWidgetItem, QFrame, QScrollArea # 통합 위젯들 추가
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QObject, Qt # 스레드 관련 추가
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node # ROS2 노드 클래스 임포트
from PyQt5.QtGui import QColor # QColor 임포트

from admin.tabs.task_request_tab import TaskRequestTab # 우리가 만든 TaskRequestTab을 임포트
from admin.tabs.heartbeat_monitor_tab import HeartbeatMonitorTab # 새로 만든 HeartbeatMonitorTab을 임포트
from admin.tabs.navigator_tab import NavigatorTab # 새로 만든 NavigatorTab을 임포트
from libo_interfaces.msg import OverallStatus  # OverallStatus 메시지 임포트 (String 대신)
from libo_interfaces.msg import TaskStatus  # TaskStatus 메시지 임포트
from libo_interfaces.msg import Heartbeat  # Heartbeat 메시지 임포트 (통합용)
from libo_interfaces.srv import TaskRequest  # TaskRequest 서비스 임포트 (통합용)
from libo_interfaces.srv import SetGoal  # SetGoal 서비스 임포트 (통합용)
from libo_interfaces.srv import NavigationResult  # NavigationResult 서비스 임포트 (통합용)

# ===== 통합 기능을 위한 헬퍼 클래스들 =====

class TaskRequestClientNode(Node):  # ROS2 서비스 클라이언트 역할을 하는 별도의 노드 클래스
    def __init__(self):
        super().__init__('integrated_task_request_client_node', automatically_declare_parameters_from_overrides=True)
        self.client = self.create_client(TaskRequest, '/task_request')  # TaskRequest 서비스 클라이언트를 생성
        self.get_logger().info('통합 TaskRequestClientNode가 준비되었습니다.')

    def send_request(self, robot_id, task_type, call_location, goal_location):
        if not self.client.wait_for_service(timeout_sec=1.0):  # 서비스가 1초 내에 사용 가능한지 확인
            self.get_logger().error('서비스가 사용 가능하지 않습니다. 요청을 보낼 수 없습니다.')
            return None  # 사용 불가능하면 None을 반환

        request = TaskRequest.Request()  # 서비스 요청 객체를 생성
        request.robot_id = robot_id
        request.task_type = task_type
        request.call_location = call_location
        request.goal_location = goal_location

        future = self.client.call_async(request)  # 비동기적으로 서비스 요청을 보냄
        return future  # 요청의 결과를 담고 있는 future 객체를 반환

class RequestWorker(QThread):  # GUI가 멈추지 않도록 백그라운드에서 ROS2 요청을 처리하는 스레드
    response_received = pyqtSignal(object)  # 응답을 전달하는 시그널

    def __init__(self, client_node, robot_id, task_type, call_location, goal_location):
        super().__init__()
        self.client_node = client_node
        self.robot_id = robot_id
        self.task_type = task_type
        self.call_location = call_location
        self.goal_location = goal_location

    def run(self):  # 스레드가 시작될 때 실행되는 함수
        future = self.client_node.send_request(self.robot_id, self.task_type, self.call_location, self.goal_location)  # 서비스 요청
        if future:
            while rclpy.ok() and not future.done():  # future가 완료될 때까지 기다림
                time.sleep(0.1)  # CPU 사용을 줄이기 위해 0.1초 대기
            
            if future.done():  # future가 완료되면
                self.response_received.emit(future.result())  # 결과와 함께 시그널을 발생시킴
        else:
            self.response_received.emit(None)  # future가 없으면 None과 함께 시그널 발생

class RosSignalBridge(QObject):  # Qt의 시그널을 사용하기 위해 QObject를 상속받는 중간 다리 역할 클래스
    heartbeat_received_signal = pyqtSignal(object)  # Heartbeat 메시지를 전달할 시그널

class HeartbeatSubscriberNode(Node):  # ROS2 통신(구독)을 전담할 별도의 노드 클래스
    def __init__(self, signal_bridge):  # 시그널을 발생시킬 bridge 객체를 외부에서 받음
        super().__init__('integrated_heartbeat_monitor_subscriber_node')  # 노드 이름 초기화
        self.signal_bridge = signal_bridge  # 전달받은 bridge 객체를 저장
        self.get_logger().info('💓 통합 Heartbeat 구독 노드 생성됨. "heartbeat" 토픽을 구독합니다.')

        qos_profile = rclpy.qos.QoSProfile(  # QoS 프로파일 생성
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # 신뢰성 정책: 최선 노력
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,  # 내구성 정책: 휘발성
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,  # 히스토리 정책: 마지막 N개만 유지
            depth=10  # 히스토리 깊이(큐 사이즈)
        )

        self.subscription = self.create_subscription(  # 토픽 구독자 생성
            Heartbeat, 'heartbeat', self.listener_callback, qos_profile)

    def listener_callback(self, msg):
        self.signal_bridge.heartbeat_received_signal.emit(msg)  # 받은 메시지를 시그널로 GUI에 전달

class NavigatorServerNode(Node):  # SetGoal 서비스 서버 노드
    def __init__(self):
        super().__init__('integrated_navigator_debug_server', automatically_declare_parameters_from_overrides=True)
        
        # 서비스 서버는 처음에 None (비활성 상태)
        self.service = None
        self.is_active = False  # 서버 활성화 상태
        
        # 수신된 메시지를 저장할 리스트
        self.received_messages = []
        
        self.get_logger().info('🧭 통합 Navigator 디버그 서버 생성됨 (비활성 상태)')
    
    def start_service(self):  # 서비스 서버 시작
        """SetGoal 서비스 서버를 시작"""
        if self.service is None:
            try:
                self.service = self.create_service(
                    SetGoal,
                    'set_navigation_goal',
                    self.set_goal_callback
                )
                self.is_active = True
                self.get_logger().info('✅ 통합 Navigator 디버그 서버 활성화됨 - set_navigation_goal 서비스 대기 중...')
                return True
            except Exception as e:
                self.get_logger().error(f'❌ 서비스 시작 실패: {e}')
                return False
        return True
    
    def stop_service(self):  # 서비스 서버 중지
        """SetGoal 서비스 서버를 중지"""
        if self.service is not None:
            try:
                self.destroy_service(self.service)
                self.service = None
                self.is_active = False
                self.get_logger().info('🔴 통합 Navigator 디버그 서버 비활성화됨')
                return True
            except Exception as e:
                self.get_logger().error(f'❌ 서비스 중지 실패: {e}')
                return False
        return True
    
    def set_goal_callback(self, request, response):  # SetGoal 서비스 콜백
        """SetGoal 요청을 받아서 처리하는 콜백"""
        current_time = time.strftime('%H:%M:%S', time.localtime())
        
        try:
            # 수신 정보 저장
            message_info = {
                'time': current_time,
                'x': request.x,
                'y': request.y,
                'status': 'received'  # 수신 상태 추가
            }
            self.received_messages.append(message_info)
            
            # 로그 출력
            self.get_logger().info(f'🎯 SetGoal 수신: ({request.x}, {request.y}) at {current_time}')
            
            # 성공 응답 생성
            response.success = True
            response.message = f"통합 디버그 서버에서 수신 완료: ({request.x}, {request.y}) at {current_time}"
            
            # 응답 상태 업데이트
            message_info['status'] = 'responded'
            message_info['response'] = 'SUCCESS'
            
            self.get_logger().info(f'✅ SetGoal 응답 전송: SUCCESS - {response.message}')
            
            return response
            
        except Exception as e:
            # 에러 처리
            self.get_logger().error(f'❌ SetGoal 처리 중 오류: {e}')
            
            # 실패 응답 생성
            response.success = False
            response.message = f"통합 디버그 서버 오류: {str(e)}"
            
            # 에러 상태 저장
            if 'message_info' in locals():
                message_info['status'] = 'error'
                message_info['response'] = f'ERROR: {str(e)}'
            
            return response
    
    def get_latest_messages(self, count=10):  # 최근 메시지 가져오기
        """최근 수신된 메시지들을 반환"""
        return self.received_messages[-count:] if self.received_messages else []

class AdminWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ros_node = rclpy.create_node('admin_gui_node')  # GUI 전체에서 사용할 ROS 노드 생성
        self.robot_status_dict = {}  # 로봇 상태를 저장할 딕셔너리 (문자열 대신)
        self.task_status_data = {}  # 작업 상태를 저장할 딕셔너리
        
        # 통합 기능을 위한 노드들 생성
        self.task_request_client_node = TaskRequestClientNode()  # TaskRequest 클라이언트 노드
        self.signal_bridge = RosSignalBridge()  # Heartbeat 시그널 브리지
        self.heartbeat_subscriber_node = HeartbeatSubscriberNode(self.signal_bridge)  # Heartbeat 구독 노드
        self.navigator_server_node = NavigatorServerNode()  # Navigator 서버 노드
        
        # NavigationResult 서비스 클라이언트 생성
        self.navigation_result_client = self.ros_node.create_client(NavigationResult, 'navigation_result')
        
        # 추가 데이터 저장
        self.heartbeat_log = []  # Heartbeat 로그 리스트
        self.heartbeat_start_time = time.time()  # Heartbeat 탭이 생성된 시간을 기록
        self.navigation_result_logs = []  # NavigationResult 로그를 저장할 리스트
        
        self.init_integrated_ui()  # 통합 UI 초기화
        self.init_robot_status_subscriber()  # OverallStatus 구독자 초기화
        self.init_task_status_subscriber()  # TaskStatus 구독자 초기화
        self.init_timer()  # ROS 통신을 위한 타이머 시작
        self.init_robot_timeout_timer()  # 로봇 타임아웃 체크 타이머 추가
        self.init_heartbeat_signal()  # Heartbeat 시그널 연결
        
        # 윈도우 설정
        self.setWindowTitle("LIBO Administrator System v2.0 - Integrated")
        self.setMinimumSize(1320, 900)
        self.resize(1320, 900)

    def init_integrated_ui(self):  # 통합된 UI 초기화
        """메인 컨트롤 탭에 모든 기능을 통합한 UI 생성"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 메인 수평 레이아웃 (왼쪽: TaskRequest+Heartbeat, 중앙: Navigator, 오른쪽: 기존 상태)
        main_layout = QHBoxLayout(central_widget)
        
        # 왼쪽 영역 (TaskRequest + HeartbeatMonitor) - 가로 2/5
        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)
        
        # TaskRequest 영역 (상단)
        self.create_task_request_widget(left_layout)
        
        # HeartbeatMonitor 영역 (하단)
        self.create_heartbeat_monitor_widget(left_layout)
        
        # 중앙 영역 (Navigator) - 가로 1/3
        center_container = QWidget()
        center_layout = QVBoxLayout(center_container)
        self.create_navigator_widget(center_layout)
        
        # 오른쪽 영역 (기존 ActiveRobots + CurrentTask) - 나머지
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        self.create_status_widgets(right_layout)
        
        # 메인 레이아웃에 추가 (1:1:1 비율로 변경)
        main_layout.addWidget(left_container, 1)  # 왼쪽 1/3
        main_layout.addWidget(center_container, 1)  # 중앙 1/3  
        main_layout.addWidget(right_container, 1)  # 오른쪽 1/3

    def create_task_request_widget(self, parent_layout):  # TaskRequest 위젯 생성
        """TaskRequest 기능을 위한 위젯 생성"""
        # TaskRequest 프레임
        task_frame = QFrame()
        task_frame.setStyleSheet("""
            QFrame {
                border: 2px solid #e74c3c;
                border-radius: 8px;
                background-color: #ffffff;
                margin: 5px;
                padding: 8px;
            }
            QLabel {
                color: #2c3e50;
                font-size: 11px;
            }
            QLineEdit {
                background-color: #f8f9fa;
                color: #2c3e50;
                border: 1px solid #95a5a6;
                border-radius: 3px;
                padding: 3px;
            }
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                padding: 5px 10px;
                border-radius: 3px;
                font-weight: bold;
                font-size: 10px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
        """)
        task_layout = QVBoxLayout(task_frame)
        
        # 제목 (컴팩트하게)
        title_label = QLabel("🚀 Task Request")
        title_label.setStyleSheet("font-size: 12px; font-weight: bold; color: #e74c3c; margin: 1px;")
        title_label.setFixedHeight(50)  # 다른 위젯들과 같은 제목 높이
        task_layout.addWidget(title_label, 0)  # stretch factor 0으로 고정 크기
        
        # 입력 필드들을 2x2 그리드로 배치
        input_grid_layout = QHBoxLayout()
        
        # 왼쪽 열
        left_column = QVBoxLayout()
        left_column.addWidget(QLabel('로봇 ID:'))
        self.robot_id_edit = QLineEdit('libo_a')
        self.robot_id_edit.setMaximumHeight(20)
        left_column.addWidget(self.robot_id_edit)
        
        left_column.addWidget(QLabel('작업 타입:'))
        self.task_type_edit = QLineEdit('escort')
        self.task_type_edit.setMaximumHeight(20)
        left_column.addWidget(self.task_type_edit)
        
        # 오른쪽 열  
        right_column = QVBoxLayout()
        right_column.addWidget(QLabel('호출지:'))
        self.call_location_edit = QLineEdit('A2')
        self.call_location_edit.setMaximumHeight(20)
        right_column.addWidget(self.call_location_edit)
        
        right_column.addWidget(QLabel('목적지:'))
        self.goal_location_edit = QLineEdit('D3')
        self.goal_location_edit.setMaximumHeight(20)
        right_column.addWidget(self.goal_location_edit)
        
        input_grid_layout.addLayout(left_column)
        input_grid_layout.addLayout(right_column)
        task_layout.addLayout(input_grid_layout, 0)  # 입력 필드는 고정 크기
        
        # 버튼들 (한 줄로)
        button_layout = QHBoxLayout()
        self.random_button = QPushButton('🎲 랜덤')
        self.random_button.setMaximumHeight(25)
        self.random_button.clicked.connect(self.set_random_values)
        button_layout.addWidget(self.random_button)
        
        self.send_button = QPushButton('🚀 전송')
        self.send_button.setMaximumHeight(25)
        self.send_button.clicked.connect(self.send_task_request)
        button_layout.addWidget(self.send_button)
        task_layout.addLayout(button_layout, 0)  # 버튼은 고정 크기
        
        # 로그 영역 (더 크게)
        task_layout.addWidget(QLabel('로그:'), 0)  # 라벨은 고정 크기
        self.task_log_text = QTextEdit()
        self.task_log_text.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                color: #2c3e50;
                border: 1px solid #95a5a6;
                border-radius: 4px;
                font-family: 'Courier New', monospace;
                font-size: 10px;
            }
        """)
        self.task_log_text.setReadOnly(True)
        task_layout.addWidget(self.task_log_text, 1)  # 로그는 남은 공간 사용
        
        parent_layout.addWidget(task_frame, 1)  # TaskRequest는 더 작은 비율

    def create_heartbeat_monitor_widget(self, parent_layout):  # HeartbeatMonitor 위젯 생성
        """HeartbeatMonitor 기능을 위한 위젯 생성"""
        # Heartbeat 프레임 (스타일 완전 제거)
        heartbeat_frame = QFrame()
        heartbeat_layout = QVBoxLayout(heartbeat_frame)
        
        # 제목 (컴팩트하게) - 고정 높이
        title_label = QLabel("💓 Heartbeat Monitor")
        title_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #9b59b6; margin: 2px;")
        title_label.setFixedHeight(50)  # 고정 높이로 설정
        heartbeat_layout.addWidget(title_label, 0)  # stretch factor 0으로 고정
        
        # 테이블 (순수한 테이블만, 아무 스타일 없음)
        self.heartbeat_table = QTableWidget()
        self.heartbeat_table.setColumnCount(3)
        self.heartbeat_table.setHorizontalHeaderLabels(['Sender ID', '경과 시간 (초)', 'Timestamp'])
        
        # 헤더 명시적으로 보이게 설정
        header = self.heartbeat_table.horizontalHeader()
        header.setVisible(True)
        header.setFixedHeight(30)
        header.setDefaultSectionSize(100)
        header.setStretchLastSection(True)
        
        # 세로 헤더는 숨김
        self.heartbeat_table.verticalHeader().setVisible(False)
        
        # 테이블 기본 설정
        self.heartbeat_table.setAlternatingRowColors(True)
        self.heartbeat_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.heartbeat_table.setSortingEnabled(False)
        
        heartbeat_layout.addWidget(self.heartbeat_table, 1)  # stretch factor 1로 남은 공간 모두 사용
        
        parent_layout.addWidget(heartbeat_frame, 3)  # HeartbeatMonitor는 더 큰 비율

    def create_navigator_widget(self, parent_layout):  # Navigator 위젯 생성
        """Navigator 기능을 위한 위젯 생성"""
        # Navigator 프레임
        nav_frame = QFrame()
        nav_frame.setStyleSheet("""
            QFrame {
                border: 2px solid #f39c12;
                border-radius: 8px;
                background-color: #ffffff;
                margin: 5px;
                padding: 10px;
            }
            QLabel {
                color: #2c3e50;
                font-size: 12px;
            }
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                padding: 6px 12px;
                border-radius: 4px;
                font-weight: bold;
                margin: 2px;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QTextEdit {
                background-color: #f8f9fa;
                color: #2c3e50;
                border: 1px solid #95a5a6;
                border-radius: 4px;
                font-family: 'Courier New', monospace;
                font-size: 11px;
            }
        """)
        nav_layout = QVBoxLayout(nav_frame)
        
        # 제목
        title_label = QLabel("🧭 Navigator 디버깅")
        title_label.setStyleSheet("font-size: 13px; font-weight: bold; color: #f39c12; margin: 2px;")
        title_label.setFixedHeight(50)  # 다른 위젯들과 같은 제목 높이
        nav_layout.addWidget(title_label, 0)  # stretch factor 0으로 고정 크기
        
        # 상태 및 제어
        control_layout = QHBoxLayout()
        self.nav_status_label = QLabel("상태: 비활성화됨")
        self.nav_status_label.setStyleSheet("color: #e74c3c; font-size: 11px;")
        control_layout.addWidget(self.nav_status_label)
        
        self.nav_toggle_button = QPushButton("🟢 활성화")
        self.nav_toggle_button.clicked.connect(self.toggle_navigator_service)
        control_layout.addWidget(self.nav_toggle_button)
        nav_layout.addLayout(control_layout, 0)  # stretch factor 0으로 고정 크기
        
        # SetGoal 수신 로그
        setgoal_label = QLabel("SetGoal 수신:")
        setgoal_label.setFixedHeight(50)
        nav_layout.addWidget(setgoal_label, 0)  # 라벨은 고정 크기
        self.nav_messages_text = QTextEdit()
        self.nav_messages_text.setReadOnly(True)
        nav_layout.addWidget(self.nav_messages_text, 1)  # 남은 공간 모두 사용
        
        # NavigationResult 테스트 버튼들
        navtest_label = QLabel("NavigationResult 테스트:")
        navtest_label.setFixedHeight(50)
        nav_layout.addWidget(navtest_label, 0)  # 라벨은 고정 크기
        result_layout = QHBoxLayout()
        
        self.success_button = QPushButton("✅ SUCCESS")
        self.success_button.clicked.connect(lambda: self.send_navigation_result("SUCCEEDED"))
        self.success_button.setStyleSheet("QPushButton { background-color: #27ae60; } QPushButton:hover { background-color: #229954; }")
        result_layout.addWidget(self.success_button)
        
        self.failed_button = QPushButton("❌ FAILED")
        self.failed_button.clicked.connect(lambda: self.send_navigation_result("FAILED"))
        self.failed_button.setStyleSheet("QPushButton { background-color: #e74c3c; } QPushButton:hover { background-color: #c0392b; }")
        result_layout.addWidget(self.failed_button)
        
        self.canceled_button = QPushButton("⏹️ CANCEL")
        self.canceled_button.clicked.connect(lambda: self.send_navigation_result("CANCELED"))
        self.canceled_button.setStyleSheet("QPushButton { background-color: #f39c12; } QPushButton:hover { background-color: #e67e22; }")
        result_layout.addWidget(self.canceled_button)
        nav_layout.addLayout(result_layout, 0)  # 버튼 영역은 고정 크기
        
        # NavigationResult 로그
        navlog_label = QLabel("NavigationResult 로그:")
        navlog_label.setFixedHeight(50)
        nav_layout.addWidget(navlog_label, 0)  # 라벨은 고정 크기
        self.nav_result_text = QTextEdit()
        self.nav_result_text.setReadOnly(True)
        nav_layout.addWidget(self.nav_result_text, 1)  # 남은 공간 모두 사용
        
        parent_layout.addWidget(nav_frame)

    def create_status_widgets(self, parent_layout):  # 기존 상태 위젯들 생성
        """기존 ActiveRobots와 CurrentTask 위젯 생성"""
        # Active Robots 프레임
        robot_frame = QFrame()
        robot_frame.setStyleSheet("""
            QFrame {
                border: 2px solid #3498db;
                border-radius: 8px;
                background-color: #ffffff;
                margin: 5px;
                padding: 10px;
            }
            QLabel {
                color: #2c3e50;
                font-size: 12px;
            }
        """)
        robot_layout = QVBoxLayout(robot_frame)
        
        # 로봇 제목과 카운트 (충분한 크기)
        robot_title_layout = QHBoxLayout()
        robot_title = QLabel("🤖 Active Robots")
        robot_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #3498db; margin: 2px;")
        robot_title.setFixedHeight(50)  # 제목 높이 더 늘림 (30 -> 40)
        robot_title_layout.addWidget(robot_title)
        
        self.robot_count_label = QLabel("Count: 0")
        self.robot_count_label.setFixedHeight(50)  # 카운트 높이도 맞춤
        robot_title_layout.addWidget(self.robot_count_label)
        robot_layout.addLayout(robot_title_layout, 0)  # stretch factor 0으로 고정 크기
        
        # 로봇 카드들을 담을 스크롤 영역
        self.robot_scroll_area = QScrollArea()
        self.robot_scroll_area.setWidgetResizable(True)
        self.robot_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.robot_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.robot_scroll_area.setStyleSheet("""
            QScrollArea {
                border: none;
                background-color: transparent;
            }
        """)
        
        # 스크롤 영역 안에 들어갈 컨테이너 위젯
        self.robot_container = QWidget()
        self.robot_container_layout = QVBoxLayout(self.robot_container)
        self.robot_container_layout.setAlignment(Qt.AlignTop)
        self.robot_container_layout.setSpacing(10)
        
        self.robot_scroll_area.setWidget(self.robot_container)
        robot_layout.addWidget(self.robot_scroll_area, 1)  # 남은 공간 사용
        
        # Current Task 프레임
        task_frame = QFrame()
        task_frame.setStyleSheet("""
            QFrame {
                border: 2px solid #e67e22;
                border-radius: 8px;
                background-color: #ffffff;
                margin: 5px;
                padding: 10px;
            }
            QLabel {
                color: #2c3e50;
                font-size: 12px;
            }
        """)
        task_layout = QVBoxLayout(task_frame)
        
        # 작업 제목 (충분한 크기)
        task_title = QLabel("📋 Current Task")
        task_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #e67e22; margin: 2px;")
        task_title.setFixedHeight(50)  # 제목 높이 더 늘림 (30 -> 40)
        task_layout.addWidget(task_title, 0)  # stretch factor 0으로 고정 크기
        
        # 작업 상태 (남은 공간 사용하되 적절한 크기)
        self.task_status_label = QLabel("활성 작업 없음")
        self.task_status_label.setStyleSheet("color: #2c3e50; font-size: 12px; background-color: #f8f9fa; border: 1px solid #dee2e6; border-radius: 4px; padding: 10px;")
        self.task_status_label.setWordWrap(True)
        task_layout.addWidget(self.task_status_label, 1)  # 남은 공간 사용
        
        parent_layout.addWidget(robot_frame, 1)
        parent_layout.addWidget(task_frame, 1)

    def init_heartbeat_signal(self):  # Heartbeat 시그널 연결
        """Heartbeat 신호를 GUI 업데이트에 연결"""
        self.signal_bridge.heartbeat_received_signal.connect(self.add_heartbeat_log_entry)

    # ===== TaskRequest 관련 메서드들 =====
    
    def set_random_values(self):  # 랜덤 값 설정
        """TaskRequest 필드에 랜덤 값 설정"""
        robot_ids = ['libo_a', 'libo_b', 'libo_c']  # 랜덤 선택을 위한 로봇 ID 목록
        task_types = ['escort', 'assist', 'delivery']  # 랜덤 선택을 위한 작업 타입 목록
        locations = ['A1', 'A2', 'A3', 'B2', 'B3', 'C1', 'C3', 'D3', 'E3']  # 랜덤 선택을 위한 위치 목록
        
        self.robot_id_edit.setText(random.choice(robot_ids))  # 로봇 ID를 랜덤으로 선택하여 설정
        self.task_type_edit.setText(random.choice(task_types))  # 작업 타입을 랜덤으로 선택하여 설정
        
        call_loc = random.choice(locations)  # 호출지를 랜덤으로 선택
        goal_loc = random.choice(locations)  # 목적지를 랜덤으로 선택
        while goal_loc == call_loc:  # 호출지와 목적지가 같으면
            goal_loc = random.choice(locations)  # 목적지를 다시 선택
        
        self.call_location_edit.setText(call_loc)  # 랜덤 호출지를 설정
        self.goal_location_edit.setText(goal_loc)  # 랜덤 목적지를 설정
        
        self.log_task_message('🎲 랜덤 값으로 설정 완료!')  # 랜덤 설정 완료 로그 출력

    def send_task_request(self):  # TaskRequest 전송
        """TaskRequest 서비스 요청 전송"""
        robot_id = self.robot_id_edit.text().strip()  # 로봇 ID 입력 필드에서 텍스트를 가져오고 양쪽 공백 제거
        task_type = self.task_type_edit.text().strip()  # 작업 타입 입력 필드에서 텍스트를 가져오고 양쪽 공백 제거
        call_location = self.call_location_edit.text().strip()  # 호출지 위치 입력 필드에서 텍스트를 가져오고 양쪽 공백 제거
        goal_location = self.goal_location_edit.text().strip()  # 목적지 위치 입력 필드에서 텍스트를 가져오고 양쪽 공백 제거

        if not all([robot_id, task_type, call_location, goal_location]):  # 모든 필드가 채워져 있는지 확인
            self.log_task_message('❌ 모든 필드를 입력해주세요.')  # 비어있는 필드가 있으면 로그 출력
            return  # 함수 실행 중단

        self.log_task_message(f'📤 요청 전송 중... {robot_id}, {task_type}')  # 전송 시작 로그 출력
        self.send_button.setEnabled(False)  # 중복 전송을 막기 위해 버튼을 비활성화
        self.send_button.setText('⏳ 전송 중...')  # 버튼 텍스트를 '전송 중'으로 변경

        self.worker = RequestWorker(self.task_request_client_node, robot_id, task_type, call_location, goal_location)  # 백그라운드 작업을 위한 워커 스레드 생성
        self.worker.response_received.connect(self.handle_task_response)  # 워커의 응답 시그널을 handle_response 함수에 연결
        self.worker.start()  # 워커 스레드 시작

    def handle_task_response(self, response):  # TaskRequest 응답 처리
        """TaskRequest 응답을 처리하는 메서드"""
        if response:  # 응답이 성공적으로 오면
            self.log_task_message(f'✅ 응답: {response.message}')  # 성공 메시지를 로그에 출력
        else:  # 응답이 없거나 실패하면
            self.log_task_message('❌ 응답 수신 실패 또는 타임아웃')  # 실패 메시지를 로그에 출력
        
        self.send_button.setEnabled(True)  # 버튼을 다시 활성화
        self.send_button.setText('🚀 전송')  # 버튼 텍스트를 원래대로 복원

    def log_task_message(self, message):  # TaskRequest 로그 메시지 출력
        """TaskRequest 로그에 메시지 추가"""
        self.task_log_text.append(f'[{time.strftime("%H:%M:%S")}] {message}')  # 현재 시간과 함께 로그 메시지를 추가
        self.task_log_text.ensureCursorVisible()  # 스크롤을 맨 아래로 내려서 항상 최신 로그가 보이게 함

    # ===== Heartbeat 관련 메서드들 =====
    
    def add_heartbeat_log_entry(self, msg):  # Heartbeat 로그 추가
        """새 Heartbeat 메시지가 도착하면 테이블에 추가"""
        # 새 메시지가 도착하면, 로그 리스트에 추가하고 테이블을 다시 그림
        log_entry = {  # 로그 항목을 딕셔너리로 구성
            'msg': msg,  # 원본 메시지
            'received_time': time.time()  # GUI가 받은 정확한 시간
        }
        self.heartbeat_log.append(log_entry)  # 리스트의 맨 뒤에 새 로그 추가
        
        # 테이블의 맨 아래에 새로운 행만 추가하여 성능을 최적화
        row_position = self.heartbeat_table.rowCount()  # 현재 행의 개수 = 새로 추가될 행의 인덱스
        self.heartbeat_table.insertRow(row_position)  # 맨 아래에 새 행 삽입

        # 새 행에 데이터를 채움
        elapsed_time = log_entry['received_time'] - self.heartbeat_start_time  # 경과 시간 계산
        timestamp = log_entry['msg'].timestamp  # 메시지의 타임스탬프
        timestamp_str = f"{timestamp.sec}.{timestamp.nanosec:09d}"  # 타임스탬프를 문자열로 변환

        self.heartbeat_table.setItem(row_position, 0, QTableWidgetItem(log_entry['msg'].sender_id))  # 0번 열: Sender ID
        self.heartbeat_table.setItem(row_position, 1, QTableWidgetItem(f"{elapsed_time:.2f}"))  # 1번 열: 경과 시간 (소수점 둘째자리까지)
        self.heartbeat_table.setItem(row_position, 2, QTableWidgetItem(timestamp_str))  # 2번 열: 타임스탬프
            
        self.heartbeat_table.scrollToBottom()  # 새 로그가 추가되면 자동으로 스크롤을 맨 아래로 내림
        
        # 컬럼 너비 자동 조절 후, 특정 컬럼 너비 수동 조정
        self.heartbeat_table.resizeColumnsToContents()  # 먼저 모든 열의 너비를 내용에 맞춤
        current_ts_width = self.heartbeat_table.columnWidth(2)  # 현재 타임스탬프 열(2번 인덱스)의 너비를 가져옴
        self.heartbeat_table.setColumnWidth(2, current_ts_width * 2)  # 해당 열의 너비를 2배로 설정

    # ===== Navigator 관련 메서드들 =====
    
    def toggle_navigator_service(self):  # Navigator 서비스 토글
        """Navigator 디버그 서비스를 on/off 토글"""
        if self.navigator_server_node.is_active:
            # 서비스 비활성화
            if self.navigator_server_node.stop_service():
                self.nav_status_label.setText("상태: 비활성화됨")
                self.nav_status_label.setStyleSheet("color: #e74c3c; font-size: 11px;")
                self.nav_toggle_button.setText("🟢 활성화")
                self.nav_messages_text.setPlainText("서비스가 비활성화되었습니다.\n'활성화' 버튼을 눌러 디버깅을 시작하세요.\n")
        else:
            # 서비스 활성화
            if self.navigator_server_node.start_service():
                self.nav_status_label.setText("상태: 활성화됨 - SetGoal 서비스 대기 중...")
                self.nav_status_label.setStyleSheet("color: #27ae60; font-size: 11px;")
                self.nav_toggle_button.setText("🔴 비활성화")
                self.nav_messages_text.setPlainText("SetGoal 메시지 대기 중...\n💡 TaskManager에서 Task Request를 보내면 여기에 SetGoal 메시지가 표시됩니다.\n")

    def send_navigation_result(self, result_type):  # NavigationResult 서비스 호출
        """TaskManager에게 NavigationResult를 보내는 메서드"""
        current_time = time.strftime('%H:%M:%S', time.localtime())
        
        # 클릭 로그 추가
        click_log = f"[{current_time}] 🖱️  {result_type} 버튼 클릭됨"
        self.navigation_result_logs.append(click_log)
        self.update_navigation_result_display()
        
        try:
            # 서비스가 준비될 때까지 대기
            if not self.navigation_result_client.wait_for_service(timeout_sec=2.0):
                error_log = f"[{current_time}] ❌ NavigationResult 서비스를 찾을 수 없음"
                self.navigation_result_logs.append(error_log)
                self.update_navigation_result_display()
                self.ros_node.get_logger().error('❌ NavigationResult 서비스를 찾을 수 없음')
                return False
            
            # NavigationResult 요청 생성
            request = NavigationResult.Request()
            request.result = result_type  # "SUCCEEDED", "FAILED", "CANCELED"
            
            send_log = f"[{current_time}] �� NavigationResult 전송: {result_type}"
            self.navigation_result_logs.append(send_log)
            self.update_navigation_result_display()
            
            self.ros_node.get_logger().info(f'📍 NavigationResult 전송: {result_type}')
            
            # 비동기 서비스 호출
            future = self.navigation_result_client.call_async(request)
            future.add_done_callback(lambda f: self.navigation_result_response_callback(f, result_type))
            
            return True
            
        except Exception as e:
            error_log = f"[{current_time}] ❌ 전송 오류: {str(e)}"
            self.navigation_result_logs.append(error_log)
            self.update_navigation_result_display()
            self.ros_node.get_logger().error(f'❌ NavigationResult 전송 중 오류: {e}')
            return False

    def navigation_result_response_callback(self, future, result_type):  # NavigationResult 응답 콜백
        """NavigationResult 서비스 응답을 처리하는 콜백"""
        current_time = time.strftime('%H:%M:%S', time.localtime())
        
        try:
            response = future.result()
            if response.success:
                success_log = f"[{current_time}] ✅ {result_type} 응답 성공: {response.message}"
                self.navigation_result_logs.append(success_log)
                self.ros_node.get_logger().info(f'✅ NavigationResult ({result_type}) 전송 성공: {response.message}')
            else:
                fail_log = f"[{current_time}] ⚠️  {result_type} 응답 실패: {response.message}"
                self.navigation_result_logs.append(fail_log)
                self.ros_node.get_logger().warning(f'⚠️  NavigationResult ({result_type}) 전송 실패: {response.message}')
        except Exception as e:
            error_log = f"[{current_time}] ❌ {result_type} 응답 처리 오류: {str(e)}"
            self.navigation_result_logs.append(error_log)
            self.ros_node.get_logger().error(f'❌ NavigationResult 응답 처리 중 오류: {e}')
        
        self.update_navigation_result_display()

    def update_navigation_result_display(self):  # NavigationResult 로그 표시 업데이트
        """NavigationResult 로그를 UI에 표시"""
        if self.navigation_result_logs:
            # 최근 10개 로그만 표시
            recent_logs = self.navigation_result_logs[-10:]
            display_text = "\n".join(recent_logs) + "\n"
        else:
            display_text = "NavigationResult 버튼을 클릭해서 테스트해보세요.\n"
        
        self.nav_result_text.setPlainText(display_text)
        
        # 스크롤을 맨 아래로
        scrollbar = self.nav_result_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def update_navigator_messages_display(self):  # Navigator 메시지 표시 업데이트
        """수신된 SetGoal 메시지들을 UI에 표시"""
        # 서비스가 비활성화되었으면 메시지 업데이트 안 함
        if not self.navigator_server_node.is_active:
            return
            
        latest_messages = self.navigator_server_node.get_latest_messages(20)  # 최근 20개 메시지
        
        if latest_messages:
            display_text = "🎯 SetGoal 메시지 수신 기록:\n\n"
            for msg in latest_messages:
                status_icon = "✅" if msg.get('status') == 'responded' else "❌" if msg.get('status') == 'error' else "⏳"
                display_text += f"{status_icon} [{msg['time']}] 좌표: ({msg['x']}, {msg['y']})\n"
                if 'response' in msg:
                    display_text += f"   📤 응답: {msg['response']}\n"
                display_text += "\n"
            
            # 통계 정보 추가
            total_count = len(self.navigator_server_node.received_messages)
            success_count = len([m for m in self.navigator_server_node.received_messages if m.get('status') == 'responded'])
            error_count = len([m for m in self.navigator_server_node.received_messages if m.get('status') == 'error'])
            
            display_text += f"📊 통계:\n"
            display_text += f"   총 수신: {total_count}개\n"
            display_text += f"   성공 응답: {success_count}개\n"
            display_text += f"   오류: {error_count}개\n"
            
        else:
            display_text = "SetGoal 메시지 대기 중...\n\n"
            display_text += "💡 TaskManager에서 Task Request를 보내면 여기에 SetGoal 메시지가 표시됩니다.\n"
        
        self.nav_messages_text.setPlainText(display_text)
        
        # 스크롤을 맨 아래로
        scrollbar = self.nav_messages_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def init_robot_status_subscriber(self):  # OverallStatus 구독자 초기화
        """robot_status 토픽을 구독해서 로봇 상태를 실시간 업데이트"""
        self.robot_status_subscription = self.ros_node.create_subscription(
            OverallStatus,  # 메시지 타입을 OverallStatus로 변경
            'robot_status',  # 토픽 이름
            self.robot_status_callback,  # 콜백 함수
            10  # QoS depth
        )

    def init_task_status_subscriber(self):  # TaskStatus 구독자 초기화
        """task_status 토픽을 구독해서 작업 상태를 실시간 업데이트"""
        self.task_status_subscription = self.ros_node.create_subscription(
            TaskStatus,  # 메시지 타입
            'task_status',  # 토픽 이름
            self.task_status_callback,  # 콜백 함수
            10  # QoS depth
        )

    def robot_status_callback(self, msg):  # OverallStatus 메시지 수신 콜백
        """OverallStatus 메시지를 받았을 때 GUI 업데이트"""
        try:
            # 로봇별로 상태 정보 저장 (모든 정보 포함)
            self.robot_status_dict[msg.robot_id] = {
                'id': msg.robot_id,  # 로봇 ID
                'state': msg.robot_state,  # 로봇 상태 (INIT, CHARGING, STANDBY 등)
                'available': msg.is_available,  # 사용 가능 여부
                'battery': msg.battery,  # 배터리 잔량
                'book_weight': msg.book_weight,  # 책 무게
                'position_x': msg.position_x,  # X 좌표
                'position_y': msg.position_y,  # Y 좌표
                'position_yaw': msg.position_yaw,  # Yaw 각도
                'last_seen': time.time()  # 마지막 수신 시간 추가
            }
            self.update_robot_status_display()  # GUI 업데이트
            
        except Exception as e:
            print(f"로봇 상태 처리 중 오류: {e}")

    def update_robot_status_display(self):  # 로봇 상태 표시 업데이트
        """활성 로봇들의 상태를 개별 카드로 표시"""
        try:
            # 로봇 개수 업데이트
            robot_count = len(self.robot_status_dict)  # 활성 로봇 개수
            self.robot_count_label.setText(f"Count: {robot_count}")  # 카운트 라벨 업데이트
            
            # 기존 로봇 카드들 제거
            for i in reversed(range(self.robot_container_layout.count())):
                child = self.robot_container_layout.itemAt(i).widget()
                if child:
                    child.deleteLater()
            
            if robot_count == 0:  # 로봇이 없다면
                # 빈 상태 메시지 표시
                empty_label = QLabel("활성 로봇 없음")
                empty_label.setStyleSheet("""
                    color: #95a5a6; 
                    font-size: 14px; 
                    font-style: italic; 
                    padding: 20px;
                    text-align: center;
                """)
                empty_label.setAlignment(Qt.AlignCenter)
                self.robot_container_layout.addWidget(empty_label)
            else:
                # 각 로봇 정보를 개별 카드로 추가
                for robot_id, status in self.robot_status_dict.items():
                    robot_card = self.create_robot_card(robot_id, status)
                    self.robot_container_layout.addWidget(robot_card)
                
        except Exception as e:
            print(f"로봇 상태 표시 중 오류: {e}")

    def create_robot_card(self, robot_id, status):  # 개별 로봇 카드 생성
        """개별 로봇의 정보를 담은 카드 위젯 생성"""
        # 로봇 카드 프레임
        card_frame = QFrame()
        
        # 상태에 따른 카드 색상 설정
        state_colors = {
            'STANDBY': '#d4edda',  # 연한 초록색
            'CHARGING': '#fff3cd',  # 연한 노란색
            'INIT': '#f8d7da',      # 연한 분홍색
            'ESCORT': '#cce5ff',    # 연한 파란색
            'DELIVERY': '#e2e3e5',  # 연한 회색
            'ASSIST': '#d1ecf1'     # 연한 청록색
        }
        
        card_color = state_colors.get(status['state'], '#f8f9fa')  # 기본값은 연한 회색
        
        card_frame.setStyleSheet(f"""
            QFrame {{
                border: 2px solid #3498db;
                border-radius: 8px;
                background-color: {card_color};
                margin: 3px;
                padding: 8px;
                max-width: 300px;
            }}
            QLabel {{
                color: #2c3e50;
                font-size: 10px;
            }}
        """)
        
        card_layout = QVBoxLayout(card_frame)
        card_layout.setSpacing(3)  # 간격 줄이기
        card_layout.setContentsMargins(5, 5, 5, 5)  # 여백 줄이기
        
        # 로봇 제목 (ID + State) - 세로 배치로 변경
        title_layout = QVBoxLayout()
        
        # 첫 번째 줄: 로봇 ID
        robot_id_label = QLabel(f"🤖 {robot_id}")
        robot_id_label.setStyleSheet("font-size: 12px; font-weight: bold; color: #2c3e50;")
        title_layout.addWidget(robot_id_label)
        
        # 두 번째 줄: State와 Available 상태
        status_layout = QHBoxLayout()
        
        # State 표시 (고정 너비로 설정)
        state_label = QLabel(f"📊 {status['state']}")
        state_label.setStyleSheet("font-size: 10px; font-weight: bold; color: #7f8c8d;")
        state_label.setFixedWidth(80)  # State 고정 너비
        status_layout.addWidget(state_label)
        
        # Available 상태 표시
        available_text = "✅ 사용가능" if status['available'] else "❌ 사용 불가"
        available_label = QLabel(available_text)
        available_label.setStyleSheet("font-size: 10px; font-weight: bold; color: #27ae60;" if status['available'] else "font-size: 10px; font-weight: bold; color: #e74c3c;")
        status_layout.addWidget(available_label)
        
        status_layout.addStretch()  # 오른쪽 정렬을 위한 공간
        title_layout.addLayout(status_layout)
        card_layout.addLayout(title_layout)
        
        # 구분선 추가
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #bdc3c7; margin: 3px 0px;")
        card_layout.addWidget(separator)
        
        # 상세 정보 (2열로 배치)
        info_layout = QHBoxLayout()
        info_layout.setSpacing(10)  # 열 간격 줄이기
        
        # 왼쪽 열
        left_column = QVBoxLayout()
        left_column.setSpacing(2)  # 간격 줄이기
        
        # 배터리 정보
        if status['battery'] == 255:
            battery_text = "🔋 N/A"
            battery_color = "#95a5a6"
        else:
            battery_text = f"🔋 {status['battery']}%"
            if status['battery'] > 50:
                battery_color = "#27ae60"  # 초록색
            elif status['battery'] > 20:
                battery_color = "#f39c12"  # 주황색
            else:
                battery_color = "#e74c3c"  # 빨간색
        
        battery_label = QLabel(battery_text)
        battery_label.setStyleSheet(f"color: {battery_color}; font-size: 10px;")
        left_column.addWidget(battery_label)
        
        # 책 무게 정보
        weight_text = f"📚 {status['book_weight']:.1f}kg" if status['book_weight'] > 0 else "📚 0.0kg"
        weight_label = QLabel(weight_text)
        weight_label.setStyleSheet("color: #2c3e50; font-size: 10px;")
        left_column.addWidget(weight_label)
        
        info_layout.addLayout(left_column)
        
        # 오른쪽 열
        right_column = QVBoxLayout()
        right_column.setSpacing(2)  # 간격 줄이기
        
        # 위치 정보 (간단하게)
        pos_text = f"📍 ({status['position_x']:.1f}, {status['position_y']:.1f})"
        pos_label = QLabel(pos_text)
        pos_label.setStyleSheet("color: #2c3e50; font-size: 10px;")
        right_column.addWidget(pos_label)
        
        # 방향 정보 (간단하게)
        yaw_text = f"🧭 {status['position_yaw']:.0f}°"
        yaw_label = QLabel(yaw_text)
        yaw_label.setStyleSheet("color: #2c3e50; font-size: 10px;")
        right_column.addWidget(yaw_label)
        
        info_layout.addLayout(right_column)
        card_layout.addLayout(info_layout)
        
        return card_frame

    def task_status_callback(self, msg):  # TaskStatus 메시지 수신 콜백
        """TaskStatus 메시지를 받았을 때 GUI 업데이트"""
        try:
            # 기존 task와 stage가 다른 경우에만 업데이트 시간 갱신
            task_changed = False
            if not self.task_status_data or self.task_status_data.get('task_id') != msg.task_id:
                task_changed = True  # 새로운 task
            elif self.task_status_data.get('task_stage') != msg.task_stage:
                task_changed = True  # stage 변경됨
            
            # 작업 상태 정보 저장
            self.task_status_data = {
                'task_id': msg.task_id,  # Task ID 추가
                'robot_id': msg.robot_id,  # 로봇 ID
                'task_type': msg.task_type,  # 작업 타입
                'task_stage': msg.task_stage,  # 작업 단계
                'call_location': msg.call_location,  # 호출 위치
                'goal_location': msg.goal_location,  # 목표 위치
                'start_time': msg.start_time.sec + msg.start_time.nanosec / 1e9,  # Task 시작 시간 (실제 시작시간)
                'last_stage_update': time.time() if task_changed else self.task_status_data.get('last_stage_update', time.time()),  # stage 변경 시간만 갱신
                'last_received': time.time()  # 메시지 수신 시간 (매번 갱신)
            }
            self.update_task_status_display()  # GUI 업데이트
            
            if task_changed:
                print(f"✅ TaskStatus 업데이트: Task[{msg.task_id}] Stage {msg.task_stage}")  # stage 변경시만 로그
            
        except Exception as e:
            print(f"❌ 작업 상태 처리 중 오류: {e}")  # 에러 메시지 개선

    def update_task_status_display(self):  # 작업 상태 표시 업데이트
        """현재 작업 상태를 위젯에 표시"""
        try:
            if not self.task_status_data:  # 작업 데이터가 없다면
                status_text = "활성 작업 없음"
            else:
                # 작업 단계 텍스트와 아이콘 변환
                stage_info = {
                    1: {"text": "시작", "icon": "🟡"},
                    2: {"text": "진행중", "icon": "🔵"}, 
                    3: {"text": "완료직전", "icon": "🟢"}
                }
                
                current_stage = self.task_status_data['task_stage']
                stage_data = stage_info.get(current_stage, {"text": f"Stage {current_stage}", "icon": "⚪"})
                stage_text = stage_data["text"]
                stage_icon = stage_data["icon"]
                
                # 시작 시간과 마지막 stage 변경 시간 포맷
                start_time_str = time.strftime('%H:%M:%S', time.localtime(self.task_status_data['start_time']))
                last_update_str = time.strftime('%H:%M:%S', time.localtime(self.task_status_data['last_stage_update']))
                
                # 작업 정보 텍스트 생성 (시간 정보 개선)
                status_text = (f"🆔 Task ID: {self.task_status_data['task_id']}\n"
                              f"🤖 로봇: {self.task_status_data['robot_id']}\n"
                              f"📋 작업: {self.task_status_data['task_type']}\n" 
                              f"{stage_icon} Stage {current_stage}: {stage_text}\n"
                              f"📍 {self.task_status_data['call_location']} → {self.task_status_data['goal_location']}\n"
                              f"⏰ 시작: {start_time_str} | 마지막변경: {last_update_str}")
                
            # 위젯 업데이트 (위젯 이름은 UI에서 추가할 예정)
            if hasattr(self, 'task_status_label'):  # task_status_label이 있다면
                self.task_status_label.setText(status_text)  # 텍스트 업데이트
                
        except Exception as e:
            print(f"❌ 작업 상태 표시 중 오류: {e}")  # 에러 메시지 개선

    def init_timer(self):
        self.ros_timer = QTimer(self)  # QTimer 객체 생성
        self.ros_timer.timeout.connect(self.spin_ros_nodes)  # 타이머가 만료될 때마다 spin_ros_nodes 함수를 호출하도록 연결
        self.ros_timer.start(100)  # 100ms (0.1초) 간격으로 타이머 시작
        
        # Navigator 메시지 업데이트 타이머 추가
        self.nav_update_timer = QTimer(self)
        self.nav_update_timer.timeout.connect(self.update_navigator_messages_display)
        self.nav_update_timer.start(500)  # 0.5초마다 업데이트

    def spin_ros_nodes(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)  # 메인 GUI의 ROS 노드를 스핀
        
        # 통합된 노드들 스핀
        rclpy.spin_once(self.task_request_client_node, timeout_sec=0)  # TaskRequest 클라이언트 노드
        rclpy.spin_once(self.heartbeat_subscriber_node, timeout_sec=0)  # Heartbeat 구독 노드
        rclpy.spin_once(self.navigator_server_node, timeout_sec=0)  # Navigator 서버 노드

    def init_robot_timeout_timer(self):  # 로봇 타임아웃 체크 타이머 초기화
        """5초마다 비활성 로봇들을 제거하는 타이머"""
        self.robot_timeout_timer = QTimer(self)  # 타이머 생성
        self.robot_timeout_timer.timeout.connect(self.check_robot_timeouts)  # 타임아웃 체크 함수 연결
        self.robot_timeout_timer.start(3000)  # 3초마다 실행

    def check_robot_timeouts(self):  # 비활성 로봇 제거
        """3초 이상 메시지가 안 온 로봇들을 제거"""
        current_time = time.time()  # 현재 시간
        timeout_seconds = 3 # 타임아웃 시간 (3초)
        
        # 로봇 타임아웃 체크
        robots_to_remove = []  # 제거할 로봇들 리스트
        for robot_id, status in self.robot_status_dict.items():  # 각 로봇 확인
            time_since_last_seen = current_time - status['last_seen']  # 마지막 수신 후 경과 시간
            if time_since_last_seen > timeout_seconds:  # 타임아웃됐다면
                robots_to_remove.append(robot_id)  # 제거 목록에 추가
                
        for robot_id in robots_to_remove:  # 타임아웃된 로봇들 제거
            del self.robot_status_dict[robot_id]  # 딕셔너리에서 제거
            print(f"🚫 로봇 {robot_id} 제거됨 (타임아웃)")  # 디버그 출력
            
        if robots_to_remove:  # 제거된 로봇이 있다면
            self.update_robot_status_display()  # GUI 업데이트
            
        # TaskStatus 타임아웃 체크
        if self.task_status_data:  # TaskStatus 데이터가 있다면
            time_since_last_task_update = current_time - self.task_status_data['last_received']  # 마지막 수신 후 경과 시간
            if time_since_last_task_update > timeout_seconds:  # 3초 타임아웃됐다면
                self.task_status_data = {}  # TaskStatus 데이터 제거
                self.update_task_status_display()  # GUI 업데이트
                print(f"🚫 작업 상태 제거됨 (타임아웃)")  # 디버그 출력

    def closeEvent(self, event):
        # 통합된 노드들 종료 처리
        print("🔄 통합 노드들 종료 중...")
        
        # Navigator 서비스가 활성화되어 있으면 먼저 중지
        if self.navigator_server_node.is_active:
            self.navigator_server_node.stop_service()
            
        # 각 노드들 정리
        self.task_request_client_node.destroy_node()  # TaskRequest 클라이언트 노드 종료
        self.heartbeat_subscriber_node.destroy_node()  # Heartbeat 구독 노드 종료
        self.navigator_server_node.destroy_node()  # Navigator 서버 노드 종료
        
        self.ros_node.destroy_node()  # 메인 ROS 노드 종료
        rclpy.shutdown()  # ROS2 시스템 전체 종료
        event.accept()  # 창 닫기 이벤트 수락

def main(args=None):
    rclpy.init(args=args)  # ROS2 시스템 초기화 (노드 생성 전에 한번만 호출)
    app = QApplication(sys.argv)  # PyQt 어플리케이션 객체 생성
    window = AdminWindow()  # 통합된 메인 윈도우 객체 생성
    window.show()  # 윈도우를 화면에 표시
    
    # 초기 로그 메시지 출력
    window.log_task_message('✅ Admin GUI v2.0 시작됨 - 모든 기능이 통합되었습니다!')
    
    sys.exit(app.exec_())  # 어플리케이션 이벤트 루프 시작 및 종료 코드 처리

if __name__ == '__main__':
    main() 