#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time # 시간 관련 기능을 사용하기 위한 모듈
import rclpy # ROS2 파이썬 클라이언트 라이브러리
from rclpy.node import Node # ROS2 노드 클래스
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QLabel, QHBoxLayout # PyQt5 위젯들
from PyQt5.QtCore import pyqtSignal, QObject # PyQt의 시그널, 기본 객체
from libo_interfaces.msg import Heartbeat # Heartbeat 메시지 타입 임포트

class RosSignalBridge(QObject): # Qt의 시그널을 사용하기 위해 QObject를 상속받는 중간 다리 역할 클래스
    heartbeat_received_signal = pyqtSignal(object) # Heartbeat 메시지를 전달할 시그널

class HeartbeatSubscriberNode(Node): # ROS2 통신(구독)을 전담할 별도의 노드 클래스
    def __init__(self, signal_bridge): # 시그널을 발생시킬 bridge 객체를 외부에서 받음
        super().__init__('heartbeat_monitor_subscriber_node') # 노드 이름 초기화
        self.signal_bridge = signal_bridge # 전달받은 bridge 객체를 저장
        self.get_logger().info('💓 Heartbeat 구독 노드 생성됨. "heartbeat" 토픽을 구독합니다.')

        qos_profile = rclpy.qos.QoSProfile( # QoS 프로파일 생성
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, # 신뢰성 정책: 최선 노력
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, # 내구성 정책: 휘발성
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # 히스토리 정책: 마지막 N개만 유지
            depth=10 # 히스토리 깊이(큐 사이즈)
        )

        self.subscription = self.create_subscription( # 토픽 구독자 생성
            Heartbeat, 'heartbeat', self.listener_callback, qos_profile)

    def listener_callback(self, msg):
        self.signal_bridge.heartbeat_received_signal.emit(msg) # 받은 메시지를 시그널로 GUI에 전달

class HeartbeatMonitorTab(QWidget): # 하트비트 모니터링 탭을 정의하는 클래스
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.main_ros_node = ros_node # 메인 GUI의 ROS 노드 (스핀용)
        
        self.signal_bridge = RosSignalBridge() # 통신용 시그널 다리 생성
        self.node = HeartbeatSubscriberNode(self.signal_bridge) # 통신 전용 노드 생성
        
        self.heartbeat_log = []  # 데이터를 딕셔너리가 아닌 **리스트**로 변경하여 모든 로그를 축적
        self.start_time = time.time() # 탭이 생성된 시간을 기록 (경과 시간의 기준점)
        
        self.init_ui() # UI 초기화
        
        self.signal_bridge.heartbeat_received_signal.connect(self.add_log_entry) # 시그널을 로그 추가 함수에 연결

    def init_ui(self):
        main_layout = QHBoxLayout(self) # 전체 레이아웃을 가로로 변경 (2/5 내용, 3/5 빈 공간)
        
        content_container = QWidget() # 내용을 담을 컨테이너 위젯 생성
        content_layout = QVBoxLayout(content_container) # 컨테이너 내부는 기존처럼 수직 레이아웃

        title_label = QLabel('💓 Heartbeat 수신 로그 (실시간 누적)') # 제목
        content_layout.addWidget(title_label) # 컨테이너에 제목 추가

        self.heartbeat_table = QTableWidget() # 테이블 위젯 생성
        self.heartbeat_table.setColumnCount(3) # 열 개수는 3개로 유지
        self.heartbeat_table.setHorizontalHeaderLabels(['Sender ID', '경과 시간 (초)', 'Timestamp']) # 헤더 순서를 [ID, 경과시간, 타임스탬프]로 변경
        content_layout.addWidget(self.heartbeat_table) # 컨테이너에 테이블 추가

        main_layout.addWidget(content_container, 2) # 메인 레이아웃의 왼쪽에 내용 컨테이너를 2의 비율로 추가
        main_layout.addStretch(3) # 메인 레이아웃의 오른쪽에 3의 비율로 빈 공간 추가

    def add_log_entry(self, msg):
        # 새 메시지가 도착하면, 로그 리스트에 추가하고 테이블을 다시 그림
        log_entry = { # 로그 항목을 딕셔너리로 구성
            'msg': msg, # 원본 메시지
            'received_time': time.time() # GUI가 받은 정확한 시간
        }
        self.heartbeat_log.append(log_entry) # 리스트의 맨 뒤에 새 로그 추가
        
        # 테이블의 맨 아래에 새로운 행만 추가하여 성능을 최적화
        row_position = self.heartbeat_table.rowCount() # 현재 행의 개수 = 새로 추가될 행의 인덱스
        self.heartbeat_table.insertRow(row_position) # 맨 아래에 새 행 삽입

        # 새 행에 데이터를 채움
        elapsed_time = log_entry['received_time'] - self.start_time # 경과 시간 계산
        timestamp = log_entry['msg'].timestamp # 메시지의 타임스탬프
        timestamp_str = f"{timestamp.sec}.{timestamp.nanosec:09d}" # 타임스탬프를 문자열로 변환

        self.heartbeat_table.setItem(row_position, 0, QTableWidgetItem(log_entry['msg'].sender_id)) # 0번 열: Sender ID
        self.heartbeat_table.setItem(row_position, 1, QTableWidgetItem(f"{elapsed_time:.2f}")) # 1번 열: 경과 시간 (소수점 둘째자리까지)
        self.heartbeat_table.setItem(row_position, 2, QTableWidgetItem(timestamp_str)) # 2번 열: 타임스탬프
            
        self.heartbeat_table.scrollToBottom() # 새 로그가 추가되면 자동으로 스크롤을 맨 아래로 내림
        
        # 컬럼 너비 자동 조절 후, 특정 컬럼 너비 수동 조정
        self.heartbeat_table.resizeColumnsToContents() # 먼저 모든 열의 너비를 내용에 맞춤
        current_ts_width = self.heartbeat_table.columnWidth(2) # 현재 타임스탬프 열(2번 인덱스)의 너비를 가져옴
        self.heartbeat_table.setColumnWidth(2, current_ts_width * 2) # 해당 열의 너비를 2배로 설정

    def shutdown(self): # 이 탭이 닫힐 때 호출될 정리 함수
        print("Heartbeat 모니터 탭 종료 중...")
        self.node.destroy_node() # 이 탭이 사용하던 노드를 안전하게 종료 