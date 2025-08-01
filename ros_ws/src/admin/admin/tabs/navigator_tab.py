#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit, QPushButton
from PyQt5.QtCore import pyqtSignal, QTimer
import time
from libo_interfaces.srv import SetGoal

class NavigatorServerNode(Node):  # SetGoal 서비스 서버 노드
    def __init__(self):
        super().__init__('navigator_debug_server', automatically_declare_parameters_from_overrides=True)
        
        # SetGoal 서비스 서버 생성 (Navigator 대신 받기)
        self.service = self.create_service(
            SetGoal,
            'set_navigation_goal',
            self.set_goal_callback
        )
        
        # 수신된 메시지를 저장할 리스트
        self.received_messages = []
        
        self.get_logger().info('🧭 Navigator 디버그 서버 준비됨 - set_navigation_goal 서비스 대기 중...')
    
    def set_goal_callback(self, request, response):  # SetGoal 서비스 콜백
        """SetGoal 요청을 받아서 처리하는 콜백"""
        current_time = time.strftime('%H:%M:%S', time.localtime())
        
        # 수신 정보 저장
        message_info = {
            'time': current_time,
            'x': request.x,
            'y': request.y
        }
        self.received_messages.append(message_info)
        
        # 로그 출력
        self.get_logger().info(f'🎯 SetGoal 수신: ({request.x}, {request.y}) at {current_time}')
        
        # 성공 응답 생성
        response.success = True
        response.message = f"디버그 서버에서 수신 완료: ({request.x}, {request.y})"
        
        return response
    
    def get_latest_messages(self, count=10):  # 최근 메시지 가져오기
        """최근 수신된 메시지들을 반환"""
        return self.received_messages[-count:] if self.received_messages else []

class NavigatorTab(QWidget):  # Navigator 디버깅 탭
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node  # 메인 앱의 ROS 노드
        self.server_node = NavigatorServerNode()  # 디버그 서버 노드 생성
        self.init_ui()  # UI 초기화
        self.init_timer()  # 업데이트 타이머 초기화
    
    def init_ui(self):  # UI 초기화
        """Navigator 디버깅 탭의 UI를 초기화"""
        layout = QVBoxLayout()
        
        # 타이틀
        title_label = QLabel("🧭 Navigator 디버깅 도구")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #2c3e50; margin: 10px;")
        layout.addWidget(title_label)
        
        # 상태 표시
        status_label = QLabel("상태: SetGoal 서비스 대기 중...")
        status_label.setStyleSheet("font-size: 14px; color: #27ae60; margin: 5px;")
        layout.addWidget(status_label)
        
        # 설명
        description_label = QLabel("TaskManager가 Navigator에게 보내는 SetGoal 메시지를 여기서 대신 받아서 디버깅합니다.")
        description_label.setStyleSheet("font-size: 12px; color: #7f8c8d; margin: 5px;")
        layout.addWidget(description_label)
        
        # 수신된 메시지 표시 영역
        messages_label = QLabel("수신된 SetGoal 메시지:")
        messages_label.setStyleSheet("font-size: 14px; font-weight: bold; margin-top: 20px;")
        layout.addWidget(messages_label)
        
        # 메시지 로그 텍스트 영역
        self.messages_text = QTextEdit()
        self.messages_text.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                border: 2px solid #dee2e6;
                border-radius: 5px;
                font-family: 'Courier New', monospace;
                font-size: 12px;
                padding: 10px;
            }
        """)
        self.messages_text.setReadOnly(True)
        self.messages_text.setPlainText("SetGoal 메시지 대기 중...\n")
        layout.addWidget(self.messages_text)
        
        # 클리어 버튼
        button_layout = QHBoxLayout()
        self.clear_button = QPushButton("🗑️ 로그 지우기")
        self.clear_button.clicked.connect(self.clear_messages)
        self.clear_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                border: none;
                padding: 10px 20px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        button_layout.addWidget(self.clear_button)
        button_layout.addStretch()
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def init_timer(self):  # 업데이트 타이머 초기화
        """주기적으로 수신된 메시지를 UI에 업데이트"""
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_messages_display)
        self.update_timer.start(500)  # 0.5초마다 업데이트
    
    def update_messages_display(self):  # 메시지 표시 업데이트
        """수신된 메시지들을 UI에 표시"""
        latest_messages = self.server_node.get_latest_messages(20)  # 최근 20개 메시지
        
        if latest_messages:
            display_text = "🎯 SetGoal 메시지 수신 기록:\n\n"
            for msg in latest_messages:
                display_text += f"[{msg['time']}] 좌표: ({msg['x']}, {msg['y']})\n"
            
            # 메시지 개수 표시
            display_text += f"\n📊 총 수신 메시지: {len(self.server_node.received_messages)}개"
            
        else:
            display_text = "SetGoal 메시지 대기 중...\n"
        
        self.messages_text.setPlainText(display_text)
        
        # 스크롤을 맨 아래로
        scrollbar = self.messages_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def clear_messages(self):  # 메시지 로그 지우기
        """수신된 메시지 로그를 지움"""
        self.server_node.received_messages.clear()
        self.messages_text.setPlainText("로그가 지워졌습니다.\nSetGoal 메시지 대기 중...\n")
    
    def shutdown(self):  # 탭 종료 시 정리
        """탭이 종료될 때 서버 노드 정리"""
        if hasattr(self, 'update_timer'):
            self.update_timer.stop()
        if hasattr(self, 'server_node'):
            self.server_node.destroy_node() 