#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
import time
import random
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit, QFrame, QTableWidgetItem, QTableWidget
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QObject, Qt
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from PyQt5.QtGui import QColor

from libo_interfaces.msg import OverallStatus, TaskStatus, Heartbeat
from libo_interfaces.srv import TaskRequest, SetGoal, NavigationResult, CancelNavigation

class NavigatorServerNode(Node):  # SetGoal 서비스 서버 노드
    def __init__(self):
        super().__init__('navigator_debug_server', automatically_declare_parameters_from_overrides=True)
        
        # 서비스 서버들은 처음에 None (비활성 상태)
        self.set_goal_service = None
        self.cancel_navigation_service = None
        self.is_active = False  # 서버 활성화 상태
        
        # 수신된 메시지를 저장할 리스트
        self.received_messages = []
        self.cancel_messages = []  # CancelNavigation 메시지 저장
        
        self.get_logger().info('🧭 Navigator 디버그 서버 생성됨 (비활성 상태)')
    
    def start_service(self):  # 서비스 서버 시작
        """SetGoal과 CancelNavigation 서비스 서버를 시작"""
        if self.set_goal_service is None:
            try:
                # SetGoal 서비스 서버 생성
                self.set_goal_service = self.create_service(
                    SetGoal,
                    'set_navigation_goal',
                    self.set_goal_callback
                )
                
                # CancelNavigation 서비스 서버 생성
                self.cancel_navigation_service = self.create_service(
                    CancelNavigation,
                    'cancel_navigation',
                    self.cancel_navigation_callback
                )
                
                self.is_active = True
                self.get_logger().info('✅ Navigator 디버그 서버 활성화됨 - set_navigation_goal, cancel_navigation 서비스 대기 중...')
                return True
            except Exception as e:
                self.get_logger().error(f'❌ 서비스 시작 실패: {e}')
                return False
        return True
    
    def stop_service(self):  # 서비스 서버 중지
        """SetGoal과 CancelNavigation 서비스 서버를 중지"""
        if self.set_goal_service is not None:
            try:
                self.destroy_service(self.set_goal_service)
                self.set_goal_service = None
                
                if self.cancel_navigation_service is not None:
                    self.destroy_service(self.cancel_navigation_service)
                    self.cancel_navigation_service = None
                
                self.is_active = False
                self.get_logger().info('🔴 Navigator 디버그 서버 비활성화됨')
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
            response.message = f"디버그 서버에서 수신 완료: ({request.x}, {request.y}) at {current_time}"
            
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
            response.message = f"디버그 서버 오류: {str(e)}"
            
            # 에러 상태 저장
            if 'message_info' in locals():
                message_info['status'] = 'error'
                message_info['response'] = f'ERROR: {str(e)}'
            
            return response
    
    def cancel_navigation_callback(self, request, response):  # CancelNavigation 서비스 콜백
        """CancelNavigation 요청을 받아서 처리하는 콜백"""
        current_time = time.strftime('%H:%M:%S', time.localtime())
        
        try:
            # 수신 정보 저장
            message_info = {
                'time': current_time,
                'status': 'received'  # 수신 상태 추가
            }
            self.cancel_messages.append(message_info)
            
            # 로그 출력
            self.get_logger().info(f'⏹️ CancelNavigation 수신 at {current_time}')
            
            # 성공 응답 생성
            response.success = True
            response.message = f"네비게이션 취소 요청 처리 완료 at {current_time}"
            
            # 응답 상태 업데이트
            message_info['status'] = 'responded'
            message_info['response'] = 'SUCCESS'
            
            self.get_logger().info(f'✅ CancelNavigation 응답 전송: SUCCESS - {response.message}')
            
            return response
            
        except Exception as e:
            # 에러 처리
            self.get_logger().error(f'❌ CancelNavigation 처리 중 오류: {e}')
            
            # 실패 응답 생성
            response.success = False
            response.message = f"디버그 서버 오류: {str(e)}"
            
            # 에러 상태 저장
            if 'message_info' in locals():
                message_info['status'] = 'error'
                message_info['response'] = f'ERROR: {str(e)}'
            
            return response
    
    def get_latest_messages(self, count=10):  # 최근 메시지 가져오기
        """최근 수신된 메시지들을 반환"""
        return self.received_messages[-count:] if self.received_messages else []
    
    def get_latest_cancel_messages(self, count=10):  # 최근 취소 메시지 가져오기
        """최근 수신된 취소 메시지들을 반환"""
        return self.cancel_messages[-count:] if self.cancel_messages else []

class MainControlTab(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node
        self.robot_status_dict = {}
        self.task_status_data = {}
        self.heartbeat_log = []
        self.heartbeat_start_time = time.time()
        self.navigation_result_logs = []
        
        # Navigator 서버 노드 생성
        self.navigator_server = NavigatorServerNode()
        
        # ROS 클라이언트들
        self.task_request_client = self.ros_node.create_client(TaskRequest, '/task_request')
        self.navigation_result_client = self.ros_node.create_client(NavigationResult, 'navigation_result')
        
        self.init_ui()
        self.init_ros_connections()
        self.init_timers()
    
    def init_ui(self):
        """UI 초기화 - main_control_tab.ui 파일 로드"""
        ui_file_path = os.path.join(get_package_share_directory('admin'), 'ui', 'main_control_tab.ui')
        uic.loadUi(ui_file_path, self)
        
        # Heartbeat 테이블 생성 및 초기화
        self.heartbeat_table = QTableWidget()
        self.heartbeat_table.setColumnCount(3)
        self.heartbeat_table.setHorizontalHeaderLabels(['Sender ID', '경과 시간 (초)', 'Timestamp'])
        
        # 기존 QWidget을 QTableWidget으로 교체
        layout = self.heartbeat_log_text.layout()
        if layout is None:
            layout = QVBoxLayout(self.heartbeat_log_text)
        layout.addWidget(self.heartbeat_table)
        
        # 시그널 연결
        self.send_task_button.clicked.connect(self.send_task_request)
        self.toggle_navigator_button.clicked.connect(self.toggle_navigator_service)
        self.send_success_button.clicked.connect(lambda: self.send_navigation_result("SUCCEEDED"))
        self.send_failed_button.clicked.connect(lambda: self.send_navigation_result("FAILED"))
        self.send_canceled_button.clicked.connect(lambda: self.send_navigation_result("CANCELED"))
        
        # CancelNavigation 로그 업데이트 타이머
        self.cancel_log_timer = QTimer()
        self.cancel_log_timer.timeout.connect(self.update_cancel_navigation_log)
        self.cancel_log_timer.start(1000)  # 1초마다 업데이트
    
    def init_ros_connections(self):
        """ROS 연결 초기화"""
        # OverallStatus 구독자
        self.robot_status_subscription = self.ros_node.create_subscription(
            OverallStatus, 'robot_status', self.robot_status_callback, 10
        )
        
        # TaskStatus 구독자
        self.task_status_subscription = self.ros_node.create_subscription(
            TaskStatus, 'task_status', self.task_status_callback, 10
        )
        
        # Heartbeat 구독자 (BEST_EFFORT QoS 사용)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # 최선 노력 수신
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,  # 휘발성
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,  # 마지막 N개 메시지만 유지
            depth=10  # 큐 깊이
        )
        self.heartbeat_subscription = self.ros_node.create_subscription(
            Heartbeat, 'heartbeat', self.heartbeat_callback, qos_profile
        )
    
    def init_timers(self):
        """타이머 초기화"""
        # 로봇 상태 업데이트 타이머
        self.robot_status_timer = QTimer()
        self.robot_status_timer.timeout.connect(self.update_robot_status_display)
        self.robot_status_timer.start(1000)  # 1초마다
        
        # 작업 상태 업데이트 타이머
        self.task_status_timer = QTimer()
        self.task_status_timer.timeout.connect(self.update_task_status_display)
        self.task_status_timer.start(1000)  # 1초마다
        
        # Navigator 서버 노드 스핀 타이머
        self.navigator_spin_timer = QTimer()
        self.navigator_spin_timer.timeout.connect(self.spin_navigator_server)
        self.navigator_spin_timer.start(10)  # 10ms마다 (100Hz)
    
    def spin_navigator_server(self):
        """Navigator 서버 노드 스핀"""
        rclpy.spin_once(self.navigator_server, timeout_sec=0.001)
    
    def send_task_request(self):
        """TaskRequest 전송"""
        robot_id = self.robot_id_edit.text()
        task_type = self.task_type_edit.text()
        call_location = self.call_location_edit.text()
        goal_location = self.goal_location_edit.text()
        
        if not self.task_request_client.wait_for_service(timeout_sec=1.0):
            self.log_task_message("❌ TaskRequest 서비스를 찾을 수 없음")
            return
        
        request = TaskRequest.Request()
        request.robot_id = robot_id
        request.task_type = task_type
        request.call_location = call_location
        request.goal_location = goal_location
        
        future = self.task_request_client.call_async(request)
        future.add_done_callback(self.task_response_callback)
        
        self.log_task_message(f"📤 TaskRequest 전송: {robot_id} | {task_type} | {call_location} → {goal_location}")
    
    def task_response_callback(self, future):
        """TaskRequest 응답 처리"""
        try:
            response = future.result()
            if response.success:
                self.log_task_message(f"✅ TaskRequest 성공: {response.message}")
            else:
                self.log_task_message(f"❌ TaskRequest 실패: {response.message}")
        except Exception as e:
            self.log_task_message(f"❌ TaskRequest 오류: {str(e)}")
    
    def log_task_message(self, message):
        """Task 로그 메시지 출력"""
        timestamp = time.strftime("%H:%M:%S")
        self.task_log_text.append(f"[{timestamp}] {message}")
    
    def heartbeat_callback(self, msg):
        """Heartbeat 메시지 수신"""
        timestamp = time.time()
        self.heartbeat_log.append({
            'msg': msg,
            'received_time': timestamp
        })
        
        # 최근 50개만 유지
        if len(self.heartbeat_log) > 50:
            self.heartbeat_log = self.heartbeat_log[-50:]
        
        # 테이블에 새 행 추가
        row_position = self.heartbeat_table.rowCount()
        self.heartbeat_table.insertRow(row_position)
        
        # 새 행에 데이터를 채움
        elapsed_time = timestamp - self.heartbeat_start_time
        timestamp_msg = msg.timestamp
        timestamp_str = f"{timestamp_msg.sec}.{timestamp_msg.nanosec:09d}"
        
        self.heartbeat_table.setItem(row_position, 0, QTableWidgetItem(msg.sender_id))
        self.heartbeat_table.setItem(row_position, 1, QTableWidgetItem(f"{elapsed_time:.2f}"))
        self.heartbeat_table.setItem(row_position, 2, QTableWidgetItem(timestamp_str))
        
        self.heartbeat_table.scrollToBottom()
        
        # 컬럼 너비 자동 조절
        self.heartbeat_table.resizeColumnsToContents()
        current_ts_width = self.heartbeat_table.columnWidth(2)
        self.heartbeat_table.setColumnWidth(2, current_ts_width * 2)
    
    def update_heartbeat_display(self):
        """Heartbeat 로그 표시 업데이트 (테이블 방식이므로 불필요)"""
        pass
    
    def toggle_navigator_service(self):
        """Navigator 서비스 토글"""
        current_text = self.toggle_navigator_button.text()
        if "Start" in current_text:
            # 서비스 시작
            if self.navigator_server.start_service():
                self.toggle_navigator_button.setText("🔴 Stop Navigator Service")
                self.toggle_navigator_button.setStyleSheet("background-color: #e74c3c;")
                self.log_navigator_message("🟢 Navigator 서비스 시작됨 - set_navigation_goal, cancel_navigation 서비스 대기 중...")
            else:
                self.log_navigator_message("❌ Navigator 서비스 시작 실패")
        else:
            # 서비스 중지
            if self.navigator_server.stop_service():
                self.toggle_navigator_button.setText("🟢 Start Navigator Service")
                self.toggle_navigator_button.setStyleSheet("background-color: #27ae60;")
                self.log_navigator_message("🔴 Navigator 서비스 중지됨")
            else:
                self.log_navigator_message("❌ Navigator 서비스 중지 실패")
    
    def send_navigation_result(self, result_type):
        """NavigationResult 서비스 호출"""
        if not self.navigation_result_client.wait_for_service(timeout_sec=1.0):
            self.log_navigator_message("❌ NavigationResult 서비스를 찾을 수 없음")
            return
        
        request = NavigationResult.Request()
        request.result = result_type
        
        future = self.navigation_result_client.call_async(request)
        future.add_done_callback(lambda f: self.navigation_result_callback(f, result_type))
        
        self.log_navigator_message(f"📤 NavigationResult 전송: {result_type}")
    
    def navigation_result_callback(self, future, result_type):
        """NavigationResult 응답 처리"""
        try:
            response = future.result()
            if response.success:
                self.log_navigator_message(f"✅ NavigationResult 성공: {result_type}")
            else:
                self.log_navigator_message(f"❌ NavigationResult 실패: {response.message}")
        except Exception as e:
            self.log_navigator_message(f"❌ NavigationResult 오류: {str(e)}")
    
    def log_navigator_message(self, message):
        """Navigator 로그 메시지 출력"""
        timestamp = time.strftime("%H:%M:%S")
        self.navigator_log_text.append(f"[{timestamp}] {message}")
    
    def robot_status_callback(self, msg):
        """OverallStatus 메시지 수신"""
        robot_id = msg.robot_id
        self.robot_status_dict[robot_id] = {
            'state': msg.robot_state,
            'is_available': msg.is_available,
            'battery': msg.battery,
            'book_weight': msg.book_weight,
            'position_x': msg.position_x,
            'position_y': msg.position_y,
            'position_yaw': msg.position_yaw,
            'timestamp': time.time()
        }
    
    def update_robot_status_display(self):
        """로봇 상태 표시 업데이트"""
        current_time = time.time()
        active_robots = []
        
        for robot_id, status in self.robot_status_dict.items():
            # 5초 이상 업데이트가 없으면 제거
            if current_time - status['timestamp'] > 5.0:
                continue
            
            # 로봇 상태에 따라 사용 가능/불가 결정
            robot_state = status['state']
            if robot_state in ['INIT', 'CHARGING']:
                available_text = "🔴 사용 불가"
            elif status['is_available']:
                available_text = "🟢 사용가능"
            else:
                available_text = "🔴 사용중"
            
            robot_info = f"🤖 {robot_id}\n"
            robot_info += f"   상태: {status['state']}\n"
            robot_info += f"   {available_text}\n"
            robot_info += f"   배터리: {status['battery']}%\n"
            robot_info += f"   무게: {status.get('book_weight', 0.0):.1f}kg\n"
            robot_info += f"   위치: ({status.get('position_x', 0.0):.1f}, {status.get('position_y', 0.0):.1f})\n"
            robot_info += f"   방향: {status.get('position_yaw', 0.0):.1f}°\n"
            robot_info += "─" * 20 + "\n"
            active_robots.append(robot_info)
        
        if active_robots:
            self.robot_list_text.setPlainText("".join(active_robots))
        else:
            self.robot_list_text.setPlainText("활성 로봇 없음")
    
    def task_status_callback(self, msg):
        """TaskStatus 메시지 수신"""
        task_id = msg.task_id
        self.task_status_data[task_id] = {
            'robot_id': msg.robot_id,
            'task_type': msg.task_type,
            'task_stage': msg.task_stage,
            'call_location': msg.call_location,
            'goal_location': msg.goal_location,
            'start_time': msg.start_time,
            'timestamp': time.time()
        }
    
    def update_task_status_display(self):
        """작업 상태 표시 업데이트"""
        current_time = time.time()
        active_tasks = []
        
        for task_id, task in self.task_status_data.items():
            # 10초 이상 업데이트가 없으면 제거
            if current_time - task['timestamp'] > 10.0:
                continue
            
            task_info = f"📋 Task[{task_id}]\n"
            task_info += f"   로봇: {task['robot_id']}\n"
            task_info += f"   타입: {task['task_type']}\n"
            task_info += f"   단계: {task['task_stage']}\n"
            task_info += f"   경로: {task['call_location']} → {task['goal_location']}\n"
            task_info += "─" * 20 + "\n"
            active_tasks.append(task_info)
        
        if active_tasks:
            self.current_task_text.setPlainText("".join(active_tasks))
        else:
            self.current_task_text.setPlainText("활성 작업 없음")
    
    def update_cancel_navigation_log(self):
        """CancelNavigation 로그 업데이트"""
        if hasattr(self, 'cancel_navigation_log_text'):
            latest_messages = self.navigator_server.get_latest_cancel_messages(5)  # 최근 5개 메시지
            
            if latest_messages:
                log_text = ""
                for msg in latest_messages:
                    status_icon = "✅" if msg.get('status') == 'responded' else "📥"
                    response_text = msg.get('response', '대기중')
                    log_text += f"[{msg['time']}] {status_icon} {response_text}\n"
                
                self.cancel_navigation_log_text.setPlainText(log_text)
            else:
                self.cancel_navigation_log_text.setPlainText("취소 요청 없음")
    
    def cleanup(self):
        """탭 종료 시 정리"""
        if hasattr(self, 'navigator_spin_timer'):
            self.navigator_spin_timer.stop()
        if hasattr(self, 'navigator_server'):
            if self.navigator_server.is_active:
                self.navigator_server.stop_service()
            self.navigator_server.destroy_node() 