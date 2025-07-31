#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import rclpy
import time  # 시간 추적용 추가
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from ament_index_python.packages import get_package_share_directory

from admin.tabs.task_request_tab import TaskRequestTab # 우리가 만든 TaskRequestTab을 임포트
from admin.tabs.heartbeat_monitor_tab import HeartbeatMonitorTab # 새로 만든 HeartbeatMonitorTab을 임포트
from libo_interfaces.msg import OverallStatus  # OverallStatus 메시지 임포트 (String 대신)
from libo_interfaces.msg import TaskStatus  # TaskStatus 메시지 임포트

class AdminWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ros_node = rclpy.create_node('admin_gui_node') # GUI 전체에서 사용할 ROS 노드 생성
        self.robot_status_dict = {}  # 로봇 상태를 저장할 딕셔너리 (문자열 대신)
        self.task_status_data = {}  # 작업 상태를 저장할 딕셔너리
        self.init_ui() # UI 파일을 로드하고 초기화하는 함수를 호출
        self.init_tabs() # 탭들을 초기화하고 추가하는 함수를 호출
        self.init_robot_status_subscriber()  # OverallStatus 구독자 초기화
        self.init_task_status_subscriber()  # TaskStatus 구독자 초기화
        self.init_timer() # ROS 통신을 위한 타이머 시작
        self.init_robot_timeout_timer()  # 로봇 타임아웃 체크 타이머 추가

    def init_ui(self):
        package_share_dir = get_package_share_directory('admin') # 'admin' 패키지 공유 디렉토리 경로를 찾음
        ui_file = os.path.join(package_share_dir, 'ui', 'main_window.ui') # UI 파일의 전체 경로를 조합
        uic.loadUi(ui_file, self) # UI 파일을 불러와 현재 객체에 적용

    def init_tabs(self):
        # Task Request 탭 추가
        self.task_request_tab = TaskRequestTab(self.ros_node) # TaskRequestTab 객체를 생성
        self.tabWidget.addTab(self.task_request_tab, "🚀 Task Request 테스트") # 'tabWidget'에 새 탭을 추가

        # Heartbeat Monitor 탭 추가
        self.heartbeat_monitor_tab = HeartbeatMonitorTab(self.ros_node) # HeartbeatMonitorTab 객체를 생성하고 메인 노드를 전달
        self.tabWidget.addTab(self.heartbeat_monitor_tab, "💓 Heartbeat 모니터") # 'tabWidget'에 새 탭을 추가

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
            # 로봇별로 상태 정보 저장 (마지막 수신 시간도 포함)
            self.robot_status_dict[msg.robot_id] = {
                'id': msg.robot_id,  # 로봇 ID
                'available': msg.is_available,  # 사용 가능 여부
                'battery': msg.battery,  # 배터리 잔량
                'position': f"({msg.position_x:.1f}, {msg.position_y:.1f})",  # 위치 정보
                'last_seen': time.time()  # 마지막 수신 시간 추가
            }
            self.update_robot_status_display()  # GUI 업데이트
            
        except Exception as e:
            print(f"로봇 상태 처리 중 오류: {e}")

    def task_status_callback(self, msg):  # TaskStatus 메시지 수신 콜백
        """TaskStatus 메시지를 받았을 때 GUI 업데이트"""
        try:
            # 작업 상태 정보 저장
            self.task_status_data = {
                'robot_id': msg.robot_id,  # 로봇 ID
                'task_type': msg.task_type,  # 작업 타입
                'task_stage': msg.task_stage,  # 작업 단계
                'call_location': msg.call_location,  # 호출 위치
                'goal_location': msg.goal_location,  # 목표 위치
                'last_updated': time.time()  # 마지막 업데이트 시간
            }
            self.update_task_status_display()  # GUI 업데이트
            print(f"📋 작업 상태 수신: {msg.robot_id} - {msg.task_type}")  # 디버그 출력
            
        except Exception as e:
            print(f"작업 상태 처리 중 오류: {e}")

    def update_robot_status_display(self):  # 로봇 상태 표시 업데이트
        """활성 로봇들의 상태를 위젯에 표시"""
        try:
            # 로봇 개수 업데이트
            robot_count = len(self.robot_status_dict)  # 활성 로봇 개수
            self.robot_count_label.setText(f"Count: {robot_count}")  # 카운트 라벨 업데이트
            
            # 로봇 목록 텍스트 생성
            if robot_count == 0:  # 로봇이 없다면
                status_text = "활성 로봇 없음"
            else:
                status_lines = []  # 로봇 정보 저장할 리스트
                for robot_id, status in self.robot_status_dict.items():  # 각 로봇에 대해
                    availability = "사용가능" if status['available'] else "사용중"  # 가용성 표시
                    status_lines.append(f"🤖 {robot_id}: {availability}")  # 로봇 정보 추가
                status_text = "\n".join(status_lines)  # 줄바꿈으로 연결
                
            self.robot_list_label.setText(status_text)  # 목록 라벨 업데이트
                
        except Exception as e:
            print(f"로봇 상태 표시 중 오류: {e}")

    def update_task_status_display(self):  # 작업 상태 표시 업데이트
        """현재 작업 상태를 위젯에 표시"""
        try:
            if not self.task_status_data:  # 작업 데이터가 없다면
                status_text = "활성 작업 없음"
            else:
                # 작업 단계 텍스트 변환
                stage_text = {1: "시작", 2: "진행중", 3: "완료"}.get(self.task_status_data['task_stage'], "알 수 없음")
                
                # 작업 정보 텍스트 생성
                status_text = (f"🤖 로봇: {self.task_status_data['robot_id']}\n"
                              f"📋 작업: {self.task_status_data['task_type']}\n" 
                              f"⚡ 단계: {stage_text}\n"
                              f"📍 {self.task_status_data['call_location']} → {self.task_status_data['goal_location']}")
                
            # 위젯 업데이트 (위젯 이름은 UI에서 추가할 예정)
            if hasattr(self, 'task_status_label'):  # task_status_label이 있다면
                self.task_status_label.setText(status_text)  # 텍스트 업데이트
                
        except Exception as e:
            print(f"작업 상태 표시 중 오류: {e}")

    def init_timer(self):
        self.ros_timer = QTimer(self) # QTimer 객체 생성
        self.ros_timer.timeout.connect(self.spin_ros_nodes) # 타이머가 만료될 때마다 spin_ros_nodes 함수를 호출하도록 연결
        self.ros_timer.start(100) # 100ms (0.1초) 간격으로 타이머 시작

    def spin_ros_nodes(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0) # 메인 GUI의 ROS 노드를 스핀
        # task_request_tab에 client_node가 존재하면 그것도 스핀
        if hasattr(self, 'task_request_tab') and hasattr(self.task_request_tab, 'client_node'):
            rclpy.spin_once(self.task_request_tab.client_node, timeout_sec=0)
        
        # heartbeat_monitor_tab에 node가 존재하면 그것도 스핀
        if hasattr(self, 'heartbeat_monitor_tab') and hasattr(self.heartbeat_monitor_tab, 'node'):
            rclpy.spin_once(self.heartbeat_monitor_tab.node, timeout_sec=0)

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
            time_since_last_task_update = current_time - self.task_status_data['last_updated']  # 마지막 업데이트 후 경과 시간
            if time_since_last_task_update > timeout_seconds:  # 3초 타임아웃됐다면
                self.task_status_data = {}  # TaskStatus 데이터 제거
                self.update_task_status_display()  # GUI 업데이트
                print(f"🚫 작업 상태 제거됨 (타임아웃)")  # 디버그 출력

    def closeEvent(self, event):
        self.task_request_tab.shutdown() # TaskRequest 탭의 정리 함수 호출
        self.heartbeat_monitor_tab.shutdown() # Heartbeat 탭의 정리 함수도 호출
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