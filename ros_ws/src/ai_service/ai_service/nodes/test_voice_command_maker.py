#!/usr/bin/env python3
# test_voice_command_maker.py
# VoiceCommand 메시지를 발행하기 위한 PyQt5 기반 테스트 도구

import sys
import os
from datetime import datetime
import pytz
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from libo_interfaces.msg import VoiceCommand
from libo_interfaces.srv import ActivateTalker, DeactivateTalker, EndTask

from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QPushButton,
                            QVBoxLayout, QHBoxLayout, QWidget, QLabel,
                            QLineEdit, QGridLayout, QStatusBar, QGroupBox,
                            QScrollArea)
from PyQt5.QtCore import Qt, pyqtSignal, QSize, QTimer
from PyQt5.QtGui import QFont, QIcon


# 한국 시간 출력 함수
def get_kr_time():
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 밀리초 3자리까지 표시


class ROS2Publisher(Node):
    """VoiceCommand 메시지 발행 및 서비스를 제공하는 ROS2 노드"""
    def __init__(self):
        super().__init__('voice_command_publisher')
        # 메시지 퍼블리셔
        self.publisher = self.create_publisher(
            VoiceCommand,
            '/voice_command',
            10
        )
        
        # 서비스 서버
        self.activate_talker_service = self.create_service(
            ActivateTalker,
            '/activate_talker',  # 슬래시(/)로 시작하는 전체 경로 사용
            self.activate_talker_callback
        )
        
        self.deactivate_talker_service = self.create_service(
            DeactivateTalker,
            '/deactivate_talker',  # 슬래시(/)로 시작하는 전체 경로 사용
            self.deactivate_talker_callback
        )
        
        # end_task 서비스는 클라이언트로 변경
        self.end_task_client = self.create_client(
            EndTask,
            '/end_task'  # 슬래시(/)로 시작하는 전체 경로 사용
        )
        
        self.get_logger().info('🚀 ROS2 노드 초기화 완료 (퍼블리셔, 서비스 서버 및 클라이언트)')
        
    def publish_command(self, robot_id, category, action):
        """VoiceCommand 메시지 발행"""
        msg = VoiceCommand()
        msg.robot_id = robot_id
        msg.category = category
        msg.action = action
        
        self.publisher.publish(msg)
        self.get_logger().info(f'📢 메시지 발행: robot_id={robot_id}, category={category}, action={action}')
        return True
        
    def activate_talker_callback(self, request, response):
        """음성 인식 활성화 서비스 콜백"""
        robot_id = request.robot_id
        
        try:
            self.get_logger().info(f'🎤 음성 인식 활성화 요청 수신: robot_id={robot_id}')
            
            # 실제로는 talker_manager.py에서 음성 인식을 활성화하는 로직을 여기서 시뮬레이션
            
            response.success = True
            response.message = f"음성 인식이 활성화되었습니다. (robot_id: {robot_id})"
            
        except Exception as e:
            self.get_logger().error(f'❌ 음성 인식 활성화 처리 중 오류: {str(e)}')
            response.success = False
            response.message = f"오류: {str(e)}"
            
        return response
        
    def deactivate_talker_callback(self, request, response):
        """음성 인식 비활성화 서비스 콜백"""
        robot_id = request.robot_id
        
        try:
            self.get_logger().info(f'🔇 음성 인식 비활성화 요청 수신: robot_id={robot_id}')
            
            # 실제로는 talker_manager.py에서 음성 인식을 비활성화하는 로직을 여기서 시뮬레이션
            
            response.success = True
            response.message = f"음성 인식이 비활성화되었습니다. (robot_id: {robot_id})"
            
        except Exception as e:
            self.get_logger().error(f'❌ 음성 인식 비활성화 처리 중 오류: {str(e)}')
            response.success = False
            response.message = f"오류: {str(e)}"
            
        return response
        
    def call_end_task(self, robot_id, task_type, callback=None):
        """작업 종료 서비스 클라이언트 호출"""
        if not self.end_task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('❌ end_task 서비스를 사용할 수 없습니다')
            return False
            
        request = EndTask.Request()
        request.robot_id = robot_id
        request.task_type = task_type
        
        self.get_logger().info(f'🛑 작업 종료 요청 전송: robot_id={robot_id}, task_type={task_type}')
        
        future = self.end_task_client.call_async(request)
        if callback:
            future.add_done_callback(callback)
            
        return True


class VoiceCommandMaker(QMainWindow):
    """음성 명령을 생성하고 발행하는 GUI 애플리케이션"""
    status_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Voice Command Maker')
        self.setGeometry(100, 100, 800, 600)
        self.setMinimumSize(800, 600)
        
        # ROS2 초기화는 별도 스레드에서 수행
        self.ros2_thread = None
        self.ros2_node = None
        self.init_ros2()
        
        # GUI 초기화
        self.init_ui()
        
        # 상태 시그널 연결
        self.status_signal.connect(self.update_status)
        
    def init_ros2(self):
        """ROS2 노드 초기화 (별도 스레드에서 실행)"""
        self.ros2_shutdown = threading.Event()  # 종료 이벤트 추가
        
        def run_ros2():
            rclpy.init()
            self.ros2_node = ROS2Publisher()
            self.status_signal.emit("🚀 ROS2 노드 및 서비스 서버 초기화 완료")
            
            # 종료 신호가 올 때까지 스핀 (종료 가능하게)
            while rclpy.ok() and not self.ros2_shutdown.is_set():
                rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
            
            # 정리
            if self.ros2_node and rclpy.ok():
                self.ros2_node.destroy_node()
            print("ROS2 스레드 종료됨")
            
        self.ros2_thread = threading.Thread(target=run_ros2, daemon=True)
        self.ros2_thread.start()
        
        # ROS2 노드 초기화 대기
        time.sleep(1)
        
    def init_ui(self):
        """GUI 요소 초기화"""
        # 중앙 위젯 설정
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 메인 레이아웃
        main_layout = QVBoxLayout(main_widget)
        
        # 로봇 ID 입력 섹션
        id_layout = QHBoxLayout()
        id_label = QLabel("로봇 ID:")
        self.id_input = QLineEdit("libo_a")  # 기본값
        id_layout.addWidget(id_label)
        id_layout.addWidget(self.id_input)
        id_layout.addStretch(1)
        
        # 탭 위젯 생성
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.North)
        self.tabs.setMovable(True)
        
        # 카테고리별 탭 추가
        self.add_category_tab("common", "일반")
        self.add_category_tab("escort", "안내")
        self.add_category_tab("delivery", "배달")
        self.add_category_tab("assist", "어시스트")
        
        # 서비스 탭 추가
        self.add_services_tab()
        
        # 상태 표시줄
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("준비됨")
        
        # 레이아웃에 위젯 추가
        main_layout.addLayout(id_layout)
        main_layout.addWidget(self.tabs)
        
    def add_services_tab(self):
        """서비스 호출 탭 추가"""
        tab = QWidget()
        
        # 스크롤 영역 추가
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        
        # 로그 표시 영역
        log_group = QGroupBox("서비스 응답 로그")
        log_layout = QVBoxLayout(log_group)
        
        self.service_log = QLabel("서비스 응답이 여기에 표시됩니다")
        self.service_log.setWordWrap(True)
        self.service_log.setMinimumHeight(100)
        self.service_log.setStyleSheet("background-color: #f0f0f0; padding: 10px; border-radius: 5px;")
        log_layout.addWidget(self.service_log)
        
        # 음성 인식 활성화/비활성화 그룹
        voice_group = QGroupBox("음성 인식 제어")
        voice_layout = QVBoxLayout(voice_group)
        
        # 서비스 테스트용 버튼
        activate_btn = QPushButton("🎤 음성 인식 활성화 (activate_talker)")
        activate_btn.setMinimumHeight(50)
        activate_btn.clicked.connect(self.test_activate_talker)
        voice_layout.addWidget(activate_btn)
        
        deactivate_btn = QPushButton("🔇 음성 인식 비활성화 (deactivate_talker)")
        deactivate_btn.setMinimumHeight(50)
        deactivate_btn.clicked.connect(self.test_deactivate_talker)
        voice_layout.addWidget(deactivate_btn)
        
        # 작업 종료 그룹
        task_group = QGroupBox("작업 종료 제어")
        task_layout = QGridLayout(task_group)
        
        # 작업 유형 선택 레이아웃
        task_type_layout = QHBoxLayout()
        task_type_label = QLabel("작업 유형:")
        self.task_type_input = QLineEdit("assist")  # 기본값
        task_type_layout.addWidget(task_type_label)
        task_type_layout.addWidget(self.task_type_input)
        
        end_task_btn = QPushButton("🛑 작업 종료 (end_task)")
        end_task_btn.setMinimumHeight(50)
        end_task_btn.clicked.connect(self.test_end_task)
        
        task_layout.addLayout(task_type_layout, 0, 0)
        task_layout.addWidget(end_task_btn, 1, 0)
        
        # 그룹을 스크롤 레이아웃에 추가
        scroll_layout.addWidget(log_group)
        scroll_layout.addWidget(voice_group)
        scroll_layout.addWidget(task_group)
        
        # 스트레치 추가 (하단 여백)
        scroll_layout.addStretch(1)
        
        # 스크롤 영역 설정 완료
        scroll.setWidget(scroll_content)
        
        # 탭 레이아웃에 스크롤 영역 추가
        tab_layout = QVBoxLayout(tab)
        tab_layout.addWidget(scroll)
        
        # 탭 추가
        self.tabs.addTab(tab, "서비스")
    
    def add_category_tab(self, category, display_name):
        """카테고리별 탭 추가"""
        tab = QWidget()
        
        # 스크롤 영역 추가
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        
        # 명령 버튼 그룹
        if category == "common":
            self.add_command_group(scroll_layout, category, "전원/초기화", [
                ("power_on", "전원 켜기"),
                ("initialized", "초기화 완료"),
                ("charging", "충전 시작"),
                ("battery_sufficient", "배터리 충분"),
            ])
            
            self.add_command_group(scroll_layout, category, "이동", [
                ("depart_base", "베이스 출발"),
                ("obstacle_detected", "장애물 감지"),
                ("reroute", "경로 재설정"),
                ("return", "복귀"),
                ("arrived_base", "베이스 도착"),
            ])
            
        elif category == "escort":
            self.add_command_group(scroll_layout, category, "안내 시작", [
                ("depart_base", "베이스 출발"),
                ("arrived_kiosk", "키오스크 도착"),
            ])
            
            self.add_command_group(scroll_layout, category, "안내 중", [
                ("lost_user", "사용자 감지 실패"),
                ("user_reconnected", "사용자 재연결"),
            ])
            
            self.add_command_group(scroll_layout, category, "안내 종료", [
                ("arrived_destination", "목적지 도착"),
                ("return", "복귀"),
                ("arrived_base", "베이스 도착"),
            ])
            
        elif category == "delivery":
            self.add_command_group(scroll_layout, category, "배달 시작", [
                ("depart_base", "베이스 출발"),
                ("arrived_admin_desk", "관리자 데스크 도착"),
                ("receive_next_goal", "목적지 수신"),
            ])
            
            self.add_command_group(scroll_layout, category, "배달 중", [
                ("arrived_destination", "목적지 도착"),
                ("called_by_staff", "직원 호출"),
            ])
            
            self.add_command_group(scroll_layout, category, "배달 종료", [
                ("return", "복귀"),
                ("arrived_base", "베이스 도착"),
            ])
            
        elif category == "assist":
            self.add_command_group(scroll_layout, category, "어시스트 시작", [
                ("depart_base", "베이스 출발"),
                ("arrived_kiosk", "키오스크 도착"),
                ("qr_authenticated", "QR 인증 완료"),
            ])
            
            self.add_command_group(scroll_layout, category, "어시스트 중", [
                ("no_person_5s", "사람 감지 실패"),
                ("person_detected", "사람 감지 성공"),
                ("called_by_staff", "직원 호출"),
                ("pause", "일시정지"),
                ("resume", "재개"),
            ])
            
            self.add_command_group(scroll_layout, category, "어시스트 종료", [
                ("return", "복귀"),
                ("arrived_base", "베이스 도착"),
            ])
        
        # 스트레치 추가 (하단 여백)
        scroll_layout.addStretch(1)
        
        # 스크롤 영역 설정 완료
        scroll.setWidget(scroll_content)
        
        # 탭 레이아웃에 스크롤 영역 추가
        tab_layout = QVBoxLayout(tab)
        tab_layout.addWidget(scroll)
        
        # 탭 추가
        self.tabs.addTab(tab, display_name)
        
    def add_command_group(self, parent_layout, category, group_name, commands):
        """명령 버튼 그룹 추가"""
        group = QGroupBox(group_name)
        group_layout = QGridLayout(group)
        
        for row, (action, display_name) in enumerate(commands):
            btn = QPushButton(display_name)
            btn.setMinimumHeight(40)
            btn.clicked.connect(lambda checked, c=category, a=action: self.send_command(c, a))
            group_layout.addWidget(btn, row, 0)
        
        parent_layout.addWidget(group)
        
    def send_command(self, category, action):
        """명령 발행"""
        robot_id = self.id_input.text().strip()
        if not robot_id:
            robot_id = "libo_a"  # 기본값
        
        if not self.ros2_node:
            self.status_signal.emit("❌ ROS2 노드가 초기화되지 않았습니다")
            return
        
        # 메시지 발행
        try:
            self.ros2_node.publish_command(robot_id, category, action)
            self.status_signal.emit(f"✅ 명령 발행 성공: [{robot_id}] {category}/{action}")
            
            # 콘솔에도 출력
            print(f"[{get_kr_time()}][🔊 TEST] VoiceCommand 발행: robot_id={robot_id}, "
                  f"category={category}, action={action}")
            
        except Exception as e:
            self.status_signal.emit(f"❌ 명령 발행 실패: {str(e)}")
            print(f"[{get_kr_time()}][❌ ERROR] 명령 발행 실패: {str(e)}")
    
    def test_activate_talker(self):
        """음성 인식 활성화 서비스 테스트"""
        robot_id = self.id_input.text().strip() or "libo_a"
        
        if not self.ros2_node:
            self.status_signal.emit("❌ ROS2 노드가 초기화되지 않았습니다")
            return
            
        # 상태 업데이트
        self.status_signal.emit(f"🎤 음성 인식 활성화 요청: robot_id={robot_id}")
            
        # 직접 활성화 서비스 콜백을 호출하여 시뮬레이션
        try:
            # 요청 객체 생성
            request = ActivateTalker.Request()
            request.robot_id = robot_id
            
            # 응답 객체 생성
            response = ActivateTalker.Response()
            
            # 서비스 콜백 직접 호출 (시뮬레이션)
            response = self.ros2_node.activate_talker_callback(request, response)
            
            # 로그 추가
            self.ros2_node.get_logger().info(f'🎤 음성 인식 활성화 요청 처리됨: robot_id={robot_id}')
            
            # 결과 처리
            if response.success:
                msg = f"✅ 음성 인식 활성화 성공: {response.message}"
                self.service_log.setText(msg)
                self.status_signal.emit(f"🎤 음성 인식 활성화 성공")
            else:
                msg = f"❌ 음성 인식 활성화 실패: {response.message}"
                self.service_log.setText(msg)
                self.status_signal.emit(msg)
                
        except Exception as e:
            error_msg = f"❌ 음성 인식 활성화 요청 중 오류: {str(e)}"
            self.service_log.setText(error_msg)
            self.status_signal.emit(error_msg)
    
    def test_deactivate_talker(self):
        """음성 인식 비활성화 서비스 테스트"""
        robot_id = self.id_input.text().strip() or "libo_a"
        
        if not self.ros2_node:
            self.status_signal.emit("❌ ROS2 노드가 초기화되지 않았습니다")
            return
            
        # 직접 비활성화 서비스 콜백을 호출하여 시뮬레이션
        try:
            # 요청 객체 생성
            request = DeactivateTalker.Request()  # 올바른 메시지 타입 사용
            request.robot_id = robot_id
            
            # 응답 객체 생성
            response = DeactivateTalker.Response()  # 올바른 메시지 타입 사용
            
            # 서비스 콜백 직접 호출 (시뮬레이션)
            response = self.ros2_node.deactivate_talker_callback(request, response)
            
            # 로그 추가
            self.ros2_node.get_logger().info(f'🔇 음성 인식 비활성화 요청 처리됨: robot_id={robot_id}')
            
            # 결과 처리
            if response.success:
                msg = f"✅ 음성 인식 비활성화 성공: {response.message}"
                self.service_log.setText(msg)
                self.status_signal.emit(f"🔇 음성 인식 비활성화 성공")
            else:
                msg = f"❌ 음성 인식 비활성화 실패: {response.message}"
                self.service_log.setText(msg)
                self.status_signal.emit(msg)
                
        except Exception as e:
            error_msg = f"❌ 음성 인식 비활성화 요청 중 오류: {str(e)}"
            self.service_log.setText(error_msg)
            self.status_signal.emit(error_msg)
    
    def test_end_task(self):
        """작업 종료 서비스 클라이언트 테스트"""
        robot_id = self.id_input.text().strip() or "libo_a"
        task_type = self.task_type_input.text().strip() or "assist"
        
        if not self.ros2_node:
            self.status_signal.emit("❌ ROS2 노드가 초기화되지 않았습니다")
            return
            
        # 작업 종료 서비스 클라이언트 호출
        try:
            self.service_log.setText(f"🔄 작업 종료 요청 중... (robot_id: {robot_id}, task_type: {task_type})")
            success = self.ros2_node.call_end_task(robot_id, task_type, self.end_task_callback)
            
            if success:
                self.status_signal.emit(f"🛑 작업 종료 요청 전송됨: {robot_id}, {task_type}")
            else:
                self.status_signal.emit(f"❌ 작업 종료 요청 실패: {robot_id}, {task_type}")
                self.service_log.setText(f"❌ 작업 종료 요청 실패: 서비스 사용 불가")
                
        except Exception as e:
            error_msg = f"❌ 작업 종료 요청 중 오류: {str(e)}"
            self.service_log.setText(error_msg)
            self.status_signal.emit(error_msg)
            
    def end_task_callback(self, future):
        """작업 종료 서비스 응답 처리"""
        try:
            response = future.result()
            if response.success:
                msg = f"✅ 작업 종료 성공: {response.message}"
                self.service_log.setText(msg)
                self.status_signal.emit(f"✅ 작업 종료 성공")
            else:
                msg = f"❌ 작업 종료 실패: {response.message}"
                self.service_log.setText(msg)
                self.status_signal.emit(msg)
        except Exception as e:
            error_msg = f"❌ 서비스 응답 처리 중 오류: {str(e)}"
            self.service_log.setText(error_msg)
            self.status_signal.emit(error_msg)
            
    def update_status(self, message):
        """상태 표시줄 업데이트"""
        self.status_bar.showMessage(f"{get_kr_time()} - {message}")
        
    def closeEvent(self, event):
        """애플리케이션 종료 시 정리"""
        print("🚪 종료 중...")
        
        # ROS2 종료 이벤트 설정
        if hasattr(self, 'ros2_shutdown'):
            self.ros2_shutdown.set()
        
        # 스레드가 끝날 때까지 최대 2초간 대기
        if hasattr(self, 'ros2_thread') and self.ros2_thread.is_alive():
            self.ros2_thread.join(timeout=2.0)
            
        # ROS2 종료
        try:
            rclpy.shutdown()
            print("🛑 ROS2 셧다운 완료")
        except Exception as e:
            print(f"❌ ROS2 셧다운 중 오류: {e}")
            
        print("✅ 프로그램이 안전하게 종료되었습니다")
        event.accept()


def main():
    # Ctrl+C 시그널 핸들러 설정
    import signal
    
    app = QApplication(sys.argv)
    window = VoiceCommandMaker()
    window.show()
    
    # Ctrl+C 핸들러 등록
    def signal_handler(sig, frame):
        print("🛑 Ctrl+C 감지됨, 프로그램을 종료합니다...")
        window.close()
        app.quit()
    
    # SIGINT 시그널(Ctrl+C) 핸들러 등록
    signal.signal(signal.SIGINT, signal_handler)
    
    # 타이머를 사용해 시그널 처리를 할 수 있도록 주기적 이벤트 발생
    timer = QTimer()
    timer.timeout.connect(lambda: None)  # 더미 함수
    timer.start(100)  # 100ms마다 타이머 이벤트 발생
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 프로그램이 Ctrl+C로 종료되었습니다.")
    except Exception as e:
        print(f"❌ 예외 발생: {e}")
    finally:
        # 마지막 정리 - ROS2가 여전히 실행 중인 경우를 대비
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
        print("👋 프로그램 종료됨")
