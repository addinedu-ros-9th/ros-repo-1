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
from rclpy.callback_groups import ReentrantCallbackGroup
from libo_interfaces.msg import VoiceCommand
from libo_interfaces.srv import ActivateTalker, DeactivateTalker, EndTask

from PyQt5.QtWidgets import (QApplication, QMainWindow, QTabWidget, QPushButton,
                            QVBoxLayout, QHBoxLayout, QWidget, QLabel,
                            QLineEdit, QGridLayout, QStatusBar, QGroupBox,
                            QScrollArea, QMessageBox, QComboBox)
from PyQt5.QtCore import Qt, pyqtSignal, QSize, QTimer
from PyQt5.QtGui import QFont, QIcon


# 한국 시간 출력 함수
def get_kr_time():
    kr_tz = pytz.timezone('Asia/Seoul')
    return datetime.now(kr_tz).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 밀리초 3자리까지 표시


class ROS2Publisher(Node):
    """VoiceCommand 메시지를 발행하고 서비스를 처리하는 ROS2 노드"""
    def __init__(self):
        super().__init__('test_node')
        
        # 콜백 그룹 생성 - 동시에 여러 콜백을 처리하기 위함
        self.callback_group = ReentrantCallbackGroup()
        
        # VoiceCommand 메시지 발행자
        self.publisher = self.create_publisher(
            VoiceCommand,
            '/voice_command',
            10
        )
        
        # ActivateTalker 서비스 클라이언트
        self.activate_talker_client = self.create_client(
            ActivateTalker, 
            '/activate_talker',
            callback_group=self.callback_group
        )
        
        # DeactivateTalker 서비스 클라이언트
        self.deactivate_talker_client = self.create_client(
            DeactivateTalker, 
            '/deactivate_talker',
            callback_group=self.callback_group
        )
        
        # EndTask 서비스 구현
        self.end_task_service = self.create_service(
            EndTask,
            '/end_task',
            self.end_task_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('테스트 노드 초기화 완료')
        print(f"[{get_kr_time()}][ROS2] 메시지 발행자 및 서비스 초기화 완료")
        print(f"[{get_kr_time()}][ROS2] EndTask 서비스 제공 중: /end_task")
    
    def publish_command(self, robot_id, category, action):
        """VoiceCommand 메시지 발행"""
        msg = VoiceCommand()
        msg.robot_id = robot_id
        msg.category = category
        msg.action = action
        
        self.publisher.publish(msg)
        self.get_logger().info(f'메시지 발행: robot_id={robot_id}, category={category}, action={action}')
        return True
    
    def call_activate_talker(self, robot_id):
        """ActivateTalker 서비스 호출"""
        if not self.activate_talker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('ActivateTalker 서비스를 사용할 수 없습니다.')
            return False, "서비스를 사용할 수 없습니다."
        
        request = ActivateTalker.Request()
        request.robot_id = robot_id
        
        future = self.activate_talker_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            return response.success, response.message
        else:
            self.get_logger().error('ActivateTalker 서비스 호출 시간 초과')
            return False, "서비스 호출 시간 초과"
    
    def call_deactivate_talker(self, robot_id):
        """DeactivateTalker 서비스 호출"""
        if not self.deactivate_talker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('DeactivateTalker 서비스를 사용할 수 없습니다.')
            return False, "서비스를 사용할 수 없습니다."
        
        request = DeactivateTalker.Request()
        request.robot_id = robot_id
        
        future = self.deactivate_talker_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            return response.success, response.message
        else:
            self.get_logger().error('DeactivateTalker 서비스 호출 시간 초과')
            return False, "서비스 호출 시간 초과"
    
    def end_task_callback(self, request, response):
        """EndTask 서비스 콜백"""
        robot_id = request.robot_id
        task_type = request.task_type
        
        print(f"[{get_kr_time()}][SERVICE] EndTask 요청 수신: robot_id={robot_id}, task_type={task_type}")
        
        # 여기서는 항상 성공으로 응답 (실제 환경에서는 작업 종료 로직 구현)
        response.success = True
        response.message = f"{robot_id} 로봇의 {task_type} 작업이 성공적으로 종료되었습니다."
        
        print(f"[{get_kr_time()}][SERVICE] EndTask 응답 반환: success=True")
        return response


class VoiceCommandMaker(QMainWindow):
    """음성 명령을 생성하고 발행하는 GUI 애플리케이션"""
    status_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS2 토커매니저 테스트 도구')
        self.setGeometry(100, 100, 900, 700)
        self.setMinimumSize(900, 700)
        
        # 스타일 설정
        self.setStyleSheet("""
            QPushButton { 
                font-weight: bold; 
                padding: 5px; 
            }
            QGroupBox { 
                font-weight: bold; 
                border: 1px solid #cccccc; 
                border-radius: 5px; 
                margin-top: 10px; 
                padding-top: 15px; 
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
        """)
        
        # ROS2 초기화는 별도 스레드에서 수행
        self.ros2_thread = None
        self.ros2_node = None
        self.init_ros2()
        
        # GUI 초기화
        self.init_ui()
        
        # 상태 시그널 연결
        self.status_signal.connect(self.update_status)
        
        # 서비스 상태 업데이트 타이머
        self.service_timer = QTimer()
        self.service_timer.timeout.connect(self.check_service_status)
        self.service_timer.start(5000)  # 5초마다 업데이트
        
    def init_ros2(self):
        """ROS2 노드 초기화 (별도 스레드에서 실행)"""
        self.ros2_shutdown = threading.Event()  # 종료 이벤트 추가
        
        def run_ros2():
            rclpy.init()
            self.ros2_node = ROS2Publisher()
            self.status_signal.emit("ROS2 노드 초기화 완료")
            
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
        
        # 토커매니저 제어 섹션 추가
        talker_control_box = QGroupBox("토커매니저 제어")
        talker_layout = QHBoxLayout(talker_control_box)
        
        self.activate_btn = QPushButton("토커매니저 활성화")
        self.activate_btn.setMinimumHeight(40)
        self.activate_btn.clicked.connect(self.activate_talker)
        self.activate_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        
        self.deactivate_btn = QPushButton("토커매니저 비활성화")
        self.deactivate_btn.setMinimumHeight(40)
        self.deactivate_btn.clicked.connect(self.deactivate_talker)
        self.deactivate_btn.setStyleSheet("background-color: #f44336; color: white;")
        
        talker_layout.addWidget(self.activate_btn)
        talker_layout.addWidget(self.deactivate_btn)
        
        # EndTask 제어 섹션 추가
        endtask_control_box = QGroupBox("작업 종료")
        endtask_layout = QHBoxLayout(endtask_control_box)
        
        self.task_type_combo = QComboBox()
        self.task_type_combo.addItem("assist", "assist")
        self.task_type_combo.addItem("delivery", "delivery")
        self.task_type_combo.addItem("escort", "escort")
        
        self.end_task_btn = QPushButton("작업 종료 정보")
        self.end_task_btn.setMinimumHeight(40)
        self.end_task_btn.clicked.connect(self.end_task)
        self.end_task_btn.setStyleSheet("background-color: #2196F3; color: white;")
        
        endtask_layout.addWidget(QLabel("작업 유형:"))
        endtask_layout.addWidget(self.task_type_combo)
        endtask_layout.addWidget(self.end_task_btn)
        
        # 서비스 상태 섹션
        service_status_box = QGroupBox("서비스 상태")
        service_status_layout = QVBoxLayout(service_status_box)
        self.service_status_label = QLabel("EndTask 서비스 제공 중\n토커매니저 상태: 알 수 없음")
        service_status_layout.addWidget(self.service_status_label)
        
        # 탭 위젯 생성
        self.tabs = QTabWidget()
        self.tabs.setTabPosition(QTabWidget.North)
        self.tabs.setMovable(True)
        
        # 카테고리별 탭 추가
        self.add_category_tab("common", "일반")
        self.add_category_tab("escort", "안내")
        self.add_category_tab("delivery", "배달")
        self.add_category_tab("assist", "어시스트")
        
        # 상태 표시줄
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("준비됨")
        
        # 레이아웃에 위젯 추가
        control_layout = QHBoxLayout()
        control_layout.addWidget(talker_control_box)
        control_layout.addWidget(endtask_control_box)
        
        main_layout.addLayout(id_layout)
        main_layout.addLayout(control_layout)
        main_layout.addWidget(service_status_box)
        main_layout.addWidget(self.tabs)
        
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
            self.status_signal.emit("ROS2 노드가 초기화되지 않았습니다")
            return
        
        # 메시지 발행
        try:
            self.ros2_node.publish_command(robot_id, category, action)
            self.status_signal.emit(f"명령 발행 성공: [{robot_id}] {category}/{action}")
            
            # 콘솔에도 출력
            print(f"[{get_kr_time()}][TEST] VoiceCommand 발행: robot_id={robot_id}, "
                  f"category={category}, action={action}")
            
        except Exception as e:
            self.status_signal.emit(f"명령 발행 실패: {str(e)}")
            print(f"[{get_kr_time()}][ERROR] 명령 발행 실패: {str(e)}")
    
    def activate_talker(self):
        """토커매니저 활성화"""
        robot_id = self.id_input.text().strip()
        if not robot_id:
            robot_id = "libo_a"  # 기본값
        
        if not self.ros2_node:
            self.status_signal.emit("ROS2 노드가 초기화되지 않았습니다")
            return
        
        # 버튼 비활성화
        self.activate_btn.setEnabled(False)
        
        try:
            success, message = self.ros2_node.call_activate_talker(robot_id)
            
            if success:
                self.status_signal.emit(f"토커매니저 활성화 성공: {message}")
                self.service_status_label.setText(f"EndTask 서비스 제공 중\n토커매니저 상태: 활성화됨 (로봇: {robot_id})")
                print(f"[{get_kr_time()}][CLIENT] ✅ ActivateTalker 호출 성공: {message}")
                
                # 성공 메시지 박스 표시
                QMessageBox.information(self, "성공", f"토커매니저가 활성화되었습니다.\n로봇: {robot_id}")
            else:
                self.status_signal.emit(f"토커매니저 활성화 실패: {message}")
                print(f"[{get_kr_time()}][CLIENT] ❌ ActivateTalker 호출 실패: {message}")
                
                # 실패 메시지 박스 표시
                QMessageBox.warning(self, "실패", f"토커매니저 활성화 실패:\n{message}")
                
        except Exception as e:
            self.status_signal.emit(f"토커매니저 활성화 중 오류 발생: {str(e)}")
            print(f"[{get_kr_time()}][ERROR] 토커매니저 활성화 오류: {str(e)}")
            
            # 오류 메시지 박스 표시
            QMessageBox.critical(self, "오류", f"토커매니저 활성화 중 오류 발생:\n{str(e)}")
        
        # 버튼 재활성화
        self.activate_btn.setEnabled(True)
    
    def deactivate_talker(self):
        """토커매니저 비활성화"""
        robot_id = self.id_input.text().strip()
        if not robot_id:
            robot_id = "libo_a"  # 기본값
        
        if not self.ros2_node:
            self.status_signal.emit("ROS2 노드가 초기화되지 않았습니다")
            return
        
        # 버튼 비활성화
        self.deactivate_btn.setEnabled(False)
        
        try:
            success, message = self.ros2_node.call_deactivate_talker(robot_id)
            
            if success:
                self.status_signal.emit(f"토커매니저 비활성화 성공: {message}")
                self.service_status_label.setText(f"EndTask 서비스 제공 중\n토커매니저 상태: 비활성화됨 (로봇: {robot_id})")
                print(f"[{get_kr_time()}][CLIENT] ✅ DeactivateTalker 호출 성공: {message}")
                
                # 성공 메시지 박스 표시
                QMessageBox.information(self, "성공", f"토커매니저가 비활성화되었습니다.\n로봇: {robot_id}")
            else:
                self.status_signal.emit(f"토커매니저 비활성화 실패: {message}")
                print(f"[{get_kr_time()}][CLIENT] ❌ DeactivateTalker 호출 실패: {message}")
                
                # 실패 메시지 박스 표시
                QMessageBox.warning(self, "실패", f"토커매니저 비활성화 실패:\n{message}")
                
        except Exception as e:
            self.status_signal.emit(f"토커매니저 비활성화 중 오류 발생: {str(e)}")
            print(f"[{get_kr_time()}][ERROR] 토커매니저 비활성화 오류: {str(e)}")
            
            # 오류 메시지 박스 표시
            QMessageBox.critical(self, "오류", f"토커매니저 비활성화 중 오류 발생:\n{str(e)}")
        
        # 버튼 재활성화
        self.deactivate_btn.setEnabled(True)
    
    def end_task(self):
        """작업 종료 - 직접 호출하지 않고 서비스만 제공함을 알림"""
        robot_id = self.id_input.text().strip()
        if not robot_id:
            robot_id = "libo_a"  # 기본값
        
        task_type = self.task_type_combo.currentData()
        
        # 메시지 표시
        QMessageBox.information(
            self, 
            "EndTask 서비스 정보", 
            f"이 애플리케이션은 EndTask 서비스를 제공합니다.\n\n"
            f"다른 노드에서 다음 정보로 호출 시 응답합니다:\n"
            f"- 로봇 ID: {robot_id}\n"
            f"- 작업 유형: {task_type}\n\n"
            f"직접 호출하는 기능은 제공하지 않습니다."
        )
        
        self.status_signal.emit(f"EndTask 서비스 설명 표시됨: robot_id={robot_id}, task_type={task_type}")
        print(f"[{get_kr_time()}][INFO] EndTask 서비스 정보 표시: robot_id={robot_id}, task_type={task_type}")
    
    def check_service_status(self):
        """서비스 가용성 확인"""
        if not self.ros2_node:
            return
            
        # ActivateTalker 서비스 확인
        activate_available = self.ros2_node.activate_talker_client.service_is_ready()
        self.activate_btn.setEnabled(activate_available)
        
        # DeactivateTalker 서비스 확인
        deactivate_available = self.ros2_node.deactivate_talker_client.service_is_ready()
        self.deactivate_btn.setEnabled(deactivate_available)
        
        # 상태 텍스트 업데이트
        status_text = "서비스 상태:\n"
        status_text += f"- ActivateTalker: {'사용 가능' if activate_available else '사용 불가'}\n"
        status_text += f"- DeactivateTalker: {'사용 가능' if deactivate_available else '사용 불가'}\n"
        status_text += f"- EndTask: 서비스 제공 중"
        
        self.service_status_label.setText(status_text)
        
    def update_status(self, message):
        """상태 표시줄 업데이트"""
        self.status_bar.showMessage(f"{get_kr_time()} - {message}")
        
    def closeEvent(self, event):
        """애플리케이션 종료 시 정리"""
        print(f"[{get_kr_time()}][SYSTEM] 종료 중...")
        
        # 타이머 중지
        if hasattr(self, 'service_timer'):
            self.service_timer.stop()
        
        # ROS2 종료 이벤트 설정
        if hasattr(self, 'ros2_shutdown'):
            self.ros2_shutdown.set()
        
        # 스레드가 끝날 때까지 최대 2초간 대기
        if hasattr(self, 'ros2_thread') and self.ros2_thread.is_alive():
            self.ros2_thread.join(timeout=2.0)
            
        # ROS2 종료
        try:
            rclpy.shutdown()
            print(f"[{get_kr_time()}][SYSTEM] ROS2 셧다운 완료")
        except Exception as e:
            print(f"[{get_kr_time()}][ERROR] ROS2 셧다운 중 오류: {e}")
            
        print(f"[{get_kr_time()}][SYSTEM] 프로그램이 안전하게 종료되었습니다")
        event.accept()


def main():
    # Ctrl+C 시그널 핸들러 설정
    import signal
    
    app = QApplication(sys.argv)
    window = VoiceCommandMaker()
    window.show()
    
    # Ctrl+C 핸들러 등록
    def signal_handler(sig, frame):
        print("Ctrl+C 감지됨, 프로그램을 종료합니다...")
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
        print("\n프로그램이 Ctrl+C로 종료되었습니다.")
    except Exception as e:
        print(f"예외 발생: {e}")
    finally:
        # 마지막 정리 - ROS2가 여전히 실행 중인 경우를 대비
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
        print("프로그램 종료됨")
