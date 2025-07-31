#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit
from PyQt5.QtCore import QThread, pyqtSignal
import time
import random
from libo_interfaces.srv import TaskRequest

class TaskRequestClientNode(Node): # ROS2 서비스 클라이언트 역할을 하는 별도의 노드 클래스
    def __init__(self):
        super().__init__('task_request_client_node', automatically_declare_parameters_from_overrides=True)
        self.client = self.create_client(TaskRequest, '/task_request') # TaskRequest 서비스 클라이언트를 생성
        self.get_logger().info('TaskRequestClientNode가 준비되었습니다.')

    def send_request(self, robot_id, task_type, call_location, goal_location):
        if not self.client.wait_for_service(timeout_sec=1.0): # 서비스가 1초 내에 사용 가능한지 확인
            self.get_logger().error('서비스가 사용 가능하지 않습니다. 요청을 보낼 수 없습니다.')
            return None # 사용 불가능하면 None을 반환

        request = TaskRequest.Request() # 서비스 요청 객체를 생성
        request.robot_id = robot_id
        request.task_type = task_type
        request.call_location = call_location
        request.goal_location = goal_location

        future = self.client.call_async(request) # 비동기적으로 서비스 요청을 보냄
        return future # 요청의 결과를 담고 있는 future 객체를 반환

class RequestWorker(QThread): # GUI가 멈추지 않도록 백그라운드에서 ROS2 요청을 처리하는 스레드
    response_received = pyqtSignal(object) # 응답을 전달하는 시그널

    def __init__(self, client_node, robot_id, task_type, call_location, goal_location):
        super().__init__()
        self.client_node = client_node
        self.robot_id = robot_id
        self.task_type = task_type
        self.call_location = call_location
        self.goal_location = goal_location

    def run(self): # 스레드가 시작될 때 실행되는 함수
        future = self.client_node.send_request(self.robot_id, self.task_type, self.call_location, self.goal_location) # 서비스 요청
        if future:
            while rclpy.ok() and not future.done(): # future가 완료될 때까지 기다림
                time.sleep(0.1) # CPU 사용을 줄이기 위해 0.1초 대기
            
            if future.done(): # future가 완료되면
                self.response_received.emit(future.result()) # 결과와 함께 시그널을 발생시킴
        else:
            self.response_received.emit(None) # future가 없으면 None과 함께 시그널 발생

class TaskRequestTab(QWidget): # QMainWindow 대신 QWidget을 상속받음. TaskRequest 테스트 툴의 모든 UI와 기능을 담는 탭 클래스
    def __init__(self, ros_node, parent=None): # 메인 앱의 ROS 노드를 받아옴
        super().__init__(parent)
        self.ros_node = ros_node # 전달받은 노드를 저장
        self.client_node = TaskRequestClientNode() # 통신을 위한 자체 클라이언트 노드 생성
        self.init_ui() # UI를 초기화하는 함수 호출

    def init_ui(self):
        # 1. 전체 탭의 메인 레이아웃을 가로(QHBoxLayout)로 설정
        #    - 내용(1/3)과 빈 공간(2/3)으로 나누기 위함
        main_layout = QHBoxLayout(self)

        # 2. 모든 컨트롤을 담을 컨테이너 위젯 생성
        #    - 이 컨테이너가 왼쪽 1/3 공간을 차지하게 됨
        content_container = QWidget()
        content_layout = QVBoxLayout(content_container) # 컨테이너는 세로(QVBoxLayout)로 위젯을 쌓음

        # 3. 입력 컨트롤들을 담을 별도의 그룹 위젯 생성
        #    - 세로 비율(1/4)을 조절하기 위함
        input_group = QWidget()
        input_layout = QVBoxLayout(input_group)

        # 기존의 입력 UI 요소들을 input_layout에 추가
        self.robot_id_edit = QLineEdit('libo_a')
        input_layout.addWidget(QLabel('로봇 ID:'))
        input_layout.addWidget(self.robot_id_edit)

        self.task_type_edit = QLineEdit('escort')
        input_layout.addWidget(QLabel('작업 타입:'))
        input_layout.addWidget(self.task_type_edit)
        
        self.call_location_edit = QLineEdit('A2')
        input_layout.addWidget(QLabel('호출지 위치:'))
        input_layout.addWidget(self.call_location_edit)

        self.goal_location_edit = QLineEdit('D3')
        input_layout.addWidget(QLabel('목적지 위치:'))
        input_layout.addWidget(self.goal_location_edit)

        button_layout = QHBoxLayout()
        self.random_button = QPushButton('🎲 랜덤 설정')
        self.random_button.clicked.connect(self.set_random_values)
        button_layout.addWidget(self.random_button)

        self.send_button = QPushButton('🚀 요청 전송')
        self.send_button.clicked.connect(self.send_request)
        button_layout.addWidget(self.send_button)
        input_layout.addLayout(button_layout)

        # 4. 로그 UI 요소 생성
        log_label = QLabel('로그:')
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)

        # 5. 위젯들을 비율에 맞게 조립
        #    - content_layout에 입력 그룹(비율 1)과 로그(비율 3)를 추가
        content_layout.addWidget(input_group, 1) # 입력 그룹이 차지할 세로 비율
        content_layout.addWidget(log_label)
        content_layout.addWidget(self.log_text, 3) # 로그 창이 차지할 세로 비율

        # 6. 메인 레이아웃에 컨테이너(비율 1)와 빈 공간(비율 2)을 추가
        main_layout.addWidget(content_container, 1) # 내용 컨테이너가 차지할 가로 비율
        main_layout.addStretch(2) # 비어있는 공간이 차지할 가로 비율

        self.log_message('✅ TaskRequest 테스트 탭 준비 완료!')

    def set_random_values(self):
        robot_ids = ['libo_a', 'libo_b', 'libo_c'] # 랜덤 선택을 위한 로봇 ID 목록
        task_types = ['escort', 'assist', 'delivery'] # 랜덤 선택을 위한 작업 타입 목록
        locations = ['A1', 'A2', 'A3', 'B2', 'B3', 'C1', 'C3', 'D3', 'E3'] # 랜덤 선택을 위한 위치 목록
        
        self.robot_id_edit.setText(random.choice(robot_ids)) # 로봇 ID를 랜덤으로 선택하여 설정
        self.task_type_edit.setText(random.choice(task_types)) # 작업 타입을 랜덤으로 선택하여 설정
        
        call_loc = random.choice(locations) # 호출지를 랜덤으로 선택
        goal_loc = random.choice(locations) # 목적지를 랜덤으로 선택
        while goal_loc == call_loc: # 호출지와 목적지가 같으면
            goal_loc = random.choice(locations) # 목적지를 다시 선택
        
        self.call_location_edit.setText(call_loc) # 랜덤 호출지를 설정
        self.goal_location_edit.setText(goal_loc) # 랜덤 목적지를 설정
        
        self.log_message('🎲 랜덤 값으로 설정 완료!') # 랜덤 설정 완료 로그 출력

    def send_request(self):
        robot_id = self.robot_id_edit.text().strip() # 로봇 ID 입력 필드에서 텍스트를 가져오고 양쪽 공백 제거
        task_type = self.task_type_edit.text().strip() # 작업 타입 입력 필드에서 텍스트를 가져오고 양쪽 공백 제거
        call_location = self.call_location_edit.text().strip() # 호출지 위치 입력 필드에서 텍스트를 가져오고 양쪽 공백 제거
        goal_location = self.goal_location_edit.text().strip() # 목적지 위치 입력 필드에서 텍스트를 가져오고 양쪽 공백 제거

        if not all([robot_id, task_type, call_location, goal_location]): # 모든 필드가 채워져 있는지 확인
            self.log_message('❌ 모든 필드를 입력해주세요.') # 비어있는 필드가 있으면 로그 출력
            return # 함수 실행 중단

        self.log_message(f'📤 요청 전송 중... {robot_id}, {task_type}') # 전송 시작 로그 출력
        self.send_button.setEnabled(False) # 중복 전송을 막기 위해 버튼을 비활성화
        self.send_button.setText('⏳ 전송 중...') # 버튼 텍스트를 '전송 중'으로 변경

        self.worker = RequestWorker(self.client_node, robot_id, task_type, call_location, goal_location) # 백그라운드 작업을 위한 워커 스레드 생성
        self.worker.response_received.connect(self.handle_response) # 워커의 응답 시그널을 handle_response 함수에 연결
        self.worker.start() # 워커 스레드 시작

    def handle_response(self, response):
        if response: # 응답이 성공적으로 오면
            self.log_message(f'✅ 응답: {response.message}') # 성공 메시지를 로그에 출력
        else: # 응답이 없거나 실패하면
            self.log_message('❌ 응답 수신 실패 또는 타임아웃') # 실패 메시지를 로그에 출력
        
        self.send_button.setEnabled(True) # 버튼을 다시 활성화
        self.send_button.setText('🚀 요청 전송') # 버튼 텍스트를 원래대로 복원

    def log_message(self, message):
        self.log_text.append(f'[{time.strftime("%H:%M:%S")}] {message}') # 현재 시간과 함께 로그 메시지를 추가
        self.log_text.ensureCursorVisible() # 스크롤을 맨 아래로 내려서 항상 최신 로그가 보이게 함

    def shutdown(self): # 이 탭이 닫힐 때 노드를 정리하기 위한 함수
        self.log_message("TaskRequest 탭 종료 중...")
        self.client_node.destroy_node() # 이 탭에서 사용한 ROS 노드를 정리 