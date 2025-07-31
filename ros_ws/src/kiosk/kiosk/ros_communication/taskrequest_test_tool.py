#!/usr/bin/env python3

import sys  # 시스템 관련 기능 사용 (프로그램 종료 등)
import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS2 노드 클래스
from libo_interfaces.srv import TaskRequest  # 우리가 만든 TaskRequest 서비스
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit  # PyQt5 GUI 위젯들
from PyQt5.QtCore import QThread, pyqtSignal  # 스레드와 시그널 기능
import time  # 시간 관련 기능

class TaskRequestClient(Node):  # TaskRequest 서비스 클라이언트
    def __init__(self):  # 클라이언트 초기화
        super().__init__('task_request_client')  # 부모 클래스 초기화하고 노드 이름 설정
        
        # TaskRequest 서비스 클라이언트 생성 ('/task_request' 서비스에 연결)
        self.client = self.create_client(TaskRequest, '/task_request')
        
        # 서비스가 준비될 때까지 대기 (1초마다 체크)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('🔄 task_request 서비스 대기 중...')  # 서비스가 아직 준비되지 않았음을 알림
        
        self.get_logger().info('✅ TaskRequestClient 준비 완료!')  # 클라이언트 준비 완료 알림
    
    def send_request(self, robot_id, task_type, call_location, goal_location):  # 요청 전송
        """작업 요청을 전송하고 응답을 받아옴"""
        try:
            # 요청 데이터 생성 (TaskRequest.Request 객체 생성)
            request = TaskRequest.Request()
            request.robot_id = robot_id  # 로봇 ID 설정
            request.task_type = task_type  # 작업 타입 설정
            request.call_location = call_location  # 호출지 위치 설정
            request.goal_location = goal_location  # 목적지 위치 설정
            
            self.get_logger().info(f'🚀 요청 전송: robot_id={robot_id}, task_type={task_type}, call_location={call_location}, goal_location={goal_location}')  # 전송 정보 로그
            
            # 서비스 호출 (비동기로 요청 전송)
            future = self.client.call_async(request)
            
            # 응답 대기 (최대 10초까지 대기)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():  # 요청이 완료되었는지 확인
                response = future.result()  # 응답 결과 가져오기
                self.get_logger().info(f'📡 응답 받음: success={response.success}, message={response.message}')  # 응답 정보 로그
                return response  # 응답 반환
            else:
                self.get_logger().error('❌ 요청 타임아웃')  # 타임아웃 에러 로그
                return None  # 실패 시 None 반환
                
        except Exception as e:  # 예외 발생 시 처리
            self.get_logger().error(f'❌ 요청 중 오류: {e}')  # 에러 로그
            return None  # 실패 시 None 반환

class RequestWorker(QThread):  # 백그라운드 요청 처리 스레드
    response_received = pyqtSignal(bool, str)  # 응답 시그널 정의 (성공여부, 메시지)
    
    def __init__(self, client, robot_id, task_type, call_location, goal_location):  # 스레드 초기화
        super().__init__()  # 부모 클래스 초기화
        self.client = client  # ROS2 클라이언트 저장
        self.robot_id = robot_id  # 로봇 ID 저장
        self.task_type = task_type  # 작업 타입 저장
        self.call_location = call_location  # 호출지 위치 저장
        self.goal_location = goal_location  # 목적지 위치 저장

    def run(self):  # 스레드 실행
        response = self.client.send_request(self.robot_id, self.task_type, self.call_location, self.goal_location)  # 요청 전송
        if response:  # 응답이 있으면
            self.response_received.emit(response.success, response.message)  # 성공 시그널 발생
        else:
            self.response_received.emit(False, "요청 실패")  # 실패 시그널 발생

class TaskRequestTestGUI(QMainWindow):  # 메인 GUI 윈도우
    def __init__(self):  # GUI 초기화
        super().__init__()  # 부모 클래스 초기화
        self.client = None  # ROS2 클라이언트 초기값
        self.init_ros()  # ROS2 초기화
        self.init_ui()  # UI 구성
    
    def init_ros(self):  # ROS2 초기화
        try:
            rclpy.init()  # ROS2 초기화
            self.client = TaskRequestClient()  # 클라이언트 생성
        except Exception as e:  # 예외 발생 시
            print(f"❌ ROS2 초기화 실패: {e}")  # 에러 메시지 출력
    
    def init_ui(self):  # UI 구성
        self.setWindowTitle('🎯 TaskRequest 테스트 툴')  # 윈도우 제목 설정
        self.setGeometry(100, 100, 400, 500)  # 윈도우 위치와 크기 설정 (x=100, y=100, width=400, height=500)
        
        # 중앙 위젯 (메인 컨테이너)
        central_widget = QWidget()  # 중앙 위젯 생성
        self.setCentralWidget(central_widget)  # 메인 윈도우의 중앙 위젯으로 설정
        
        # 메인 레이아웃 (세로 배치)
        layout = QVBoxLayout()  # 세로 레이아웃 생성
        central_widget.setLayout(layout)  # 중앙 위젯에 레이아웃 설정
        
        # 입력 필드들
        self.robot_id_edit = QLineEdit()  # 로봇 ID 입력란 생성
        self.robot_id_edit.setPlaceholderText('예: libo_a')  # 힌트 텍스트 설정
        layout.addWidget(QLabel('로봇 ID:'))  # 라벨 추가
        layout.addWidget(self.robot_id_edit)  # 입력란 추가
        
        self.task_type_edit = QLineEdit()  # 작업 타입 입력란 생성
        self.task_type_edit.setPlaceholderText('예: escort, assist, delivery')  # 힌트 텍스트 설정
        layout.addWidget(QLabel('작업 타입:'))  # 라벨 추가
        layout.addWidget(self.task_type_edit)  # 입력란 추가
        
        self.call_location_edit = QLineEdit()  # 호출지 위치 입력란 생성
        self.call_location_edit.setPlaceholderText('예: A2, B1, C3')  # 힌트 텍스트 설정
        layout.addWidget(QLabel('호출지 위치:'))  # 라벨 추가
        layout.addWidget(self.call_location_edit)  # 입력란 추가
        
        self.goal_location_edit = QLineEdit()  # 목적지 위치 입력란 생성
        self.goal_location_edit.setPlaceholderText('예: D3, E2, A1')  # 힌트 텍스트 설정
        layout.addWidget(QLabel('목적지 위치:'))  # 라벨 추가
        layout.addWidget(self.goal_location_edit)  # 입력란 추가
        
        # 전송 버튼
        self.send_button = QPushButton('🚀 요청 전송')  # 전송 버튼 생성
        self.send_button.clicked.connect(self.send_request)  # 버튼 클릭 시 send_request 함수 호출
        layout.addWidget(self.send_button)  # 버튼 추가
        
        # 로그 출력 영역
        layout.addWidget(QLabel('로그:'))  # 로그 라벨 추가
        self.log_text = QTextEdit()  # 텍스트 편집 영역 생성 (로그 출력용)
        self.log_text.setMaximumHeight(200)  # 최대 높이 제한
        layout.addWidget(self.log_text)  # 로그 영역 추가
        
        # 기본값 설정
        self.robot_id_edit.setText('libo_a')  # 기본 로봇 ID 설정
        self.task_type_edit.setText('escort')  # 기본 작업 타입 설정
        self.call_location_edit.setText('A2')  # 기본 호출지 위치 설정
        self.goal_location_edit.setText('D3')  # 기본 목적지 위치 설정
        
        self.log_message('✅ TaskRequest 테스트 툴 시작됨')  # 시작 메시지 출력
    
    def send_request(self):  # 요청 전송 처리
        if not self.client:  # 클라이언트가 없으면
            self.log_message('❌ ROS2 클라이언트가 초기화되지 않았습니다.')  # 에러 메시지
            return
        
        # 입력값 가져오기
        robot_id = self.robot_id_edit.text().strip()  # 로봇 ID 가져오기 (공백 제거)
        task_type = self.task_type_edit.text().strip()  # 작업 타입 가져오기 (공백 제거)
        call_location = self.call_location_edit.text().strip()  # 호출지 위치 가져오기 (공백 제거)
        goal_location = self.goal_location_edit.text().strip()  # 목적지 위치 가져오기 (공백 제거)
        
        # 입력값 검증 (모든 필드가 비어있지 않은지 확인)
        if not all([robot_id, task_type, call_location, goal_location]):
            self.log_message('❌ 모든 필드를 입력해주세요.')  # 에러 메시지
            return
        
        self.log_message(f'📤 요청 전송 중... robot_id={robot_id}, task_type={task_type}')  # 전송 시작 메시지
        
        # 전송 버튼 비활성화 (중복 클릭 방지)
        self.send_button.setEnabled(False)  # 버튼 비활성화
        self.send_button.setText('⏳ 전송 중...')  # 버튼 텍스트 변경
        
        # 백그라운드에서 요청 처리 (UI가 멈추지 않도록)
        self.worker = RequestWorker(self.client, robot_id, task_type, call_location, goal_location)  # 작업 스레드 생성
        self.worker.response_received.connect(self.handle_response)  # 응답 시그널 연결
        self.worker.start()  # 스레드 시작
    
    def handle_response(self, success, message):  # 응답 처리
        if success:  # 성공이면
            self.log_message(f'✅ 응답: {message}')  # 성공 메시지 출력
        else:
            self.log_message(f'❌ 응답: {message}')  # 실패 메시지 출력
        
        # 전송 버튼 재활성화
        self.send_button.setEnabled(True)  # 버튼 활성화
        self.send_button.setText('🚀 요청 전송')  # 버튼 텍스트 복원
    
    def log_message(self, message):  # 로그 메시지 출력
        self.log_text.append(f'[{time.strftime("%H:%M:%S")}] {message}')  # 현재 시간과 함께 메시지 추가
        self.log_text.ensureCursorVisible()  # 커서가 보이도록 스크롤
    
    def closeEvent(self, event):  # 윈도우 종료 처리
        if self.client:  # 클라이언트가 있으면
            self.client.destroy_node()  # 노드 정리
        rclpy.shutdown()  # ROS2 종료
        event.accept()  # 종료 이벤트 수락

def main():  # 메인 함수
    app = QApplication(sys.argv)  # PyQt5 애플리케이션 생성
    window = TaskRequestTestGUI()  # GUI 윈도우 생성
    window.show()  # 윈도우 표시
    sys.exit(app.exec_())  # 애플리케이션 실행 (종료 시 프로그램 종료)

if __name__ == '__main__':  # 이 파일이 직접 실행될 때만
    main()  # 메인 함수 호출 