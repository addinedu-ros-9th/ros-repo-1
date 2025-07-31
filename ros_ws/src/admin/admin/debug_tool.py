#!/usr/bin/env python3

import sys  # 시스템 관련 기능
import json  # JSON 파일 읽기
import time  # 시간 관련 기능
import hashlib  # 파일 해시 계산용
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem, QLabel, QTabWidget  # PyQt5 GUI 위젯들
from PyQt5.QtCore import QTimer  # 타이머 기능

class TaskManagerDebugTool(QMainWindow):  # TaskManager 디버깅 툴
    def __init__(self):  # GUI 초기화
        super().__init__()  # 부모 클래스 초기화
        self.tasks_file = "/tmp/current_tasks.json"  # 작업 목록 파일 경로
        self.heartbeat_log_file = "/tmp/heartbeat_log.json"  # Heartbeat 로그 파일 경로
        
        # 파일 해시 저장 (변경사항 체크용)
        self.last_tasks_hash = None  # 마지막 작업 목록 해시
        self.last_heartbeat_hash = None  # 마지막 Heartbeat 로그 해시
        
        self.init_ui()  # UI 구성
        self.setup_timer()  # 타이머 설정
    
    def init_ui(self):  # UI 구성
        self.setWindowTitle('🎯 TaskManager Debug Tool')  # 윈도우 제목 설정
        self.setGeometry(100, 100, 1000, 600)  # 윈도우 크기 설정
        
        # 중앙 위젯
        central_widget = QWidget()  # 중앙 위젯 생성
        self.setCentralWidget(central_widget)  # 메인 윈도우에 설정
        
        # 메인 레이아웃
        layout = QVBoxLayout()  # 세로 레이아웃 생성
        central_widget.setLayout(layout)  # 레이아웃 설정
        
        # 제목 라벨
        title_label = QLabel('🎯 TaskManager - 현재 작업 목록 & Heartbeat 로그 (실시간 업데이트)')  # 제목 라벨 생성
        layout.addWidget(title_label)  # 제목 추가
        
        # 탭 위젯 생성
        self.tab_widget = QTabWidget()  # 탭 위젯 생성
        layout.addWidget(self.tab_widget)  # 탭 위젯 추가
        
        # 작업 목록 탭
        self.create_tasks_tab()  # 작업 목록 탭 생성
        
        # Heartbeat 로그 탭
        self.create_heartbeat_tab()  # Heartbeat 로그 탭 생성
        
        # 초기 데이터 로드
        self.load_tasks()  # 작업 목록 로드
        self.load_heartbeat_logs()  # Heartbeat 로그 로드
    
    def create_tasks_tab(self):  # 작업 목록 탭 생성
        """작업 목록을 표시하는 탭 생성"""
        tasks_widget = QWidget()  # 작업 목록 위젯 생성
        
        # 레이아웃
        layout = QVBoxLayout()  # 세로 레이아웃 생성
        tasks_widget.setLayout(layout)  # 레이아웃 설정
        
        # 제목 라벨
        title_label = QLabel('📋 현재 작업 목록')  # 제목 라벨 생성
        layout.addWidget(title_label)  # 제목 추가
        
        # 테이블 위젯 (작업 목록 표시용)
        self.tasks_table = QTableWidget()  # 테이블 위젯 생성
        self.tasks_table.setColumnCount(7)  # 7개 컬럼 설정
        self.tasks_table.setHorizontalHeaderLabels(['Task ID', 'Robot ID', 'Task Type', 'Call Location', 'Goal Location', 'Start Time', 'Status'])  # 컬럼 헤더 설정
        layout.addWidget(self.tasks_table)  # 테이블 추가
        
        # 탭에 추가
        self.tab_widget.addTab(tasks_widget, "📋 작업 목록")  # 탭 추가
    
    def create_heartbeat_tab(self):  # Heartbeat 로그 탭 생성
        """Heartbeat 로그를 표시하는 탭 생성"""
        heartbeat_widget = QWidget()  # Heartbeat 로그 위젯 생성
        
        # 레이아웃
        layout = QVBoxLayout()  # 세로 레이아웃 생성
        heartbeat_widget.setLayout(layout)  # 레이아웃 설정
        
        # 제목 라벨
        title_label = QLabel('💓 Heartbeat 로그')  # 제목 라벨 생성
        layout.addWidget(title_label)  # 제목 추가
        
        # 테이블 위젯 (Heartbeat 로그 표시용)
        self.heartbeat_table = QTableWidget()  # 테이블 위젯 생성
        self.heartbeat_table.setColumnCount(4)  # 4개 컬럼 설정
        self.heartbeat_table.setHorizontalHeaderLabels(['Sender ID', 'Received Time', 'Timestamp', 'Age (초)'])  # 컬럼 헤더 설정
        layout.addWidget(self.heartbeat_table)  # 테이블 추가
        
        # 탭에 추가
        self.tab_widget.addTab(heartbeat_widget, "💓 Heartbeat 로그")  # 탭 추가
    
    def setup_timer(self):  # 타이머 설정
        """1초마다 파일을 다시 읽어서 테이블 업데이트"""
        self.timer = QTimer()  # 타이머 생성
        self.timer.timeout.connect(self.update_all_data)  # 타이머 만료 시 update_all_data 함수 호출
        self.timer.start(1000)  # 1초(1000ms)마다 실행
    
    def update_all_data(self):  # 모든 데이터 업데이트
        """작업 목록과 Heartbeat 로그를 모두 업데이트"""
        self.load_tasks()  # 작업 목록 로드
        self.load_heartbeat_logs()  # Heartbeat 로그 로드
    
    def calculate_file_hash(self, file_path):  # 파일 해시 계산
        """파일의 MD5 해시를 계산"""
        try:
            with open(file_path, 'rb') as f:  # 바이너리 모드로 파일 읽기
                file_content = f.read()  # 파일 내용 읽기
                return hashlib.md5(file_content).hexdigest()  # MD5 해시 계산
        except FileNotFoundError:  # 파일이 없으면
            return None  # None 반환
    
    def load_tasks(self):  # 작업 목록 로드
        """JSON 파일에서 작업 목록을 읽어서 테이블에 표시"""
        try:
            # 파일 해시 체크
            current_hash = self.calculate_file_hash(self.tasks_file)  # 현재 파일 해시 계산
            if current_hash == self.last_tasks_hash:  # 해시가 같으면 (변경사항 없음)
                return  # 업데이트하지 않음
            
            with open(self.tasks_file, 'r') as f:  # 파일 읽기 모드로 열기
                tasks_data = json.load(f)  # JSON 데이터 읽기
            
            # 테이블 행 수 설정
            self.tasks_table.setRowCount(len(tasks_data))  # 작업 개수만큼 행 생성
            
            # 각 작업을 테이블에 추가
            for row, task in enumerate(tasks_data):  # 각 작업에 대해
                self.tasks_table.setItem(row, 0, QTableWidgetItem(task['task_id']))  # Task ID
                self.tasks_table.setItem(row, 1, QTableWidgetItem(task['robot_id']))  # Robot ID
                self.tasks_table.setItem(row, 2, QTableWidgetItem(task['task_type']))  # Task Type
                self.tasks_table.setItem(row, 3, QTableWidgetItem(task['call_location']))  # Call Location
                self.tasks_table.setItem(row, 4, QTableWidgetItem(task['goal_location']))  # Goal Location
                
                # 시작 시간을 읽기 쉬운 형태로 변환
                start_time = time.strftime('%H:%M:%S', time.localtime(task['start_time']))  # 시간 포맷 변환
                self.tasks_table.setItem(row, 5, QTableWidgetItem(start_time))  # Start Time
                
                self.tasks_table.setItem(row, 6, QTableWidgetItem(task['status']))  # Status
            
            # 테이블 크기 자동 조정
            self.tasks_table.resizeColumnsToContents()  # 컬럼 너비 자동 조정
            
            # 해시 업데이트
            self.last_tasks_hash = current_hash  # 해시 저장
            
        except FileNotFoundError:  # 파일이 없으면
            self.tasks_table.setRowCount(0)  # 테이블 비우기
        except Exception as e:  # 기타 에러
            print(f"❌ 작업 목록 로드 실패: {e}")  # 에러 메시지 출력
    
    def load_heartbeat_logs(self):  # Heartbeat 로그 로드
        """JSON 파일에서 Heartbeat 로그를 읽어서 테이블에 표시"""
        try:
            # 파일 해시 체크
            current_hash = self.calculate_file_hash(self.heartbeat_log_file)  # 현재 파일 해시 계산
            if current_hash == self.last_heartbeat_hash:  # 해시가 같으면 (변경사항 없음)
                return  # 업데이트하지 않음
            
            with open(self.heartbeat_log_file, 'r') as f:  # 파일 읽기 모드로 열기
                heartbeat_data = json.load(f)  # JSON 데이터 읽기
            
            # 테이블 행 수 설정
            self.heartbeat_table.setRowCount(len(heartbeat_data))  # 로그 개수만큼 행 생성
            
            # 각 Heartbeat 로그를 테이블에 추가 (최신 순으로 표시)
            for row, heartbeat in enumerate(reversed(heartbeat_data)):  # 최신 순으로 표시
                self.heartbeat_table.setItem(row, 0, QTableWidgetItem(heartbeat['sender_id']))  # Sender ID
                self.heartbeat_table.setItem(row, 1, QTableWidgetItem(heartbeat['received_time_str']))  # Received Time
                
                # 타임스탬프를 읽기 쉬운 형태로 변환 (수신 시간 사용)
                timestamp_str = heartbeat['received_time_str']  # 수신 시간 문자열 사용
                self.heartbeat_table.setItem(row, 2, QTableWidgetItem(timestamp_str))  # Timestamp
                
                # 경과 시간 계산 (현재 시간 - 수신 시간)
                age = time.time() - heartbeat['received_time']  # 경과 시간 계산
                age_str = f"{age:.1f}"  # 소수점 1자리까지 표시
                self.heartbeat_table.setItem(row, 3, QTableWidgetItem(age_str))  # Age
            
            # 테이블 크기 자동 조정
            self.heartbeat_table.resizeColumnsToContents()  # 컬럼 너비 자동 조정
            
            # 해시 업데이트
            self.last_heartbeat_hash = current_hash  # 해시 저장
            
        except FileNotFoundError:  # 파일이 없으면
            print(f"❌ Heartbeat 로그 파일 없음: {self.heartbeat_log_file}")  # 디버깅용 출력
            self.heartbeat_table.setRowCount(0)  # 테이블 비우기
        except Exception as e:  # 기타 에러
            print(f"❌ Heartbeat 로그 로드 실패: {e}")  # 에러 메시지 출력
            self.heartbeat_table.setRowCount(0)  # 테이블 비우기

def main():  # 메인 함수
    app = QApplication(sys.argv)  # PyQt5 애플리케이션 생성
    window = TaskManagerDebugTool()  # 디버깅 툴 윈도우 생성
    window.show()  # 윈도우 표시
    sys.exit(app.exec_())  # 애플리케이션 실행

if __name__ == '__main__':  # 이 파일이 직접 실행될 때만
    main()  # 메인 함수 호출