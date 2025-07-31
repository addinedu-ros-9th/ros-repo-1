#!/usr/bin/env python3

import sys  # 시스템 관련 기능
import json  # JSON 파일 읽기
import time  # 시간 관련 기능
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QLabel  # PyQt5 GUI 위젯들
from PyQt5.QtCore import QTimer  # 타이머 기능

class CurrentTaskDebugTool(QMainWindow):  # 현재 작업 디버깅 툴
    def __init__(self):  # GUI 초기화
        super().__init__()  # 부모 클래스 초기화
        self.tasks_file = "/tmp/current_tasks.json"  # 작업 목록 파일 경로
        self.init_ui()  # UI 구성
        self.setup_timer()  # 타이머 설정
    
    def init_ui(self):  # UI 구성
        self.setWindowTitle('📋 Current Tasks Debug Tool')  # 윈도우 제목 설정
        self.setGeometry(100, 100, 800, 400)  # 윈도우 크기 설정
        
        # 중앙 위젯
        central_widget = QWidget()  # 중앙 위젯 생성
        self.setCentralWidget(central_widget)  # 메인 윈도우에 설정
        
        # 레이아웃
        layout = QVBoxLayout()  # 세로 레이아웃 생성
        central_widget.setLayout(layout)  # 레이아웃 설정
        
        # 제목 라벨
        title_label = QLabel('🎯 현재 작업 목록 (실시간 업데이트)')  # 제목 라벨 생성
        layout.addWidget(title_label)  # 제목 추가
        
        # 테이블 위젯 (작업 목록 표시용)
        self.table = QTableWidget()  # 테이블 위젯 생성
        self.table.setColumnCount(7)  # 7개 컬럼 설정
        self.table.setHorizontalHeaderLabels(['Task ID', 'Robot ID', 'Task Type', 'Call Location', 'Goal Location', 'Start Time', 'Status'])  # 컬럼 헤더 설정
        layout.addWidget(self.table)  # 테이블 추가
        
        # 작업 목록 로드
        self.load_tasks()  # 작업 목록 로드
    
    def setup_timer(self):  # 타이머 설정
        """1초마다 파일을 다시 읽어서 테이블 업데이트"""
        self.timer = QTimer()  # 타이머 생성
        self.timer.timeout.connect(self.load_tasks)  # 타이머 만료 시 load_tasks 함수 호출
        self.timer.start(1000)  # 1초(1000ms)마다 실행
    
    def load_tasks(self):  # 작업 목록 로드
        """JSON 파일에서 작업 목록을 읽어서 테이블에 표시"""
        try:
            with open(self.tasks_file, 'r') as f:  # 파일 읽기 모드로 열기
                tasks_data = json.load(f)  # JSON 데이터 읽기
            
            # 테이블 행 수 설정
            self.table.setRowCount(len(tasks_data))  # 작업 개수만큼 행 생성
            
            # 각 작업을 테이블에 추가
            for row, task in enumerate(tasks_data):  # 각 작업에 대해
                self.table.setItem(row, 0, QTableWidgetItem(task['task_id']))  # Task ID
                self.table.setItem(row, 1, QTableWidgetItem(task['robot_id']))  # Robot ID
                self.table.setItem(row, 2, QTableWidgetItem(task['task_type']))  # Task Type
                self.table.setItem(row, 3, QTableWidgetItem(task['call_location']))  # Call Location
                self.table.setItem(row, 4, QTableWidgetItem(task['goal_location']))  # Goal Location
                
                # 시작 시간을 읽기 쉬운 형태로 변환
                start_time = time.strftime('%H:%M:%S', time.localtime(task['start_time']))  # 시간 포맷 변환
                self.table.setItem(row, 5, QTableWidgetItem(start_time))  # Start Time
                
                self.table.setItem(row, 6, QTableWidgetItem(task['status']))  # Status
            
            # 테이블 크기 자동 조정
            self.table.resizeColumnsToContents()  # 컬럼 너비 자동 조정
            
        except FileNotFoundError:  # 파일이 없으면
            self.table.setRowCount(0)  # 테이블 비우기
        except Exception as e:  # 기타 에러
            print(f"❌ 작업 목록 로드 실패: {e}")  # 에러 메시지 출력

def main():  # 메인 함수
    app = QApplication(sys.argv)  # PyQt5 애플리케이션 생성
    window = CurrentTaskDebugTool()  # 디버깅 툴 윈도우 생성
    window.show()  # 윈도우 표시
    sys.exit(app.exec_())  # 애플리케이션 실행

if __name__ == '__main__':  # 이 파일이 직접 실행될 때만
    main()  # 메인 함수 호출 