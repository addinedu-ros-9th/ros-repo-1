#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json # JSON 파일을 다루기 위한 모듈
import time # 시간 관련 기능을 사용하기 위한 모듈
import hashlib # 파일의 해시 값을 계산하기 위한 모듈
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QLabel # PyQt5 위젯들
from PyQt5.QtCore import QTimer # 일정 시간마다 반복 작업을 실행하기 위한 타이머

class HeartbeatMonitorTab(QWidget): # QWidget을 상속받아 하트비트 모니터링 탭을 정의
    def __init__(self, parent=None):
        super().__init__(parent) # 부모 클래스의 초기화 함수를 호출
        self.heartbeat_log_file = "/tmp/heartbeat_log.json" # 감시할 하트비트 로그 파일의 경로
        self.last_heartbeat_hash = None # 파일의 변경 여부를 확인하기 위해 마지막으로 읽은 파일의 해시 값을 저장할 변수
        self.init_ui() # UI를 초기화하는 함수 호출
        self.setup_timer() # 데이터를 주기적으로 업데이트하는 타이머를 설정하는 함수 호출

    def init_ui(self):
        layout = QVBoxLayout(self) # 위젯들을 수직으로 배치할 레이아웃 생성
        
        title_label = QLabel('💓 실시간 Heartbeat 로그') # 탭의 제목 라벨 생성
        layout.addWidget(title_label) # 레이아웃에 제목 라벨 추가

        self.heartbeat_table = QTableWidget() # 하트비트 로그를 표시할 테이블 위젯 생성
        self.heartbeat_table.setColumnCount(4) # 테이블에 4개의 열(column)을 설정
        self.heartbeat_table.setHorizontalHeaderLabels(['Sender ID', 'Received Time', 'Timestamp', 'Age (초)']) # 각 열의 제목을 설정
        layout.addWidget(self.heartbeat_table) # 레이아웃에 테이블 위젯 추가

    def setup_timer(self):
        self.timer = QTimer() # QTimer 객체 생성
        self.timer.timeout.connect(self.load_heartbeat_logs) # 타이머가 만료될 때마다 load_heartbeat_logs 함수가 호출되도록 연결
        self.timer.start(1000) # 1000ms (1초) 간격으로 타이머를 시작

    def calculate_file_hash(self, file_path): # 파일의 내용으로 MD5 해시 값을 계산하는 함수
        try:
            with open(file_path, 'rb') as f: # 파일을 바이너리 읽기 모드로 열기
                file_content = f.read() # 파일 전체 내용을 읽음
                return hashlib.md5(file_content).hexdigest() # 내용의 MD5 해시를 계산하여 16진수 문자열로 반환
        except FileNotFoundError: # 파일을 찾을 수 없을 경우
            return None # None을 반환

    def load_heartbeat_logs(self): # 하트비트 로그 파일을 읽어 테이블을 업데이트하는 함수
        try:
            current_hash = self.calculate_file_hash(self.heartbeat_log_file) # 현재 로그 파일의 해시 값을 계산
            if current_hash == self.last_heartbeat_hash: # 이전에 읽은 해시와 동일하다면 (파일 내용 변경 없음)
                return # 함수를 종료하여 불필요한 업데이트를 방지

            with open(self.heartbeat_log_file, 'r') as f: # 로그 파일을 읽기 모드로 열기
                heartbeat_data = json.load(f) # JSON 형식의 데이터를 파이썬 객체로 변환
            
            self.heartbeat_table.setRowCount(len(heartbeat_data)) # 데이터의 개수만큼 테이블의 행 수를 설정
            
            for row, heartbeat in enumerate(reversed(heartbeat_data)): # 최신 로그가 맨 위에 오도록 데이터를 뒤집어서 반복
                self.heartbeat_table.setItem(row, 0, QTableWidgetItem(heartbeat['sender_id'])) # 0번 열에 'sender_id'를 표시
                self.heartbeat_table.setItem(row, 1, QTableWidgetItem(heartbeat['received_time_str'])) # 1번 열에 수신 시간을 문자열로 표시
                
                timestamp_str = heartbeat.get('received_time_str', 'N/A') # 'received_time_str' 키가 없을 경우를 대비해 기본값 설정
                self.heartbeat_table.setItem(row, 2, QTableWidgetItem(timestamp_str)) # 2번 열에 타임스탬프를 표시
                
                age = time.time() - heartbeat['received_time'] # 현재 시간과 로그 기록 시간의 차이를 계산하여 경과 시간을 구함
                age_str = f"{age:.1f}" # 경과 시간을 소수점 첫째 자리까지의 문자열로 변환
                self.heartbeat_table.setItem(row, 3, QTableWidgetItem(age_str)) # 3번 열에 경과 시간을 표시
            
            self.heartbeat_table.resizeColumnsToContents() # 각 열의 너비를 내용물에 맞게 자동으로 조절
            self.last_heartbeat_hash = current_hash # 현재 파일의 해시 값을 마지막 해시로 저장
            
        except FileNotFoundError: # 파일을 찾을 수 없는 예외가 발생하면
            self.heartbeat_table.setRowCount(0) # 테이블의 모든 데이터를 지움
        except Exception as e: # 그 외 다른 예외가 발생하면
            print(f"❌ Heartbeat 로그 로드 실패: {e}") # 콘솔에 에러 메시지를 출력
            self.heartbeat_table.setRowCount(0) # 테이블의 모든 데이터를 지움 