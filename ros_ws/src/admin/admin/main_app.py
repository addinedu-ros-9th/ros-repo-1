#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys  # 시스템 관련 기능을 사용하기 위해 임포트
import os   # 운영체제와 상호작용하기 위해 임포트 (파일 경로 등)
import rclpy # ROS2 파이썬 클라이언트 라이브러리
from PyQt5.QtWidgets import QApplication, QMainWindow # PyQt5에서 GUI 어플리케이션과 메인 윈도우를 만들기 위한 클래스
from PyQt5 import uic # .ui 파일을 파이썬 코드에서 사용할 수 있게 해주는 모듈
from ament_index_python.packages import get_package_share_directory # ROS2 패키지 경로를 쉽게 찾기 위한 함수

class AdminWindow(QMainWindow): # 메인 윈도우를 정의하는 클래스
    def __init__(self, parent=None):
        super().__init__(parent) # 부모 클래스(QMainWindow)의 초기화 함수를 호출
        self.init_ui() # UI 파일을 로드하는 함수를 호출

    def init_ui(self):
        package_share_dir = get_package_share_directory('admin') # 'admin' 패키지가 설치된 경로에서 'share' 디렉토리의 위치를 찾음
        ui_file = os.path.join(package_share_dir, 'ui', 'main_window.ui') # 'share' 디렉토리 아래 'ui/main_window.ui' 파일의 전체 경로를 만듦
        uic.loadUi(ui_file, self) # .ui 파일의 내용을 현재 객체에 적용 (self.위젯이름으로 접근 가능)

def main(args=None): # 프로그램을 실행하는 메인 함수
    rclpy.init(args=args) # ROS2 시스템을 초기화
    
    app = QApplication(sys.argv) # 모든 PyQt 어플리케이션은 하나의 QApplication 객체가 필요
    window = AdminWindow() # 우리가 만든 AdminWindow 클래스의 인스턴스(객체)를 생성
    window.show() # 생성된 윈도우를 화면에 보여줌
    
    exit_code = app.exec_() # 어플리케이션의 이벤트 루프를 시작 (창이 닫히면 종료 코드 반환)
    
    rclpy.shutdown() # ROS2 시스템을 깔끔하게 종료
    sys.exit(exit_code) # 이벤트 루프에서 받은 종료 코드로 시스템을 종료

if __name__ == '__main__': # 이 스크립트가 직접 실행될 때만 main() 함수를 호출
    main() 