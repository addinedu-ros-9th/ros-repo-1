#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5 import uic
import os

class DebugToolWindow(QWidget): # QWidget을 상속받는 클래스를 만들어.
    def __init__(self): # 클래스가 생성될 때 호출되는 초기화 함수야.
        super().__init__() # 부모 클래스(QWidget)의 초기화 함수를 먼저 호출해.
        self.init_ui() # UI를 초기화하는 함수를 호출해.
        self.setup_connections() # 버튼 클릭 같은 이벤트를 연결하는 함수를 호출해.

    def init_ui(self):
        ui_file = os.path.join(os.path.dirname(__file__), 'debug_tool.ui') # UI 파일 경로를 만들어.
        uic.loadUi(ui_file, self) # uic.loadUi로 UI 파일을 불러와서 현재 클래스(self)에 적용해.

    def setup_connections(self): # 이런저런 ui 인풋에 대한 아웃풋
        self.btnSend.clicked.connect(self.on_send_clicked) # btnSend 버튼이 클릭되면 on_send_clicked 함수를 실행하도록 연결해.
        # 각 입력창에서 엔터키를 눌러도 전송되도록 연결해.
        self.linex.returnPressed.connect(self.on_send_clicked)
        self.liney.returnPressed.connect(self.on_send_clicked)
        self.lineyaw.returnPressed.connect(self.on_send_clicked)

    def on_send_clicked(self): # 버튼 클릭시~ 
        try:
            x_val = float(self.linex.text()) # linex 입력창의 텍스트를 float으로 변환해.
            y_val = float(self.liney.text()) # liney 입력창의 텍스트를 float으로 변환해.
            yaw_val = float(self.lineyaw.text()) # lineyaw 입력창의 텍스트를 float으로 변환해.
        except ValueError:
            self.log.append("에러: 유효한 숫자를 입력해주세요.") # 만약 숫자로 바꿀 수 없으면, 로그 창에 에러 메시지를 출력하고 함수를 끝내.
            return

        log_message = f"Send 버튼 클릭! x: {x_val:.1f}, y: {y_val:.1f}, yaw: {yaw_val:.1f}" # 로그 창에 표시할 메시지를 만들어.
        self.log.append(log_message) # 로그 창(QTextEdit)에 메시지를 한 줄 추가해.

def main():
    app = QApplication(sys.argv) # 모든 PyQt5 앱은 QApplication 객체가 하나 필요해.
    window = DebugToolWindow() # 우리가 만든 윈도우 클래스의 인스턴스를 생성해.
    window.show() # 창을 화면에 보여줘.
    sys.exit(app.exec_()) # 앱의 이벤트 루프를 시작해.

if __name__ == '__main__':
    main()