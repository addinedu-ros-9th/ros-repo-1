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

    def on_send_clicked(self): # 버튼 클릭시~ 
        x_val = self.linex.text() # linex 입력창의 텍스트를 가져와.
        y_val = self.liney.text() # liney 입력창의 텍스트를 가져와.
        yaw_val = self.lineyaw.text() # lineyaw 입력창의 텍스트를 가져와. (UI에 z가 아니라 yaw로 되어있어)

        print(f"Send 버튼 클릭! x: {x_val}, y: {y_val}, yaw: {yaw_val}") # 가져온 값들을 터미널에 출력해.

def main():
    app = QApplication(sys.argv) # 모든 PyQt5 앱은 QApplication 객체가 하나 필요해.
    window = DebugToolWindow() # 우리가 만든 윈도우 클래스의 인스턴스를 생성해.
    window.show() # 창을 화면에 보여줘.
    sys.exit(app.exec_()) # 앱의 이벤트 루프를 시작해.

if __name__ == '__main__':
    main()