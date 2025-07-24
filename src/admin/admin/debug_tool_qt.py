#!/usr/bin/env python3
import sys
import os
import rclpy
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import QTimer
from PyQt5 import uic
from libo_interfaces.msg import GoalPose # 우리가 만든 GoalPose 메시지를 임포트해.
from ament_index_python.packages import get_package_share_directory # 패키지 경로를 찾기 위한 함수를 임포트해.

class DebugToolWindow(QWidget): # QWidget을 상속받는 클래스를 만들어.
    def __init__(self): # 클래스가 생성될 때 호출되는 초기화 함수야.
        super().__init__() # 부모 클래스(QWidget)의 초기화 함수를 먼저 호출해.
        self.node = None
        self.publisher = None
        self.init_ui() # UI를 초기화하는 함수를 호출해.
        self.init_ros() # ROS2 관련 기능을 초기화하는 함수를 호출해.
        self.setup_connections() # 버튼 클릭 같은 이벤트를 연결하는 함수를 호출해.

    def init_ui(self):
        # colcon build로 설치된 패키지의 share 디렉토리 경로를 가져와.
        package_share_dir = get_package_share_directory('admin')
        # share 디렉토리 안의 ui 폴더에서 UI 파일을 찾아.
        ui_file = os.path.join(package_share_dir, 'ui', 'debug_tool.ui')
        uic.loadUi(ui_file, self)

    def init_ros(self):
        self.node = rclpy.create_node('debug_tool_qt_publisher') # 'debug_tool_qt_publisher'라는 이름의 노드를 만들어.
        self.publisher = self.node.create_publisher(GoalPose, '/goal_pose', 10) # '/goal_pose' 토픽에 GoalPose 메시지를 발행할 퍼블리셔를 만들어.

        # QTimer를 사용해서 주기적으로 ROS 이벤트를 처리해. UI가 멈추는 걸 방지하는 가장 좋은 방법이야.
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_ros_node)
        self.ros_timer.start(100) # 100ms (0.1초) 마다 spin_ros_node 함수를 실행해.
        self.log.append("✅ ROS2 Publisher 초기화 완료.")

    def spin_ros_node(self):
        rclpy.spin_once(self.node, timeout_sec=0) # ROS 이벤트를 한 번 처리해.

    def setup_connections(self):
        self.btnSend.clicked.connect(self.on_send_clicked) # btnSend 버튼이 클릭되면 on_send_clicked 함수를 실행하도록 연결해.
        # 각 입력창에서 엔터키를 눌러도 전송되도록 연결해.
        self.linex.returnPressed.connect(self.on_send_clicked)
        self.liney.returnPressed.connect(self.on_send_clicked)
        self.lineyaw.returnPressed.connect(self.on_send_clicked)

    def on_send_clicked(self):
        try:
            x_val = round(float(self.linex.text()), 1) # 입력값을 float으로 바꾸고, 소수점 첫째 자리에서 반올림해.
            y_val = round(float(self.liney.text()), 1) # 입력값을 float으로 바꾸고, 소수점 첫째 자리에서 반올림해.
            yaw_val = round(float(self.lineyaw.text()), 1) # 입력값을 float으로 바꾸고, 소수점 첫째 자리에서 반올림해.
        except ValueError:
            self.log.append("에러: 유효한 숫자를 입력해주세요.") # 만약 숫자로 바꿀 수 없으면, 로그 창에 에러 메시지를 출력하고 함수를 끝내.
            return

        msg = GoalPose() # GoalPose 메시지 객체를 만들어.
        msg.x = x_val
        msg.y = y_val
        msg.yaw = yaw_val

        self.publisher.publish(msg) # 만들어진 메시지를 토픽으로 발행(publish)해.
        self.log.append(f"➡️ 토픽 발행: x={msg.x:.1f}, y={msg.y:.1f}, yaw={msg.yaw:.1f}") # 로그 창에 발행 정보를 표시해.

    def closeEvent(self, event):
        self.log.append("디버깅 툴 종료 중...")
        self.ros_timer.stop() # 타이머를 멈춰.
        self.node.destroy_node() # ROS 노드를 깔끔하게 종료해.
        event.accept() # 창 닫기 이벤트를 수락해.

def main():
    rclpy.init() # ROS2 시스템을 초기화해.
    app = QApplication(sys.argv) # 모든 PyQt5 앱은 QApplication 객체가 하나 필요해.
    window = DebugToolWindow() # 우리가 만든 윈도우 클래스의 인스턴스를 생성해.
    window.show() # 창을 화면에 보여줘.
    exit_code = app.exec_() # 앱의 이벤트 루프를 시작해.
    rclpy.shutdown() # 앱이 끝나면 ROS2 시스템을 종료해.
    sys.exit(exit_code)

if __name__ == '__main__':
    main()