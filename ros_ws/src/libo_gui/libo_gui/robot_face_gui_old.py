#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont
import math


class RobotFaceWidget(QWidget):
    """로봇 얼굴을 그리는 위젯"""
    
    def __init__(self):
        super().__init__()
        self.face_state = "normal"  # normal, happy, sad, thinking, error
        self.setMinimumSize(400, 300)
        self.setStyleSheet("background-color: black;")
        
        # 애니메이션 관련 변수들
        self.blink_timer = QTimer()
        self.blink_timer.timeout.connect(self.blink_animation)
        self.blink_timer.start(3000)  # 3초마다 깜빡임
        
        self.blink_state = 0  # 0: 열린 상태, 1: 닫히는 중, 2: 닫힌 상태, 3: 열리는 중
        self.blink_progress = 0.0  # 깜빡임 진행도 (0.0 ~ 1.0)
        
        # 깜빡임 애니메이션 타이머
        self.blink_anim_timer = QTimer()
        self.blink_anim_timer.timeout.connect(self.update_blink_animation)
        self.blink_anim_timer.start(30)  # 30ms마다 업데이트 (더 부드럽게)
        
        # 상태별 애니메이션 타이머
        self.state_anim_timer = QTimer()
        self.state_anim_timer.timeout.connect(self.state_animation)
        self.state_anim_timer.start(50)  # 50ms마다 업데이트
        
        # 호흡 애니메이션 타이머
        self.breathing_timer = QTimer()
        self.breathing_timer.timeout.connect(self.breathing_animation)
        self.breathing_timer.start(100)  # 100ms마다 업데이트
        
        self.animation_progress = 0.0
        self.animation_direction = 1  # 1: 증가, -1: 감소
        self.breathing_progress = 0.0
        self.breathing_direction = 1
        
        # 색상 전환 애니메이션
        self.color_transition_progress = 0.0
        self.previous_color = None
        self.target_color = None
        
    def set_face_state(self, state):
        """얼굴 상태를 설정하고 화면을 업데이트"""
        if self.face_state != state:
            # 색상 전환 시작
            self.previous_color = self._get_eye_color()
            self.face_state = state
            self.target_color = self._get_eye_color()
            self.color_transition_progress = 0.0
            
        self.animation_progress = 0.0
        self.animation_direction = 1
        self.update()
        
    def blink_animation(self):
        """깜빡임 애니메이션 시작"""
        self.blink_state = 1
        self.blink_progress = 0.0
        
    def update_blink_animation(self):
        """깜빡임 애니메이션 업데이트"""
        if self.blink_state == 1:  # 닫히는 중
            self.blink_progress += 0.15  # 더 부드럽게
            if self.blink_progress >= 1.0:
                self.blink_state = 2
                self.blink_progress = 1.0
        elif self.blink_state == 2:  # 닫힌 상태
            self.blink_state = 3
        elif self.blink_state == 3:  # 열리는 중
            self.blink_progress -= 0.15  # 더 부드럽게
            if self.blink_progress <= 0.0:
                self.blink_state = 0
                self.blink_progress = 0.0
                
        self.update()
        
    def state_animation(self):
        """상태별 애니메이션 업데이트"""
        if self.face_state in ["happy", "thinking"]:
            self.animation_progress += 0.03 * self.animation_direction
            if self.animation_progress >= 1.0:
                self.animation_direction = -1
            elif self.animation_progress <= 0.0:
                self.animation_direction = 1
        else:
            self.animation_progress = 0.0
            
        # 색상 전환 애니메이션
        if self.previous_color and self.target_color and self.color_transition_progress < 1.0:
            self.color_transition_progress += 0.05
            if self.color_transition_progress >= 1.0:
                self.previous_color = None
                self.target_color = None
                self.color_transition_progress = 0.0
                
        self.update()
        
    def breathing_animation(self):
        """호흡 애니메이션"""
        self.breathing_progress += 0.02 * self.breathing_direction
        if self.breathing_progress >= 1.0:
            self.breathing_direction = -1
        elif self.breathing_progress <= 0.0:
            self.breathing_direction = 1
            
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 배경을 검은색으로 설정
        painter.fillRect(self.rect(), QColor(0, 0, 0))
        
        # 화면 중앙 계산
        center_x = self.width() // 2
        center_y = self.height() // 2
        
        # 호흡 효과 적용
        breathing_factor = 1.0 + 0.1 * math.sin(self.breathing_progress * 2 * math.pi)
        
        # 눈의 크기와 위치 설정
        base_eye_width = 60
        base_eye_height = 80
        eye_spacing = 100
        
        # 호흡에 따른 크기 변화
        eye_width = int(base_eye_width * breathing_factor)
        eye_height = int(base_eye_height * breathing_factor)
        
        # 깜빡임 효과 적용
        blink_factor = 1.0 - self.blink_progress * 0.8  # 최대 80%까지 닫힘
        current_eye_height = int(eye_height * blink_factor)
        
        # 눈 색상 설정 (상태에 따라)
        eye_color = self._get_eye_color_with_transition()
        
        # 왼쪽 눈 그리기
        left_eye_x = center_x - eye_spacing // 2 - eye_width // 2
        left_eye_y = center_y - current_eye_height // 2
        painter.setBrush(QBrush(eye_color))
        painter.setPen(QPen(eye_color))
        painter.drawEllipse(left_eye_x, left_eye_y, eye_width, current_eye_height)
        
        # 오른쪽 눈 그리기
        right_eye_x = center_x + eye_spacing // 2 - eye_width // 2
        right_eye_y = center_y - current_eye_height // 2
        painter.drawEllipse(right_eye_x, right_eye_y, eye_width, current_eye_height)
        
        # 상태에 따른 추가 표현 그리기
        self._draw_expression(painter, center_x, center_y)
        
    def _get_eye_color_with_transition(self):
        """색상 전환을 포함한 눈 색상 반환"""
        if self.previous_color and self.target_color and self.color_transition_progress < 1.0:
            # 색상 전환 중
            r1, g1, b1 = self.previous_color.red(), self.previous_color.green(), self.previous_color.blue()
            r2, g2, b2 = self.target_color.red(), self.target_color.green(), self.target_color.blue()
            
            r = int(r1 + (r2 - r1) * self.color_transition_progress)
            g = int(g1 + (g2 - g1) * self.color_transition_progress)
            b = int(b1 + (b2 - b1) * self.color_transition_progress)
            
            return QColor(r, g, b)
        else:
            return self._get_eye_color()
        
    def _get_eye_color(self):
        """상태에 따른 눈 색상 반환"""
        colors = {
            "normal": QColor(144, 238, 144),  # 연한 초록색
            "happy": QColor(255, 255, 0),     # 노란색
            "sad": QColor(0, 0, 255),         # 파란색
            "thinking": QColor(255, 165, 0),   # 주황색
            "error": QColor(255, 0, 0),       # 빨간색
        }
        return colors.get(self.face_state, colors["normal"])
        
    def _draw_expression(self, painter, center_x, center_y):
        """상태에 따른 추가 표현 그리기"""
        if self.face_state == "happy":
            # 행복한 표정 - 움직이는 미소
            painter.setPen(QPen(QColor(255, 255, 0), 3))
            smile_height = 40 + int(15 * math.sin(self.animation_progress * 2 * math.pi))
            painter.drawArc(center_x - 50, center_y - 20, 100, smile_height, 0, 180)
            
        elif self.face_state == "sad":
            # 슬픈 표정 - 눈꼬리가 내려간 모양
            painter.setPen(QPen(QColor(0, 0, 255), 3))
            painter.drawArc(center_x - 50, center_y + 20, 100, 40, 180, 180)
            
        elif self.face_state == "thinking":
            # 생각하는 표정 - 움직이는 점들
            painter.setPen(QPen(QColor(255, 165, 0), 2))
            for i in range(3):
                offset = int(8 * math.sin(self.animation_progress * 2 * math.pi + i * math.pi/2))
                painter.drawEllipse(center_x - 30 + i * 20, center_y + 30 + offset, 8, 8)
                
        elif self.face_state == "error":
            # 에러 표정 - 깜빡이는 X 표시
            if int(self.animation_progress * 8) % 2 == 0:  # 깜빡임 효과
                painter.setPen(QPen(QColor(255, 0, 0), 3))
                painter.drawLine(center_x - 20, center_y + 20, center_x + 20, center_y + 40)
                painter.drawLine(center_x + 20, center_y + 20, center_x - 20, center_y + 40)


class RobotFaceGUI(QMainWindow):
    """로봇 얼굴 GUI 메인 윈도우"""
    
    def __init__(self):
        super().__init__()
        self.ros_node = None
        self.init_ui()
        
    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("Robot Face GUI")
        self.setGeometry(100, 100, 600, 500)
        
        # 중앙 위젯 생성
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 레이아웃 설정
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        # 로봇 얼굴 위젯 추가
        self.face_widget = RobotFaceWidget()
        layout.addWidget(self.face_widget)
        
        # 상태 표시 라벨
        self.status_label = QLabel("상태: normal")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("color: white; font-size: 16px; background-color: #333; padding: 10px;")
        layout.addWidget(self.status_label)
        
        # 버튼 레이아웃
        button_layout = QHBoxLayout()
        layout.addLayout(button_layout)
        
        # 상태 변경 버튼들
        states = ["normal", "happy", "sad", "thinking", "error"]
        for state in states:
            btn = QPushButton(state.capitalize())
            btn.clicked.connect(lambda checked, s=state: self.change_face_state(s))
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #444;
                    color: white;
                    border: 2px solid #666;
                    border-radius: 5px;
                    padding: 10px;
                    font-size: 14px;
                }
                QPushButton:hover {
                    background-color: #555;
                }
                QPushButton:pressed {
                    background-color: #333;
                }
            """)
            button_layout.addWidget(btn)
            
    def change_face_state(self, state):
        """얼굴 상태 변경"""
        self.face_widget.set_face_state(state)
        self.status_label.setText(f"상태: {state}")
        
        # ROS 노드가 있으면 상태 메시지 발행
        if self.ros_node:
            msg = String()
            msg.data = state
            self.ros_node.get_logger().info(f"Face state changed to: {state}")


class RobotFaceNode(Node):
    """ROS2 노드"""
    
    def __init__(self):
        super().__init__('robot_face_gui')
        self.face_state_pub = self.create_publisher(String, 'robot_face_state', 10)
        self.get_logger().info('Robot Face GUI Node started')


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    # ROS 노드 생성
    ros_node = RobotFaceNode()
    
    # PyQt5 애플리케이션 생성
    app = QApplication(sys.argv)
    
    # GUI 윈도우 생성
    gui = RobotFaceGUI()
    gui.ros_node = ros_node
    gui.show()
    
    # 타이머로 ROS 이벤트 처리
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)  # 10ms 간격
    
    # 애플리케이션 실행
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 