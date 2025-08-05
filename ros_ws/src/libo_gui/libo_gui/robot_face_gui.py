#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PyQt5 import uic
import math
import os

# Libo 인터페이스 메시지 임포트
try:
    from libo_interfaces.msg import FaceExpression
except ImportError:
    print("⚠️ libo_interfaces 패키지를 찾을 수 없습니다. 기본 메시지를 사용합니다.")
    FaceExpression = None


class RobotFaceWidget(QWidget):
    """로봇 얼굴을 그리는 위젯 - 통합된 FaceExpression 시스템"""
    
    def __init__(self):
        super().__init__()
        # 통합된 상태 시스템
        self.current_state = "normal"  # 통합된 현재 상태
        self.current_robot_id = "libo_a"  # 현재 로봇 ID
        
        self.setMinimumSize(800, 400)
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
        self.blink_anim_timer.start(30)  # 30ms마다 업데이트
        
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
        
        # 음성 상태별 특수 효과
        self.voice_animation_progress = 0.0
        self.voice_animation_direction = 1
        
    def set_face_state(self, state, robot_id=None):
        """FaceExpression 상태 설정"""
        if robot_id:
            self.current_robot_id = robot_id
            
        if self.current_state != state:
            # 색상 전환 시작
            self.previous_color = self._get_eye_color()
            self.current_state = state
            self.target_color = self._get_eye_color()
            self.color_transition_progress = 0.0
            
        self.animation_progress = 0.0
        self.animation_direction = 1
        self.update()
        
    def get_current_state(self):
        """현재 상태 반환"""
        return self.current_state
        
    def blink_animation(self):
        """깜빡임 애니메이션 시작"""
        self.blink_state = 1
        self.blink_progress = 0.0
        
    def update_blink_animation(self):
        """깜빡임 애니메이션 업데이트"""
        if self.blink_state == 1:  # 닫히는 중
            self.blink_progress += 0.15
            if self.blink_progress >= 1.0:
                self.blink_state = 2
                self.blink_progress = 1.0
        elif self.blink_state == 2:  # 닫힌 상태
            self.blink_state = 3
        elif self.blink_state == 3:  # 열리는 중
            self.blink_progress -= 0.15
            if self.blink_progress <= 0.0:
                self.blink_state = 0
                self.blink_progress = 0.0
                
        self.update()
        
    def state_animation(self):
        """상태별 애니메이션 업데이트"""
        # Face 애니메이션
        if self.current_state in ["happy", "focused"]:
            self.animation_progress += 0.03 * self.animation_direction
            if self.animation_progress >= 1.0:
                self.animation_direction = -1
            elif self.animation_progress <= 0.0:
                self.animation_direction = 1
        else:
            self.animation_progress = 0.0
            
        # Voice 애니메이션 (listening, speaking 상태)
        if self.current_state in ["listening", "speaking"]:
            self.voice_animation_progress += 0.05 * self.voice_animation_direction
            if self.voice_animation_progress >= 1.0:
                self.voice_animation_direction = -1
            elif self.voice_animation_progress <= 0.0:
                self.voice_animation_direction = 1
        else:
            self.voice_animation_progress = 0.0
            
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
        
        # 모든 상태에서 일반 눈 그리기
        # 눈의 크기와 위치 설정
        base_eye_width = 120
        base_eye_height = 160
        eye_spacing = 500
        
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
        self._draw_face_expression(painter, center_x, center_y)
        
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
            # FaceExpression 상태들
            "normal": QColor(144, 238, 144),    # 연한 초록색
            "focused": QColor(255, 165, 0),     # 주황색
            "charging": QColor(70, 130, 225),   # 파란색
            "heavy": QColor(139, 69, 19),       # 갈색
            "happy": QColor(255, 255, 0),       # 노란색
            "sad": QColor(220, 20, 60),         # 빨간색
            # 음성 상태들 (FaceExpression으로 통일)
            "listening": QColor(138, 43, 226),  # 보라색
            "speaking": QColor(255, 140, 0),    # 주황색
        }
        return colors.get(self.current_state, colors["normal"])
        
    def _draw_face_expression(self, painter, center_x, center_y):
        """FaceExpression 상태에 따른 추가 표현 그리기"""
        if self.current_state == "happy":
            painter.setPen(QPen(QColor(255, 255, 0), 6))
            painter.drawArc(center_x - 50, center_y + 120, 100, 40, 180 * 16, 180 * 16)
                
        elif self.current_state == "sad":
            # 슬픈 표정 - 아래로 굽은 입 (슬픈 입)
            painter.setPen(QPen(QColor(220, 20, 60), 6))
            painter.drawArc(center_x - 50, center_y + 120, 100, 40, 0, 180 * 16)
            
        elif self.current_state == "focused":
            # 집중하는 표정 - 움직이는 점들
            painter.setPen(QPen(QColor(255, 165, 0), 2))
            for i in range(3):
                offset = int(8 * math.sin(self.animation_progress * 2 * math.pi + i * math.pi/2))
                painter.drawEllipse(center_x - 30 + i * 20, center_y + 100 + offset, 8, 8)
                
        elif self.current_state == "charging":
            # 충전 중 - 번개 표시 (육각형 프레임 스타일)
            # 육각형 프레임 그리기
            painter.setPen(QPen(QColor(255, 215, 0), 3))  # 노란색
            painter.setBrush(QBrush(QColor(255, 255, 255)))  # 흰색 배경
            
            # 육각형 프레임 좌표
            hex_points = [
                (center_x - 60, center_y + 60),      # 위쪽 꼭짓점
                (center_x - 40, center_y + 30),      # 위쪽 왼쪽
                (center_x + 40, center_y + 30),      # 위쪽 오른쪽
                (center_x + 60, center_y + 60),      # 오른쪽 꼭짓점
                (center_x + 40, center_y + 90),      # 아래쪽 오른쪽
                (center_x - 40, center_y + 90),      # 아래쪽 왼쪽
            ]
            
            # 육각형 그리기
            for i in range(len(hex_points)):
                painter.drawLine(hex_points[i][0], hex_points[i][1], 
                               hex_points[(i+1) % len(hex_points)][0], 
                               hex_points[(i+1) % len(hex_points)][1])
            
            # 번개 모양 그리기 (더 굵고 날카롭게)
            painter.setPen(QPen(QColor(255, 215, 0), 12))  # 노란색, 굵은 선
            lightning_points = [
                (center_x - 8, center_y + 35),       # 번개 시작
                (center_x + 12, center_y + 50),      # 첫 번째 꺾임
                (center_x - 5, center_y + 65),       # 두 번째 꺾임
                (center_x + 18, center_y + 80)       # 번개 끝
            ]
            
            for i in range(len(lightning_points) - 1):
                painter.drawLine(lightning_points[i][0], lightning_points[i][1], 
                               lightning_points[i+1][0], lightning_points[i+1][1])
                
        elif self.current_state == "heavy":
            # 무거운 표정 - 땀방울
            painter.setPen(QPen(QColor(70, 130, 225), 2))
            painter.setBrush(QBrush(QColor(70, 130, 225)))
            # 땀방울 그리기
            painter.drawEllipse(center_x - 500, center_y - 50, 50, 80)
            painter.drawEllipse(center_x - 570, center_y - 100, 50, 80)
            painter.drawEllipse(center_x + 500, center_y - 40, 60, 90)
            
        elif self.current_state == "listening":
            # 듣는 중 - 와이파이 모양 (양쪽에 4개씩, 중앙에서 간격 벌림)
            painter.setPen(QPen(QColor(138, 43, 226), 3))
            
            # 애니메이션 효과를 위한 크기 변화
            base_size = 20
            wave_size = int(base_size + 10 * math.sin(self.voice_animation_progress * 4 * math.pi))
            
            # 왼쪽 와이파이 신호들 (4개, 오른쪽으로 퍼짐)
            left_base_x = center_x - 500  # 중앙에서 더 멀리
            left_base_y = center_y - 30
            
            for i in range(4):
                # 각 신호의 크기와 위치 (안쪽에서 바깥쪽으로 커짐)
                signal_size = wave_size + i * 20  # 크기 증가폭도 늘림
                signal_x = left_base_x - i * 35  # 간격을 더 벌림
                signal_y = left_base_y
                
                # 와이파이 신호 그리기 (90도에서 시작해서 180도 스윕)
                painter.drawArc(signal_x - signal_size//2, signal_y - signal_size//2, 
                              signal_size, signal_size, 90 * 16, 180 * 16)
            
            # 오른쪽 와이파이 신호들 (4개, 왼쪽으로 퍼짐)
            right_base_x = center_x + 500  # 중앙에서 더 멀리
            right_base_y = center_y - 30
            
            for i in range(4):
                # 각 신호의 크기와 위치 (안쪽에서 바깥쪽으로 커짐)
                signal_size = wave_size + i * 20  # 크기 증가폭도 늘림
                signal_x = right_base_x + i * 35  # 간격을 더 벌림
                signal_y = right_base_y
                
                # 와이파이 신호 그리기 (270도에서 시작해서 180도 스윕)
                painter.drawArc(signal_x - signal_size//2, signal_y - signal_size//2, 
                              signal_size, signal_size, 270 * 16, 180 * 16)
            
        elif self.current_state == "speaking":
            # 말하는 중 - 입 모양 애니메이션
            painter.setPen(QPen(QColor(255, 140, 0), 3))
            
            # 입 위치
            mouth_x = center_x
            mouth_y = center_y + 80
            
            # 애니메이션에 따른 입 크기 변화
            mouth_width = int(60 + 20 * math.sin(self.voice_animation_progress * 8 * math.pi))
            mouth_height = int(30 + 15 * math.sin(self.voice_animation_progress * 6 * math.pi))
            
            # 말하는 중일 때는 입이 열리고 닫히는 애니메이션
            if self.voice_animation_progress < 0.5:
                # 입이 열린 상태 (타원)
                painter.drawEllipse(mouth_x - mouth_width//2, mouth_y - mouth_height//2, 
                                  mouth_width, mouth_height)
            else:
                # 입이 닫힌 상태 (선)
                painter.drawLine(mouth_x - mouth_width//2, mouth_y, 
                               mouth_x + mouth_width//2, mouth_y)


class RobotFaceGUI(QMainWindow):
    """로봇 얼굴 GUI 메인 윈도우 - 통합된 FaceExpression 시스템"""
    
    def __init__(self):
        super().__init__()
        self.ros_node = None
        self.current_state = "normal"
        self.current_robot_id = "libo_a"
        
        self.load_ui()
        self.setup_connections()
        
    def load_ui(self):
        """UI 파일 로드"""
        # UI 파일 경로 설정 - 여러 경로에서 찾기
        ui_file_path = None
        
        # 1. 개발 환경에서의 경로
        dev_path = os.path.join(os.path.dirname(__file__), '..', 'ui', 'robot_face_gui.ui')
        if os.path.exists(dev_path):
            ui_file_path = dev_path
        else:
            # 2. 설치된 패키지에서의 경로 (ROS2 Jazzy 업데이트)
            share_paths = [
                '/opt/ros/jazzy/share/libo_gui/ui/robot_face_gui.ui',
                '/opt/ros/humble/share/libo_gui/ui/robot_face_gui.ui',
                os.path.join(os.getcwd(), 'install/libo_gui/share/libo_gui/ui/robot_face_gui.ui'),
                os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'ui', 'robot_face_gui.ui')
            ]
            
            for path in share_paths:
                if os.path.exists(path):
                    ui_file_path = path
                    break
        
        if ui_file_path is None:
            raise FileNotFoundError("robot_face_gui.ui 파일을 찾을 수 없습니다.")
            
        print(f"UI 파일 경로: {ui_file_path}")
        uic.loadUi(ui_file_path, self)
        
        # face_widget을 RobotFaceWidget으로 교체
        self.face_widget = RobotFaceWidget()
        # 기존 face_widget의 위치에 새로운 위젯 삽입
        layout = self.centralwidget.layout()
        layout.replaceWidget(self.findChild(QWidget, "face_widget"), self.face_widget)
        
    def setup_connections(self):
        """새로운 버튼 연결 설정"""
        # 기본 상태 버튼들
        self.normal_button.clicked.connect(lambda: self.change_face_state("normal"))
        self.focused_button.clicked.connect(lambda: self.change_face_state("focused"))
        self.charging_button.clicked.connect(lambda: self.change_face_state("charging"))
        self.heavy_button.clicked.connect(lambda: self.change_face_state("heavy"))
        
        # 특별 표정 버튼들
        self.happy_button.clicked.connect(lambda: self.change_face_state("happy"))
        self.sad_button.clicked.connect(lambda: self.change_face_state("sad"))
        
        # 음성 상태 버튼들 (FaceExpression으로 통일)
        self.listening_button.clicked.connect(lambda: self.change_face_state("listening"))
        self.speaking_button.clicked.connect(lambda: self.change_face_state("speaking"))
            
    def change_face_state(self, state):
        """FaceExpression 상태 변경"""
        self.current_state = state
        self.face_widget.set_face_state(state, self.current_robot_id)
        self.update_status_display()
        
        # ROS 노드를 통해 FaceExpression 메시지 발행 (테스트용)
        if self.ros_node and hasattr(self.ros_node, 'publish_face_expression'):
            self.ros_node.publish_face_expression(self.current_robot_id, state, f"Manual change to {state}")
    
    def update_status_display(self):
        """상태 표시 업데이트"""
        current_state = self.face_widget.get_current_state()
        emoji = self._get_state_emoji(current_state)
        
        status_text = f"{emoji} 상태: {current_state}"
        self.status_label.setText(status_text)
        
    def _get_state_emoji(self, state):
        """상태에 따른 이모지 반환"""
        emojis = {
            # FaceExpression 상태들
            "normal": "🟢",
            "focused": "🟡", 
            "charging": "🔵",
            "heavy": "🟤",
            "happy": "💛",
            "sad": "🔴",
            # 음성 상태들 (FaceExpression으로 통일)
            "listening": "🟣",
            "speaking": "🟠"
        }
        return emojis.get(state, "🟢")
    
    def handle_face_expression_message(self, msg):
        """외부에서 받은 FaceExpression 메시지 처리"""
        if msg.robot_id == self.current_robot_id:
            self.current_state = msg.expression_type
            self.face_widget.set_face_state(msg.expression_type, msg.robot_id)
            self.update_status_display()
            
            if self.ros_node:
                self.ros_node.get_logger().info(
                    f"🎭 FaceExpression 수신: {msg.robot_id} -> {msg.expression_type} ({msg.description})"
                )


class RobotFaceNode(Node):
    """ROS2 노드 - 통합된 FaceExpression 시스템"""
    
    def __init__(self):
        super().__init__('robot_face_gui')
        
        # 구독자 (외부에서 메시지 받기)
        if FaceExpression:
            self.face_expression_sub = self.create_subscription(
                FaceExpression,
                'libo_face_expression',
                self.face_expression_callback,
                10
            )
        
        # 발행자 (테스트용 메시지 발행)
        if FaceExpression:
            self.face_expression_pub = self.create_publisher(FaceExpression, 'libo_face_expression', 10)
        
        self.gui = None  # GUI 인스턴스 저장용
        
        self.get_logger().info('🤖 Libo Robot Face GUI Node started - 통합된 FaceExpression 시스템!')
        self.get_logger().info(f'   📥 구독 토픽: libo_face_expression')
        self.get_logger().info(f'   📤 발행 토픽: libo_face_expression')
        self.get_logger().info(f'   🎯 상태 시스템: 모든 상태가 FaceExpression으로 통일')
    
    def face_expression_callback(self, msg):
        """FaceExpression 메시지 수신 콜백"""
        if self.gui:
            self.gui.handle_face_expression_message(msg)
    
    def publish_face_expression(self, robot_id, expression_type, description=""):
        """FaceExpression 메시지 발행 (테스트용)"""
        if FaceExpression and hasattr(self, 'face_expression_pub'):
            msg = FaceExpression()
            msg.robot_id = robot_id
            msg.expression_type = expression_type
            msg.description = description
            self.face_expression_pub.publish(msg)
            self.get_logger().info(f"📤 FaceExpression 발행: {robot_id} -> {expression_type}")


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
    ros_node.gui = gui  # 양방향 연결
    gui.show()
    
    # 타이머로 ROS 이벤트 처리
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)  # 10ms 간격
    
    # 애플리케이션 시작 로그
    print("🚀 Libo Robot Face GUI 시작!")
    print("   🎭 지원 표정: normal, focused, charging, heavy, happy, sad, listening, speaking")
    print("   🔗 ROS2 토픽: libo_face_expression (통일된 시스템)")
    print("   🎯 상태 시스템: 모든 상태가 FaceExpression으로 통일")
    
    # 애플리케이션 실행
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\n🛑 사용자 중단")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        print("🏁 Libo Robot Face GUI 종료")


if __name__ == '__main__':
    main()