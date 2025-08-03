#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 필요한 ROS2 및 메시지/서비스 타입들을 임포트합니다.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from libo_interfaces.msg import HumanInfo, DetectionStatus
from libo_interfaces.srv import ActivateTracker, DeactivateTracker
from libo_interfaces.msg import TalkCommand # 음성 명령 처리를 위한 메시지 타입

class PIDController:
    """
    PID(Proportional-Integral-Derivative) 제어기 클래스.
    목표값에 부드럽고 안정적으로 도달하기 위해 사용됩니다.
    """
    def __init__(self, kp, ki, kd, max_output=None, min_output=None):
        # PID 게인(Gain) 및 출력 제한값 초기화
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_output, self.min_output = max_output, min_output
        # 내부 상태 변수 초기화
        self.previous_error, self.integral = 0.0, 0.0

    def update(self, error, dt=0.1):
        """새로운 오차 값을 입력받아 PID 제어 출력을 계산합니다."""
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        # 출력값이 설정된 최댓값/최솟값을 넘지 않도록 제한 (Clamping)
        if self.max_output is not None: output = min(output, self.max_output)
        if self.min_output is not None: output = max(output, self.min_output)
        self.previous_error = error
        return output

    def reset(self):
        """제어기의 내부 상태(누적 오차 등)를 초기화합니다."""
        self.previous_error, self.integral = 0.0, 0.0

class AdvancedAssistFollowFSM(Node):
    """
    [최종] 사람 추종, 장애물 회피, QR 인증, 음성 명령을 총괄하는 고도화된 FSM 노드.
    """
    def __init__(self):
        super().__init__('advanced_assist_follow_fsm')

        # --- 1. ROS 파라미터 선언 ---
        # 코드 수정 없이 launch 파일에서 쉽게 값을 변경(튜닝)할 수 있도록 파라미터화합니다.
        self.declare_parameter('target_distance', 0.5, description='로봇-사용자 간 목표 거리 (m)')
        self.declare_parameter('safe_distance_min', 0.3, description='로봇-사용자 간 최소 안전 거리 (m)')
        self.declare_parameter('max_linear_vel', 0.3, description='최대 전진/후진 속도 (m/s)')
        self.declare_parameter('max_angular_vel', 0.5, description='최대 회전 속도 (rad/s)')
        # 장애물 회피 기동 관련 파라미터
        self.declare_parameter('avoidance_linear_vel', 0.1, description='회피 기동 시 직진 속도 (m/s)')
        self.declare_parameter('avoidance_angular_vel', 0.2, description='회피 기동 시 회전 속도 (rad/s)')
        self.declare_parameter('avoidance_backup_vel', -0.1, description='회피 기동 시 후진 속도 (m/s)')
        self.declare_parameter('avoidance_backup_duration_s', 1.5, description='회피 기동 시 후진 지속 시간 (초)')
        self.declare_parameter('avoidance_turn_duration_s', 1.0, description='회피 기동 시 회전 지속 시간 (초)')
        self.declare_parameter('avoidance_straight_duration_s', 1.5, description='회피 기동 시 직진 지속 시간 (초)')
        # PID 제어기 게인 파라미터 (이 값들을 튜닝하여 로봇의 반응성을 조절)
        self.declare_parameter('dist_kp', 1.0); self.declare_parameter('dist_ki', 0.0); self.declare_parameter('dist_kd', 0.1)
        self.declare_parameter('angle_kp', 1.5); self.declare_parameter('angle_ki', 0.0); self.declare_parameter('angle_kd', 0.2)
        
        # 선언된 파라미터 값을 변수로 가져와 사용
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.safe_distance_min = self.get_parameter('safe_distance_min').get_parameter_value().double_value
        self.avoidance_linear_vel = self.get_parameter('avoidance_linear_vel').get_parameter_value().double_value
        self.avoidance_angular_vel = self.get_parameter('avoidance_angular_vel').get_parameter_value().double_value
        self.avoidance_backup_vel = self.get_parameter('avoidance_backup_vel').get_parameter_value().double_value
        self.avoidance_backup_duration = self.get_parameter('avoidance_backup_duration_s').get_parameter_value().double_value
        self.avoidance_turn_duration = self.get_parameter('avoidance_turn_duration_s').get_parameter_value().double_value
        self.avoidance_straight_duration = self.get_parameter('avoidance_straight_duration_s').get_parameter_value().double_value

        # --- 2. PID 제어기 생성 ---
        # 거리(선속도)와 각도(각속도)를 독립적으로 제어하기 위해 2개의 PID 제어기를 생성
        self.distance_pid = PIDController(
            kp=self.get_parameter('dist_kp').get_parameter_value().double_value, ki=self.get_parameter('dist_ki').get_parameter_value().double_value, kd=self.get_parameter('dist_kd').get_parameter_value().double_value,
            max_output=self.get_parameter('max_linear_vel').get_parameter_value().double_value, min_output=-self.get_parameter('max_linear_vel').get_parameter_value().double_value)
        self.angle_pid = PIDController(
            kp=self.get_parameter('angle_kp').get_parameter_value().double_value, ki=self.get_parameter('angle_ki').get_parameter_value().double_value, kd=self.get_parameter('angle_kd').get_parameter_value().double_value,
            max_output=self.get_parameter('max_angular_vel').get_parameter_value().double_value, min_output=-self.get_parameter('max_angular_vel').get_parameter_value().double_value)

        # --- 3. 상태 변수 및 통신 인터페이스 초기화 ---
        self.qr_authenticated = False       # QR 인증 완료 여부. 전체 기능의 마스터 스위치
        self.is_following = False           # 로봇이 현재 사용자를 따라 움직이는 중인지 여부
        self.is_paused_by_voice = False     # 음성 명령('stop')으로 추종이 '일시정지' 되었는지 여부
        self.obstacle_status = None         # 가장 최근에 수신한 장애물 센서 정보
        self.state = "FOLLOWING"            # FSM의 현재 상태 (FOLLOWING, AVOIDING_BACKUP, ...)
        self.avoidance_timer = None         # 회피 기동 시 사용되는 시간 기반 타이머 객체
        self.avoidance_turn_direction = None # 회피 시 회전해야 할 방향 ('LEFT' 또는 'RIGHT')

        # ROS2 통신 인터페이스 설정
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 로봇 속도 명령 발행용
        self.human_info_sub = self.create_subscription(HumanInfo, '/human_info', self.human_info_callback, 10) # 사람 위치 정보 수신용
        self.detection_status_sub = self.create_subscription(DetectionStatus, '/detection_status', self.detection_status_callback, 10) # 장애물 정보 수신용
        self.talk_command_sub = self.create_subscription(TalkCommand, '/talk_command', self.talk_command_callback, 10) # 음성 명령 수신용
        self.activate_srv = self.create_service(ActivateTracker, '/activate_tracker', self.handle_activate_tracker) # 모드 활성화 서비스 제공용
        self.deactivate_srv = self.create_service(DeactivateTracker, '/deactivate_tracker', self.handle_deactivate_tracker) # 모드 비활성화 서비스 제공용

        self.get_logger().info('✅ Advanced Assist Follow FSM 노드 시작 완료 (음성 명령 처리 기능 포함)')

    # --- 4. 서비스 핸들러 (외부 노드와의 통신) ---
    def handle_activate_tracker(self, request, response):
        """'/activate_tracker' 서비스 요청 처리. QR 인증 완료 시 호출됩니다."""
        self.get_logger().info(f"🟢 Activate 요청 수신 - robot_id: {request.robot_id}")
        self.qr_authenticated = True # 전체 FSM을 활성화
        response.success = True
        response.message = "Assist mode activated with advanced FSM."
        return response

    def handle_deactivate_tracker(self, request, response):
        """'/deactivate_tracker' 서비스 요청 처리. 모드 종료 시 호출됩니다."""
        self.get_logger().info(f"🔴 Deactivate 요청 수신 - robot_id: {request.robot_id}")
        self.qr_authenticated = False # 전체 FSM을 비활성화
        self.stop_robot() # 안전을 위해 로봇을 즉시 정지하고 모든 상태를 초기화
        response.success = True
        response.message = "Assist mode deactivated."
        return response

    # --- 5. 콜백 함수 (데이터 수신 및 메인 로직) ---
    def detection_status_callback(self, msg: DetectionStatus):
        """'/detection_status' 토픽으로부터 장애물 감지 정보를 비동기적으로 수신하여 저장합니다."""
        self.obstacle_status = msg

    def talk_command_callback(self, msg: TalkCommand):
        """'/talk_command' 토픽을 통해 음성 명령을 처리하는 콜백 함수."""
        # QR 인증 전이거나, 이 로봇을 위한 명령이 아니면 무시
        if not self.qr_authenticated or msg.robot_id != "libo_a": # TODO: 'libo_a'를 파라미터로 관리하면 더 유연해짐
            return

        if msg.action == "stop":
            # '정지' 또는 '일시중지' 명령을 받았을 때
            if not self.is_paused_by_voice:
                self.get_logger().info("🎤 음성 명령으로 추종을 일시 중지합니다.")
                self.is_paused_by_voice = True # '일시정지' 상태 플래그를 True로 설정
                self.stop_robot() # 로봇을 물리적으로 정지시킴
        
        elif msg.action == "follow":
            # '다시 따라와' (재개) 명령을 받았을 때
            if self.is_paused_by_voice:
                self.get_logger().info("🎤 음성 명령으로 추종을 재개합니다.")
                self.is_paused_by_voice = False # '일시정지' 상태 플래그를 False로 해제
                # 플래그만 해제하면, 다음 human_info_callback 루프에서 사람이 감지될 시 자동으로 추종을 시작
        
    def human_info_callback(self, msg: HumanInfo):
        """사람 위치 정보를 받을 때마다 호출되는 메인 콜백. 모든 판단과 행동 결정이 여기서 이뤄집니다."""
        # 마스터 스위치 확인: QR 인증 전이면 모든 동작 중지
        if not self.qr_authenticated: return

        # 데이터 수신 확인: 장애물 정보가 아직 없다면 대기
        if self.obstacle_status is None:
            self.get_logger().info('장애물 감지 정보 수신 대기 중...', once=True)
            return

        # 최우선 순위: 비상 정지. 다른 어떤 상태보다 먼저 확인하여 안전 확보
        if self.obstacle_status.center_detected or \
           (self.obstacle_status.left_detected and self.obstacle_status.right_detected):
            self.get_logger().warn('🚨 전방 또는 양측 장애물 동시 감지! 비상 정지!', throttle_duration_sec=1)
            self.stop_robot()
            return

        # FSM (유한 상태 머신) 기반 행동 결정
        # 'self.state' 변수의 현재 값에 따라 다른 행동을 수행
        if self.state == "FOLLOWING":
            # 측면에 장애물이 감지되면 회피 기동 상태로 전환
            if self.obstacle_status.left_detected:
                self.get_logger().info("좌측 장애물 감지. 회피 기동(후진->우회전) 시작.")
                self.state = "AVOIDING_BACKUP"
                self.avoidance_turn_direction = 'RIGHT'
            elif self.obstacle_status.right_detected:
                self.get_logger().info("우측 장애물 감지. 회피 기동(후진->좌회전) 시작.")
                self.state = "AVOIDING_BACKUP"
                self.avoidance_turn_direction = 'LEFT'
            else: # 회피할 장애물이 없으면 정상 추종
                self.perform_following_with_pid(msg)
                return

        # 회피 기동 상태 처리
        if self.state == "AVOIDING_BACKUP":
            cmd_msg = Twist(); cmd_msg.linear.x = self.avoidance_backup_vel
            self.cmd_vel_pub.publish(cmd_msg)
            if self.avoidance_timer is None: self.avoidance_timer = self.create_timer(self.avoidance_backup_duration, self.transition_to_avoid_turn)
            return

        if self.state == "AVOIDING_TURN":
            cmd_msg = Twist(); cmd_msg.angular.z = self.avoidance_angular_vel if self.avoidance_turn_direction == 'LEFT' else -self.avoidance_angular_vel
            self.cmd_vel_pub.publish(cmd_msg)
            if self.avoidance_timer is None: self.avoidance_timer = self.create_timer(self.avoidance_turn_duration, self.transition_to_avoid_straight)
            return

        if self.state == "AVOIDING_STRAIGHT":
            cmd_msg = Twist(); cmd_msg.linear.x = self.avoidance_linear_vel
            self.cmd_vel_pub.publish(cmd_msg)
            if self.avoidance_timer is None: self.avoidance_timer = self.create_timer(self.avoidance_straight_duration, self.transition_to_following)
            return

    # --- 6. 핵심 기능 함수 ---
    def perform_following_with_pid(self, msg: HumanInfo):
        """PID 제어기를 사용하여 사람을 추종하는 핵심 로직"""
        # 음성 명령으로 '일시정지'된 상태라면, 사람이 보여도 추종 로직을 실행하지 않음
        if self.is_paused_by_voice:
            return

        # 사람이 감지되지 않으면 정지
        if not msg.is_detected:
            if self.is_following:
                self.get_logger().warn('사람을 놓쳤습니다. 즉시 정지합니다.')
                self.stop_robot()
            return
        self.is_following = True

        # 사람이 최소 안전 거리보다 가까우면 안전을 위해 정지
        if msg.distance < self.safe_distance_min:
            self.get_logger().warn(f'사람이 너무 가까움! ({msg.distance:.2f}m) 정지!', throttle_duration_sec=1)
            self.stop_robot()
            return

        # PID 제어기에 입력할 오차 값 계산 및 속도 명령 생성
        distance_error = msg.distance - self.target_distance
        angle_error = msg.horizontal_offset
        cmd_msg = Twist()
        cmd_msg.linear.x = self.distance_pid.update(distance_error)
        cmd_msg.angular.z = -self.angle_pid.update(angle_error) # 화면-로봇 좌표계 변환을 위해 부호 반전
        self.cmd_vel_pub.publish(cmd_msg)
    
    # --- 7. 상태 전이 콜백 함수 (타이머에 의해 호출) ---
    def transition_to_avoid_turn(self):
        self.get_logger().info("후진 완료. 회전 시작.")
        self.state = "AVOIDING_TURN"
        if self.avoidance_timer: self.avoidance_timer.cancel(); self.avoidance_timer = None

    def transition_to_avoid_straight(self):
        self.get_logger().info("회전 완료. 회피 직진 시작.")
        self.state = "AVOIDING_STRAIGHT"
        if self.avoidance_timer: self.avoidance_timer.cancel(); self.avoidance_timer = None

    def transition_to_following(self):
        self.get_logger().info("회피 기동 완료. 추종 모드로 복귀.")
        self.stop_robot() # 바로 추종을 시작하지 않고, 일단 정지하여 상황을 다시 판단하도록 함

    def stop_robot(self):
        """로봇을 정지시키고 모든 관련 상태를 초기화하는 가장 중요한 안전 및 리셋 함수."""
        self.get_logger().info('🛑 로봇 정지 및 상태 초기화.')
        self.is_following = False
        # 진행 중이던 회피 타이머가 있다면 즉시 취소
        if self.avoidance_timer: self.avoidance_timer.cancel(); self.avoidance_timer = None
        # FSM 상태를 기본값인 'FOLLOWING'으로 리셋
        self.state = "FOLLOWING"
        self.avoidance_turn_direction = None
        # 정지 명령 발행
        self.cmd_vel_pub.publish(Twist())
        # PID 제어기 내부 누적값 초기화
        self.distance_pid.reset()
        self.angle_pid.reset()
        # [주의] 이 함수는 is_paused_by_voice 상태는 초기화하지 않음.
        # 음성 명령에 의한 '일시정지'는 명시적인 '재개' 명령으로만 해제되어야 하기 때문.

# --- 8. 메인 실행 부분 ---
def main(args=None):
    rclpy.init(args=args)
    node = None
    # try...finally 구문을 사용하여 어떤 상황에서든 노드가 종료될 때 로봇이 안전하게 정지하도록 보장
    try:
        node = AdvancedAssistFollowFSM()
        rclpy.spin(node) # 노드를 계속 실행하며 콜백 함수들을 처리
    except KeyboardInterrupt: # Ctrl+C 입력 시
        if node: node.get_logger().info('키보드 인터럽트로 노드를 종료합니다.')
    finally:
        if node and rclpy.ok():
            node.get_logger().info('안전한 종료를 위해 로봇을 정지합니다.')
            node.stop_robot() # 로봇 정지
            node.destroy_node() # 노드 소멸
        rclpy.shutdown()

if __name__ == '__main__':
    main()