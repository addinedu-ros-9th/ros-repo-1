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
        # [수정] Foxy/Galactic 호환을 위해 'description' 인자 제거
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('safe_distance_min', 0.3)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('avoidance_linear_vel', 0.1)
        self.declare_parameter('avoidance_angular_vel', 0.2)
        self.declare_parameter('avoidance_backup_vel', -0.1)
        self.declare_parameter('avoidance_backup_duration_s', 1.5)
        self.declare_parameter('avoidance_turn_duration_s', 1.0)
        self.declare_parameter('avoidance_straight_duration_s', 1.5)
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
        self.distance_pid = PIDController(
            kp=self.get_parameter('dist_kp').get_parameter_value().double_value, ki=self.get_parameter('dist_ki').get_parameter_value().double_value, kd=self.get_parameter('dist_kd').get_parameter_value().double_value,
            max_output=self.get_parameter('max_linear_vel').get_parameter_value().double_value, min_output=-self.get_parameter('max_linear_vel').get_parameter_value().double_value)
        self.angle_pid = PIDController(
            kp=self.get_parameter('angle_kp').get_parameter_value().double_value, ki=self.get_parameter('angle_ki').get_parameter_value().double_value, kd=self.get_parameter('angle_kd').get_parameter_value().double_value,
            max_output=self.get_parameter('max_angular_vel').get_parameter_value().double_value, min_output=-self.get_parameter('max_angular_vel').get_parameter_value().double_value)

        # --- 3. 상태 변수 및 통신 인터페이스 초기화 ---
        self.qr_authenticated = False
        self.is_following = False
        self.is_paused_by_voice = False
        self.obstacle_status = None
        self.state = "FOLLOWING"
        self.avoidance_timer = None
        self.avoidance_turn_direction = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.human_info_sub = self.create_subscription(HumanInfo, '/human_info', self.human_info_callback, 10)
        self.detection_status_sub = self.create_subscription(DetectionStatus, '/detection_status', self.detection_status_callback, 10)
        self.talk_command_sub = self.create_subscription(TalkCommand, '/talk_command', self.talk_command_callback, 10)
        self.activate_srv = self.create_service(ActivateTracker, '/activate_tracker', self.handle_activate_tracker)
        self.deactivate_srv = self.create_service(DeactivateTracker, '/deactivate_tracker', self.handle_deactivate_tracker)

        self.get_logger().info('✅ Advanced Assist Follow FSM 노드 시작 완료 (Foxy/Galactic 호환)')

    # --- 서비스 핸들러 및 콜백 함수들 (이하 변경 없음) ---
    def handle_activate_tracker(self, request, response):
        self.get_logger().info(f"🟢 Activate 요청 수신 - robot_id: {request.robot_id}")
        self.qr_authenticated = True
        response.success = True
        response.message = "Assist mode activated with advanced FSM."
        return response

    def handle_deactivate_tracker(self, request, response):
        self.get_logger().info(f"🔴 Deactivate 요청 수신 - robot_id: {request.robot_id}")
        self.qr_authenticated = False
        self.stop_robot()
        response.success = True
        response.message = "Assist mode deactivated."
        return response

    def detection_status_callback(self, msg: DetectionStatus):
        self.obstacle_status = msg

    def talk_command_callback(self, msg: TalkCommand):
        if not self.qr_authenticated or msg.robot_id != "libo_a":
            return

        if msg.action == "stop":
            if not self.is_paused_by_voice:
                self.get_logger().info("🎤 음성 명령으로 추종을 일시 중지합니다.")
                self.is_paused_by_voice = True
                self.stop_robot()
        
        elif msg.action == "follow":
            if self.is_paused_by_voice:
                self.get_logger().info("🎤 음성 명령으로 추종을 재개합니다.")
                self.is_paused_by_voice = False
        
    def human_info_callback(self, msg: HumanInfo):
        if not self.qr_authenticated: return
        if self.obstacle_status is None:
            self.get_logger().info('장애물 감지 정보 수신 대기 중...', once=True)
            return

        if self.obstacle_status.center_detected or \
           (self.obstacle_status.left_detected and self.obstacle_status.right_detected):
            self.get_logger().warn('🚨 전방 또는 양측 장애물 동시 감지! 비상 정지!', throttle_duration_sec=1)
            self.stop_robot()
            return

        if self.state == "FOLLOWING":
            if self.obstacle_status.left_detected:
                self.get_logger().info("좌측 장애물 감지. 회피 기동(후진->우회전) 시작.")
                self.state = "AVOIDING_BACKUP"
                self.avoidance_turn_direction = 'RIGHT'
            elif self.obstacle_status.right_detected:
                self.get_logger().info("우측 장애물 감지. 회피 기동(후진->좌회전) 시작.")
                self.state = "AVOIDING_BACKUP"
                self.avoidance_turn_direction = 'LEFT'
            else:
                self.perform_following_with_pid(msg)
                return

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

    def perform_following_with_pid(self, msg: HumanInfo):
        if self.is_paused_by_voice:
            return
        if not msg.is_detected:
            if self.is_following:
                self.get_logger().warn('사람을 놓쳤습니다. 즉시 정지합니다.')
                self.stop_robot()
            return
        self.is_following = True
        if msg.distance < self.safe_distance_min:
            self.get_logger().warn(f'사람이 너무 가까움! ({msg.distance:.2f}m) 정지!', throttle_duration_sec=1)
            self.stop_robot()
            return
        distance_error = msg.distance - self.target_distance
        angle_error = msg.horizontal_offset
        cmd_msg = Twist()
        cmd_msg.linear.x = self.distance_pid.update(distance_error)
        cmd_msg.angular.z = -self.angle_pid.update(angle_error)
        self.cmd_vel_pub.publish(cmd_msg)
    
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
        self.stop_robot() 

    def stop_robot(self):
        self.get_logger().info('🛑 로봇 정지 및 상태 초기화.')
        self.is_following = False
        if self.avoidance_timer:
            self.avoidance_timer.cancel()
            self.avoidance_timer = None
        self.state = "FOLLOWING"
        self.avoidance_turn_direction = None
        self.cmd_vel_pub.publish(Twist())
        self.distance_pid.reset()
        self.angle_pid.reset()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AdvancedAssistFollowFSM()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info('키보드 인터럽트로 노드를 종료합니다.')
    finally:
        if node and rclpy.ok():
            node.get_logger().info('안전한 종료를 위해 로봇을 정지합니다.')
            node.stop_robot()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()