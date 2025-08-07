#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from libo_interfaces.msg import HumanInfo, DetectionStatus
from libo_interfaces.srv import ActivateTracker, DeactivateTracker
from libo_interfaces.msg import TalkCommand, VoiceCommand
from std_srvs.srv import Trigger

class PIDController:
    """
    PID(Proportional-Integral-Derivative) 제어기 클래스.
    """
    def __init__(self, kp, ki, kd, max_output=None, min_output=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_output, self.min_output = max_output, min_output
        self.previous_error, self.integral = 0.0, 0.0

    def update(self, error, dt=0.1):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        if self.max_output is not None: output = min(output, self.max_output)
        if self.min_output is not None: output = max(output, self.min_output)
        self.previous_error = error
        return output

    def reset(self):
        self.previous_error, self.integral = 0.0, 0.0
    
    def set_output_limits(self, min_output, max_output):
        self.min_output = min_output
        self.max_output = max_output

class AdvancedAssistFollowFSM(Node):
    """
    [최종 수정] 로봇 특성을 고려한 '후진 선회' 회피 로직 적용 FSM 노드.
    """
    def __init__(self):
        super().__init__('advanced_assist_follow_fsm_revised')

        # --- 1. ROS 파라미터 선언 ---
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('safe_distance_min', 1.0)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('avoidance_max_angular_vel', 0.1)
        self.declare_parameter('following_max_angular_vel', 0.3)
        self.declare_parameter('avoidance_linear_vel', 0.1)
        self.declare_parameter('avoidance_angular_vel', 0.2)
        self.declare_parameter('avoidance_backup_vel', -0.1)
        # [수정] 회피 기동 시간을 단일 파라미터로 통합
        self.declare_parameter('avoidance_reverse_turn_duration_s', 1.0) 
        self.declare_parameter('avoidance_straight_duration_s', 1.5)
        self.declare_parameter('dist_kp', 1.0); self.declare_parameter('dist_ki', 0.0); self.declare_parameter('dist_kd', 0.1)
        self.declare_parameter('angle_kp', 1.5); self.declare_parameter('angle_ki', 0.0); self.declare_parameter('angle_kd', 0.2)
        
        # 파라미터 값 가져오기
        self.target_distance = self.get_parameter('target_distance').value
        self.safe_distance_min = self.get_parameter('safe_distance_min').value
        self.avoidance_linear_vel = self.get_parameter('avoidance_linear_vel').value
        self.avoidance_angular_vel = self.get_parameter('avoidance_angular_vel').value
        self.avoidance_backup_vel = self.get_parameter('avoidance_backup_vel').value
        # [수정] 새로운 시간 파라미터 사용
        self.avoidance_reverse_turn_duration = self.get_parameter('avoidance_reverse_turn_duration_s').value
        self.avoidance_straight_duration = self.get_parameter('avoidance_straight_duration_s').value
        self.following_max_angular_vel = self.get_parameter('following_max_angular_vel').value
        self.avoidance_max_angular_vel = self.get_parameter('avoidance_max_angular_vel').value

        # --- 2. PID 제어기 생성 ---
        self.distance_pid = PIDController(
            kp=self.get_parameter('dist_kp').value, ki=self.get_parameter('dist_ki').value, kd=self.get_parameter('dist_kd').value,
            max_output=self.get_parameter('max_linear_vel').value, min_output=-self.get_parameter('max_linear_vel').value)
        self.angle_pid = PIDController(
            kp=self.get_parameter('angle_kp').value, ki=self.get_parameter('angle_ki').value, kd=self.get_parameter('angle_kd').value,
            max_output=self.avoidance_max_angular_vel, min_output=-self.avoidance_max_angular_vel)

        # --- 3. 상태 변수 및 통신 인터페이스 초기화 ---
        self.qr_authenticated = False
        self.is_following = False
        self.is_paused_by_voice = False
        self.obstacle_status = None
        self.state = "FOLLOWING"
        self.avoidance_timer = None
        self.avoidance_turn_direction = None
        self.honk_played = False
        self.last_cmd_vel = Twist()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_cmd_pub = self.create_publisher(VoiceCommand, '/voice_command', 10)
        
        self.human_info_sub = self.create_subscription(HumanInfo, '/human_info', self.human_info_callback, 10)
        self.detection_status_sub = self.create_subscription(DetectionStatus, '/detection_status', self.detection_status_callback, 10)
        self.talk_command_sub = self.create_subscription(TalkCommand, '/talk_command', self.talk_command_callback, 10)
        
        self.activate_srv = self.create_service(ActivateTracker, '/activate_tracker', self.handle_activate_tracker)
        self.deactivate_srv = self.create_service(DeactivateTracker, '/deactivate_tracker', self.handle_deactivate_tracker)
        self.arrived_srv = self.create_service(Trigger, '/trigger_arrival', self.handle_arrival_trigger)

        self.get_logger().info('✅ Advanced Assist Follow FSM 노드 시작 완료 (후진 선회 로직 적용됨)')

    def gradual_stop(self, duration=0.5, steps=20):
        initial_vel = self.last_cmd_vel
        self.get_logger().info('...부드럽게 감속합니다...')
        for i in range(steps + 1):
            ratio = 1.0 - (i / steps)
            cmd_msg = Twist()
            cmd_msg.linear.x = initial_vel.linear.x * ratio
            cmd_msg.angular.z = initial_vel.angular.z * ratio
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(duration / steps)
        self.last_cmd_vel = Twist()

    def send_voice_command(self, category, action):
        msg = VoiceCommand()
        msg.robot_id = "libo_a"
        msg.category = category
        msg.action = action
        self.voice_cmd_pub.publish(msg)
        self.get_logger().info(f'📢 음성 명령 전송: category="{category}", action="{action}"')

    def handle_arrival_trigger(self, request, response):
        self.get_logger().info('📍 목적지 도착! 안내 음성을 송출하고 모든 동작을 중지합니다.')
        self.send_voice_command("escort", "arrived")
        self.qr_authenticated = False
        self.stop_robot()
        response.success = True
        response.message = "Arrival sequence triggered."
        return response

    def handle_activate_tracker(self, request, response):
        self.get_logger().info(f"🟢 Activate 요청 수신 - robot_id: {request.robot_id}")
        self.qr_authenticated = True
        response.success = True
        response.message = "Assist mode activated."
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
        if not self.qr_authenticated or self.is_paused_by_voice: return
        if self.obstacle_status is None:
            self.get_logger().info('장애물 감지 정보 수신 대기 중...', once=True)
            return

        if self.obstacle_status.center_detected or \
           (self.obstacle_status.left_detected and self.obstacle_status.right_detected):
            if not self.honk_played:
                self.send_voice_command("common", "obstacle_detected")
                self.honk_played = True
            self.get_logger().warn('🚨 전방 또는 양측 장애물 동시 감지! 비상 정지!', throttle_duration_sec=1)
            self.stop_robot()
            return

        # --- [수정] 회피 로직 전체 수정 ---
        if self.state == "FOLLOWING":
            if self.obstacle_status.left_detected or self.obstacle_status.right_detected:
                self.get_logger().info("장애물 감지! 회전 속도를 안전 모드로 변경합니다.")
                self.angle_pid.set_output_limits(-self.avoidance_max_angular_vel, self.avoidance_max_angular_vel)
                
                if self.obstacle_status.left_detected:
                    if not self.honk_played: self.send_voice_command("common", "obstacle_detected"); self.honk_played = True
                    self.get_logger().info("좌측 장애물 감지. 후진 선회 기동(우회전) 시작.")
                    self.state = "AVOIDING_REVERSE_TURN"
                    self.avoidance_turn_direction = 'RIGHT'
                else: # self.obstacle_status.right_detected
                    if not self.honk_played: self.send_voice_command("common", "obstacle_detected"); self.honk_played = True
                    self.get_logger().info("우측 장애물 감지. 후진 선회 기동(좌회전) 시작.")
                    self.state = "AVOIDING_REVERSE_TURN"
                    self.avoidance_turn_direction = 'LEFT'
            else:
                self.perform_following_with_pid(msg)
            return

        cmd_msg = Twist()
        # [신규] 후진과 회전을 동시에 수행하는 상태
        if self.state == "AVOIDING_REVERSE_TURN":
            cmd_msg.linear.x = self.avoidance_backup_vel # 후진
            cmd_msg.angular.z = self.avoidance_angular_vel if self.avoidance_turn_direction == 'LEFT' else -self.avoidance_angular_vel # 회전
            
            if self.avoidance_timer is None:
                self.avoidance_timer = self.create_timer(self.avoidance_reverse_turn_duration, self.transition_to_avoid_straight)

        # [유지] 회피 기동 후 직진 상태
        elif self.state == "AVOIDING_STRAIGHT":
            cmd_msg.linear.x = self.avoidance_linear_vel
            if self.avoidance_timer is None: 
                self.avoidance_timer = self.create_timer(self.avoidance_straight_duration, self.transition_to_following)
        
        self.cmd_vel_pub.publish(cmd_msg)
        self.last_cmd_vel = cmd_msg

    def perform_following_with_pid(self, msg: HumanInfo):
        if self.honk_played:
            self.get_logger().info("장애물 없음. 경고음 상태를 리셋합니다.")
            self.honk_played = False
        
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

        self.angle_pid.set_output_limits(-self.following_max_angular_vel, self.following_max_angular_vel)
        
        distance_error = msg.distance - self.target_distance
        angle_error = msg.horizontal_offset
        
        cmd_msg = Twist()
        cmd_msg.linear.x = self.distance_pid.update(distance_error)
        cmd_msg.angular.z = -self.angle_pid.update(angle_error)
        
        self.cmd_vel_pub.publish(cmd_msg)
        self.last_cmd_vel = cmd_msg
    
    # [삭제] 불필요해진 상태 전환 함수
    # def transition_to_avoid_turn(self):
    #     ...

    def transition_to_avoid_straight(self):
        self.get_logger().info("후진 선회 완료. 회피 직진 시작.")
        self.state = "AVOIDING_STRAIGHT"
        if self.avoidance_timer: self.avoidance_timer.cancel(); self.avoidance_timer = None

    def transition_to_following(self):
        self.get_logger().info("회피 기동 완료. 추종 모드로 복귀.")
        self.stop_robot() # 추종 모드 복귀 전 부드럽게 정지
        self.state = "FOLLOWING"
        self.distance_pid.reset()
        self.angle_pid.reset()

    def stop_robot(self):
        self.get_logger().info('🛑 로봇 정지 절차 시작.')
        self.is_following = False
        if self.avoidance_timer:
            self.avoidance_timer.cancel()
            self.avoidance_timer = None
        
        if abs(self.last_cmd_vel.linear.x) > 0.01 or abs(self.last_cmd_vel.angular.z) > 0.01:
            self.gradual_stop()
        else:
            self.cmd_vel_pub.publish(Twist())
        
        # [수정] 정지 시 상태는 FOLLOWING으로 초기화하되, PID 리셋은 복귀 시에만 수행하도록 변경
        # self.state = "FOLLOWING" # 이 부분은 transition_to_following에서 관리하는 것이 더 명확함
        # self.avoidance_turn_direction = None
        # self.distance_pid.reset()
        # self.angle_pid.reset()


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
            # stop_robot()은 마지막 속도를 기반으로 감속하므로, destroy 전에 호출
            node.get_logger().info('안전한 종료를 위해 로봇을 정지합니다.')
            node.stop_robot() 
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()