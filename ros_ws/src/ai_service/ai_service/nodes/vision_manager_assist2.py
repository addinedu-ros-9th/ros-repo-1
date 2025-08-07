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
    [최종 수정] 로봇 특성을 고려한 '후진 선회' 회피 로직 및 '사용자 재탐색' 로직 적용 FSM 노드.
    """
    def __init__(self):
        super().__init__('advanced_assist_follow_fsm_revised_with_search')

        # --- 1. ROS 파라미터 선언 ---
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('safe_distance_min', 1.0)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('avoidance_max_angular_vel', 0.1)
        self.declare_parameter('following_max_angular_vel', 0.3)
        self.declare_parameter('search_angular_vel', 0.4) # [신규] 탐색 시 회전 속도
        self.declare_parameter('avoidance_linear_vel', 0.1)
        self.declare_parameter('avoidance_angular_vel', 0.2)
        self.declare_parameter('avoidance_backup_vel', -0.1)
        self.declare_parameter('avoidance_reverse_turn_duration_s', 1.0) 
        self.declare_parameter('avoidance_straight_duration_s', 1.5)
        self.declare_parameter('search_timeout_s', 5.0) # [신규] 탐색 시간 초과
        self.declare_parameter('dist_kp', 1.0); self.declare_parameter('dist_ki', 0.0); self.declare_parameter('dist_kd', 0.1)
        self.declare_parameter('angle_kp', 1.5); self.declare_parameter('angle_ki', 0.0); self.declare_parameter('angle_kd', 0.2)
        
        # 파라미터 값 가져오기
        self.target_distance = self.get_parameter('target_distance').value
        self.safe_distance_min = self.get_parameter('safe_distance_min').value
        self.avoidance_linear_vel = self.get_parameter('avoidance_linear_vel').value
        self.avoidance_angular_vel = self.get_parameter('avoidance_angular_vel').value
        self.avoidance_backup_vel = self.get_parameter('avoidance_backup_vel').value
        self.avoidance_reverse_turn_duration = self.get_parameter('avoidance_reverse_turn_duration_s').value
        self.avoidance_straight_duration = self.get_parameter('avoidance_straight_duration_s').value
        self.following_max_angular_vel = self.get_parameter('following_max_angular_vel').value
        self.avoidance_max_angular_vel = self.get_parameter('avoidance_max_angular_vel').value
        self.search_angular_vel = self.get_parameter('search_angular_vel').value # [신규]
        self.search_timeout = self.get_parameter('search_timeout_s').value # [신규]

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
        self.state = "FOLLOWING"  # [수정] 상태: FOLLOWING, AVOIDING_REVERSE_TURN, AVOIDING_STRAIGHT, SEARCHING
        self.avoidance_timer = None
        self.search_timer = None # [신규] 탐색 타임아웃을 위한 타이머
        self.avoidance_turn_direction = None
        self.last_known_angle_error = 0.0 # [신규] 마지막으로 감지된 사용자의 각도 오차
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

        self.get_logger().info('✅ Advanced Assist Follow FSM 노드 시작 완료 (후진 선회 및 재탐색 로직 적용됨)')

    def gradual_stop(self, duration=0.5, steps=20):
        # ... (기존과 동일)
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
        # ... (기존과 동일)
        msg = VoiceCommand()
        msg.robot_id = "libo_a"
        msg.category = category
        msg.action = action
        self.voice_cmd_pub.publish(msg)
        self.get_logger().info(f'📢 음성 명령 전송: category="{category}", action="{action}"')

    def handle_arrival_trigger(self, request, response):
        # ... (기존과 동일)
        self.get_logger().info('📍 목적지 도착! 안내 음성을 송출하고 모든 동작을 중지합니다.')
        self.send_voice_command("escort", "arrived")
        self.qr_authenticated = False
        self.stop_robot()
        response.success = True
        response.message = "Arrival sequence triggered."
        return response

    def handle_activate_tracker(self, request, response):
        # ... (기존과 동일)
        self.get_logger().info(f"🟢 Activate 요청 수신 - robot_id: {request.robot_id}")
        self.qr_authenticated = True
        response.success = True
        response.message = "Assist mode activated."
        return response

    def handle_deactivate_tracker(self, request, response):
        # ... (기존과 동일)
        self.get_logger().info(f"🔴 Deactivate 요청 수신 - robot_id: {request.robot_id}")
        self.qr_authenticated = False
        self.stop_robot()
        response.success = True
        response.message = "Assist mode deactivated."
        return response

    def detection_status_callback(self, msg: DetectionStatus):
        self.obstacle_status = msg

    def talk_command_callback(self, msg: TalkCommand):
        # ... (기존과 동일)
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
        
        # [신규] 탐색 상태에서는 장애물 감지 로직을 건너뛰고 사용자 감지 여부만 확인
        if self.state == "SEARCHING":
            self.perform_searching(msg)
            return

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
        
        cmd_msg = Twist()
        if self.state == "FOLLOWING":
            if self.obstacle_status.left_detected or self.obstacle_status.right_detected:
                self.get_logger().info("장애물 감지! 회전 속도를 안전 모드로 변경합니다.")
                self.angle_pid.set_output_limits(-self.avoidance_max_angular_vel, self.avoidance_max_angular_vel)
                
                # [신규] 회피 기동 시작 전, 마지막 사용자 각도 저장
                self.last_known_angle_error = msg.horizontal_offset

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

        elif self.state == "AVOIDING_REVERSE_TURN":
            cmd_msg.linear.x = self.avoidance_backup_vel
            cmd_msg.angular.z = self.avoidance_angular_vel if self.avoidance_turn_direction == 'LEFT' else -self.avoidance_angular_vel
            if self.avoidance_timer is None:
                self.avoidance_timer = self.create_timer(self.avoidance_reverse_turn_duration, self.transition_to_avoid_straight)

        elif self.state == "AVOIDING_STRAIGHT":
            cmd_msg.linear.x = self.avoidance_linear_vel
            if self.avoidance_timer is None: 
                # [수정] 회피 직진이 끝나면 추종이 아닌 탐색 상태로 전환
                self.avoidance_timer = self.create_timer(self.avoidance_straight_duration, self.transition_to_searching)
        
        self.cmd_vel_pub.publish(cmd_msg)
        self.last_cmd_vel = cmd_msg

    def perform_following_with_pid(self, msg: HumanInfo):
        # ... (기존과 동일)
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

    def perform_searching(self, msg: HumanInfo):
        """
        [신규] 사용자를 재탐색하기 위해 마지막으로 감지된 방향으로 회전하는 함수.
        """
        # 탐색 중 사용자가 다시 감지되면, 탐색을 중단하고 추종 모드로 복귀
        if msg.is_detected:
            self.get_logger().info("사용자 재탐지 성공! 추종 모드로 복귀합니다.")
            if self.search_timer:
                self.search_timer.cancel()
                self.search_timer = None
            self.transition_to_following()
            return

        # 사용자가 감지되지 않으면, 마지막 위치를 기반으로 회전
        cmd_msg = Twist()
        # last_known_angle_error > 0 이면 사용자가 오른쪽에 있었으므로, 오른쪽으로 회전 (음수 각속도)
        # last_known_angle_error < 0 이면 사용자가 왼쪽에 있었으므로, 왼쪽으로 회전 (양수 각속도)
        if self.last_known_angle_error > 0.05: # 오른쪽으로 회전
            cmd_msg.angular.z = -self.search_angular_vel
        elif self.last_known_angle_error < -0.05: # 왼쪽으로 회전
            cmd_msg.angular.z = self.search_angular_vel
        else: # 중앙 근처에 있었으면 회전하지 않고 정지
            cmd_msg.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_msg)
        self.last_cmd_vel = cmd_msg

    def transition_to_avoid_straight(self):
        self.get_logger().info("후진 선회 완료. 회피 직진 시작.")
        self.state = "AVOIDING_STRAIGHT"
        if self.avoidance_timer: self.avoidance_timer.cancel(); self.avoidance_timer = None

    def transition_to_searching(self):
        """
        [신규] 회피 기동 완료 후 탐색 상태로 전환하는 함수.
        """
        self.get_logger().info("회피 기동 완료. 사용자 탐색 모드로 전환합니다.")
        self.stop_robot() # 탐색 전 잠시 정지
        self.state = "SEARCHING"
        if self.avoidance_timer: self.avoidance_timer.cancel(); self.avoidance_timer = None
        
        # 탐색 시간 초과 타이머 설정
        if self.search_timer: self.search_timer.cancel()
        self.search_timer = self.create_timer(self.search_timeout, self.handle_search_timeout)

    def handle_search_timeout(self):
        """
        [신규] 탐색 시간이 초과되면 호출되는 콜백 함수.
        """
        if self.state == "SEARCHING":
            self.get_logger().warn(f"{self.search_timeout}초 동안 사용자를 찾지 못했습니다. 탐색을 중단하고 정지합니다.")
            if self.search_timer: self.search_timer.cancel(); self.search_timer = None
            self.stop_robot()
            # 정지 후에는 다시 추종 대기 상태로 돌아감
            self.state = "FOLLOWING"

    def transition_to_following(self):
        self.get_logger().info("추종 모드로 복귀합니다.")
        self.stop_robot() # 추종 모드 복귀 전 부드럽게 정지
        self.state = "FOLLOWING"
        self.distance_pid.reset()
        self.angle_pid.reset()

    def stop_robot(self):
        self.get_logger().info('🛑 로봇 정지 절차 시작.')
        self.is_following = False
        
        # [수정] 모든 타이머(회피, 탐색)를 정지 시 비활성화
        if self.avoidance_timer:
            self.avoidance_timer.cancel()
            self.avoidance_timer = None
        if self.search_timer:
            self.search_timer.cancel()
            self.search_timer = None
        
        if abs(self.last_cmd_vel.linear.x) > 0.01 or abs(self.last_cmd_vel.angular.z) > 0.01:
            self.gradual_stop()
        else:
            self.cmd_vel_pub.publish(Twist())

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