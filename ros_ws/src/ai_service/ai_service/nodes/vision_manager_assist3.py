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
    [최종 수정] 전륜 구동 특성을 반영한 회피 로직 적용 FSM 노드.
    [v4] 후진 속도 증가 및 3차 회전 시간 증대를 통해 회피 기동을 조정한 버전.
    """
    def __init__(self):
        super().__init__('advanced_assist_follow_fsm_final_fwd_v4')

        # --- 1. ROS 파라미터 선언 ---
        self.declare_parameter('avoidance_turn_first_duration_s', 1.0)
        self.declare_parameter('avoidance_backup_duration_s', 1.5)
        self.declare_parameter('avoidance_forward_duration_s', 1.2)
        self.declare_parameter('avoidance_counter_turn_duration_s', 1.0)
        # [수정] 3차 회전 시간을 2차 회전보다 길게 설정
        self.declare_parameter('avoidance_realign_turn_duration_s', 2.0) 

        self.declare_parameter('avoidance_forward_vel', 0.15)
        self.declare_parameter('search_turn_duration_s', 0.7)
        self.declare_parameter('search_pause_duration_s', 0.5)
        self.declare_parameter('search_timeout_s', 8.0)
        self.declare_parameter('avoidance_turn_vel', 0.3)
        # [수정] 후진 속도 증가
        self.declare_parameter('avoidance_backup_vel', -0.15)
        self.declare_parameter('search_angular_vel', 0.4)
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('safe_distance_min', 1.0)
        self.declare_parameter('max_linear_vel', 0.25)
        self.declare_parameter('following_max_angular_vel', 0.35)
        self.declare_parameter('dist_kp', 1.0); self.declare_parameter('dist_ki', 0.0); self.declare_parameter('dist_kd', 0.1)
        self.declare_parameter('angle_kp', 1.5); self.declare_parameter('angle_ki', 0.0); self.declare_parameter('angle_kd', 0.2)
        
        # 파라미터 값 가져오기
        self.avoidance_turn_first_duration = self.get_parameter('avoidance_turn_first_duration_s').value
        self.avoidance_backup_duration = self.get_parameter('avoidance_backup_duration_s').value
        self.avoidance_forward_duration = self.get_parameter('avoidance_forward_duration_s').value
        self.avoidance_counter_turn_duration = self.get_parameter('avoidance_counter_turn_duration_s').value
        self.avoidance_realign_turn_duration = self.get_parameter('avoidance_realign_turn_duration_s').value
        
        self.avoidance_forward_vel = self.get_parameter('avoidance_forward_vel').value
        self.search_turn_duration = self.get_parameter('search_turn_duration_s').value
        self.search_pause_duration = self.get_parameter('search_pause_duration_s').value
        self.search_timeout = self.get_parameter('search_timeout_s').value
        self.avoidance_turn_vel = self.get_parameter('avoidance_turn_vel').value
        self.avoidance_backup_vel = self.get_parameter('avoidance_backup_vel').value
        self.search_angular_vel = self.get_parameter('search_angular_vel').value
        self.target_distance = self.get_parameter('target_distance').value
        self.safe_distance_min = self.get_parameter('safe_distance_min').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.following_max_angular_vel = self.get_parameter('following_max_angular_vel').value

        # --- 2. PID 제어기 생성 ---
        self.distance_pid = PIDController(
            kp=self.get_parameter('dist_kp').value, ki=self.get_parameter('dist_ki').value, kd=self.get_parameter('dist_kd').value,
            max_output=self.max_linear_vel, min_output=-self.max_linear_vel)
        self.angle_pid = PIDController(
            kp=self.get_parameter('angle_kp').value, ki=self.get_parameter('angle_ki').value, kd=self.get_parameter('angle_kd').value,
            max_output=self.following_max_angular_vel, min_output=-self.following_max_angular_vel)

        # --- 3. 상태 변수 및 통신 인터페이스 초기화 ---
        self.qr_authenticated = False
        self.is_paused_by_voice = False
        self.obstacle_status = None
        self.state = "FOLLOWING"
        self.is_following = False
        self.state_timer = None
        self.search_timeout_timer = None 
        self.avoidance_turn_direction = 1 
        self.last_known_angle_error = 0.0
        self.honk_played = False
        self.last_cmd_vel = Twist()

        # ROS 인터페이스
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_cmd_pub = self.create_publisher(VoiceCommand, '/voice_command', 10)
        self.human_info_sub = self.create_subscription(HumanInfo, '/human_info', self.human_info_callback, 10)
        self.detection_status_sub = self.create_subscription(DetectionStatus, '/detection_status', self.detection_status_callback, 10)
        self.talk_command_sub = self.create_subscription(TalkCommand, '/talk_command', self.talk_command_callback, 10)
        self.activate_srv = self.create_service(ActivateTracker, '/activate_tracker', self.handle_activate_tracker)
        self.deactivate_srv = self.create_service(DeactivateTracker, '/deactivate_tracker', self.handle_deactivate_tracker)
        self.arrived_srv = self.create_service(Trigger, '/trigger_arrival', self.handle_arrival_trigger)

        self.get_logger().info('✅ Advanced Assist Follow FSM 노드 시작 완료 (v4: 회피 기동 조정)')


    def human_info_callback(self, msg: HumanInfo):
        if not self.qr_authenticated or self.is_paused_by_voice: return
        if self.obstacle_status is None:
            self.get_logger().info('장애물 감지 정보 수신 대기 중...', once=True)
            return

        if self.obstacle_status.center_detected or \
           (self.obstacle_status.left_detected and self.obstacle_status.right_detected) or \
           (msg.is_detected and msg.distance < self.safe_distance_min):
            if self.state != "FOLLOWING" or self.is_following:
                 self.get_logger().warn('🚨 비상 정지 조건 충족! 모든 동작을 즉시 중지합니다!', throttle_duration_sec=1)
                 self.transition_to_following(stop_first=True)
            return

        cmd_msg = Twist()
        
        if self.state == "FOLLOWING":
            if self.obstacle_status.left_detected or self.obstacle_status.right_detected:
                if not self.honk_played: self.send_voice_command("common", "obstacle_detected"); self.honk_played = True
                self.last_known_angle_error = msg.horizontal_offset
                self.avoidance_turn_direction = 1 if self.obstacle_status.left_detected else -1
                self.transition_to_state("AVOIDING_TURN_FIRST")
            else:
                self.perform_following_with_pid(msg)
            return

        elif self.state == "AVOIDING_TURN_FIRST":
            cmd_msg.angular.z = self.avoidance_turn_vel * self.avoidance_turn_direction
        
        elif self.state == "AVOIDING_BACKUP":
            cmd_msg.linear.x = self.avoidance_backup_vel

        elif self.state == "AVOIDING_COUNTER_TURN":
            cmd_msg.angular.z = self.avoidance_turn_vel * -self.avoidance_turn_direction

        elif self.state == "AVOIDING_FORWARD":
            cmd_msg.linear.x = self.avoidance_forward_vel
        
        elif self.state == "AVOIDING_REALIGN_TURN":
            cmd_msg.angular.z = self.avoidance_turn_vel * self.avoidance_turn_direction

        elif self.state == "SEARCHING_TURN":
            turn_dir = 1 if self.last_known_angle_error < 0 else -1
            cmd_msg.angular.z = self.search_angular_vel * turn_dir

        elif self.state == "SEARCHING_PAUSE":
            if msg.is_detected:
                self.get_logger().info("🎉 사용자 재탐지 성공! 추종 모드로 복귀합니다.")
                self.transition_to_following(stop_first=True)
                return
        
        if self.state not in ["FOLLOWING"]:
            self.cmd_vel_pub.publish(cmd_msg)
            self.last_cmd_vel = cmd_msg

    def perform_following_with_pid(self, msg: HumanInfo):
        if self.honk_played:
            self.get_logger().info("장애물 없음. 경고음 상태를 리셋합니다.")
            self.honk_played = False
        
        if not msg.is_detected:
            if self.is_following:
                self.get_logger().warn('사람을 놓쳤습니다. 탐색을 시작합니다.')
                self.last_known_angle_error = self.angle_pid.previous_error
                self.transition_to_state("SEARCHING_TURN")
            return
        
        self.is_following = True
        distance_error = msg.distance - self.target_distance
        angle_error = msg.horizontal_offset
        
        cmd_msg = Twist()
        cmd_msg.linear.x = self.distance_pid.update(distance_error)
        cmd_msg.angular.z = -self.angle_pid.update(angle_error)
        
        self.cmd_vel_pub.publish(cmd_msg)
        self.last_cmd_vel = cmd_msg
    
    def transition_to_state(self, new_state):
        self.get_logger().info(f"상태 변경: {self.state} -> {new_state}")
        self.state = new_state
        
        if self.state_timer:
            self.state_timer.cancel()
            self.state_timer = None

        if new_state == "AVOIDING_TURN_FIRST":
            self.stop_robot(gradual=False)
            self.state_timer = self.create_timer(self.avoidance_turn_first_duration, lambda: self.transition_to_state("AVOIDING_BACKUP"))
        
        elif new_state == "AVOIDING_BACKUP":
            self.stop_robot(gradual=False)
            self.state_timer = self.create_timer(self.avoidance_backup_duration, lambda: self.transition_to_state("AVOIDING_COUNTER_TURN"))
        
        elif new_state == "AVOIDING_COUNTER_TURN":
            self.stop_robot(gradual=False)
            self.state_timer = self.create_timer(self.avoidance_counter_turn_duration, lambda: self.transition_to_state("AVOIDING_FORWARD"))

        elif new_state == "AVOIDING_FORWARD":
            self.stop_robot(gradual=False)
            self.state_timer = self.create_timer(self.avoidance_forward_duration, lambda: self.transition_to_state("AVOIDING_REALIGN_TURN"))
        
        elif new_state == "AVOIDING_REALIGN_TURN":
            self.stop_robot(gradual=False)
            self.state_timer = self.create_timer(self.avoidance_realign_turn_duration, lambda: self.transition_to_state("SEARCHING_TURN"))

        elif new_state == "SEARCHING_TURN":
            self.stop_robot(gradual=False)
            if not self.search_timeout_timer or self.search_timeout_timer.is_canceled():
                 self.search_timeout_timer = self.create_timer(self.search_timeout, self.handle_search_timeout)
            self.state_timer = self.create_timer(self.search_turn_duration, lambda: self.transition_to_state("SEARCHING_PAUSE"))

        elif new_state == "SEARCHING_PAUSE":
            self.stop_robot(gradual=False)
            self.state_timer = self.create_timer(self.search_pause_duration, lambda: self.transition_to_state("SEARCHING_TURN"))

    def handle_search_timeout(self):
        if "SEARCHING" in self.state:
            self.get_logger().warn("탐색 시간 초과. 사용자를 찾지 못했습니다. 정지합니다.")
            self.transition_to_following(stop_first=True)

    def transition_to_following(self, stop_first=False):
        self.state = "FOLLOWING"
        if self.state_timer: self.state_timer.cancel()
        if self.search_timeout_timer: self.search_timeout_timer.cancel()
        self.state_timer = None
        self.search_timeout_timer = None
        
        if stop_first or self.is_following:
            self.stop_robot()
        
        self.distance_pid.reset()
        self.angle_pid.reset()
        self.is_following = False

    def stop_robot(self, gradual=True):
        if gradual and (abs(self.last_cmd_vel.linear.x) > 0.01 or abs(self.last_cmd_vel.angular.z) > 0.01):
            self.gradual_stop()
        else:
            self.cmd_vel_pub.publish(Twist())
            self.last_cmd_vel = Twist()

    # 나머지 유틸리티 및 서비스 핸들러 함수들
    def gradual_stop(self, duration=0.5, steps=20):
        initial_vel = self.last_cmd_vel
        for i in range(steps + 1):
            ratio = 1.0 - (i / steps)
            cmd_msg = Twist()
            cmd_msg.linear.x = initial_vel.linear.x * ratio
            cmd_msg.angular.z = initial_vel.angular.z * ratio
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(duration / steps)
        self.last_cmd_vel = Twist()
    def send_voice_command(self, category, action):
        msg = VoiceCommand(); msg.robot_id = "libo_a"; msg.category = category; msg.action = action
        self.voice_cmd_pub.publish(msg)
    def detection_status_callback(self, msg: DetectionStatus): self.obstacle_status = msg
    def talk_command_callback(self, msg: TalkCommand):
        if not self.qr_authenticated or msg.robot_id != "libo_a": return
        if msg.action == "stop":
            if not self.is_paused_by_voice: self.is_paused_by_voice = True; self.transition_to_following(stop_first=True)
        elif msg.action == "activate":
            if self.is_paused_by_voice: self.is_paused_by_voice = False
    def handle_activate_tracker(self, request, response): self.qr_authenticated = True; response.success = True; return response
    def handle_deactivate_tracker(self, request, response): self.qr_authenticated = False; self.transition_to_following(stop_first=True); response.success = True; return response
    def handle_arrival_trigger(self, request, response): self.qr_authenticated = False; self.send_voice_command("escort", "arrived"); self.transition_to_following(stop_first=True); response.success = True; return response


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
            node.transition_to_following(stop_first=True) 
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()