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
    [v7] 정면 장애물을 제외한 후방, 양측 장애물은 비상 정지(Stop&Wait)하도록 수정한 버전.
    """
    def __init__(self):
        super().__init__('advanced_assist_follow_fsm_final_fwd_v7')

        # ROS 파라미터 선언
        self.declare_parameter('avoidance_backup_duration_s', 1.5)
        self.declare_parameter('avoidance_escape_turn_duration_s', 1.3)
        self.declare_parameter('avoidance_forward_duration_s', 1.5)
        self.declare_parameter('avoidance_forward_vel', 0.15)
        self.declare_parameter('avoidance_turn_vel', 0.3)
        self.declare_parameter('avoidance_backup_vel', -0.15)
        self.declare_parameter('search_turn_duration_s', 0.7)
        self.declare_parameter('search_pause_duration_s', 3.0)
        self.declare_parameter('search_timeout_s', 20.0)
        self.declare_parameter('search_angular_vel', 0.45)
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('safe_distance_min', 1.0)
        self.declare_parameter('max_linear_vel', 0.25)
        self.declare_parameter('following_max_angular_vel', 0.35)
        self.declare_parameter('dist_kp', 1.0); self.declare_parameter('dist_ki', 0.0); self.declare_parameter('dist_kd', 0.1)
        self.declare_parameter('angle_kp', 1.5); self.declare_parameter('angle_ki', 0.0); self.declare_parameter('angle_kd', 0.2)
        
        # 파라미터 값 가져오기
        self.avoidance_backup_duration = self.get_parameter('avoidance_backup_duration_s').value
        self.avoidance_escape_turn_duration = self.get_parameter('avoidance_escape_turn_duration_s').value
        self.avoidance_forward_duration = self.get_parameter('avoidance_forward_duration_s').value
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

        # PID 제어기 생성
        self.distance_pid = PIDController(
            kp=self.get_parameter('dist_kp').value, ki=self.get_parameter('dist_ki').value, kd=self.get_parameter('dist_kd').value,
            max_output=self.max_linear_vel, min_output=-self.max_linear_vel)
        self.angle_pid = PIDController(
            kp=self.get_parameter('angle_kp').value, ki=self.get_parameter('angle_ki').value, kd=self.get_parameter('angle_kd').value,
            max_output=self.following_max_angular_vel, min_output=-self.following_max_angular_vel)

        # 상태 변수 및 통신 인터페이스 초기화
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
        self.front_obstacle_timer = None

        # ROS 통신 인터페이스
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_cmd_pub = self.create_publisher(VoiceCommand, '/voice_command', 10)
        self.human_info_sub = self.create_subscription(HumanInfo, '/human_info', self.human_info_callback, 10)
        self.detection_status_sub = self.create_subscription(DetectionStatus, '/detection_status', self.detection_status_callback, 10)
        self.talk_command_sub = self.create_subscription(TalkCommand, '/talk_command', self.talk_command_callback, 10)
        self.activate_srv = self.create_service(ActivateTracker, '/activate_tracker', self.handle_activate_tracker)
        self.deactivate_srv = self.create_service(DeactivateTracker, '/deactivate_tracker', self.handle_deactivate_tracker)
        self.arrived_srv = self.create_service(Trigger, '/trigger_arrival', self.handle_arrival_trigger)

        self.get_logger().info('✅ Advanced Assist Follow FSM 노드 시작 완료 (v7)')

    def human_info_callback(self, msg: HumanInfo):
        # 1순위: 시스템 비활성화 및 일시정지 확인
        if not self.qr_authenticated or self.is_paused_by_voice: return
        if self.obstacle_status is None:
            self.get_logger().info('장애물 감지 정보 수신 대기 중...', once=True)
            return

        # 2순위: [정면 장애물] 조건부 로직
        if self.obstacle_status.center_detected:
            # <<< 상세 로그: 판단 시작 >>>
            self.get_logger().info(f"정면 장애물 감지됨. 거리 판단 시작 (현재: {msg.distance:.2f}m, 목표: {self.target_distance:.2f}m)", throttle_duration_sec=3)
            
            # 목표 거리보다 멀리 있을 때만 5초 대기 후 회피 시도
            if msg.distance > self.target_distance:
                if self.front_obstacle_timer is None:
                    # <<< 상세 로그: 5초 대기 결정 >>>
                    self.get_logger().warn("  -> [판단] 목표보다 멀리 있어 5초 대기 후 회피를 시도합니다. 타이머를 시작합니다.")
                    self.transition_to_following(stop_first=True)
                    self.front_obstacle_timer = self.create_timer(5.0, self.handle_front_obstacle_timeout)
            # 목표 거리보다 가깝거나 같으면 그냥 정지
            else:
                # <<< 상세 로그: 현위치 정지 결정 >>>
                self.get_logger().warn("  -> [판단] 목표 거리 내에 있으므로 후진 없이 현 위치에서 대기합니다.")
                self.transition_to_following(stop_first=True)
                # 혹시 다른 조건으로 타이머가 실행 중이었다면 안전하게 취소
                if self.front_obstacle_timer is not None:
                    self.get_logger().info("  -> 기존에 실행 중이던 전방 장애물 타이머를 취소합니다.")
                    self.front_obstacle_timer.cancel()
                    self.front_obstacle_timer = None
            return # 다른 로직 실행 방지
        else:
            if self.front_obstacle_timer is not None:
                # <<< 상세 로그: 타이머 취소 >>>
                self.get_logger().info('✅ 전방 장애물이 사라졌습니다. 대기 타이머를 취소하고 정상 동작을 재개합니다.')
                self.front_obstacle_timer.cancel()
                self.front_obstacle_timer = None

        # 3순위: [v7 수정] 그 외 비상 정지 (후방, 양측, 최소거리) - 자동회피 없이 정지만 수행
        estop_reason = None
        if self.obstacle_status.rear_detected:
            estop_reason = "후방 장애물 감지!"
        elif self.obstacle_status.left_detected and self.obstacle_status.right_detected:
            estop_reason = "양측 경로 막힘!"
        elif msg.is_detected and msg.distance < self.safe_distance_min:
            estop_reason = f"사용자와의 최소 안전거리({self.safe_distance_min}m) 미만!"
        
        if estop_reason:
            self.get_logger().warn(f'🚨 비상 정지! (사유: {estop_reason})', throttle_duration_sec=1)
            self.transition_to_following(stop_first=True)
            return # 다른 로직 실행 방지
            
        # --- FSM: 상태에 따른 행동 분기 ---
        cmd_msg = Twist()
        if self.state == "FOLLOWING":
            # 4순위: 측면 장애물 회피 (좌 또는 우 단독)
            if self.obstacle_status.left_detected or self.obstacle_status.right_detected:
                if not self.honk_played: self.send_voice_command("common", "obstacle_detected"); self.honk_played = True
                self.last_known_angle_error = msg.horizontal_offset
                self.avoidance_turn_direction = -1 if self.obstacle_status.left_detected else 1
                self.get_logger().info(f"측면 장애물 감지. {'우측' if self.avoidance_turn_direction == -1 else '좌측'}으로 회피합니다.")
                self.transition_to_state("AVOIDING_BACKUP")
            else:
                # 5 & 6순위: 사용자 탐색 및 정상 추종
                self.perform_following_with_pid(msg)
            return

        elif self.state == "AVOIDING_BACKUP":
            cmd_msg.linear.x = self.avoidance_backup_vel
        elif self.state == "AVOIDING_ESCAPE_TURN":
            cmd_msg.angular.z = self.avoidance_turn_vel * self.avoidance_turn_direction
        elif self.state == "AVOIDING_FORWARD":
            cmd_msg.linear.x = self.avoidance_forward_vel
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
        
        # 5순위: 사용자 탐색
        if not msg.is_detected:
            if self.is_following:
                self.get_logger().warn('사람을 놓쳤습니다. 탐색을 시작합니다.')
                self.last_known_angle_error = self.angle_pid.previous_error
                self.transition_to_state("SEARCHING_TURN")
            return
        
        self.is_following = True
        
        # 6순위: 정상 추종
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
        
        if self.state_timer: self.state_timer.cancel(); self.state_timer = None

        if new_state == "AVOIDING_BACKUP":
            self.stop_robot(gradual=False)
            self.state_timer = self.create_timer(self.avoidance_backup_duration, lambda: self.transition_to_state("AVOIDING_ESCAPE_TURN"))
        elif new_state == "AVOIDING_ESCAPE_TURN":
            self.stop_robot(gradual=False)
            self.state_timer = self.create_timer(self.avoidance_escape_turn_duration, lambda: self.transition_to_state("AVOIDING_FORWARD"))
        elif new_state == "AVOIDING_FORWARD":
            self.stop_robot(gradual=False)
            self.state_timer = self.create_timer(self.avoidance_forward_duration, lambda: self.transition_to_state("SEARCHING_TURN"))
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

    def handle_front_obstacle_timeout(self):
        # <<< 상세 로그: 타이머 만료 >>>
        self.get_logger().info("⏰ 전방 장애물 5초 대기 타이머 만료.")
        if self.obstacle_status and self.obstacle_status.center_detected:
            # <<< 상세 로그: 회피 시작 >>>
            self.get_logger().warn("  -> [조치] 장애물이 아직 존재하므로 후진 회피를 시작합니다.")
            self.avoidance_turn_direction = 1  # 후진 후 회전 방향은 좌측으로 고정
            self.transition_to_state("AVOIDING_BACKUP")
        else:
            # <<< 상세 로그: 회피 불필요 >>>
            self.get_logger().info("  -> [조치] 타이머 만료 시점에는 장애물이 사라져 별도 조치 없이 정상 추종을 재개합니다.")
        self.front_obstacle_timer = None # 타이머 완료 후 리셋

    def transition_to_following(self, stop_first=False):
        self.state = "FOLLOWING"
        if self.state_timer: self.state_timer.cancel()
        if self.search_timeout_timer: self.search_timeout_timer.cancel()
        if self.front_obstacle_timer: self.front_obstacle_timer.cancel()
        self.state_timer = None; self.search_timeout_timer = None; self.front_obstacle_timer = None
        
        if stop_first or self.is_following: self.stop_robot()
        
        self.distance_pid.reset(); self.angle_pid.reset()
        self.is_following = False

    def stop_robot(self, gradual=True):
        if gradual and (abs(self.last_cmd_vel.linear.x) > 0.01 or abs(self.last_cmd_vel.angular.z) > 0.01):
            self.gradual_stop()
        else:
            self.cmd_vel_pub.publish(Twist())
            self.last_cmd_vel = Twist()

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

    def detection_status_callback(self, msg: DetectionStatus):
        self.obstacle_status = msg
        
    def talk_command_callback(self, msg: TalkCommand):
        # 로봇 ID가 다르면 무시 (로그 기록 전에 체크)
        if msg.robot_id != "libo_a": return

        self.get_logger().info(f"🎤 음성 명령 수신: '{msg.action}'") 

        # 추종 기능이 꺼져있으면, 명령을 처리하지 않음
        if not self.qr_authenticated:
            self.get_logger().warn("   -> 추적 비활성화 상태이므로 음성 명령을 처리하지 않습니다.") 
            return
        
        if msg.action == "stop":
            if not self.is_paused_by_voice:
                self.get_logger().info("   -> 음성 명령으로 추적을 일시 중지합니다.") 
                self.is_paused_by_voice = True
                self.transition_to_following(stop_first=True)
        elif msg.action == "activate":
            if self.is_paused_by_voice:
                self.get_logger().info("   -> 음성 명령으로 추적을 다시 시작합니다.") 
                self.is_paused_by_voice = False

    def handle_activate_tracker(self, request, response):
        self.get_logger().info('▶️  추적 활성화 (Activate) 서비스 수신') 
        self.qr_authenticated = True
        response.success = True
        return response
        
    def handle_deactivate_tracker(self, request, response):
        self.get_logger().info('⏹️  추적 비활성화 (Deactivate) 서비스 수신') 
        self.qr_authenticated = False
        self.transition_to_following(stop_first=True)
        response.success = True
        return response
        
    def handle_arrival_trigger(self, request, response):
        self.get_logger().info('🏁 도착 완료 (Arrival) 트리거 수신. 추적을 종료합니다.') 
        self.qr_authenticated = False
        self.send_voice_command("escort", "arrived")
        self.transition_to_following(stop_first=True)
        response.success = True
        return response

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