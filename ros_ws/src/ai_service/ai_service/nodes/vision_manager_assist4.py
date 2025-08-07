#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 필요한 ROS 2 라이브러리와 메시지/서비스 타입을 임포트합니다.
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist  # 로봇의 속도 명령을 위한 메시지
from libo_interfaces.msg import HumanInfo, DetectionStatus  # 사용자 추적 정보, 장애물 감지 정보 메시지
from libo_interfaces.srv import ActivateTracker, DeactivateTracker  # 추종 활성화/비활성화 서비스
from libo_interfaces.msg import TalkCommand, VoiceCommand  # 음성 명령 수신/송신 메시지
from std_srvs.srv import Trigger  # 간단한 트리거(신호)를 위한 표준 서비스

# --- PID 제어기 클래스 ---
# 목표값(예: 목표 거리)과 현재값의 차이(오차)를 이용해
# 로봇의 출력을 제어하여 목표값에 안정적으로 도달하게 만드는 제어기입니다.
class PIDController:
    """
    비례(P), 적분(I), 미분(D) 제어를 수행하는 클래스.
    """
    def __init__(self, kp, ki, kd, max_output=None, min_output=None):
        # PID 게인(Gain) 값 설정. 이 값들을 튜닝하여 로봇의 반응성을 조절합니다.
        self.kp, self.ki, self.kd = kp, ki, kd
        # 제어기 출력의 최댓값과 최솟값. 로봇의 속도를 제한하여 안정성을 높입니다.
        self.max_output, self.min_output = max_output, min_output
        # 이전 오차값과 오차의 누적값(적분항)을 저장하기 위한 변수
        self.previous_error, self.integral = 0.0, 0.0

    def update(self, error, dt=0.1):
        """
        새로운 오차값을 받아 PID 계산을 수행하고 제어 출력값을 반환합니다.
        :param error: 현재 오차 (목표값 - 현재값)
        :param dt: 시간 변화량 (delta time)
        :return: 계산된 제어 출력값 (예: 로봇의 속도)
        """
        # 오차를 시간에 따라 누적합니다 (적분항 I).
        self.integral += error * dt
        # 현재 오차와 이전 오차의 차이를 통해 오차의 변화율을 계산합니다 (미분항 D).
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        
        # P, I, D 각 항에 게인을 곱하여 최종 출력값을 계산합니다.
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # 출력값이 설정된 최대/최소 범위를 벗어나지 않도록 제한합니다.
        if self.max_output is not None: output = min(output, self.max_output)
        if self.min_output is not None: output = max(output, self.min_output)
        
        # 현재 오차를 다음 계산을 위해 저장합니다.
        self.previous_error = error
        return output
    
    def reset(self):
        """PID 제어기를 초기 상태로 리셋합니다."""
        self.previous_error, self.integral = 0.0, 0.0
        
    def set_output_limits(self, min_output, max_output):
        """제어기 출력 제한값을 동적으로 설정합니다."""
        self.min_output = min_output
        self.max_output = max_output

# --- 메인 로직: 지능형 보조 추종 유한 상태 머신(FSM) 클래스 ---
class AdvancedAssistFollowFSM(Node):
    """
    [v5] 복잡성을 줄이고 '후진-회전-전진'의 단순 회피 로직으로 수정한 버전.
    로봇의 행동을 '상태(State)'로 정의하고, 조건에 따라 상태를 전환하며 동작을 제어합니다.
    """
    def __init__(self):
        super().__init__('advanced_assist_follow_fsm_final_fwd_v5')

        # --- 1. ROS 파라미터 선언: 로봇의 행동을 쉽게 튜닝하기 위한 변수들 ---
        # 이 값들은 launch 파일 등에서 쉽게 변경할 수 있습니다.
        
        # --- 회피 기동 관련 파라미터 ---
        self.declare_parameter('avoidance_backup_duration_s', 1.5)      # 회피 시 후진할 시간 (초)
        self.declare_parameter('avoidance_escape_turn_duration_s', 1.3) # 회피 시 회전할 시간 (초)
        self.declare_parameter('avoidance_forward_duration_s', 1.5)     # 회피 후 전진할 시간 (초)
        self.declare_parameter('avoidance_forward_vel', 0.15)           # 회피 시 전진 속도 (m/s)
        self.declare_parameter('avoidance_turn_vel', 0.3)               # 회피 시 회전 각속도 (rad/s)
        self.declare_parameter('avoidance_backup_vel', -0.15)           # 회피 시 후진 속도 (m/s, 음수)
        
        # --- 사용자 탐색 관련 파라미터 ---
        self.declare_parameter('search_turn_duration_s', 0.7)           # 탐색 시 한 방향으로 회전할 시간 (초)
        self.declare_parameter('search_pause_duration_s', 0.5)          # 탐색 회전 후 잠시 멈출 시간 (초)
        self.declare_parameter('search_timeout_s', 8.0)                 # 탐색을 포기하기까지의 총 시간 (초)
        self.declare_parameter('search_angular_vel', 0.4)               # 탐색 시 회전 각속도 (rad/s)

        # --- 일반 추종 관련 파라미터 ---
        self.declare_parameter('target_distance', 1.2)                  # 사용자와 유지할 목표 거리 (m)
        self.declare_parameter('safe_distance_min', 1.0)                # 비상 정지를 발동할 최소 안전 거리 (m)
        self.declare_parameter('max_linear_vel', 0.25)                  # 로봇의 최대 직진 속도 (m/s)
        self.declare_parameter('following_max_angular_vel', 0.35)       # 추종 중 최대 회전 각속도 (rad/s)
        
        # --- PID 제어기 게인 파라미터 ---
        self.declare_parameter('dist_kp', 1.0); self.declare_parameter('dist_ki', 0.0); self.declare_parameter('dist_kd', 0.1)
        self.declare_parameter('angle_kp', 1.5); self.declare_parameter('angle_ki', 0.0); self.declare_parameter('angle_kd', 0.2)
        
        # 선언된 파라미터들의 실제 값을 가져와 클래스 변수에 저장합니다.
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

        # --- 2. PID 제어기 생성 ---
        # 거리 제어용 PID. 로봇의 직진 속도를 제어.
        self.distance_pid = PIDController(
            kp=self.get_parameter('dist_kp').value, ki=self.get_parameter('dist_ki').value, kd=self.get_parameter('dist_kd').value,
            max_output=self.max_linear_vel, min_output=-self.max_linear_vel)
        # 각도 제어용 PID. 로봇의 회전 속도를 제어.
        self.angle_pid = PIDController(
            kp=self.get_parameter('angle_kp').value, ki=self.get_parameter('angle_ki').value, kd=self.get_parameter('angle_kd').value,
            max_output=self.following_max_angular_vel, min_output=-self.following_max_angular_vel)

        # --- 3. 상태 변수 및 통신 인터페이스 초기화 ---
        # 동작 플래그 및 상태 변수
        self.qr_authenticated = False       # QR 인증을 통해 추종이 활성화되었는지 여부
        self.is_paused_by_voice = False     # 음성 명령으로 일시정지되었는지 여부
        self.obstacle_status = None         # 장애물 감지 센서의 최신 상태 저장
        self.state = "FOLLOWING"            # 로봇의 현재 행동 상태 (FSM의 핵심)
        self.is_following = False           # 실제로 사용자를 따라가고 있는지 여부
        self.state_timer = None             # 상태 전환을 위한 타이머
        self.search_timeout_timer = None    # 탐색 시간 초과를 감지하기 위한 타이머
        self.avoidance_turn_direction = 1   # 회피 방향 (1: 좌회전, -1: 우회전)
        self.last_known_angle_error = 0.0   # 사용자를 놓치기 직전의 각도 오차
        self.honk_played = False            # 장애물 경고음을 재생했는지 여부
        self.last_cmd_vel = Twist()         # 마지막으로 발행한 속도 명령

        # ROS 통신 인터페이스 (Publisher, Subscriber, Service)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # 로봇 속도 명령 발행
        self.voice_cmd_pub = self.create_publisher(VoiceCommand, '/voice_command', 10) # 음성 출력 명령 발행
        self.human_info_sub = self.create_subscription(HumanInfo, '/human_info', self.human_info_callback, 10) # 사용자 정보 수신
        self.detection_status_sub = self.create_subscription(DetectionStatus, '/detection_status', self.detection_status_callback, 10) # 장애물 정보 수신
        self.talk_command_sub = self.create_subscription(TalkCommand, '/talk_command', self.talk_command_callback, 10) # 음성 명령 수신
        self.activate_srv = self.create_service(ActivateTracker, '/activate_tracker', self.handle_activate_tracker) # 추종 활성화 서비스 서버
        self.deactivate_srv = self.create_service(DeactivateTracker, '/deactivate_tracker', self.handle_deactivate_tracker) # 추종 비활성화 서비스 서버
        self.arrived_srv = self.create_service(Trigger, '/trigger_arrival', self.handle_arrival_trigger) # 목적지 도착 신호 서비스 서버

        self.get_logger().info('✅ Advanced Assist Follow FSM 노드 시작 완료 (v5: 단순 회피 로직)')


    def human_info_callback(self, msg: HumanInfo):
        """
        사용자 추적 정보(/human_info)를 받을 때마다 호출되는 메인 콜백 함수.
        이 함수 안에서 로봇의 모든 상태 판단과 행동 결정이 이루어집니다.
        """
        # 추종이 활성화되지 않았거나, 음성명령으로 일시정지 상태이면 아무것도 하지 않음.
        if not self.qr_authenticated or self.is_paused_by_voice: return
        # 아직 장애물 정보를 받지 못했다면 대기.
        if self.obstacle_status is None:
            self.get_logger().info('장애물 감지 정보 수신 대기 중...', once=True)
            return

        # [최우선 순위] 비상 정지 조건 확인
        # 전방 중앙, 혹은 좌/우 동시 장애물 감지 또는 사용자와의 거리가 최소 안전거리 미만일 경우
        if self.obstacle_status.center_detected or \
           (self.obstacle_status.left_detected and self.obstacle_status.right_detected) or \
           (msg.is_detected and msg.distance < self.safe_distance_min):
            # 회피/탐색 등 다른 동작 중에도 즉시 정지해야 하므로 조건을 확인하고 정지 명령을 내림.
            if self.state != "FOLLOWING" or self.is_following:
                 self.get_logger().warn('🚨 비상 정지 조건 충족! 모든 동작을 즉시 중지합니다!', throttle_duration_sec=1)
                 self.transition_to_following(stop_first=True) # 추종 상태로 복귀하면서 정지
            return

        # 로봇에게 보낼 속도 명령 메시지 초기화
        cmd_msg = Twist()
        
        # --- FSM: 상태에 따른 행동 분기 ---
        if self.state == "FOLLOWING":
            # 좌측 또는 우측에 장애물이 감지되면 회피 기동 시작
            if self.obstacle_status.left_detected or self.obstacle_status.right_detected:
                if not self.honk_played: self.send_voice_command("common", "obstacle_detected"); self.honk_played = True
                self.last_known_angle_error = msg.horizontal_offset # 사용자를 놓칠 경우를 대비해 마지막 각도 저장
                self.avoidance_turn_direction = 1 if self.obstacle_status.left_detected else -1 # 좌측 장애물이면 좌회전(1), 아니면 우회전(-1)
                # 첫 회피 동작인 '후진' 상태로 전환
                self.transition_to_state("AVOIDING_BACKUP")
            else:
                # 장애물이 없으면 일반적인 사용자 추종 수행
                self.perform_following_with_pid(msg)
            return

        elif self.state == "AVOIDING_BACKUP":
            # 후진 속도를 설정
            cmd_msg.linear.x = self.avoidance_backup_vel

        elif self.state == "AVOIDING_ESCAPE_TURN":
            # 회피 방향으로 회전 속도를 설정
            cmd_msg.angular.z = self.avoidance_turn_vel * self.avoidance_turn_direction

        elif self.state == "AVOIDING_FORWARD":
            # 전진 속도를 설정
            cmd_msg.linear.x = self.avoidance_forward_vel

        elif self.state == "SEARCHING_TURN":
            # 사용자를 마지막으로 봤던 방향의 반대쪽으로 회전하며 탐색
            turn_dir = 1 if self.last_known_angle_error < 0 else -1
            cmd_msg.angular.z = self.search_angular_vel * turn_dir

        elif self.state == "SEARCHING_PAUSE":
            # 탐색 중 잠시 멈춰서 사용자가 감지되는지 확인
            if msg.is_detected:
                self.get_logger().info("🎉 사용자 재탐지 성공! 추종 모드로 복귀합니다.")
                self.transition_to_following(stop_first=True)
                return
        
        # 'FOLLOWING' 상태가 아닐 때 (즉, 회피 또는 탐색 중일 때) 계산된 속도 명령을 발행
        if self.state not in ["FOLLOWING"]:
            self.cmd_vel_pub.publish(cmd_msg)
            self.last_cmd_vel = cmd_msg

    def perform_following_with_pid(self, msg: HumanInfo):
        """PID 제어기를 사용하여 사용자를 부드럽게 따라가는 동작을 수행합니다."""
        if self.honk_played:
            self.get_logger().info("장애물 없음. 경고음 상태를 리셋합니다.")
            self.honk_played = False
        
        # 사용자가 감지되지 않으면 탐색 상태로 전환
        if not msg.is_detected:
            if self.is_following: # 추종 중에 놓쳤을 때만 탐색 시작
                self.get_logger().warn('사람을 놓쳤습니다. 탐색을 시작합니다.')
                self.last_known_angle_error = self.angle_pid.previous_error # PID의 마지막 오차를 사용
                self.transition_to_state("SEARCHING_TURN")
            return
        
        self.is_following = True # 추종 시작 플래그
        
        # 거리와 각도에 대한 오차 계산
        distance_error = msg.distance - self.target_distance
        angle_error = msg.horizontal_offset
        
        # PID 제어기로부터 직진/회전 속도를 계산
        cmd_msg = Twist()
        cmd_msg.linear.x = self.distance_pid.update(distance_error)
        cmd_msg.angular.z = -self.angle_pid.update(angle_error) # 화면 좌표계와 로봇 좌표계의 방향이 반대일 수 있으므로 - 부호 사용
        
        self.cmd_vel_pub.publish(cmd_msg)
        self.last_cmd_vel = cmd_msg
    
    def transition_to_state(self, new_state):
        """로봇의 상태를 전환하고, 다음 상태 전환을 위한 타이머를 설정합니다."""
        self.get_logger().info(f"상태 변경: {self.state} -> {new_state}")
        self.state = new_state
        
        # 기존에 실행 중이던 타이머가 있다면 취소
        if self.state_timer:
            self.state_timer.cancel()
            self.state_timer = None

        # [단순화된 상태 전환 로직]
        if new_state == "AVOIDING_BACKUP":
            self.stop_robot(gradual=False) # 상태 전환 전 일단 정지
            # 설정된 시간만큼 후진한 뒤, 'AVOIDING_ESCAPE_TURN' 상태로 전환
            self.state_timer = self.create_timer(self.avoidance_backup_duration, lambda: self.transition_to_state("AVOIDING_ESCAPE_TURN"))
        
        elif new_state == "AVOIDING_ESCAPE_TURN":
            self.stop_robot(gradual=False)
            # 설정된 시간만큼 회전한 뒤, 'AVOIDING_FORWARD' 상태로 전환
            self.state_timer = self.create_timer(self.avoidance_escape_turn_duration, lambda: self.transition_to_state("AVOIDING_FORWARD"))

        elif new_state == "AVOIDING_FORWARD":
            self.stop_robot(gradual=False)
            # 설정된 시간만큼 전진한 뒤, 'SEARCHING_TURN' 상태로 전환하여 사용자 탐색 시작
            self.state_timer = self.create_timer(self.avoidance_forward_duration, lambda: self.transition_to_state("SEARCHING_TURN"))

        elif new_state == "SEARCHING_TURN":
            self.stop_robot(gradual=False)
            # 탐색 상태에 처음 진입했다면, 타임아웃 타이머 시작
            if not self.search_timeout_timer or self.search_timeout_timer.is_canceled():
                 self.search_timeout_timer = self.create_timer(self.search_timeout, self.handle_search_timeout)
            # 설정된 시간만큼 회전한 뒤, 'SEARCHING_PAUSE' 상태로 전환
            self.state_timer = self.create_timer(self.search_turn_duration, lambda: self.transition_to_state("SEARCHING_PAUSE"))

        elif new_state == "SEARCHING_PAUSE":
            self.stop_robot(gradual=False)
            # 설정된 시간만큼 멈춘 뒤, 다시 'SEARCHING_TURN' 상태로 전환하여 회전-정지 반복
            self.state_timer = self.create_timer(self.search_pause_duration, lambda: self.transition_to_state("SEARCHING_TURN"))

    # --- 유틸리티 및 서비스/콜백 핸들러 함수들 ---
    
    def handle_search_timeout(self):
        """탐색 시간이 초과되면 호출되는 함수."""
        if "SEARCHING" in self.state:
            self.get_logger().warn("탐색 시간 초과. 사용자를 찾지 못했습니다. 정지합니다.")
            self.transition_to_following(stop_first=True)

    def transition_to_following(self, stop_first=False):
        """모든 동작을 중단하고 기본 'FOLLOWING' 상태로 안전하게 복귀합니다."""
        self.state = "FOLLOWING"
        # 모든 타이머를 취소하고 PID 제어기를 리셋
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
        """로봇을 정지시킵니다."""
        if gradual and (abs(self.last_cmd_vel.linear.x) > 0.01 or abs(self.last_cmd_vel.angular.z) > 0.01):
            self.gradual_stop()
        else:
            self.cmd_vel_pub.publish(Twist()) # 속도를 0으로 하여 즉시 정지
            self.last_cmd_vel = Twist()

    def gradual_stop(self, duration=0.5, steps=20):
        """로봇을 부드럽게 감속하여 정지시킵니다."""
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
        """음성 출력 노드에게 음성 재생을 요청합니다."""
        msg = VoiceCommand(); msg.robot_id = "libo_a"; msg.category = category; msg.action = action
        self.voice_cmd_pub.publish(msg)

    def detection_status_callback(self, msg: DetectionStatus):
        """장애물 감지 정보를 수신하여 클래스 변수에 저장합니다."""
        self.obstacle_status = msg
        
    def talk_command_callback(self, msg: TalkCommand):
        """음성 명령을 수신하여 로봇의 동작을 제어합니다."""
        if not self.qr_authenticated or msg.robot_id != "libo_a": return
        if msg.action == "stop": # "멈춰"
            if not self.is_paused_by_voice: self.is_paused_by_voice = True; self.transition_to_following(stop_first=True)
        elif msg.action == "activate": # "따라와"
            if self.is_paused_by_voice: self.is_paused_by_voice = False

    def handle_activate_tracker(self, request, response):
        """추종 활성화 서비스 요청을 처리합니다."""
        self.qr_authenticated = True; response.success = True; return response
        
    def handle_deactivate_tracker(self, request, response):
        """추종 비활성화 서비스 요청을 처리합니다."""
        self.qr_authenticated = False; self.transition_to_following(stop_first=True); response.success = True; return response
        
    def handle_arrival_trigger(self, request, response):
        """목적지 도착 신호를 처리합니다."""
        self.qr_authenticated = False; self.send_voice_command("escort", "arrived"); self.transition_to_following(stop_first=True); response.success = True; return response

# --- 메인 실행 함수 ---
def main(args=None):
    rclpy.init(args=args) # ROS 2 초기화
    node = None
    try:
        # FSM 노드 인스턴스 생성 및 실행
        node = AdvancedAssistFollowFSM()
        rclpy.spin(node) # 노드가 종료될 때까지 계속 실행
    except KeyboardInterrupt:
        # Ctrl+C 입력 시 처리
        if node: node.get_logger().info('키보드 인터럽트로 노드를 종료합니다.')
    finally:
        # 노드 종료 시 안전하게 로봇을 정지하고 리소스를 해제
        if node and rclpy.ok():
            node.get_logger().info('안전한 종료를 위해 로봇을 정지합니다.')
            node.transition_to_following(stop_first=True) 
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()