#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from libo_interfaces.msg import HumanInfo, DetectionStatus
from libo_interfaces.srv import ActivateTracker, DeactivateTracker

# [신규 추가] HumanCoordinatorNode의 PID 컨트롤러 클래스
class PIDController:
    """
    PID(Proportional-Integral-Derivative) 제어기 클래스.
    목표값에 부드럽고 안정적으로 도달하기 위해 사용됩니다.
    """
    def __init__(self, kp, ki, kd, max_output=None, min_output=None):
        # --- PID 게인(Gain) ---
        self.kp = kp  # 비례(Proportional) 게인: 현재 오차에 비례하여 출력을 결정
        self.ki = ki  # 적분(Integral) 게인: 과거 오차의 누적값에 비례하여 정상상태 오차(steady-state error)를 없앰
        self.kd = kd  # 미분(Derivative) 게인: 오차의 변화율에 비례하여 오버슈트(overshoot)를 줄이고 응답을 안정시킴

        # --- 출력 제한 ---
        self.max_output = max_output  # 제어기 출력의 최댓값 (예: 로봇의 최대 속도)
        self.min_output = min_output  # 제어기 출력의 최솟값

        # --- 내부 상태 변수 ---
        self.previous_error = 0.0  # 이전 오차값 (미분 항 계산에 사용)
        self.integral = 0.0        # 오차의 누적값 (적분 항 계산에 사용)

    def update(self, error, dt=0.1):
        """
        새로운 오차 값을 입력받아 PID 제어 출력을 계산하고 반환합니다.
        :param error: 목표값과 현재 측정값의 차이
        :param dt: 시간 변화량 (delta time). 이전 update 호출과의 시간 간격
        :return: PID 제어 공식에 따른 제어 출력값
        """
        # 적분항 계산: 오차를 시간에 따라 계속 누적
        self.integral += error * dt
        
        # 미분항 계산: 현재 오차와 이전 오차의 차이를 통해 오차의 변화율(기울기)을 계산
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        
        # PID 제어 수식: P항 + I항 + D항
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # 계산된 출력값이 설정된 최댓값/최솟값을 넘지 않도록 제한 (Clamping)
        if self.max_output is not None:
            output = min(output, self.max_output)
        if self.min_output is not None:
            output = max(output, self.min_output)
            
        # 다음 계산을 위해 현재 오차를 '이전 오차'로 저장
        self.previous_error = error
        return output

    def reset(self):
        """
        제어기의 내부 상태(누적 오차 등)를 초기화합니다.
        로봇이 정지하거나 상태가 변경될 때 호출하여 이전 상태가 다음 제어에 영향을 주지 않도록 합니다.
        """
        self.previous_error = 0.0
        self.integral = 0.0

class AdvancedAssistFollowFSM(Node):
    """
    [업데이트] 사람 추종과 장애물 회피, QR 인증을 총괄하는 고도화된 FSM 노드.
    PID 제어와 시간 기반 회피 기동 로직을 포함하여 안정적이고 지능적인 동작을 수행합니다.
    """
    def __init__(self):
        super().__init__('advanced_assist_follow_fsm')

        # --- 1. [신규 추가] ROS 파라미터 선언 및 로드 ---
        # 코드 수정 없이 launch 파일 등에서 쉽게 값을 변경(튜닝)할 수 있도록 파라미터화합니다.
        
        # 사람 추종 관련 파라미터
        self.declare_parameter('target_distance', 0.5, description='로봇이 사용자와 유지하려는 목표 거리 (m)')
        self.declare_parameter('safe_distance_min', 0.3, description='로봇이 사용자와 최소한으로 유지해야 할 안전 거리 (m)')
        self.declare_parameter('max_linear_vel', 0.3, description='최대 선속도 (m/s)')
        self.declare_parameter('max_angular_vel', 0.5, description='최대 각속도 (rad/s)')
        
        # 장애물 회피 관련 파라미터
        self.declare_parameter('avoidance_linear_vel', 0.1, description='회피 기동 시 직진 속도 (m/s)')
        self.declare_parameter('avoidance_angular_vel', 0.2, description='회피 기동 시 회전 속도 (rad/s)')
        self.declare_parameter('avoidance_backup_vel', -0.1, description='회피 기동 시 후진 속도 (m/s)')
        self.declare_parameter('avoidance_backup_duration_s', 1.5, description='회피 기동 시 후진 지속 시간 (초)')
        self.declare_parameter('avoidance_turn_duration_s', 1.0, description='회피 기동 시 회전 지속 시간 (초)')
        self.declare_parameter('avoidance_straight_duration_s', 1.5, description='회피 기동 시 직진 지속 시간 (초)')

        # PID 제어기 게인 파라미터 (이 값들을 튜닝하여 로봇의 반응성을 조절)
        self.declare_parameter('dist_kp', 1.0); self.declare_parameter('dist_ki', 0.0); self.declare_parameter('dist_kd', 0.1) # 거리 제어 PID 게인
        self.declare_parameter('angle_kp', 1.5); self.declare_parameter('angle_ki', 0.0); self.declare_parameter('angle_kd', 0.2) # 각도 제어 PID 게인

        # 선언된 파라미터 값을 변수로 가져와 사용
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.safe_distance_min = self.get_parameter('safe_distance_min').get_parameter_value().double_value
        self.avoidance_linear_vel = self.get_parameter('avoidance_linear_vel').get_parameter_value().double_value
        self.avoidance_angular_vel = self.get_parameter('avoidance_angular_vel').get_parameter_value().double_value
        self.avoidance_backup_vel = self.get_parameter('avoidance_backup_vel').get_parameter_value().double_value
        self.avoidance_backup_duration = self.get_parameter('avoidance_backup_duration_s').get_parameter_value().double_value
        self.avoidance_turn_duration = self.get_parameter('avoidance_turn_duration_s').get_parameter_value().double_value
        self.avoidance_straight_duration = self.get_parameter('avoidance_straight_duration_s').get_parameter_value().double_value

        # --- 2. [신규 추가] PID 제어기 생성 ---
        # 거리(선속도)와 각도(각속도)를 독립적으로 제어하기 위해 2개의 PID 제어기를 생성합니다.
        self.distance_pid = PIDController(
            kp=self.get_parameter('dist_kp').get_parameter_value().double_value,
            ki=self.get_parameter('dist_ki').get_parameter_value().double_value,
            kd=self.get_parameter('dist_kd').get_parameter_value().double_value,
            max_output=self.get_parameter('max_linear_vel').get_parameter_value().double_value, 
            min_output=-self.get_parameter('max_linear_vel').get_parameter_value().double_value) # 후진도 가능하도록 min_output 설정
        self.angle_pid = PIDController(
            kp=self.get_parameter('angle_kp').get_parameter_value().double_value,
            ki=self.get_parameter('angle_ki').get_parameter_value().double_value,
            kd=self.get_parameter('angle_kd').get_parameter_value().double_value,
            max_output=self.get_parameter('max_angular_vel').get_parameter_value().double_value, 
            min_output=-self.get_parameter('max_angular_vel').get_parameter_value().double_value)

        # --- 3. 상태 변수 및 통신 인터페이스 ---
        self.qr_authenticated = False   # QR 인증 성공 여부 (전체 기능 활성화 스위치)
        self.is_following = False       # 현재 사람을 추종하고 있는지 여부
        self.obstacle_status = None     # 가장 최근에 수신한 장애물 상태 정보
        self.state = "FOLLOWING"        # 현재 로봇의 FSM 상태 (FOLLOWING, AVOIDING_BACKUP, AVOIDING_TURN, AVOIDING_STRAIGHT)
        self.avoidance_timer = None     # 회피 기동 시 사용되는 타이머 객체
        self.avoidance_turn_direction = None # 회피 기동 시 회전해야 할 방향 ('LEFT' 또는 'RIGHT')

        # ROS2 통신 인터페이스 설정
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 로봇 속도 명령 발행
        self.human_info_sub = self.create_subscription(HumanInfo, '/human_info', self.human_info_callback, 10) # 사람 위치 정보 수신
        self.detection_status_sub = self.create_subscription(DetectionStatus, '/detection_status', self.detection_status_callback, 10) # 장애물 감지 정보 수신
        self.activate_srv = self.create_service(ActivateTracker, '/activate_tracker', self.handle_activate_tracker) # 추종 모드 활성화 서비스
        self.deactivate_srv = self.create_service(DeactivateTracker, '/deactivate_tracker', self.handle_deactivate_tracker) # 추종 모드 비활성화 서비스

        self.get_logger().info('✅ Advanced Assist Follow FSM 노드 시작 완료')

    # --- 4. 서비스 핸들러 (외부 제어) ---
    def handle_activate_tracker(self, request, response):
        """/activate_tracker 서비스 요청 처리. QR 인증 완료 시 호출됩니다."""
        self.get_logger().info(f"🟢 Activate 요청 수신 - robot_id: {request.robot_id}")
        self.qr_authenticated = True # 전체 FSM을 활성화합니다.
        response.success = True
        response.message = "Assist mode activated with advanced FSM."
        return response

    def handle_deactivate_tracker(self, request, response):
        """/deactivate_tracker 서비스 요청 처리. 모드 종료 시 호출됩니다."""
        self.get_logger().info(f"🔴 Deactivate 요청 수신 - robot_id: {request.robot_id}")
        self.qr_authenticated = False # 전체 FSM을 비활성화합니다.
        self.stop_robot() # 안전을 위해 로봇을 즉시 정지하고 모든 상태를 초기화합니다.
        response.success = True
        response.message = "Assist mode deactivated."
        return response

    # --- 5. 콜백 함수 (정보 수신 및 메인 로직) ---
    def detection_status_callback(self, msg: DetectionStatus):
        """/detection_status 토픽으로부터 장애물 감지 정보를 비동기적으로 수신하여 저장합니다."""
        self.obstacle_status = msg

    def human_info_callback(self, msg: HumanInfo):
        """
        /human_info 토픽을 받을 때마다 호출되는 메인 콜백 함수.
        이곳에서 모든 판단과 행동 결정이 순차적으로 이루어집니다.
        """
        # 1. 마스터 스위치 확인: QR 인증이 안 된 상태면 모든 동작을 중지하고 즉시 종료
        if not self.qr_authenticated:
            return

        # 2. 데이터 수신 확인: 장애물 정보가 아직 없다면 대기
        if self.obstacle_status is None:
            self.get_logger().info('장애물 감지 정보 수신 대기 중...', once=True)
            return

        # 3. [최우선] 비상 정지 시나리오: 다른 어떤 상태보다 먼저 확인하여 안전 확보
        if self.obstacle_status.center_detected or \
           (self.obstacle_status.left_detected and self.obstacle_status.right_detected):
            self.get_logger().warn('🚨 전방 또는 양측 장애물 동시 감지! 비상 정지!', throttle_duration_sec=1)
            self.stop_robot() # 즉시 정지 및 상태 초기화
            return # 다른 로직을 실행하지 않고 종료

        # 4. 상태 머신(FSM) 기반 행동 결정
        # 현재 'state' 변수에 따라 다른 행동을 수행합니다.
        
        # 상태 1: 사람 추종 (FOLLOWING)
        if self.state == "FOLLOWING":
            # 측면에 장애물이 감지되면 회피 기동 상태로 전환
            if self.obstacle_status.left_detected:
                self.get_logger().info("좌측 장애물 감지. 회피 기동(후진->우회전) 시작.")
                self.state = "AVOIDING_BACKUP"      # 상태를 '후진'으로 변경
                self.avoidance_turn_direction = 'RIGHT' # 좌측 장애물이므로 우측으로 회전하도록 방향 저장
                # return 없이 바로 아래의 AVOIDING_BACKUP 로직을 실행하도록 둡니다.
            
            elif self.obstacle_status.right_detected:
                self.get_logger().info("우측 장애물 감지. 회피 기동(후진->좌회전) 시작.")
                self.state = "AVOIDING_BACKUP"      # 상태를 '후진'으로 변경
                self.avoidance_turn_direction = 'LEFT'  # 우측 장애물이므로 좌측으로 회전하도록 방향 저장
            
            else:
                # 회피할 장애물이 없으면 정상적으로 사람을 추종
                self.perform_following_with_pid(msg)
                return # 추종 행동 후 이번 콜백은 종료

        # 상태 2: 회피를 위해 후진 (AVOIDING_BACKUP)
        if self.state == "AVOIDING_BACKUP":
            cmd_msg = Twist()
            cmd_msg.linear.x = self.avoidance_backup_vel # 설정된 속도로 후진
            self.cmd_vel_pub.publish(cmd_msg)
            # 타이머가 아직 없다면, 지정된 시간 후 회전 상태로 전환하는 타이머를 생성 (최초 한 번만 실행)
            if self.avoidance_timer is None:
                self.avoidance_timer = self.create_timer(self.avoidance_backup_duration, self.transition_to_avoid_turn)
            return # 후진 명령 후 이번 콜백 종료

        # 상태 3: 제자리 회전 (AVOIDING_TURN)
        if self.state == "AVOIDING_TURN":
            cmd_msg = Twist()
            # 이전에 저장해 둔 회전 방향에 따라 회전 속도 결정
            cmd_msg.angular.z = self.avoidance_angular_vel if self.avoidance_turn_direction == 'LEFT' else -self.avoidance_angular_vel
            self.cmd_vel_pub.publish(cmd_msg)
            # 타이머가 아직 없다면, 지정된 시간 후 직진 상태로 전환하는 타이머를 생성
            if self.avoidance_timer is None:
                self.avoidance_timer = self.create_timer(self.avoidance_turn_duration, self.transition_to_avoid_straight)
            return # 회전 명령 후 이번 콜백 종료

        # 상태 4: 회전 후 직진 (AVOIDING_STRAIGHT)
        if self.state == "AVOIDING_STRAIGHT":
            cmd_msg = Twist()
            cmd_msg.linear.x = self.avoidance_linear_vel # 설정된 속도로 직진
            self.cmd_vel_pub.publish(cmd_msg)
            # 타이머가 아직 없다면, 지정된 시간 후 추종 상태로 복귀하는 타이머를 생성
            if self.avoidance_timer is None:
                self.avoidance_timer = self.create_timer(self.avoidance_straight_duration, self.transition_to_following)
            return # 직진 명령 후 이번 콜백 종료

    # --- 6. 핵심 기능 함수 ---
    def perform_following_with_pid(self, msg: HumanInfo):
        """장애물이 없을 때 PID 제어기를 사용하여 사람을 추종하는 핵심 로직"""
        # 사람이 감지되지 않으면 정지
        if not msg.is_detected:
            if self.is_following:
                self.get_logger().warn('사람을 놓쳤습니다. 즉시 정지합니다.')
                self.stop_robot()
            return

        self.is_following = True # 추종 상태 플래그 설정

        # 사람이 최소 안전 거리보다 가까우면 안전을 위해 정지
        if msg.distance < self.safe_distance_min:
            self.get_logger().warn(f'사람이 너무 가까움! ({msg.distance:.2f}m) 정지!', throttle_duration_sec=1)
            self.stop_robot()
            return

        # PID 제어기에 입력할 오차 값 계산
        distance_error = msg.distance - self.target_distance # 거리 오차 = 현재 거리 - 목표 거리
        angle_error = msg.horizontal_offset                 # 각도 오차 = 화면 중앙으로부터의 픽셀 또는 각도 변위
        
        # PID 제어기로부터 최적의 속도 값 계산
        cmd_msg = Twist()
        cmd_msg.linear.x = self.distance_pid.update(distance_error)
        cmd_msg.angular.z = -self.angle_pid.update(angle_error) # 화면-로봇 좌표계 변환을 위해 부호 반전 (오른쪽이 +offset이면 좌회전(-angular.z) 필요)
        
        self.cmd_vel_pub.publish(cmd_msg)
    
    # --- 7. [신규 추가] 상태 전이(State Transition) 콜백 함수들 ---
    # 이 함수들은 create_timer에 의해 지정된 시간이 지나면 자동으로 한 번씩 호출됩니다.
    
    def transition_to_avoid_turn(self):
        """(타이머 콜백) 후진 완료 후 -> 회전 상태로 전환"""
        self.get_logger().info("후진 완료. 회전 시작.")
        self.state = "AVOIDING_TURN"
        if self.avoidance_timer:
            self.avoidance_timer.cancel(); self.avoidance_timer = None # 사용한 타이머는 파괴하고, 다음 상태에서 새 타이머를 만들 수 있도록 None으로 초기화

    def transition_to_avoid_straight(self):
        """(타이머 콜백) 회전 완료 후 -> 직진 상태로 전환"""
        self.get_logger().info("회전 완료. 회피 직진 시작.")
        self.state = "AVOIDING_STRAIGHT"
        if self.avoidance_timer:
            self.avoidance_timer.cancel(); self.avoidance_timer = None

    def transition_to_following(self):
        """(타이머 콜백) 회피 기동 완료 후 -> 추종 상태로 복귀"""
        self.get_logger().info("회피 기동 완료. 추종 모드로 복귀.")
        # 바로 추종을 시작하지 않고, 일단 정지하여 상황을 다시 판단하도록 함
        # 이렇게 하면 회피 기동 직후의 불안정한 상태를 방지할 수 있음
        self.stop_robot() 

    def stop_robot(self):
        """
        로봇을 정지시키고 모든 관련 상태를 초기화하는 가장 중요한 안전 및 리셋 함수.
        비상 상황, 모드 종료, 상태 초기화 등 필요 시 호출됩니다.
        """
        self.get_logger().info('🛑 로봇 정지 및 모든 상태 초기화.')
        self.is_following = False

        # 진행 중이던 회피 타이머가 있다면 즉시 취소하여 의도치 않은 동작 방지
        if self.avoidance_timer:
            self.avoidance_timer.cancel()
            self.avoidance_timer = None
        
        # FSM 상태를 기본값인 'FOLLOWING'으로 리셋
        self.state = "FOLLOWING"
        self.avoidance_turn_direction = None

        # 정지 명령(모든 속도가 0인 Twist 메시지) 발행
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        # PID 제어기 내부 누적값(integral) 등을 초기화하여 과거의 오차가 미래에 영향을 주지 않도록 함
        self.distance_pid.reset()
        self.angle_pid.reset()

def main(args=None):
    rclpy.init(args=args)
    node = None
    # try...finally 구문을 사용하여 Ctrl+C (KeyboardInterrupt)로 노드를 종료할 때
    # 로봇이 안전하게 정지하도록 보장합니다.
    try:
        node = AdvancedAssistFollowFSM()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info('키보드 인터럽트로 노드를 종료합니다.')
    finally:
        # 프로그램이 어떤 이유로든 종료될 때, 로봇을 정지시키고 노드를 소멸시킴
        if node and rclpy.ok():
            node.get_logger().info('안전한 종료를 위해 로봇을 정지합니다.')
            node.stop_robot()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()