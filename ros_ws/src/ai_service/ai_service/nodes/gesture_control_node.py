#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from libo_interfaces.msg import DetectionStatus, GestureResult, TalkCommand, VoiceCommand
from libo_interfaces.srv import ActivateGesture, DeactivateGesture

class GestureControlNode(Node):
    """
    제스처 인식 기반 제어 노드.
    [수정]
    - 지능형 회전: 회전 방향 변경 시 부드럽게 전환합니다.
    - 장애물 탈출 로직: 전/후방 장애물 감지 시, 특정 탈출 명령만 허용합니다.
    """
    def __init__(self):
        super().__init__('gesture_control_node')

        self.declare_parameter('forward_vel', 0.2)
        self.declare_parameter('backward_vel', -0.1)
        self.declare_parameter('turn_vel', 0.3)
        self.declare_parameter('stop_duration_s', 0.3) # 부드러운 정지에 걸리는 시간

        self.forward_vel = self.get_parameter('forward_vel').value
        self.backward_vel = self.get_parameter('backward_vel').value
        self.turn_vel = self.get_parameter('turn_vel').value
        self.stop_duration = self.get_parameter('stop_duration_s').value

        self.gesture_activated = False
        self.is_paused_by_command = False
        self.obstacle_status = None
        self.last_cmd_vel = Twist()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_cmd_pub = self.create_publisher(VoiceCommand, '/voice_command', 10)
        
        self.gesture_result_sub = self.create_subscription(GestureResult, '/gesture_result', self.gesture_result_callback, 10)
        self.detection_status_sub = self.create_subscription(DetectionStatus, '/detection_status', self.detection_status_callback, 10)
        self.talk_command_sub = self.create_subscription(TalkCommand, '/talk_command', self.talk_command_callback, 10)
        
        self.activate_srv = self.create_service(ActivateGesture, '/activate_gesture', self.handle_activate_gesture)
        self.deactivate_srv = self.create_service(DeactivateGesture, '/deactivate_gesture', self.handle_deactivate_gesture)
        self.arrived_srv = self.create_service(Trigger, '/trigger_arrival', self.handle_arrival_trigger)

        self.get_logger().info('✅ 제스처 인식 제어 노드 (장애물 탈출 로직 적용)가 시작되었습니다.')

    # --- [함수 수정] 새로운 장애물 탈출 로직 적용 ---
    def gesture_result_callback(self, msg: GestureResult):
        if not self.gesture_activated or self.is_paused_by_command:
            return
            
        if self.obstacle_status is None:
            self.get_logger().info('장애물 감지 정보 수신 대기 중...', once=True)
            return

        gesture = msg.gesture

        # 1. 최우선 순위: 장애물 상황별 제어
        # [NEW] 전방 장애물: 'back' 명령만 허용
        if self.obstacle_status.center_detected:
            if gesture == 'back':
                self.get_logger().info("전방 장애물 감지 중... 후진 명령을 수행합니다.")
                cmd_msg = Twist()
                cmd_msg.linear.x = self.backward_vel
                self.cmd_vel_pub.publish(cmd_msg)
                self.last_cmd_vel = cmd_msg
            else:
                self.get_logger().warn('🚨 전방 장애물! 후진 외 다른 동작은 불가능합니다. 정지합니다.', throttle_duration_sec=1)
                self.stop_robot()
            return

        # [NEW] 후방 장애물: 'go' 명령만 허용
        if self.obstacle_status.rear_detected:
            if gesture == 'go':
                self.get_logger().info("후방 장애물 감지 중... 전진 명령을 수행합니다.")
                cmd_msg = Twist()
                cmd_msg.linear.x = self.forward_vel
                self.cmd_vel_pub.publish(cmd_msg)
                self.last_cmd_vel = cmd_msg
            else:
                self.get_logger().warn('🚨 후방 장애물! 전진 외 다른 동작은 불가능합니다. 정지합니다.', throttle_duration_sec=1)
                self.stop_robot()
            return
            
        # 양측 장애물: 모든 동작 비상 정지 (기존 로직 유지)
        if self.obstacle_status.left_detected and self.obstacle_status.right_detected:
            self.get_logger().warn('🚨 양측 동시 장애물 감지! 비상 정지합니다!', throttle_duration_sec=1)
            self.stop_robot()
            return

        # 2. 장애물이 없을 경우에만 일반 제스처 처리
        intended_cmd = Twist()
        if gesture == 'go':
            intended_cmd.linear.x = self.forward_vel
        elif gesture == 'back':
            intended_cmd.linear.x = self.backward_vel
        elif gesture == 'left':
            intended_cmd.angular.z = -self.turn_vel
        elif gesture == 'right':
            intended_cmd.angular.z = self.turn_vel
        elif gesture in ['stop', 'none']:
            self.stop_robot()
            return
        else:
            self.get_logger().warn(f"알 수 없는 제스처 수신: '{gesture}'")
            return

        # 3. 회전 방향 전환 시 부드러운 처리
        is_new_turn = intended_cmd.angular.z != 0
        was_turning = self.last_cmd_vel.angular.z != 0
        is_changing_direction = is_new_turn and was_turning and (intended_cmd.angular.z * self.last_cmd_vel.angular.z < 0)

        if is_changing_direction:
            self.get_logger().info('🔄 회전 방향 전환. 부드러운 전환을 위해 잠시 정지합니다.')
            self.stop_robot()

        # 4. 최종 명령 발행
        self.cmd_vel_pub.publish(intended_cmd)
        self.last_cmd_vel = intended_cmd

    def gradual_stop(self, steps=20):
        """현재 속도에서 점진적으로 감속하여 정지합니다."""
        initial_vel = self.last_cmd_vel
        
        if abs(initial_vel.linear.x) < 0.01 and abs(initial_vel.angular.z) < 0.01:
            return
            
        for i in range(steps + 1):
            ratio = 1.0 - (i / steps)
            cmd_msg = Twist()
            cmd_msg.linear.x = initial_vel.linear.x * ratio
            cmd_msg.angular.z = initial_vel.angular.z * ratio
            self.cmd_vel_pub.publish(cmd_msg)
            time.sleep(self.stop_duration / steps)
        
        self.last_cmd_vel = Twist()

    def stop_robot(self):
        """로봇을 부드럽게 정지시킵니다."""
        self.gradual_stop()

    def detection_status_callback(self, msg: DetectionStatus):
        self.obstacle_status = msg

    def talk_command_callback(self, msg: TalkCommand):
        if not self.gesture_activated or msg.robot_id != "libo_a":
            return
            
        if msg.action == "stop":
            if not self.is_paused_by_command:
                self.get_logger().info("명령으로 인해 동작을 일시 정지합니다.")
                self.is_paused_by_command = True
                self.stop_robot()
        elif msg.action == "activate":
            if self.is_paused_by_command:
                self.get_logger().info("명령으로 인해 동작을 재개합니다.")
                self.is_paused_by_command = False
    
    def handle_activate_gesture(self, request, response):
        self.get_logger().info("제스처 제어가 활성화되었습니다.")
        self.gesture_activated = True
        self.is_paused_by_command = False
        response.success = True
        response.message = "제스처 제어 활성화 완료"
        return response

    def handle_deactivate_gesture(self, request, response):
        self.get_logger().info("제스처 제어가 비활성화되었습니다.")
        self.gesture_activated = False
        self.stop_robot()
        response.success = True
        response.message = "제스처 제어 비활성화 완료"
        return response

    def handle_arrival_trigger(self, request, response):
        self.get_logger().info("목적지에 도착하여 모든 동작을 종료합니다.")
        self.gesture_activated = False
        self.send_voice_command("escort", "arrived")
        self.stop_robot()
        response.success = True
        return response
        
    def send_voice_command(self, category, action):
        msg = VoiceCommand()
        msg.robot_id = "libo_a"
        msg.category = category
        msg.action = action
        self.voice_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('키보드 인터럽트로 노드를 종료합니다.')
    finally:
        if node and rclpy.ok():
            node.get_logger().info('안전한 종료를 위해 로봇을 정지합니다.')
            node.stop_robot()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()