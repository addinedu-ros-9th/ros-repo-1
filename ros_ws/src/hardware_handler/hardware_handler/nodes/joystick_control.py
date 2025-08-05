#!/usr/bin/env python3
# encoding: utf-8

import math
import rclpy
from enum import Enum
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


# ─────────────────────────────
def val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
# ─────────────────────────────

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.min_value = 0.1
        self.declare_parameter('max_linear', 0.2)
        self.declare_parameter('max_angular', 0.2)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.get_logger().info(f'Max linear velocity: {self.max_linear}')

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        self.last_axes = dict(zip(AXES_MAP, [0.0] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0] * len(BUTTON_MAP)))

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('Joystick controller started')

        # 비상정지
        self.nav2_lifecycle_client = self.create_client(
            ChangeState,
            '/controller_server/change_state'
        )
        # while not self.nav2_lifecycle_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warn('⏳ Waiting for /controller_server/change_state service...')


    def send_lifecycle_request(self, transition_id: int, description: str):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.nav2_lifecycle_client.call_async(req)

        def callback(f):
            try:
                result = f.result()
                self.get_logger().warn(f"{description} 완료됨: success={result.success}")
            except Exception as e:
                self.get_logger().error(f"Lifecycle request 실패: {str(e)}")

        future.add_done_callback(callback)


    def get_node_state(self, request, response):
        response.success = True
        return response

    def axes_callback(self, axes):
        twist = Twist()

        # Deadzone 처리
        for key in ['lx', 'ly', 'rx', 'ry']:
            if abs(axes[key]) < self.min_value:
                axes[key] = 0.0

        # 기본 메커넘/탱크 스타일 이동 처리
        twist.linear.x = val_map(axes['ly'], -1, 1, -self.max_linear, self.max_linear)
        twist.linear.y = val_map(axes['lx'], -1, 1, -self.max_linear, self.max_linear)
        twist.angular.z = val_map(axes['rx'], -1, 1, -self.max_angular, self.max_angular)
        # twist.angular.z = val_map(-axes['rx'], -1, 1, -self.max_angular, self.max_angular)


        self.cmd_vel_pub.publish(twist)

    def select_callback(self, new_state): pass
    def l1_callback(self, new_state): pass
    def l2_callback(self, new_state): pass
    def r1_callback(self, new_state): pass
    def r2_callback(self, new_state): pass

    def square_callback(self, new_state):
        if new_state in [ButtonState.Pressed, ButtonState.Holding]:
            print("1")
       

    def cross_callback(self, new_state):
        # if new_state in [ButtonState.Pressed, ButtonState.Holding]:
        #     twist = Twist()
        #     self.cmd_vel_pub.publish(twist)
        #     self.get_logger().warn("❌ X 버튼으로 강제 정지 유지 중")
        # A button
        if new_state in [ButtonState.Pressed, ButtonState.Holding]:
            self.get_logger().warn("💥 X 버튼 - 비상정지: controller_server deactivate")
            self.send_lifecycle_request(Transition.TRANSITION_DEACTIVATE, "Nav2 정지")
            self.cmd_vel_pub.publish(Twist())  # 즉시 정지
        # elif new_state == ButtonState.Released:
        #     self.get_logger().info("✅ X 버튼 해제 - controller_server activate")
        #     self.send_lifecycle_request(Transition.TRANSITION_ACTIVATE, "Nav2 재시작")

    def circle_callback(self, new_state): 
        # B button
        if new_state in [ButtonState.Pressed, ButtonState.Holding]:
           self.get_logger().info("✅ circle")
           self.get_logger().info("✅ X 버튼 해제 - controller_server activate")
           self.send_lifecycle_request(Transition.TRANSITION_ACTIVATE, "Nav2 재시작")


    def triangle_callback(self, new_state): pass
    def start_callback(self, new_state): pass
    def hat_xl_callback(self, new_state): pass
    def hat_xr_callback(self, new_state): pass
    def hat_yd_callback(self, new_state): pass
    def hat_yu_callback(self, new_state): pass

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
        buttons = list(joy_msg.buttons)
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))

        axes_changed = any(self.last_axes[k] != v for k, v in axes.items())
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                self.get_logger().error(str(e))

        for key, value in buttons.items():
            prev = self.last_buttons[key]
            if value != prev:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal

            if new_state != ButtonState.Normal:
                callback_name = f"{key}_callback"
                if hasattr(self, callback_name):
                    try:
                        getattr(self, callback_name)(new_state)
                    except Exception as e:
                        self.get_logger().error(str(e))

        self.last_axes = axes
        self.last_buttons = buttons

def main():
    node = JoystickController('joystick_control')
    rclpy.spin(node)

if __name__ == "__main__":
    main()
