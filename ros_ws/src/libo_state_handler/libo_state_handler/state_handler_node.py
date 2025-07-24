import rclpy
from rclpy.node import Node

# 2단계에서 만든 우리만의 메시지를 가져오자!
from libo_interfaces.msg import Heartbeat, RobotStatus

class StateHandlerNode(Node):
    """
    Libo 로봇의 초기 상태를 설정하고, 주기적으로 Heartbeat와 Status를 발행하는 노드
    """
    def __init__(self):
        super().__init__('state_handler_node')
        self.get_logger().info('Libo State Handler Node has been started! ✨')

        # --- 초기화 ---
        self.robot_id = 'libo_A'  # 로봇 ID 설정 (나중에 여러 대가 되면 바꿀 수 있어!)
        self.availability = 'available' # 현재 상태를 'available'로 초기화
        self.get_logger().info(f'[{self.robot_id}] is now {self.availability}.')

        # --- 퍼블리셔 생성 ---
        self.heartbeat_publisher_ = self.create_publisher(Heartbeat, 'robot_heartbeat', 10)
        self.status_publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)

        # --- 타이머 생성 (1초마다 콜백 함수 실행) ---
        timer_period = 1.0  # seconds
        self.create_timer(timer_period, self.publish_heartbeat)
        self.create_timer(timer_period, self.publish_status)

    def publish_heartbeat(self):
        """1초마다 Heartbeat 메시지를 발행"""
        msg = Heartbeat()
        msg.sender_id = self.robot_id
        msg.timestamp = self.get_clock().now().to_msg()
        self.heartbeat_publisher_.publish(msg)
        # 로그는 너무 많이 찍히면 정신 없으니, 필요할 때만 주석을 풀어서 확인하자!
        # self.get_logger().info(f'Publishing Heartbeat: ID={msg.sender_id}')

    def publish_status(self):
        """1초마다 RobotStatus 메시지를 발행"""
        msg = RobotStatus()
        msg.robot_id = self.robot_id
        msg.availability = self.availability
        msg.battery_percentage = 98.0  # 임시 배터리 값
        self.status_publisher_.publish(msg)
        self.get_logger().info(f'Status Update: [{msg.robot_id}] is {msg.availability}')


def main(args=None):
    rclpy.init(args=args)
    state_handler_node = StateHandlerNode()
    try:
        rclpy.spin(state_handler_node)
    except KeyboardInterrupt:
        state_handler_node.get_logger().info('🛑 Node stopped cleanly.')
    finally:
        state_handler_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()