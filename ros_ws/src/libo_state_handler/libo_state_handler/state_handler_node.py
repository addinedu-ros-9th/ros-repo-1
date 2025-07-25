import rclpy
from rclpy.node import Node

# 우리가 정의한 커스텀 메시지들을 가져옵니다.
from libo_interfaces.msg import Heartbeat, RobotStatus

class StateHandlerNode(Node):
    """
    Libo 로봇의 초기 상태를 설정하고, 주기적으로 Heartbeat와 Status를 발행하는 노드 (v2)
    """
    def __init__(self):
        # 노드 초기화 및 로거 설정
        super().__init__('state_handler_node')
        self.get_logger().info('Libo State Handler Node (v2) has been started! ✨')

        self.robot_id = 'libo_01'
        self.availability = 'available'
        
        self.mode = 'idle'
        self.get_logger().info(f'[{self.robot_id}] is now {self.availability} in {self.mode} mode.')

        # --- 퍼블리셔 생성 ---
        # [cite_start]Heartbeat 메시지를 '/robot_heartbeat' 토픽으로 발행합니다.
        self.heartbeat_publisher_ = self.create_publisher(Heartbeat, 'robot_heartbeat', 10)
        
        # [cite_start]RobotStatus 메시지를 '/robot_status' 토픽으로 발행합니다.
        self.status_publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)

        # --- 타이머 생성 (1초마다 콜백 함수 실행) ---
        timer_period = 1.0  # 1초
        # 1초마다 periodic_publish 콜백 함수를 실행하여 메시지를 주기적으로 발행합니다.
        self.timer = self.create_timer(timer_period, self.periodic_publish)

    def periodic_publish(self):
        """1초마다 Heartbeat와 RobotStatus 메시지를 생성하고 발행합니다."""
        # 현재 시간을 한 번만 가져와서 모든 메시지에 동일하게 사용합니다.
        current_time = self.get_clock().now().to_msg()

        # --- Heartbeat 메시지 발행 ---
        heartbeat_msg = Heartbeat()
        heartbeat_msg.sender_id = self.robot_id
        heartbeat_msg.timestamp = current_time
        self.heartbeat_publisher_.publish(heartbeat_msg)

        # --- RobotStatus 메시지 발행 (명세서에 맞게 수정) ---
        status_msg = RobotStatus()
        status_msg.sender_id = self.robot_id
        status_msg.availability = self.availability
        status_msg.mode = self.mode
        status_msg.battery_level = 98.0  # 임시 배터리 값 (float32)
        status_msg.timestamp = current_time
        self.status_publisher_.publish(status_msg)
        
        self.get_logger().info(f'Status Update: [{status_msg.sender_id}] is {status_msg.availability} ({status_msg.mode})')


def main(args=None):
    rclpy.init(args=args)
    state_handler_node = StateHandlerNode()
    try:
        # 노드를 계속 실행하며 콜백 함수를 처리합니다.
        rclpy.spin(state_handler_node)
    except KeyboardInterrupt:
        state_handler_node.get_logger().info('🛑 Node stopped cleanly by user.')
    finally:
        # 노드 종료 시 자원을 정리합니다.
        state_handler_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()