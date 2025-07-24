#!/usr/bin/env python3
import sys
import rclpy
from libo_interfaces.srv import Navigate

def main(args=None):
    rclpy.init(args=args)

    # 터미널에서 입력받은 인자가 4개가 아니면 사용법을 알려주고 종료해
    # (파일이름, x, y, yaw) 이렇게 4개여야 해
    if len(sys.argv) != 4:
        print("사용법: ros2 run admin debug_tool <x> <y> <yaw>")
        return

    # 서비스 클라이언트로 사용할 임시 노드를 만들어
    node = rclpy.create_node('debug_tool_client')
    # 'navigate' 서비스를 호출할 클라이언트를 생성해
    client = node.create_client(Navigate, 'navigate')

    # 서비스 서버가 켜질 때까지 1초마다 확인하며 기다려
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('서비스 연결 대기중... (robot_commander 실행했어?)')

    # 서비스에 보낼 요청 메시지를 만들어
    request = Navigate.Request()
    request.x = float(sys.argv[1]) # 첫 번째 인자를 x 좌표로
    request.y = float(sys.argv[2]) # 두 번째 인자를 y 좌표로
    request.yaw = float(sys.argv[3]) # 세 번째 인자를 yaw 각도로

    # 서비스를 비동기적으로 호출하고 응답을 기다려
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # 응답 결과를 확인하고 로그를 남겨
    try:
        response = future.result()
        node.get_logger().info(f"서비스 응답 받음: 성공={response.success}, 메시지='{response.message}'")
    except Exception as e:
        node.get_logger().error(f'서비스 호출 실패! {e}')

    # 노드를 깔끔하게 종료해
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()