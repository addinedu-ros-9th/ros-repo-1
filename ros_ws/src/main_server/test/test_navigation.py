#!/usr/bin/env python3

import rclpy
import sys
import os

# 상위 디렉토리의 main_server 모듈을 찾을 수 있도록 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# robot_commander에서 SimpleNavigator 클래스를 임포트
from main_server.services.robot_commander import SimpleNavigator

def main(args=None):
    """
    로봇 네비게이션 테스트를 위한 메인 함수.
    원하는 좌표를 설정하고 직접 실행하여 테스트할 수 있습니다.
    """
    rclpy.init(args=args)
    
    navigator = SimpleNavigator()

    # --- 여기에 실험하고 싶은 좌표를 넣으세요 ---
    # 예: (X=1.5, Y=2.0) 위치로 90도 방향을 보며 이동
    x = 1.5
    y = 2.0
    yaw = 90.0
    # -----------------------------------------
    
    navigator.get_logger().info(f"테스트 목표 전송: X={x}, Y={y}, Yaw={yaw}")
    navigator.send_goal(x=x, y=y, yaw_degrees=yaw)

    # 액션이 완료되거나 Ctrl+C로 종료할 때까지 spin
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('사용자에 의해 노드 종료')
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()