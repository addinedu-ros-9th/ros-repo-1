#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libo_interfaces.srv import TaskRequest

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # TaskRequest 서비스 서버 생성
        self.service = self.create_service(
            TaskRequest,
            'task_request',
            self.task_request_callback
        )
        
        self.get_logger().info('🎯 Task Manager 시작됨 - task_request 서비스 대기 중...')
    
    def task_request_callback(self, request, response):
        """TaskRequest 서비스 콜백"""
        self.get_logger().info(f'📥 Task Request 받음!')
        self.get_logger().info(f'   - Task Type: {request.task_type}')
        self.get_logger().info(f'   - Task Data: {request.task_data}')
        
        # 응답 설정
        response.success = True
        response.message = "Task request 잘 받았음!"
        response.task_id = "task_001"
        
        self.get_logger().info(f'✅ Task Request 처리 완료: {response.message}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    task_manager = TaskManager()
    
    try:
        rclpy.spin(task_manager)
    except KeyboardInterrupt:
        pass
    finally:
        task_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
