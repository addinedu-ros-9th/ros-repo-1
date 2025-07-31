from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """무게 센서 수신 노드 launch 파일"""
    
    weight_subscriber_node = Node(
        package='main_server',
        executable='weight_subscriber',
        name='weight_subscriber',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    return LaunchDescription([
        weight_subscriber_node,
    ]) 