from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_service',
            executable='talker_manager',
            name='talker_manager',
            output='screen'
        ),
        Node(
            package='ai_service',
            executable='velocity_manager',
            name='velocity_manager',
            output='screen'
        ),
        Node(
            package='ai_service',
            executable='vision_manager',
            name='vision_manager',
            output='screen'
        ),
    ])
