from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_handler',
            executable='cam_sender',
            name='cam_sender',
            output='screen'
        ),
        Node(
            package='hardware_handler',
            executable='io_controller_node',
            name='io_controller_node',
            output='screen'
        ),
        Node(
            package='hardware_handler',
            executable='mic_streamer',
            name='mic_streamer',
            output='screen'
        ),
        Node(
            package='hardware_handler',
            executable='rgb_led_controller',
            name='rgb_led_controller',
            output='screen'
        ),
        Node(
            package='hardware_handler',
            executable='speaker_node',
            name='speaker_node',
            output='screen'
        ),
    ])
