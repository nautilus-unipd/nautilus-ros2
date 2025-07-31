from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='debug_server',
            executable='debug_server',
            name='stereo_debug_server',
            parameters=[{
                'left_camera_topic': '/camera_left/image',
                'right_camera_topic': '/camera_right/image',
                'web_port': 8080
            }]
        )
    ])
