from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('camera_module'),
        'config',
        'stereo_camera.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='camera_module',
            executable='capture_node',
            name='capture_node_left',
            namespace='front/camera_left',
            parameters=[
                config_file,
                {
                    'camera_index': 0,  # Override camera_index for left camera
                    'system_position': 'front',
                    'camera_side': 'left',
                    'quality': 85  # High quality for front cameras
                }
            ]
        ),
        Node(
            package='camera_module',
            executable='capture_node',
            name='capture_node_right',
            namespace='front/camera_right',
            parameters=[
                config_file,
                {
                    'camera_index': 1,  # Override camera_index for right camera
                    'system_position': 'front',
                    'camera_side': 'right',
                    'quality': 85  # High quality for front cameras
                }
            ]
        )
    ])
