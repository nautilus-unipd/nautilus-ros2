from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    system_position_arg = DeclareLaunchArgument(
        'system_position',
        default_value='front',
        description='Position of the camera system (front/back)'
    )
    
    left_camera_index_arg = DeclareLaunchArgument(
        'left_camera_index',
        default_value='0',
        description='Index of the left camera device'
    )
    
    right_camera_index_arg = DeclareLaunchArgument(
        'right_camera_index',
        default_value='1',
        description='Index of the right camera device'
    )
    
    quality_arg = DeclareLaunchArgument(
        'quality',
        default_value='85',
        description='Camera image quality (0-100, higher is better)'
    )
    
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('camera_module'),
        'config',
        'stereo_camera.yaml'
    )
    
    # Get launch configuration values
    system_position = LaunchConfiguration('system_position')
    left_camera_index = LaunchConfiguration('left_camera_index')
    right_camera_index = LaunchConfiguration('right_camera_index')
    quality = LaunchConfiguration('quality')
    
    return LaunchDescription([
        system_position_arg,
        left_camera_index_arg,
        right_camera_index_arg,
        quality_arg,
        
        Node(
            package='camera_module',
            executable='capture_node',
            name='capture_node_left',
            namespace=[system_position, '/camera_left'],
            parameters=[
                config_file,
                {
                    'camera_index': left_camera_index,
                    'system_position': system_position,
                    'camera_side': 'left',
                    'quality': quality
                }
            ]
        ),
        Node(
            package='camera_module',
            executable='capture_node',
            name='capture_node_right',
            namespace=[system_position, '/camera_right'],
            parameters=[
                config_file,
                {
                    'camera_index': right_camera_index,
                    'system_position': system_position,
                    'camera_side': 'right',
                    'quality': quality
                }
            ]
        )
    ])
