from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Define some names
    name_package = "node_pump"
    name_node_pump = "node_pump"
    # Retrieve node pump parameters
    param_node_pump = PathJoinSubstitution([FindPackageShare(name_package), "config", "node_pump_config.yaml"])
    # Create and configure the node
    launch_node_pump = Node(
        package = name_package,
        executable = name_node_pump,
        name = name_node_pump,
        namespace = name_package,
        parameters = [param_node_pump],
        output = "log",
    )
    # Return the node
    return LaunchDescription([
        launch_node_pump
    ])
