from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    log_topic = LaunchConfiguration("pressure_topic")
    log_path = LaunchConfiguration("log_path")
    logging_node = Node(
        package="log_to_file",
        executable="pressure_logger",
        name="pressure_logger",
        arguments=[log_topic, log_path]
    )

    # List containing nodes + launch variables to start on launch
    nodes_to_start = [
        DeclareLaunchArgument("pressure_topic"),
        DeclareLaunchArgument("log_path", default_value=""),
        logging_node
    ]

    # Return a launch description generated from node list
    return LaunchDescription(nodes_to_start)
