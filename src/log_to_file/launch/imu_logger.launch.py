import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    log_path = LaunchConfiguration("log_path")
    logging_node = Node(
        package="log_to_file",
        executable="imu_logger",
        name="imu_logger",
        arguments=[log_path]
    )

    # List containing nodes + launch variables to start on launch
    nodes_to_start = [
        DeclareLaunchArgument("log_path", default_value=""),
        logging_node
    ]

    # Return a launch description generated from node list
    return LaunchDescription(nodes_to_start)
