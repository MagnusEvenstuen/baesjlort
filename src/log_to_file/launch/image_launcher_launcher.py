from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    log_topic_1 = LaunchConfiguration("image_topic_1")
    log_topic_2 = LaunchConfiguration("image_topic_2")
    log_path = LaunchConfiguration("log_path")
    
    logging_node_1 = Node(
        package="log_to_file",
        executable="image_logger",
        name="image_logger_1",
        namespace="logger1",
        arguments=[log_topic_1, log_path]
    )
    
    logging_node_2 = Node(
        package="log_to_file",
        executable="image_logger", 
        name="image_logger_2",
        namespace="logger2",
        arguments=[log_topic_2, log_path]
    )

    nodes_to_start = [
        DeclareLaunchArgument("image_topic_1", description="First topic to log"),
        DeclareLaunchArgument("image_topic_2", description="Second topic to log"), 
        DeclareLaunchArgument("log_path", default_value="", description="Path to save logs"),
        logging_node_1,
        logging_node_2
    ]

    return LaunchDescription(nodes_to_start)