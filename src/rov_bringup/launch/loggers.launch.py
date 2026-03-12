from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    left_video_topic = LaunchConfiguration("left_video_topic")
    right_video_topic = LaunchConfiguration("right_video_topic")

    left_video_logger = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('log_to_file'),
                'launch',
                "video_logger.launch.py"
                ]),
                launch_arguments={
                    "video_topic": left_video_topic
                }.items()
            )

    right_video_logger = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('log_to_file'),
                'launch',
                "video_logger.launch.py"
                ]),
                launch_arguments={
                    "video_topic": right_video_topic
                }.items()
            )

    # Return a launch description generated from node list
    return LaunchDescription([
            DeclareLaunchArgument("left_video_topic", default_value="/gbr/cam_left/image_raw"),
            DeclareLaunchArgument("right_video_topic", default_value="/gbr/cam_right/image_raw"),
            left_video_logger,
            right_video_logger
        ])
