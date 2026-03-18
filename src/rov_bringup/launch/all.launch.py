from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    left_video_topic = LaunchConfiguration("left_video_topic")
    right_video_topic = LaunchConfiguration("right_video_topic")

    loggers = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('rov_bringup'),
                'launch',
                "loggers.launch.py"
                ]),
                launch_arguments={
                    "left_video_topic": left_video_topic,
                    "right_video_topic": right_video_topic
                }.items()
            )

    rov = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('rov_bringup'),
                'launch',
                "rov.launch.py"
                ])
            )

    # Return a launch description generated from node list
    return LaunchDescription([
            DeclareLaunchArgument("left_video_topic", default_value="/gbr/cam_left/image_raw/compressed"),
            DeclareLaunchArgument("right_video_topic", default_value="/gbr/cam_right/image_raw/compressed"),
            loggers,
            rov
        ])
