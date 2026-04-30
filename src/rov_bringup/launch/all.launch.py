from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    left_compressed_video_topic = LaunchConfiguration("left_compressed_video_topic")
    right_compressed_video_topic = LaunchConfiguration("right_compressed_video_topic")
    left_raw_video_topic = LaunchConfiguration("left_raw_video_topic")
    right_raw_video_topic = LaunchConfiguration("right_raw_video_topic")

    receivers = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('h264_receiver'),
                'launch',
                "receivers.launch.py"
                ]),
                launch_arguments={
                    "left_video_topic": left_compressed_video_topic,
                    "right_video_topic": right_compressed_video_topic
                }.items()
            )

    loggers = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('rov_bringup'),
                'launch',
                "loggers.launch.py"
                ]),
                launch_arguments={
                    "left_video_topic": left_raw_video_topic,
                    "right_video_topic": right_raw_video_topic
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
            DeclareLaunchArgument("left_compressed_video_topic", default_value="/gbr/cam_left/image_compressed"),
            DeclareLaunchArgument("right_compressed_video_topic", default_value="/gbr/cam_right/image_compressed"),
            DeclareLaunchArgument("left_raw_video_topic", default_value="/gbr/cam_left/image_rect"),
            DeclareLaunchArgument("right_raw_video_topic", default_value="/gbr/cam_right/image_rect"),
            receivers,
            loggers,
            rov
        ])
