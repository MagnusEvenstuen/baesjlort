from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    left_video_topic = LaunchConfiguration("left_video_topic")
    right_video_topic = LaunchConfiguration("right_video_topic")

    left_cam_receiver = Node(
            package="h264_receiver",
            executable="h264_receiver",
            name="left_cam_receiver",
            namespace="gbr/cam_left",
            arguments=[left_video_topic]
        )

    right_cam_receiver = Node(
            package="h264_receiver",
            executable="h264_receiver",
            name="right_cam_receiver",
            namespace="gbr/cam_right",
            arguments=[right_video_topic]
        )

    # Return a launch description generated from node list
    return LaunchDescription([
            DeclareLaunchArgument("left_video_topic", default_value="image_compressed"),
            DeclareLaunchArgument("right_video_topic", default_value="image_compressed"),
            left_cam_receiver,
            right_cam_receiver
        ])
