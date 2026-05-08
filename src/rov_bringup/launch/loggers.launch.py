from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    left_video_topic = LaunchConfiguration("left_video_topic")
    right_video_topic = LaunchConfiguration("right_video_topic")

    navigator_imu_topic = LaunchConfiguration("navigator_imu_topic")

    return LaunchDescription([
        DeclareLaunchArgument("left_video_topic", default_value="/gbr/cam_left/image_raw"),
        DeclareLaunchArgument("right_video_topic", default_value="/gbr/cam_right/image_raw"),
        DeclareLaunchArgument("navigator_imu_topic", default_value="/gbr/navigator_imu"),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 left_video_topic,
                 right_video_topic,
                 navigator_imu_topic,
                 "/gbr/imu1",
                 "/gbr/imu2",
                 "/gbr/imu3",
                 "/gbr/thrusters",
                 "/gbr/pressure"
                 ],
            cwd=f"{os.getcwd()}/rosbag",
            output='screen'
        )
    ])
