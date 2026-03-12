from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    left_cam_path = LaunchConfiguration("left_cam_path")
    right_cam_path = LaunchConfiguration("right_cam_path")

    left_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="left_cam",
        namespace="/gbr/cam_left",
        parameters=[{ "video_device": left_cam_path }] 
    )

    right_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="right_cam",
        namespace="/gbr/cam_right",
        parameters=[{ "video_device": right_cam_path }] 
    )

    # Return a launch description generated from node list
    return LaunchDescription([
        DeclareLaunchArgument("left_cam_path", default_value="/dev/video0"),
        DeclareLaunchArgument("right_cam_path", default_value="/dev/video4"),
        left_cam,
        right_cam
    ])
