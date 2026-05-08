from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    left_video_topic = LaunchConfiguration("left_video_topic")
    right_video_topic = LaunchConfiguration("right_video_topic")
    left_info = os.path.join(get_package_share_directory("h264_receiver"), "config", "left.yaml")
    right_info = os.path.join(get_package_share_directory("h264_receiver"), "config", "right.yaml")

    use_nvidia = False

    left_cam_receiver = Node(
            package = "h264_receiver",
            executable = "h264_receiver",
            name = "left_cam_receiver",
            namespace = "/gbr/cam_left",
            parameters = [
                { "use_nvidia": use_nvidia },
                { "cam_info_path": left_info }
                ],
            arguments = [left_video_topic],
            output = "both"
        )
    left_rectify = Node(
        package = "image_proc",
        executable = "rectify_node",
        name = "rectify_node",
        namespace = "/gbr/cam_left",
        remappings = [
            ("image", "image_raw"),
        ],
        # output = "screen"
    )

    right_cam_receiver = Node(
            package = "h264_receiver",
            executable = "h264_receiver",
            name = "right_cam_receiver",
            namespace = "/gbr/cam_right",
            parameters = [
                { "use_nvidia": use_nvidia },
                { "cam_info_path": right_info }
                ],
            arguments = [right_video_topic],
            output = "both"
        )

    right_rectify = Node(
        package = "image_proc",
        executable = "rectify_node",
        name = "rectify_node",
        namespace = "/gbr/cam_right",
        remappings = [
            ("image", "image_raw"),
        ],
        # output = "screen"
    )

    # Return a launch description generated from node list
    return LaunchDescription([
            DeclareLaunchArgument("left_video_topic", default_value="image_compressed"),
            DeclareLaunchArgument("right_video_topic", default_value="image_compressed"),
            left_cam_receiver,
            right_cam_receiver,
            # left_rectify,
            # right_rectify
        ])
