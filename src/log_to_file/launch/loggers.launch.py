from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    left_video_topic = LaunchConfiguration("left_video_topic")
    right_video_topic = LaunchConfiguration("right_video_topic")

    imu_topic = LaunchConfiguration("imu_topic")

    # left_video_logger = GroupAction(
    #         actions=[
    #             PushROSNamespace("/gbr/cam_left"),
    #             IncludeLaunchDescription(
    #                 PathJoinSubstitution([
    #                     FindPackageShare('log_to_file'),
    #                     'launch',
    #                     "video_logger.launch.py"
    #                     ]),
    #                 launch_arguments={
    #                     "video_topic": left_video_topic
    #                     }.items()
    #                 )
    #             ]
    #         )

    left_video_logger = GroupAction(
            actions=[
                PushROSNamespace("/gbr/cam_left"),
                IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare('log_to_file'),
                        'launch',
                        "image_logger.launch.py"
                        ]),
                    launch_arguments={
                        "image_topic": left_video_topic
                        }.items()
                    )
                ]
            )

    # right_video_logger = GroupAction(
    #         actions=[
    #             PushROSNamespace("/gbr/cam_right"),
    #             IncludeLaunchDescription(
    #                 PathJoinSubstitution([
    #                     FindPackageShare('log_to_file'),
    #                     'launch',
    #                     "video_logger.launch.py"
    #                     ]),
    #                 launch_arguments={
    #                     "video_topic": right_video_topic
    #                     }.items()
    #                 )
    #             ]
    #         )

    right_video_logger = GroupAction(
            actions=[
                PushROSNamespace("/gbr/cam_right"),
                IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare('log_to_file'),
                        'launch',
                        "image_logger.launch.py"
                        ]),
                    launch_arguments={
                        "image_topic": right_video_topic
                        }.items()
                    )
                ]
            )

    imu_logger = GroupAction(
            actions=[
                PushROSNamespace("/gbr"),
                IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare('log_to_file'),
                        'launch',
                        "imu_logger.launch.py"
                        ]),
                    launch_arguments={
                        "imu_topic": imu_topic
                        }.items()
                    )
                ]
            )

    # Return a launch description generated from node list
    return LaunchDescription([
            DeclareLaunchArgument("left_image_topic", default_value="image_rect"),
            DeclareLaunchArgument("right_image_topic", default_value="image_rect"),
            DeclareLaunchArgument("imu_topic", default_value="navigator_imu"),
            left_video_logger,
            right_video_logger,
            imu_logger
        ])
