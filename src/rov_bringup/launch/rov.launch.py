from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rov_controller = Node(
        package="ROV_controller",
        executable="ROV_controller",
        name="rov_controller",
        namespace="/gbr"
    )

    controller_input = Node(
        package="controller_input",
        executable="controller_input",
        name="controller_input",
        namespace="/gbr"
    )

    # Return a launch description generated from node list
    return LaunchDescription([
        rov_controller,
        controller_input
    ])
