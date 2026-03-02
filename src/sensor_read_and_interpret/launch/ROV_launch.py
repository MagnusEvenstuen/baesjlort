from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_read_and_interpret',
            executable='sensor_subscriber_node',
            name='sensor_subscriber_node',
            output='screen'
        ),
        
        Node(
            package='ros2_orb_slam3',
            executable='stereo_vision_node',
            name='stereo_vision_node'
        ),
        
        Node(
            package='yolo_tac_objects_detection',
            executable='yolo_node',
            name='yolo_node'
        ),
    ])