/*
* Originally adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
* Author: Azmyin Md. Kamal
* Version: 1.0
* Date: 01/01/2024
* Compatible for ROS2 Humble
*/

//* Import all necessary modules
#include "ros2_orb_slam3/common.hpp" //* equivalent to orbslam3_ros/include/common.h
#include "ros2_orb_slam3/stereo_node.hpp" //* equivalent to orbslam3_ros/include/common.h

//* main
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<StereoMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}

// ------------------------------------------------------------ EOF ---------------------------------------------


