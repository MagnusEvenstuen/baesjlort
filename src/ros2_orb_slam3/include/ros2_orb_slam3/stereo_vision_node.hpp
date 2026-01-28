#ifndef STEREO_VISION_NODE_HPP
#define STEREO_VISION_NODE_HPP

//This code is inspired by https://github.com/MagnusEvenstuen/baesjlort/blob/632bd9b6321b4bde06fa1419d00f04613052ec7a/src/ros2_orb_slam3/include/ros2_orb_slam3/stereo_node.hpp which is AI generated

#include <memory>
#include <mutex>
#include <string>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cv_bridge/cv_bridge.hpp>

// ORB_SLAM3
#include "System.h"
#include "Sophus/sophus/se3.hpp"

// OpenCV
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

class stereo_vision_node : public rclcpp::Node
{
public: 
    stereo_vision_node() : Node("stereo_vision_node")
    {
        RCLCPP_INFO(this->get_logger(), "Slamming my balls with ORB_SLAM3 Stereo...");
        //TODO fix absolute paths
        slam_system_ = std::make_unique<ORB_SLAM3::System>(
            "/home/gud/Skole/baesjlort/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin",     //Path to vocabulary file 
            "/home/gud/Skole/baesjlort/src/ros2_orb_slam3/config/stereo_gbr_sim.yaml",               //Path to camera settings file 
            ORB_SLAM3::System::STEREO, 
            true                                                                                   //Disable viewer
        );

        //Publisher for estimated pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orb_slam3/pose",
            10
        );

        left_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/gbr/cam_left/image_color",
            10,
            std::bind(&stereo_vision_node::left_image_callback, this, std::placeholders::_1)
        );

        right_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/gbr/cam_right/image_color",
            10,
            std::bind(&stereo_vision_node::right_image_callback, this, std::placeholders::_1)
        );
    }

    ~stereo_vision_node()
    {
        if (slam_system_) {
            slam_system_->Shutdown();
        }
    }

private:
    void left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        //Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        left_image_ = cv_ptr->image;
        left_received_ = true;
        process_stereo_pair(msg->header);
    }

    void right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        //Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        right_image_ = cv_ptr->image;
        right_received_ = true;
        process_stereo_pair(msg->header);
    }

    void process_stereo_pair(const std_msgs::msg::Header& header)
    {
        //Check if both images have been received
        if (!left_received_ || !right_received_) {
            return;
        }

        //Process stereo images with ORB_SLAM3
        Sophus::SE3f pose = slam_system_->TrackStereo(left_image_, right_image_, rclcpp::Time(header.stamp).seconds());

        //Publish if valid pose
        if (!pose.matrix().isIdentity()) {
            publishPose(pose, header);
        }

        //Reset flags
        left_received_ = false;
        right_received_ = false;
    }

    void publishPose(const Sophus::SE3f& pose, const std_msgs::msg::Header& img_header)
    {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        
        //Set header
        pose_msg.header = img_header;
        pose_msg.header.frame_id = "world";  //ORB_SLAM3 coordinate system
        
        //Convert Sophus::SE3f to ROS pose
        //Sophus::SE3f contains rotation (so3) and translation (translation)
        Eigen::Quaternionf quat(pose.rotationMatrix());
        
        //Set position and orientation
        pose_msg.pose.position.x = pose.translation().x();
        pose_msg.pose.position.y = pose.translation().y();
        pose_msg.pose.position.z = pose.translation().z();
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();
        
        RCLCPP_INFO(this->get_logger(), "Publishing pose: [x: %.2f, y: %.2f, z: %.2f, w: %.2f, x: %.2f, y: %.2f, z: %.2f]", 
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
            pose_msg.pose.orientation.w,
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z
        );

        //Publish pose
        pose_publisher_->publish(pose_msg);
    }

private:
    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    cv::Mat left_image_;
    cv::Mat right_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    bool left_received_ = false;
    bool right_received_ = false;
};



#endif // STEREO_VISION_NODE_HPP