#ifndef STEREO_VISION_NODE_HPP
#define STEREO_VISION_NODE_HPP

//This code is inspired by https://github.com/MagnusEvenstuen/baesjlort/blob/632bd9b6321b4bde06fa1419d00f04613052ec7a/src/ros2_orb_slam3/include/ros2_orb_slam3/stereo_node.hpp which is AI generated

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include <chrono>
//Fix absolute path later
#include "/home/gud/Skole/baesjlort/src/sensor_read_and_interpret/include/structs.hpp"

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
            30
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

        //Subscribes to averaged IMU data from sensor_subscriber node
        //avg_gyro_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //    "/average_gyro",
        //    1000,
        //    std::bind(&stereo_vision_node::avg_gyro_callback, this, std::placeholders::_1)
        //);

        //avg_acc_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //    "/average_acceleration",
        //    1000,
        //    std::bind(&stereo_vision_node::avg_acc_callback, this, std::placeholders::_1)
        //);

        imu_center_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_center_perfect", 
            100,
            std::bind(&stereo_vision_node::imu_center_callback, this, std::placeholders::_1)
        );

        //Publisher for thruster commands to create movement during initialization
        thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gbr/thrusters",
            10
        );

        init_start_time_ = this->now().seconds();
        
        //init_timer_ = this->create_wall_timer(
        //    std::chrono::milliseconds(100),
        //    std::bind(&stereo_vision_node::init, this)
        //);
        

        //this->create_wall_timer(
        //    std::chrono::seconds(250),
        //    [this]() {
        //        RCLCPP_INFO(this->get_logger(), "Initialization complete!");
        //        init_timer_->cancel();  // Stopp initialiseringsløkken
        //        auto msg = std_msgs::msg::Float64MultiArray();
        //        msg.data = std::vector<double>({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
        //        thrust_publisher_->publish(msg);
        //    }
        //);
    }

    ~stereo_vision_node()
    {
        if (slam_system_) {
            slam_system_->Shutdown();
        }
    }

private:
    void init()
    {
        if (this->now().seconds() - init_start_time_ < 5.0)         //Back
        {
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = std::vector<double>({-10.0f, -10.0f, 10.0f, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f});
            thrust_publisher_->publish(msg);
        } else if(this->now().seconds() - init_start_time_ < 10.0)      //Left
        {
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = std::vector<double>({3.0f, -6.0f, 3.0f, -6.0f, 0.0f, 0.0f, 0.0f, 0.0f});
            thrust_publisher_->publish(msg);
        } else if(this->now().seconds() - init_start_time_ < 15.0)      //Forward
        {
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = std::vector<double>({3.0f, 3.0f, -3.0f, -3.0f, -10.0f, -10.0f, -10.0f, 0.0f});
            thrust_publisher_->publish(msg);
        } else if(this->now().seconds() - init_start_time_ < 20.0)      //Right
        {
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = std::vector<double>({-3.0f, 3.0f, -3.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f});
            thrust_publisher_->publish(msg);
        }
        else if(this->now().seconds() - init_start_time_ < 25.0)        //Still
        {
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = std::vector<double>({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
            thrust_publisher_->publish(msg);
            init_start_time_ = this->now().seconds();
        }
    }

    void left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Slamming left ball...");
        std::lock_guard<std::mutex> lock(image_mutex_);
        //Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        left_image_ = cv_ptr->image;
        left_received_ = true;
        image_msg_ = msg;
    }

    void right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Slamming right ball...");
        std::lock_guard<std::mutex> lock(image_mutex_);
        //Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        right_image_ = cv_ptr->image;
        right_received_ = true;
        image_msg_ = msg;
    }

    void imu_center_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Receiving IMU center data...");
        std::lock_guard<std::mutex> lock(imu_mutex_);
        // Store gyro data with current timestamp
        current_gyro_ = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        current_acc_ = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        last_imu_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        RCLCPP_WARN(this->get_logger(), "Time: %f", last_imu_time_);

        //Both gyro and acc data are available, add to IMU buffer
        ORB_SLAM3::IMU::Point imu_point = {
            current_acc_.x, current_acc_.y, current_acc_.z,
            current_gyro_.x, current_gyro_.y, current_gyro_.z,
            last_imu_time_
        };
        imu_buffer_.push_back(imu_point);

        //Reset current data to indicate they have been used
        current_gyro_ = {-10000.0, -10000.0, -10000.0};
        current_acc_ = {-10000.0, -10000.0, -10000.0};
        process_stereo_pair(image_msg_->header);
    }

    void avg_gyro_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Rotating my balls...");
        std::lock_guard<std::mutex> lock(imu_mutex_);
        // Store gyro data with current timestamp
        current_gyro_ = {msg->data[0], msg->data[1], msg->data[2]};
        last_imu_time_ = this->now().seconds();
        
        ORB_SLAM3::IMU::Point imu_point = {
                msg->data[0], msg->data[1], msg->data[2],
                msg->data[3], msg->data[4], msg->data[5],
                last_imu_time_
            };
        imu_buffer_.push_back(imu_point);

        current_acc_.x = -10000.0;
        if (current_acc_.x != -10000.0 && 
            current_acc_.y != -10000.0 && 
            current_acc_.z != -10000.0) 
        {
            //Both gyro and acc data are available, add to IMU buffer
            ORB_SLAM3::IMU::Point imu_point = {
                current_acc_.x, current_acc_.y, current_acc_.z,
                current_gyro_.x, current_gyro_.y, current_gyro_.z,
                last_imu_time_
            };
            imu_buffer_.push_back(imu_point);

            //Reset current data to indicate they have been used
            current_gyro_ = {-10000.0, -10000.0, -10000.0};
            current_acc_ = {-10000.0, -10000.0, -10000.0};
        }
    }

    void avg_acc_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Accelerating my balls...");
        std::lock_guard<std::mutex> lock(imu_mutex_);
        // Store acceleration data with current timestamp
        current_acc_ = {msg->data[0], msg->data[1], msg->data[2]};
        last_imu_time_ = this->now().seconds();

        current_gyro_.x = -10000.0;
        //Checking if numbers are valid
        if (current_gyro_.x != -10000.0 && 
            current_gyro_.y != -10000.0 && 
            current_gyro_.z != -10000.0) 
        {
            //Both gyro and acc data are available, add to IMU buffer
            ORB_SLAM3::IMU::Point imu_point = {
                current_acc_.x, current_acc_.y, current_acc_.z,
                current_gyro_.x, current_gyro_.y, current_gyro_.z,
                last_imu_time_
            };
            imu_buffer_.push_back(imu_point);

            //Reset current data to indicate they have been used
            current_gyro_ = {-10000.0, -10000.0, -10000.0};
            current_acc_ = {-10000.0, -10000.0, -10000.0};
        }
    }


    //This function could be improved with better synchronization if needed
    void process_stereo_pair(const std_msgs::msg::Header& header)
    {
        //RCLCPP_INFO(this->get_logger(), "Processing my balls...");
        //Check if both images have been received
        if (!left_received_ || !right_received_) 
        {
            return;
        }

        //std::lock_guard<std::mutex> imu_lock(imu_mutex_);
        std::lock_guard<std::mutex> imu_lock(image_mutex_);
        //Process stereo images with ORB_SLAM3
        if (imu_buffer_.empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "IMU buffer is empty, skipping frame...");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Calculating balls pose...");
        Sophus::SE3f pose = slam_system_->TrackStereo(left_image_, 
                                                    right_image_, 
                                                    header.stamp.sec + header.stamp.nanosec * 1e-9);
        RCLCPP_INFO(this->get_logger(), "Calculated balls pose...");

        //Only keep recent IMU data
        imu_buffer_.clear();
        //Publish if valid pose
        if (!pose.matrix().isIdentity()) 
        {
            publishPose(pose, header);
        } else 
        {
            RCLCPP_WARN(this->get_logger(), "Pose is identity, not publishing...");
        }

        left_received_ = false;
        right_received_ = false;
    }

    void publishPose(const Sophus::SE3f& pose, const std_msgs::msg::Header& img_header)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing my balls...");
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        
        //Set header
        pose_msg.header = img_header;
        pose_msg.header.frame_id = "world";
        
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
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr avg_gyro_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr avg_acc_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_center_;
    double last_imu_time_;
    Vector3 current_gyro_ = {-10000.0, -10000.0, -10000.0};
    Vector3 current_acc_ = {-10000.0, -10000.0, -10000.0};
    std::vector<ORB_SLAM3::IMU::Point> imu_buffer_;
    std::mutex imu_mutex_;
    std::mutex image_mutex_;
    double init_start_time_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    bool left_received_ = false;
    bool right_received_ = false;
    sensor_msgs::msg::Image::SharedPtr image_msg_;
};



#endif // STEREO_VISION_NODE_HPP