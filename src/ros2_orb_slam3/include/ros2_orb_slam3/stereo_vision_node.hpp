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
        //Information needed for ORB_SLAM3 initialization
        slam_system_ = std::make_unique<ORB_SLAM3::System>(
            "/home/gud/Skole/baesjlort/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin",     //Path to vocabulary file 
            "/home/gud/Skole/baesjlort/src/ros2_orb_slam3/config/stereo_gbr_sim.yaml",               //Path to camera settings file 
            ORB_SLAM3::System::STEREO, 
            true                                                                                   //Disable viewer
        );

        //ROS2 publishers and subscribers
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

        orientation_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/average_orientation",
            100,
            std::bind(&stereo_vision_node::orientation_callback, this, std::placeholders::_1)
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
        //Initial movement for testing. Can be removed later
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

    void orientation_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        //Sets the correct orientation from IMU data
        orientation_ = {
            msg->data[0],
            msg->data[1],
            msg->data[2],
            msg->data[3]
        };
    }

    void left_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        //Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        left_image_ = cv_ptr->image;
        //Set flag and process if both images are received
        left_received_ = true;
        image_msg_ = msg;
        process_stereo_pair(image_msg_->header);
    }

    void right_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        //Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        right_image_ = cv_ptr->image;
        //Set flag and process if both images are received
        right_received_ = true;
        image_msg_ = msg;
        process_stereo_pair(image_msg_->header);
    }


    //This function could be improved with better synchronization if needed
    void process_stereo_pair(const std_msgs::msg::Header& header)
    {
        static unsigned int pose_identity_counter = 0;
        //RCLCPP_INFO(this->get_logger(), "Processing my balls...");
        //Check if both images have been received
        if (!left_received_ || !right_received_) 
        {
            return;
        }

        //Process stereo images with ORB_SLAM3
        Sophus::SE3f pose = slam_system_->TrackStereo(left_image_, 
                                                    right_image_, 
                                                    header.stamp.sec + header.stamp.nanosec * 1e-9);

        //Publish if valid pose
        if (!pose.matrix().isIdentity()) 
        {
            publishPose(pose, header);
            pose_identity_counter = 0;  //Reset counter if we get a valid pose
        } else 
        {
            RCLCPP_WARN(this->get_logger(), "Pose is identity, not publishing...");
            pose_identity_counter++;
            if (pose_identity_counter >= 5)
            {
                //Reseting SLAM system after consecutive identity poses to improve speed estimation
                RCLCPP_WARN(this->get_logger(), "Pose is identity, to long. Resetting...");
                pose_identity_counter = 0;
                slam_system_->Reset();
            }
        }

        //Reset flags
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
        //Sets the orientation to custome quaternion. SLAM orientation is not used anywhere else
        Quaternion quat_non_eigen = {
            quat.w(),
            quat.x(),
            quat.y(),
            quat.z()
        };
        //Sets the position
        Vector3 position = {
            pose.translation().x(),
            pose.translation().y(),
            pose.translation().z()
        };

        //Correct position with orientation for IMU coordinate frame
        Quaternion correction = orientation_ * quat_non_eigen;
        correction.normalize();
        Vector3 position_corrected = correction.rotate_vector(position);

        //Set position and orientation
        pose_msg.pose.position.x = position_corrected.x;
        pose_msg.pose.position.y = position_corrected.y;
        pose_msg.pose.position.z = position_corrected.z;
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();
        
        //Publishes everything. Orientation from SLAM is not used anywhere else
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

    //This isn't used as it also enhances noise. Can maybe be used later with better tuning
    cv::Mat apply_clahe(const cv::Mat& image) {
        cv::Mat lab_image, result;
        
        //Convert to LAB color space
        cv::cvtColor(image, lab_image, cv::COLOR_BGR2Lab);
        
        //Split channels
        std::vector<cv::Mat> lab_planes;
        cv::split(lab_image, lab_planes);
        
        //Apply CLAHE
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(2.0);
        clahe->apply(lab_planes[0], lab_planes[0]);
        
        //Merge back and convert to BGR
        cv::merge(lab_planes, lab_image);
        cv::cvtColor(lab_image, result, cv::COLOR_Lab2BGR);
        
        return result;
    }

private:
    //Defines stuff
    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    cv::Mat left_image_;
    cv::Mat right_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr orientation_subscriber_;
    
    std::mutex image_mutex_;
    double init_start_time_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    Quaternion orientation_;

    bool left_received_ = false;
    bool right_received_ = false;
    sensor_msgs::msg::Image::SharedPtr image_msg_;
};



#endif // STEREO_VISION_NODE_HPP