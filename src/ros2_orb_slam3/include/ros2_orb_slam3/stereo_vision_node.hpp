#ifndef STEREO_VISION_NODE_HPP
#define STEREO_VISION_NODE_HPP

//This code is inspired by https://github.com/MagnusEvenstuen/baesjlort/blob/632bd9b6321b4bde06fa1419d00f04613052ec7a/src/ros2_orb_slam3/include/ros2_orb_slam3/stereo_node.hpp which is AI generated

#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <array>
#include <chrono>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

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
        //Information needed for ORB_SLAM3 initialization
        slam_system_ = std::make_unique<ORB_SLAM3::System>(
            "src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin",     //Path to vocabulary file 
            "src/ros2_orb_slam3/config/stereo_gbr_sim.yaml",               //Path to camera settings file 
            ORB_SLAM3::System::STEREO, 
            true                                                                                   //Enable viewer
        );

        //ROS2 publishers and subscribers
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orb_slam3/pose",
            30
        );

        left_subscriber_.subscribe(
            this, 
            "/gbr/cam_left/image_rect",
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
        
        right_subscriber_.subscribe(
            this,
            "/gbr/cam_right/image_rect",
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
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

        //Synchronizer for stereo image topics. Written by AI
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10),
            left_subscriber_,
            right_subscriber_
        );
        
        sync_->registerCallback(&stereo_vision_node::stereo_callback, this);

        processing_thread_ = std::jthread([this](std::stop_token st) {
            while (!st.stop_requested())
            {
                std::unique_lock<std::mutex> lock(image_mutex_);
                cv_.wait(lock, st, [this](){
                            return new_frame_;
                        });

                if (new_frame_)
                {
                    new_frame_ = false;
                    lock.unlock();
                    process_stereo_pair(last_header_);
                }
            }
        });
    }

    ~stereo_vision_node()
    {
        if (slam_system_)
        {
            slam_system_->Shutdown();
        }
    }

private:
    void stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
    {
        {
            // Needs to be at the beginning due to usage of toCvShare
            // Prevent the memory from being accessed when copying the ptr
            std::lock_guard<std::mutex> lock(image_mutex_);
            //Convert images
            auto left_img_ = cv_bridge::toCvShare(left_msg, "bgr8");
            auto right_img_ = cv_bridge::toCvShare(right_msg, "bgr8");
            
            //Apply CLAHE (this might not be nessacerry)
            //left_image_ = apply_clahe(left_img_->image);
            //right_image_ = apply_clahe(right_img_->image);
            //left_image_ = apply_homomorphic(left_img_->image);
            //right_image_ = apply_homomorphic(right_img_->image);

            left_image_ = (left_img_->image);
            right_image_ = (right_img_->image);
            
            // Process stereo pair immediately (message_filters ensures both are available)
            last_header_ = left_msg->header;
            new_frame_ = true;
        }
        cv_.notify_one();
    }

    void orientation_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(orientation_mutex_);
        //Sets the correct orientation from IMU data
        orientation_ = Eigen::Quaterniond(
            msg->data[0],
            msg->data[1],
            msg->data[2],
            msg->data[3]
        );
    }

    //This function could be improved with better synchronization if needed
    void process_stereo_pair(const std_msgs::msg::Header& header)
    {
        static unsigned int pose_identity_counter = 0;
        //RCLCPP_INFO(this->get_logger(), "Processing my balls...");
        //Check if both images have been received

        //Process stereo images with ORB_SLAM3
        cv::Mat left_image;
        cv::Mat right_image;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            /// If slow, test this. Not immune to others modifying the data
            // std::swap(left_image, left_image_);
            // std::swap(right_image, right_image_);
            left_image = left_image_.clone();
            right_image = right_image_.clone();
        }
        Sophus::SE3f pose = slam_system_->TrackStereo(left_image, 
                                                    right_image, 
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
            if (pose_identity_counter >= 2000)
            {
                //Reseting SLAM system after consecutive identity poses to improve speed estimation
                RCLCPP_WARN(this->get_logger(), "Pose is identity, to long. Resetting...");
                pose_identity_counter = 0;
                slam_system_->Reset();
            }
        }
    }

    void publishPose(const Sophus::SE3f& pose, const std_msgs::msg::Header& img_header)
    {
        //RCLCPP_INFO(this->get_logger(), "Publishing my balls...");
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        
        //Set header
        pose_msg.header = img_header;
        pose_msg.header.frame_id = "world";
        
        Eigen::Quaternionf quat(pose.rotationMatrix());
        Eigen::Quaternionf orientation;

        Eigen::Vector3f position = Eigen::Vector3f(pose.translation().x(), pose.translation().y(), pose.translation().z());
        {
            std::lock_guard<std::mutex> lock(orientation_mutex_);
            orientation = orientation_.cast<float>();
        }
        position = quat.conjugate() * position;
        //position = orientation * position;

        //Set position and orientation
        pose_msg.pose.position.x = position.x();
        pose_msg.pose.position.y = position.y();
        pose_msg.pose.position.z = position.z();
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();
        
        //Publishes everything.
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

    cv::Mat apply_homomorphic(const cv::Mat& image) {
        cv::Mat gray, log_img, blur, high_freq, result;
        
        // Konverter til grayscale med vekting som fremhever røde kanaler
        // (vann absorberer rødt, så rød-kanalen har faktisk mer tekstur)
        // Vekter: B=0.1, G=0.3, R=0.6 (motsatt av standard BGR-grayscale)
        cv::Mat channels[3];
        cv::split(image, channels);

        result = 0.1 * channels[0] + 0.3 * channels[1] + 0.6 * channels[2];
        result.convertTo(gray, CV_32F);
        
        cv::log(gray + 1.0f, log_img);
        
        cv::GaussianBlur(log_img, blur, cv::Size(63, 63), 15.0);
        high_freq = log_img - blur;
        
        cv::exp(high_freq, result);
        cv::normalize(result, result, 0, 255, cv::NORM_MINMAX);
        result.convertTo(result, CV_8U);
        
        cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);
        return result;
    }

private:
    //The message filter part is written by AI
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image
    > SyncPolicy;
    
    // Message filters subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> left_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    //Defines stuff
    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    cv::Mat left_image_;
    cv::Mat right_image_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr orientation_subscriber_;
    
    double init_start_time_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    Eigen::Quaterniond orientation_;

    std::jthread processing_thread_;
    std::mutex image_mutex_;
    std::mutex orientation_mutex_;
    std::condition_variable_any cv_;
    bool new_frame_ = false;
    sensor_msgs::msg::Image::SharedPtr image_msg_;
    std_msgs::msg::Header last_header_;

};



#endif // STEREO_VISION_NODE_HPP
