#ifndef STEREO_NODE_HPP
#define STEREO_NODE_HPP

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

class StereoMode : public rclcpp::Node
{
public:
    StereoMode() : Node("stereo_orb_slam3_node")
    {
        // Deklarer parametre
        this->declare_parameter<std::string>("vocabulary_path", "/home/gud/Skole/baesjlort/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin");
        this->declare_parameter<std::string>("settings_path", "/home/gud/Skole/baesjlort/src/ros2_orb_slam3/config/stereo_gbr_sim.yaml");
        this->declare_parameter<bool>("enable_viewer", false);
        this->declare_parameter<std::string>("left_camera_topic", "/gbr/cam_left/image_color");
        this->declare_parameter<std::string>("right_camera_topic", "/gbr/cam_right/image_color");

        // Hent parametre
        std::string vocab_path = this->get_parameter("vocabulary_path").as_string();
        std::string settings_path = this->get_parameter("settings_path").as_string();
        bool enable_viewer = this->get_parameter("enable_viewer").as_bool();
        std::string left_topic = this->get_parameter("left_camera_topic").as_string();
        std::string right_topic = this->get_parameter("right_camera_topic").as_string();

        // Sjekk at filbaner er satt
        if (vocab_path.empty() || settings_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Vocabulary eller settings filbane er ikke satt!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Initialiserer ORB_SLAM3 Stereo...");
        
        // Initialiser ORB_SLAM3 i stereo-modus
        slam_system_ = std::make_unique<ORB_SLAM3::System>(
            vocab_path, 
            settings_path, 
            ORB_SLAM3::System::STEREO, 
            enable_viewer
        );

        // Opprett subscribers for stereo-kameraene
        auto qos = rclcpp::SensorDataQoS();
        
        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            left_topic,
            qos,
            std::bind(&StereoMode::leftImageCallback, this, std::placeholders::_1)
        );

        right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            right_topic,
            qos,
            std::bind(&StereoMode::rightImageCallback, this, std::placeholders::_1)
        );

        // Opprett publisher for kamera-pose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/orb_slam3/pose",
            10
        );

        RCLCPP_INFO(this->get_logger(), "Stereo node klar. Abonnerer på:");
        RCLCPP_INFO(this->get_logger(), "  Venstre: %s", left_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Høyre: %s", right_topic.c_str());
    }

    ~StereoMode()
    {
        if (slam_system_) {
            slam_system_->Shutdown();
        }
    }

private:
    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            left_image_ = cv_ptr->image.clone();
            left_timestamp_ = rclcpp::Time(msg->header.stamp).seconds();
            left_received_ = true;
            
            // Sjekk om vi har mottatt begge bildene
            processStereoPair(msg->header);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            right_image_ = cv_ptr->image.clone();
            right_timestamp_ = rclcpp::Time(msg->header.stamp).seconds();
            right_received_ = true;
            
            // Sjekk om vi har mottatt begge bildene
            processStereoPair(msg->header);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void processStereoPair(const std_msgs::msg::Header& header)
    {
        // Sjekk om vi har mottatt begge bildene
        if (!left_received_ || !right_received_) {
            return;
        }

        // Sjekk at tidsstemplene er nærme hverandre (innenfor 0.1 sekund)
        double time_diff = std::abs(left_timestamp_ - right_timestamp_);

        // Kjør ORB_SLAM3 tracking med stereo-bildene
        if (!left_image_.empty() && !right_image_.empty()) {
            // Bruk gjennomsnittlig tidstempel
            double stereo_timestamp = (left_timestamp_ + right_timestamp_) / 2.0;
            
            // TrackStereo returnerer Sophus::SE3f
            Sophus::SE3f pose = slam_system_->TrackStereo(
                left_image_, 
                right_image_, 
                stereo_timestamp
            );

            // Publiser posen hvis tracking var vellykket
            if (!pose.matrix().isIdentity()) {  // Sjekk om vi fikk en gyldig pose
                publishPose(pose, header);
            }

            // Reset for neste bildepar
            left_received_ = false;
            right_received_ = false;
        }
    }

    void publishPose(const Sophus::SE3f& pose, const std_msgs::msg::Header& img_header)
    {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        
        // Sett header
        pose_msg.header = img_header;
        pose_msg.header.frame_id = "world";  // ORB_SLAM3 koordinatsystem
        
        // Konverter Sophus::SE3f til ROS pose
        // Sophus::SE3f inneholder rotasjon (so3) og translasjon (translation)
        Eigen::Quaternionf quat(pose.rotationMatrix());
        
        // Sett posisjon
        pose_msg.pose.position.x = pose.translation().x();
        pose_msg.pose.position.y = pose.translation().y();
        pose_msg.pose.position.z = pose.translation().z();
        
        // Sett orientering
        pose_msg.pose.orientation.x = quat.x();
        pose_msg.pose.orientation.y = quat.y();
        pose_msg.pose.orientation.z = quat.z();
        pose_msg.pose.orientation.w = quat.w();
        
        // Publiser
        pose_publisher_->publish(pose_msg);
        
        // Logg for debugging
        RCLCPP_DEBUG(this->get_logger(), 
                    "Pose: [%.3f, %.3f, %.3f]", 
                    pose.translation().x(),
                    pose.translation().y(),
                    pose.translation().z());
    }

    // ORB_SLAM3 system
    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    
    // ROS subscribers og publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    
    // Image buffers
    cv::Mat left_image_;
    cv::Mat right_image_;
    double left_timestamp_ = 0.0;
    double right_timestamp_ = 0.0;
    bool left_received_ = false;
    bool right_received_ = false;
    
    // Mutex for bilde-buffere
    std::mutex img_mutex_;
};

#endif // STEREO_NODE_HPP