#ifndef SENSOR_SUBSCRIBER_HPP
#define SENSOR_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <array>
#include <chrono>
#include "image_display_and_handle.hpp"
#include "structs.hpp"
#include "sensor_handler.hpp"
#include "IMU_class.hpp"

class sensor_subscriber : public rclcpp::Node
{
public:
    sensor_subscriber() : Node("sensor_subscriber"),
        //Setup IMU objects with their positions on the robot and initial orientations
        IMU_center_(Vector3{0.0f, -0.003f, -0.0013}, Quaternion{1.0f, 0.0f, 0.0f, 0.0f}),
        IMU_center1_(Vector3{0.04f, -0.003f, -0.0013}, Quaternion{1.0f, 0.0f, 0.0f, 0.0f}),
        IMU_center2_(Vector3{-0.04f, -0.003f, -0.0013}, Quaternion{1.0f, 0.0f, 0.0f, 0.0f}),
        IMU_front1_(Vector3{-0.03f, 0.05f, -0.0013}, Quaternion{1.0f, 0.0f, 0.0f, 0.0f}),
        IMU_front2_(Vector3{0.03f, 0.05f, -0.0013}, Quaternion{1.0f, 0.0f, 0.0f, 0.0f}),
        IMU_rear1_(Vector3{-0.03f, -0.05f, -0.0013}, Quaternion{1.0f, 0.0f, 0.0f, 0.0f}),
        IMU_rear2_(Vector3{0.03f, -0.05f, -0.0013}, Quaternion{0.924f, 0.0f, 0.0f, -0.383f}),
        IMU_rear3_(Vector3{0.00f, -0.05f, -0.0013}, Quaternion{0.924f, 0.0f, 0.0f, 0.383f})
    {
        time_ = std::chrono::steady_clock::now();
        prev_time_ = time_;
    }

    void init()
    {
        // Subscriber for Image messages
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        left_cam_ = it_->subscribe("/gbr/cam_left/image_color", 10, 
            std::bind(&sensor_subscriber::image_left_callback, this, std::placeholders::_1));
        right_cam_ = it_->subscribe("/gbr/cam_right/image_color", 10, 
            std::bind(&sensor_subscriber::image_right_callback, this, std::placeholders::_1));

        // Subscribers for IMU messages
        imu_center_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_center", 100, 
            std::bind(&sensor_subscriber::imu_callback0, this, std::placeholders::_1));
        imu_center1_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_center1", 100, 
            std::bind(&sensor_subscriber::imu_callback1, this, std::placeholders::_1));
        imu_center2_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_center2", 100, 
            std::bind(&sensor_subscriber::imu_callback2, this, std::placeholders::_1));
        imu_front1_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_front1", 100, 
            std::bind(&sensor_subscriber::imu_callback3, this, std::placeholders::_1));
        imu_rear1_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_rear1", 100, 
            std::bind(&sensor_subscriber::imu_callback4, this, std::placeholders::_1));
        imu_front2_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_front2", 100, 
            std::bind(&sensor_subscriber::imu_callback5, this, std::placeholders::_1));
        imu_rear2_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_rear2", 100, 
            std::bind(&sensor_subscriber::imu_callback6, this, std::placeholders::_1));
        imu_rear3_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_rear3", 100, 
            std::bind(&sensor_subscriber::imu_callback7, this, std::placeholders::_1));

        // Subscriber for Fluid Pressure messages
        pressure_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "/gbr/pressure", 100, 
            std::bind(&sensor_subscriber::pressure_callback, this, std::placeholders::_1));

        // Subscriber odemetry
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/gbr/odometry_perfect", 100,
            std::bind(&sensor_subscriber::odometry_callback, this, std::placeholders::_1));
    }

private:
    void image_right_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        display_and_handle.display_image(msg, "Right Camera", this->get_logger());
    }

    void image_left_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        display_and_handle.display_image(msg, "Left Camera", this->get_logger());
    }

    void imu_callback0(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[0])
        {
            IMU_center_.update(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
            Vector3 acc = IMU_center_.get_acceleration();
            Quaternion orientation = IMU_center_.get_orientation();
            acc_.x += acc.x;
            acc_.y += acc.y;
            acc_.z += acc.z;
            orientation_.w += orientation.w;
            orientation_.x += orientation.x;
            orientation_.y += orientation.y;
            orientation_.z += orientation.z;
            orientation_.normalize();
            recieved[0] = true;
        }
    }

    void imu_callback1(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[1])
        {
            IMU_center1_.update(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
            Vector3 acc = IMU_center1_.get_acceleration();
            Quaternion orientation = IMU_center1_.get_orientation();
            acc_.x += acc.x;
            acc_.y += acc.y;
            acc_.z += acc.z;
            orientation_.w += orientation.w;
            orientation_.x += orientation.x;
            orientation_.y += orientation.y;
            orientation_.z += orientation.z;
            orientation_.normalize();
            recieved[1] = true;
        }
    }

    void imu_callback2(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[2])
        {
            IMU_center2_.update(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
            Vector3 acc = IMU_center2_.get_acceleration();
            Quaternion orientation = IMU_center2_.get_orientation();
            acc_.x += acc.x;
            acc_.y += acc.y;
            acc_.z += acc.z;
            orientation_.w += orientation.w;
            orientation_.x += orientation.x;
            orientation_.y += orientation.y;
            orientation_.z += orientation.z;
            orientation_.normalize();
            recieved[2] = true;
        }
    }

    void imu_callback3(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[3])
        {
            IMU_front1_.update(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
            Vector3 acc = IMU_front1_.get_acceleration();
            Quaternion orientation = IMU_front1_.get_orientation();
            acc_.x += acc.x;
            acc_.y += acc.y;
            acc_.z += acc.z;
            orientation_.w += orientation.w;
            orientation_.x += orientation.x;
            orientation_.y += orientation.y;
            orientation_.z += orientation.z;
            orientation_.normalize();
            recieved[3] = true;
        }
    }

    void imu_callback4(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[4])
        {
            IMU_front2_.update(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
            Vector3 acc = IMU_front2_.get_acceleration();
            Quaternion orientation = IMU_front2_.get_orientation();
            acc_.x += acc.x;
            acc_.y += acc.y;
            acc_.z += acc.z;
            orientation_.w += orientation.w;
            orientation_.x += orientation.x;
            orientation_.y += orientation.y;
            orientation_.z += orientation.z;
            orientation_.normalize();
            recieved[4] = true;
        }
    }

    void imu_callback5(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[5])
        {
            IMU_rear1_.update(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
            Vector3 acc = IMU_rear1_.get_acceleration();
            Quaternion orientation = IMU_rear1_.get_orientation();
            acc_.x += acc.x;
            acc_.y += acc.y;
            acc_.z += acc.z;
            orientation_.w += orientation.w;
            orientation_.x += orientation.x;
            orientation_.y += orientation.y;
            orientation_.z += orientation.z;
            orientation_.normalize();
            recieved[5] = true;
        }
    }

    void imu_callback6(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[6])
        {
            IMU_rear2_.update(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
            Vector3 acc = IMU_rear2_.get_acceleration();
            Quaternion orientation = IMU_rear2_.get_orientation();
            acc_.x += acc.x;
            acc_.y += acc.y;
            acc_.z += acc.z;
            orientation_.w += orientation.w;
            orientation_.x += orientation.x;
            orientation_.y += orientation.y;
            orientation_.z += orientation.z;
            orientation_.normalize();
            recieved[6] = true;
        }
    }

    void imu_callback7(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[7])
        {
            IMU_rear3_.update(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
            Vector3 acc = IMU_rear3_.get_acceleration();
            Quaternion orientation = IMU_rear3_.get_orientation();
            acc_.x += acc.x;
            acc_.y += acc.y;
            acc_.z += acc.z;
            orientation_.w += orientation.w;
            orientation_.x += orientation.x;
            orientation_.y += orientation.y;
            orientation_.z += orientation.z;
            orientation_.normalize();
            recieved[7] = true;
        }
    }

    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
    {
        sensor_handler_.update_truth_odometry(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
    }

    void imu_data_sender()
    {
        //Check how many IMU messages have been recieved
        unsigned int recieved_counter = 0;
        for (bool r : recieved)
        {
            if (r)
                recieved_counter++;
        }

        if (recieved_counter == 0)
        {
            return;
        }

        //Calculate average values from the recieved IMU messages
        Vector3 avg_acc = {
            acc_.x / recieved_counter,
            acc_.y / recieved_counter,
            acc_.z / recieved_counter
        };
        RCLCPP_INFO(this->get_logger(), "Acc - x: %.4f, y: %.4f, z: %.4f", avg_acc.x, avg_acc.y,  avg_acc.z);
        
        Quaternion avg_orientation = {
            orientation_.w / recieved_counter,
            orientation_.x / recieved_counter,
            orientation_.y / recieved_counter,
            orientation_.z / recieved_counter
        };
        
        recieved[0] = false;
        recieved[1] = false;
        recieved[2] = false;
        recieved[3] = false;
        recieved[4] = false;
        recieved[5] = false;
        recieved[6] = false;
        recieved[7] = false;
        //Updates sensor handler with averaged IMU data
        sensor_handler_.update(avg_acc, avg_orientation);

        Vector3 position = sensor_handler_.get_position();
        RCLCPP_INFO(this->get_logger(), 
            "Posisjon - x: %.2f, y: %.2f, z: %.2f", 
            position.x, position.y, position.z);

        acc_ = {0.0f, 0.0f, 0.0f};
        orientation_ = {0.0f, 0.0f, 0.0f, 0.0f};
    }

    void pressure_callback(const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg)
    {
        sensor_handler_.update_depth(msg->fluid_pressure);
        imu_data_sender();
    }

private:
    std::chrono::steady_clock::time_point time_;
    std::chrono::steady_clock::time_point prev_time_;
    image_transport::Subscriber left_cam_;
    image_transport::Subscriber right_cam_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_center_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_front1_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_rear1_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_front2_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_rear2_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_center1_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_center2_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_rear3_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_center_perfect_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_display_and_handle display_and_handle;
    sensor_handler sensor_handler_;

    float current_speed_x = 0.0;
    float current_speed_y = 0.0;
    float current_speed_z = 0.0;
    unsigned int imu_msg_count_ = 0;
    Vector3 acc_ = {0.0f, 0.0f, 0.0f};
    Quaternion orientation_ = {0.0f, 0.0f, 0.0f, 0.0f};
    std::array<bool, 8> recieved = {false, false, false, false, false, false, false, false};
    IMU IMU_center_;
    IMU IMU_center1_;
    IMU IMU_center2_;
    IMU IMU_front1_;
    IMU IMU_front2_;
    IMU IMU_rear1_;
    IMU IMU_rear2_;
    IMU IMU_rear3_;

};
#endif // SENSOR_SUBSCRIBER_HPP