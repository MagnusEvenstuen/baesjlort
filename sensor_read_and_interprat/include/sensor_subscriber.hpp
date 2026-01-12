#ifndef SENSOR_SUBSCRIBER_HPP
#define SENSOR_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "image_display_and_handle.hpp"
#include "structs.hpp"
#include "sensor_handler.hpp"

class sensor_subscriber : public rclcpp::Node
{
public:
    sensor_subscriber() : Node("sensor_subscriber")
    {
        time_ = std::time(nullptr);
        prev_time_ = std::time(nullptr);
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
        imu_center_perfect_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/gbr/imu_center_perfect", 100, 
            std::bind(&sensor_subscriber::imu_perfect_callback, this, std::placeholders::_1));

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
            acc.x += msg->linear_acceleration.x;
            acc.y += msg->linear_acceleration.y;
            acc.z += msg->linear_acceleration.z;
            gyro.x += msg->angular_velocity.x;
            gyro.y += msg->angular_velocity.y;
            gyro.z += msg->angular_velocity.z;
            recieved[0] = true;
            imu_data_sender();
        }
    }

    void imu_callback1(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[1])
        {
            acc.x += msg->linear_acceleration.x;
            acc.y += msg->linear_acceleration.y;
            acc.z += msg->linear_acceleration.z;
            gyro.x += msg->angular_velocity.x;
            gyro.y += msg->angular_velocity.y;
            gyro.z += msg->angular_velocity.z;
            recieved[1] = true;
            imu_data_sender();
        }
    }

    void imu_callback2(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[2])
        {
            acc.x += msg->linear_acceleration.x;
            acc.y += msg->linear_acceleration.y;
            acc.z += msg->linear_acceleration.z;
            gyro.x += msg->angular_velocity.x;
            gyro.y += msg->angular_velocity.y;
            gyro.z += msg->angular_velocity.z;
            recieved[2] = true;
            imu_data_sender();
        }
    }

    void imu_callback3(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[3])
        {
            acc.x += msg->linear_acceleration.x;
            acc.y += msg->linear_acceleration.y;
            acc.z += msg->linear_acceleration.z;
            gyro.x += msg->angular_velocity.x;
            gyro.y += msg->angular_velocity.y;
            gyro.z += msg->angular_velocity.z;
            recieved[3] = true;
            imu_data_sender();
        }
    }

    void imu_callback4(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[4])
        {
            acc.x += msg->linear_acceleration.x;
            acc.y += msg->linear_acceleration.y;
            acc.z += msg->linear_acceleration.z;
            gyro.x += msg->angular_velocity.x;
            gyro.y += msg->angular_velocity.y;
            gyro.z += msg->angular_velocity.z;
            recieved[4] = true;
            imu_data_sender();
        }
    }

    void imu_callback5(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[5])
        {
            acc.x += msg->linear_acceleration.x;
            acc.y += msg->linear_acceleration.y;
            acc.z += msg->linear_acceleration.z;
            gyro.x += msg->angular_velocity.x;
            gyro.y += msg->angular_velocity.y;
            gyro.z += msg->angular_velocity.z;
            recieved[5] = true;
            imu_data_sender();
        }
    }

    void imu_callback6(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[6])
        {
            acc.x += msg->linear_acceleration.x;
            acc.y += msg->linear_acceleration.y;
            acc.z += msg->linear_acceleration.z;
            gyro.x += msg->angular_velocity.x;
            gyro.y += msg->angular_velocity.y;
            gyro.z += msg->angular_velocity.z;
            recieved[6] = true;
            imu_data_sender();
        }
    }

    void imu_callback7(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        if (!recieved[7])
        {
            acc.x += msg->linear_acceleration.x;
            acc.y += msg->linear_acceleration.y;
            acc.z += msg->linear_acceleration.z;
            gyro.x += msg->angular_velocity.x;
            gyro.y += msg->angular_velocity.y;
            gyro.z += msg->angular_velocity.z;
            recieved[7] = true;
            imu_data_sender();
        }
    }

    void imu_perfect_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        /**
        if (imu_msg_count_ >= 100){
            sensor_handler_.update_truth(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z,
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
        }*/
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
        //Check if all imu messages have been received
        if (!(recieved[0] && recieved[1] && recieved[2] && recieved[3] &&
              recieved[4] && recieved[5] && recieved[6] && recieved[7]))
        {
            return;
        }
        
        Vector3 avg_acc = {
            acc.x / 8.0f,
            acc.y / 8.0f,
            acc.z / 8.0f
        };
        
        Vector3 avg_gyro = {
            gyro.x / 8.0f,
            gyro.y / 8.0f,
            gyro.z / 8.0f
        };
        
        acc = {0.0f, 0.0f, 0.0f};
        gyro = {0.0f, 0.0f, 0.0f};
        recieved[0] = false;
        recieved[1] = false;
        recieved[2] = false;
        recieved[3] = false;
        recieved[4] = false;
        recieved[5] = false;
        recieved[6] = false;
        recieved[7] = false;
        
        if (imu_msg_count_ < 100)
        {
            // Assumes standing still for calibration
            Vector3 grav_vector = sensor_handler_.calibrate_gravity(
                avg_acc.x, avg_acc.y, avg_acc.z);
            RCLCPP_INFO(this->get_logger(), 
                "Kalibrering %d/100 - Gravitasjonsvektor: x: %.2f, y: %.2f, z: %.2f", 
                imu_msg_count_ + 1, grav_vector.x, grav_vector.y, grav_vector.z);
            imu_msg_count_++;
        }
        else
        {
            sensor_handler_.update(avg_acc.x, avg_acc.y, avg_acc.z,
                               avg_gyro.x, avg_gyro.y, avg_gyro.z);

            Vector3 position = sensor_handler_.get_position();
            RCLCPP_INFO(this->get_logger(), 
                "Posisjon - x: %.2f, y: %.2f, z: %.2f", 
                position.x, position.y, position.z);
        }
    }

    void pressure_callback(const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg)
    {
        sensor_handler_.update_depth(msg->fluid_pressure);
    }

private:
    std::time_t time_;
    std::time_t prev_time_;
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
    Vector3 acc = {0.0f, 0.0f, 0.0f};
    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    bool recieved[8] = {false, false, false, false, false, false, false, false};
};
#endif // SENSOR_SUBSCRIBER_HPP