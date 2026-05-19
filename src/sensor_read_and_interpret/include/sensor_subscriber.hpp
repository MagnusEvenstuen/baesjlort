#ifndef SENSOR_SUBSCRIBER_HPP
#define SENSOR_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <image_transport/image_transport.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <array>
#include <chrono>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "image_display_and_handle.hpp"
#include "structs.hpp"
#include "sensor_handler.hpp"
#include "IMU_class.hpp"
#include "VIMU_filter.hpp"

class sensor_subscriber : public rclcpp::Node
{
public:
    sensor_subscriber() : Node("sensor_subscriber"),
        //Setup IMU objects with their positions on the robot and initial orientations
        IMU_center_(Eigen::Vector3d(0.0f, -0.003f, -0.0013), Eigen::Quaterniond(1.0f, 0.0f, 0.0f, 0.0f)),
        IMU_center1_(Eigen::Vector3d(0.04f, -0.003f, -0.0013), Eigen::Quaterniond(-0.3827, 0.0, 0.0, 0.9239)),  //Rotated 225 degrees around z
        IMU_center2_(Eigen::Vector3d(-0.04f, -0.003f, -0.0013), Eigen::Quaterniond(1.0f, 0.0f, 0.0f, 0.0f)),    //Not rotated at all
        IMU_front1_(Eigen::Vector3d(-0.03f, 0.05f, -0.0013), Eigen::Quaterniond(0.7071, 0.0, 0.0, 0.7071)),     //Rotated 90 degrees around z
        IMU_front2_(Eigen::Vector3d(0.03f, 0.05f, -0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, 0.383f)),
        IMU_rear1_(Eigen::Vector3d(-0.03f, -0.05f, -0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, -0.383f)),
        IMU_rear2_(Eigen::Vector3d(0.03f, -0.05f, -0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, -0.383f)),
        IMU_rear3_(Eigen::Vector3d(0.00f, -0.05f, -0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, -0.383f)),

        imu_objects_{IMU_center_, IMU_center1_, IMU_center2_, IMU_front1_, IMU_front2_, IMU_rear1_, IMU_rear2_, IMU_rear3_},
        imu_topic_map_{
            //{"/gbr/imu_center_perfect", 0},
            {"/gbr/navigator_imu1", 3},
            {"/gbr/imu1", 0},       //Currently only 1, 2, and 3 are used.
            {"/gbr/imu2", 1},
            {"/gbr/imu3", 2},
            {"/gbr/imu_front2", 4},
            {"/gbr/imu_rear1", 5},
            {"/gbr/imu_rear2", 6},
            {"/gbr/imu_rear3", 7}
        },
        vimu_filter(3)
    {
        time_ = std::chrono::steady_clock::now();
        prev_time_ = time_;
        processing_thread_ = std::thread([this]() {
            while (running_)
            {
                imu_data_sender();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
        //vimu_filter.set_imu_geometry(Eigen::Vector3d(0.0f, 0.003f, 0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, 0.383f), 0);
        vimu_filter.set_imu_geometry(Eigen::Vector3d(-0.04f, 0.003f, 0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, -0.383f), 0);
        vimu_filter.set_imu_geometry(Eigen::Vector3d(0.04f, 0.003f, 0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, -0.383f), 1);
        vimu_filter.set_imu_geometry(Eigen::Vector3d(0.03f, -0.05f, 0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, -0.383f), 2);
        //vimu_filter.set_imu_geometry(Eigen::Vector3d(-0.03f, -0.05f, 0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, -0.383f), 4);
        //vimu_filter.set_imu_geometry(Eigen::Vector3d(0.03f, 0.05f, 0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, 0.383f), 5);
        //vimu_filter.set_imu_geometry(Eigen::Vector3d(-0.03f, 0.05f, 0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, 0.383f), 6);
        //vimu_filter.set_imu_geometry(Eigen::Vector3d(-0.00f, 0.05f, 0.0013), Eigen::Quaterniond(0.924f, 0.0f, 0.0f, 0.383f), 7);
    }

    ~sensor_subscriber()
    {
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }
    }

    void init()
    {
        // Subscribers for IMU messages
        for (const auto& [topic_name, imu_index] : imu_topic_map_) {
            auto subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
                topic_name, rclcpp::SensorDataQoS(),
                [this, imu_index](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
                    imu_callback(msg, imu_index);
                });
            imu_subscribers_.push_back(subscriber);
        }
        thrust_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/gbr/thrusters", 100, 
            std::bind(&sensor_subscriber::thrust_callback, this, std::placeholders::_1));

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/orb_slam3/pose", 30,
            std::bind(&sensor_subscriber::pose_callback, this, std::placeholders::_1));

        avg_gyro_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/average_gyro", 1000);
        avg_acc_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/average_acceleration", 1000);

        //Publishes orientation for SLAMming balls
        orientation_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/average_orientation", 100);

        position_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/position", 100);

        // Subscriber for Fluid Pressure messages
        pressure_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "/gbr/pressure", rclcpp::SensorDataQoS(), 
            std::bind(&sensor_subscriber::pressure_callback, this, std::placeholders::_1));

        // Subscriber odemetry
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/gbr/odometry_perfect", 100,
            std::bind(&sensor_subscriber::odometry_callback, this, std::placeholders::_1));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg, int imu_index)
    {
        //Updates stuff from each IMU
        if (!recieved[imu_index])
        {
            bool fuse_sensors = true;
            auto& imu_obj = imu_objects_[imu_index];

            //Only fuse sensors when thrusters are not generating force. Also check in function to make sure no fusion happens when
            //to high non lineary acceleration.
            for (int i = 0; i < thrust_.size(); i++)
            {
                if (thrust_[i] != 0.0)
                {
                    fuse_sensors = false;
                    break;
                }
            }

            imu_obj.update2_electric_boogalo(
                msg->linear_acceleration.x/8,
                msg->linear_acceleration.y/8,
                msg->linear_acceleration.z/8,
                msg->angular_velocity.x/16,
                msg->angular_velocity.y/16,
                msg->angular_velocity.z/16,
                fuse_sensors);
            
            Eigen::Vector3d acc = imu_obj.get_acceleration();
            Eigen::Quaterniond orientation = imu_obj.get_orientation();
            Eigen::Vector3d gyro = imu_obj.get_gyro();
            
            {
                const std::lock_guard<std::mutex> lock(vector_mutex);
                acc_vector.emplace_back(acc);
                gyro_vector.emplace_back(gyro);
                orientation_vector.emplace_back(orientation);
                id_vector.emplace_back(imu_index);
            }
            
            recieved[imu_index] = true;
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        //Recievs estimated pose from SLAM
        static double last_time = this->now().seconds();
        double current_time = this->now().seconds();
        double dt = current_time - last_time;
        RCLCPP_INFO(this->get_logger(), "DT between poses: %.4f", dt);
        
        sensor_handler_.update_pose_estimate(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w,
            dt
        );
        last_time = this->now().seconds();
    }

    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
    {
        //Recievs perfect odometry for comparison and logging
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

    void thrust_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
    {
        //Recievs current thruster commands for Kalman filter
        thrust_ = {msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]};
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

        if (recieved_counter <= 0)
        {
            sensor_handler_.non_measurement_prediction(thrust_);
            return;
        }

        //Calculate average values from the recieved IMU messages
        {
            //const std::lock_guard<std::mutex> lock(vector_mutex);
            //vimu_filter.update(acc_vector, gyro_vector, id_vector);
            //acc_vector.clear();
            //gyro_vector.clear();
            //id_vector.clear();
        }
        //Eigen::Vector3d vimu_acc = vimu_filter.get_acceleration();
        //Eigen::Vector3d vimu_gyro = vimu_filter.get_gyro();
        //Eigen::Vector3d avg_acc = vimu_acc;
        //Eigen::Vector3d avg_gyro = vimu_gyro;
        Eigen::Vector3d avg_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d avg_gyro = Eigen::Vector3d::Zero();
        Eigen::Quaterniond avg_orientation_IMU = Eigen::Quaterniond::Identity();
        {
            const std::lock_guard<std::mutex> lock(vector_mutex);
            for (const auto& acc : acc_vector)
            {
                avg_acc += acc;
            }
            for (const auto& gyro : gyro_vector)
            {
                avg_gyro += gyro;
            }
            for (const auto& orientation : orientation_vector) {
                avg_orientation_IMU.coeffs() += orientation.coeffs();
            }
            if (!acc_vector.empty())
            {
                avg_acc /= acc_vector.size();
            }
            if (!gyro_vector.empty())
            {
                avg_gyro /= gyro_vector.size();
            }
            avg_orientation_IMU.normalize();
            acc_vector.clear();
            gyro_vector.clear();
            orientation_vector.clear();
            id_vector.clear();
        }

        RCLCPP_INFO(this->get_logger(), "Acc - x: %.4f, y: %.4f, z: %.4f", avg_acc.y(), avg_acc.x(),  avg_acc.z());
        
        //Publish data for SYSID
        auto gyro_msg = std_msgs::msg::Float32MultiArray();
        gyro_msg.data = {avg_gyro.x(), avg_gyro.y(), avg_gyro.z()};
        avg_gyro_publisher_->publish(gyro_msg);

        auto acc_msg = std_msgs::msg::Float32MultiArray();
        acc_msg.data = {avg_acc.x(), avg_acc.y(), avg_acc.z()};
        avg_acc_publisher_->publish(acc_msg);

        ////Publish data for Slamming balls
        Eigen::Quaterniond ori_from_handler = sensor_handler_.get_orientation();
        auto orientation_msg = std_msgs::msg::Float32MultiArray();
        orientation_msg.data = {ori_from_handler.w(), ori_from_handler.x(), ori_from_handler.y(), ori_from_handler.z()};
        orientation_publisher_->publish(orientation_msg);
        Eigen::Vector3d position = sensor_handler_.get_position();
        auto position_msg = std_msgs::msg::Float32MultiArray();
        position_msg.data = {-position.x(), -position.y(), position.z()};
        position_publisher_->publish(position_msg);

        //Reset recieved flags
        recieved[0] = false;
        recieved[1] = false;
        recieved[2] = false;
        recieved[3] = false;
        recieved[4] = false;
        recieved[5] = false;
        recieved[6] = false;
        recieved[7] = false;
        //Updates sensor handler with averaged IMU data
        sensor_handler_.update(avg_acc, avg_gyro, thrust_, avg_orientation_IMU);

        RCLCPP_INFO(this->get_logger(), 
            "Posisjon - x: %.2f, y: %.2f, z: %.2f", 
            -position.x(), -position.y(), position.z());

        acc_ = Eigen::Vector3d::Zero();
        gyro_ = Eigen::Vector3d::Zero();
        orientation_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    }

    void pressure_callback(const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg)
    {
        sensor_handler_.update_depth(msg->fluid_pressure);
    }

private:
    //Variables and shit
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
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr avg_gyro_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr avg_acc_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr orientation_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> imu_subscribers_;
    Eigen::Vector3d gyro_ = {0.0, 0.0, 0.0};
    std::mutex vector_mutex;
    
    std::vector<Eigen::Vector3d> acc_vector;
    std::vector<Eigen::Vector3d> gyro_vector;
    std::vector<int> id_vector;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_display_and_handle display_and_handle;
    sensor_handler sensor_handler_;
    const Eigen::Vector3d center_of_gravity_ = Eigen::Vector3d(0.0, -0.003, -0.0013);
    std::array<float, 8> thrust_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<Eigen::Quaterniond> orientation_vector;
    float current_speed_x = 0.0;
    float current_speed_y = 0.0;
    float current_speed_z = 0.0;
    unsigned int imu_msg_count_ = 0;
    Eigen::Vector3d acc_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    Eigen::Quaterniond last_orientation_ = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    std::array<bool, 8> recieved = {false, false, false, false, false, false, false, false};
    IMU IMU_center_;
    IMU IMU_center1_;
    IMU IMU_center2_;
    IMU IMU_front1_;
    IMU IMU_front2_;
    IMU IMU_rear1_;
    IMU IMU_rear2_;
    IMU IMU_rear3_;
    VIMU_filter vimu_filter;
    std::unordered_map<std::string, int> imu_topic_map_;
    std::array<IMU, 8> imu_objects_;
    std::thread processing_thread_;
    bool running_ = true;
};
#endif // SENSOR_SUBSCRIBER_HPP