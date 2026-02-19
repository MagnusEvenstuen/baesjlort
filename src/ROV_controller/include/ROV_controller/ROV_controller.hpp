#ifndef ROV_CONTROLLER_HPP
#define ROV_CONTROLLER_HPP

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "PID_controller.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <thread>

class ROV_controller : public rclcpp::Node
{
public:
    ROV_controller() : Node("rov_controller"), PID_x(0.8, 0.0, 0.1), PID_y(0.8, 0.0, 0.1), PID_z(0.8, 0.01, 0.1),
                        PID_orientation(1.2, 0.001, 0.8)
    {
        orientation_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/average_orientation", 100,
            std::bind(&ROV_controller::orientation_callback, this, std::placeholders::_1));

        position_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/position", 100,
            std::bind(&ROV_controller::position_callback, this, std::placeholders::_1));

        thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gbr/thrusters", 10);

        processing_thread_ = std::thread([this]() {
            while (true)
            {
                update_PID();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        PID_x.set_target_position(0.0);
        PID_y.set_target_position(0.0);
        PID_z.set_target_position(0.0);
        PID_orientation.set_target_quaternion(
            Eigen::Quaterniond(
                Eigen::AngleAxisd(-3.1415/2, Eigen::Vector3d::UnitX()) *   // roll
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *   // pitch
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())     // yaw
            )
        );
    }

private:
    void orientation_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
    {
        current_orientation = Eigen::Quaterniond(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    }

    void position_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
    {
        current_position = Eigen::Vector3d(msg->data[0], msg->data[1], msg->data[2]);
    }

    void update_PID()
    { 
        static double last_time = this->now().seconds();
        double current_time = this->now().seconds();
        double dt = current_time - last_time;
        last_time = current_time;

        Eigen::Vector3d orientation_output = PID_orientation.update(current_orientation, dt);
        Eigen::Vector3d force_world = Eigen::Vector3d(PID_x.update(current_position.x(), dt),
               PID_y.update(current_position.y(), dt),
               PID_z.update(current_position.z(), dt));
        force_world = current_orientation.conjugate() * force_world;
        Eigen::VectorXd forces(6);

        forces << force_world(0),
                  force_world(1),
                  force_world(2),
                  orientation_output(0),
                  orientation_output(1),
                  orientation_output(2);

        Eigen::VectorXd gain = thrust_map_matrix*forces;
        gain = gain.cwiseMax(-10.0).cwiseMin(10.0);
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = std::vector<double>({gain(0), gain(1), gain(2), gain(3), gain(4), gain(5), gain(6), gain(7)});
        thrust_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), 
            "Posisjon - T1: %.2f, T2: %.2f, T3: %.2f, T4: %.2f, T5: %.2f, T6: %.2f, T7: %.2f, T8: %.2f", 
            gain(0), gain(1), gain(2), gain(3), gain(4), gain(5), gain(6), gain(7));

        //Get errors for logging
        Eigen::Vector3d position_error(
            PID_x.get_error_float(),
            PID_y.get_error_float(),
            PID_z.get_error_float()
        );
        Eigen::Vector3d orientation_error = PID_orientation.get_error_quat();
        log_errors_to_csv(position_error, orientation_error, current_time);
    }

    void log_errors_to_csv(const Eigen::Vector3d& position_error, const Eigen::Vector3d& orientation_error, double timestamp)
    {
        static std::ofstream error_csv_file_;
        static bool first_open_ = true;
        
        if (first_open_)
        {
            error_csv_file_.open("/home/gud/Skole/baesjlort/scripts/plotting_program/data_files/error_data.csv", std::ios::app);
            if (error_csv_file_.is_open())
            {
                error_csv_file_ << "timestamp,pos_error_x,pos_error_y,pos_error_z,"
                                "orient_error_x,orient_error_y,orient_error_z\n";
            }
            first_open_ = false;
        }
        
        if (error_csv_file_.is_open())
        {
            error_csv_file_ << std::fixed << std::setprecision(6)
                        << timestamp << ","
                        << position_error.x() << "," << position_error.y() << "," << position_error.z() << ","
                        << orientation_error.x() << "," << orientation_error.y() << "," << orientation_error.z() << "\n";
            error_csv_file_.flush();
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr orientation_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_publisher_;
    Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond current_orientation = Eigen::Quaterniond::Identity();
    std::thread processing_thread_;

    PID_controller PID_x;
    PID_controller PID_y;
    PID_controller PID_z;
    PID_controller PID_orientation;

    Eigen::Matrix<double, 8, 6> thrust_map_matrix = 
    (Eigen::Matrix<double, 8, 6>() <<
    -1.0, 1.0, 0.0, 0.0, 0.0, 1.0,   //Thruster 1
    1.0, 1.0, 0.0, 0.0, 0.0, -1.0,  //Thruster 2
    -1.0, -1.0, 0.0, 0.0, 0.0, -1.0, //Thruster 3
    1.0, -1.0, 0.0, 0.0, 0.0, 1.0,  //Thruster 4
    0.0, 0.0, 1.0, -1.0, 1.0, 0.0, //Thruster 5
    0.0, 0.0, 1.0, 1.0, 1.0, 0.0,  //Thruster 6
    0.0, 0.0, 1.0, -1.0, -1.0, 0.0,    //Thruster 7
    0.0, 0.0, 1.0, 1.0, -1.0, 0.0    //Thruster 8
    ).finished();
};

#endif // ROV_CONTROLLER_HPP