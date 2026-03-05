#ifndef ROV_CONTROLLER_HPP
#define ROV_CONTROLLER_HPP

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include "PID_controller.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <thread>
#include "controller_msgs/msg/controller_state.hpp"

enum ROV_classes_to_detect
{
    //Theese might not be correct (Structure and valve are correct)
    aruco_marker = 0,
    structure = 1,
    tube = 2,
    valve = 3
};

class ROV_controller : public rclcpp::Node
{
public:
    ROV_controller() : Node("rov_controller"), PID_x(1.2, 0.0, 0.4), PID_y(1.2, 0.0, 0.4), PID_z(1.2, 0.01, 0.1),
                        PID_orientation(0.5, 0.0, 0.01)
    {
        //Sets up ROS2 publishers and subscribers
        orientation_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/average_orientation", 1,
            std::bind(&ROV_controller::orientation_callback, this, std::placeholders::_1));

        position_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/position", 1,
            std::bind(&ROV_controller::position_callback, this, std::placeholders::_1));

        thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gbr/thrusters", 10);
        
        object_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/distance_to_object", 1,
            std::bind(&ROV_controller::object_position_callback, this, std::placeholders::_1));

        controller_subscriber_ = this->create_subscription<controller_msgs::msg::ControllerState>(
                "/controller_state", 1,
                std::bind(&ROV_controller::controller_callback, this, std::placeholders::_1));


        //Runs the PID process
        if (auto_)
        {
            processing_thread_ = std::thread([this]() {
                while (running_)
                {
                    update_PID();
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            });
            //Sets targets
            PID_x.set_target_position(target_position_.x());
            PID_y.set_target_position(target_position_.y());
            PID_z.set_target_position(target_position_.z());
            PID_orientation.set_target_quaternion(target_orientation_);
        }
    }

    ~ROV_controller()
    {
        running_ = false;
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }
    }

private:
    void orientation_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
    {
        current_orientation_ = Eigen::Quaterniond(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    }

    void position_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg)
    {
        current_position_ = Eigen::Vector3d(msg->data[0], msg->data[1], msg->data[2]);
    }

    void object_position_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
    {
        object_position_ = Eigen::Vector3d(msg->data[0], msg->data[2], -msg->data[1]);

        //Uses enum, now Jon Aksel is happy :-)
        if (object_position_.y() < 3 && msg->data[3] == ROV_classes_to_detect::valve)
        {
            found_object = true;
            PID_x.set_target_position((current_position_.x() + object_position_.x()));
            PID_y.set_target_position((current_position_.y() + object_position_.y()));
            PID_z.set_target_position((current_position_.z() + object_position_.z()) - 1.0);


            target_position_ = Eigen::Vector3d(PID_x.get_target_position(), PID_y.get_target_position(), PID_z.get_target_position());

            RCLCPP_INFO(this->get_logger(), 
                "Ny Target - X: %.2f, Y: %.2f, Z: %.2f", 
                PID_x.get_target_position(), PID_y.get_target_position(), PID_z.get_target_position());

            Eigen::Vector3d direction = target_position_ - current_position_;

            RCLCPP_INFO(this->get_logger(), 
                "Forskjell fra nå pos - X: %.2f, Y: %.2f, Z: %.2f", 
                direction.x(), 
                direction.y(), 
                direction.z());

            direction.normalize();

            target_object_direction_ = Eigen::Quaterniond::FromTwoVectors(
                                                Eigen::Vector3d::UnitY(),   //UnitY because that is forward on the ROV
                                                direction
                                            );
        } else if (msg->data[3] == ROV_classes_to_detect::structure)
        {
            RCLCPP_INFO(this->get_logger(), 
                "Structure detected - X: %.2f, Y: %.2f, Z: %.2f", 
                object_position_.x(), object_position_.y(), object_position_.z());
        } else if (msg->data[3] == ROV_classes_to_detect::tube)
        {
            RCLCPP_INFO(this->get_logger(), 
                "Tube detected - X: %.2f, Y: %.2f, Z: %.2f", 
                object_position_.x(), object_position_.y(), object_position_.z());
        } else if (msg->data[3] == ROV_classes_to_detect::aruco_marker)
        {
            RCLCPP_INFO(this->get_logger(), 
                "Aruco marker detected - X: %.2f, Y: %.2f, Z: %.2f", 
                object_position_.x(), object_position_.y(), object_position_.z());
        }
    }

    void controller_callback(controller_msgs::msg::ControllerState::UniquePtr msg)
    {
        if ((msg->l1 && !prev_l1_) ||
                (msg->r1 && !prev_r1_))
        {
            manual_alt_mode_ = !manual_alt_mode_;
        }

        if (msg->up && !prev_dpad_up_)
        {
            speed_percentage_ += 0.05;
            speed_percentage_ = std::min(speed_percentage_, 1.0);
        } else if (msg->down && !prev_dpad_down_)
        {
            speed_percentage_ -= 0.05;
            speed_percentage_ = std::max(speed_percentage_, 0.5);
        }

        if (msg->left && !prev_dpad_left_)
        {
            light_percentage_ += 0.05;
            light_percentage_ = std::min(light_percentage_, 1.0);
        } else if (msg->right && !prev_dpad_right_)
        {
            light_percentage_ -= 0.05;
            light_percentage_ = std::max(light_percentage_, 0.5);
        }

        Eigen::Vector3d xyz;
        Eigen::Vector3d rpy;

        if (manual_alt_mode_)
        {
            // Control Roll/Pitch
            rpy.x() = msg->lx / 100;
            // Controller gives negative values up
            rpy.y() = -msg->ly / 100;

        } else
        {
            // Yaw and z control 
            // Controller gives negative values up
            xyz.z() = -msg->ly / 100;
            // Right hand rule is :( Give me NED-FRD
            rpy.z() = -msg->lx / 100;
        }

        // Control x/y in both modes
        xyz.x() = msg->rx / 100;
        // Controller gives negative values up
        xyz.y() = -msg->ry / 100;

        Eigen::VectorXd setpoints(6);
        setpoints << xyz.x(), xyz.y(), xyz.z(),
            rpy.x(), rpy.y(), rpy.z();

        setpoints *= speed_percentage_;

        set_thrust(setpoints);

        
        prev_ps_ = msg->ps;
        prev_l1_ = msg->l1;
        prev_r1_ = msg->r1;
        prev_dpad_up_ = msg->up;
        prev_dpad_down_ = msg->down;
        prev_dpad_left_ = msg->left;
        prev_dpad_right_ = msg->right;
    }

    void update_PID()
    {
        //Updates delta time
        static double last_time = this->now().seconds();
        double current_time = this->now().seconds();
        double dt = current_time - last_time;
        last_time = current_time;

        float distance_from_target = Eigen::Vector3f(PID_x.get_error_float(), PID_y.get_error_float(), PID_z.get_error_float()).norm();

        //Looks towards target position if far away to improve SLAM
        //RCLCPP_INFO(this->get_logger(), 
        //"Distance: %.2f", distance_from_target);
        Eigen::Vector3d direction = (target_position_ - current_position_);
        direction.normalize();

        Eigen::Vector3d error_orien = PID_orientation.get_error_quat();
        Eigen::Quaterniond direction_to_target = Eigen::Quaterniond::FromTwoVectors(
                                                Eigen::Vector3d::UnitY(),   //UnitY because that is forward on the ROV
                                                direction
                                            );
        
        if (!found_object)
        {
            Eigen::Quaterniond direction_target = target_orientation_.slerp(std::min(1.0f, distance_from_target), direction_to_target);
            current_direction_target = current_direction_target.slerp(0.01, direction_target);
            PID_orientation.set_target_quaternion(direction_to_target);
        } else 
        {
            PID_orientation.set_target_quaternion(target_object_direction_);
        }

        //Runs the PID function, and rotates the force for positional correction to the correct frame (not done with rotation due to rotation being in body, and not world frame)
        Eigen::Vector3d orientation_output = PID_orientation.update(current_orientation_, dt);
        Eigen::Vector3d force_world = Eigen::Vector3d(PID_x.update(current_position_.x(), dt),
               PID_y.update(current_position_.y(), dt),
               PID_z.update(current_position_.z(), dt));
        force_world = current_orientation_.conjugate() * force_world;
        Eigen::VectorXd forces(6);

        //Reduce translation force if orientation error is big to make sure the ROV always points in the right direction
        double orientation_error_norm = PID_orientation.get_error_quat().norm();
        force_world *= 1.0 / (1.0 + 5.0 * orientation_error_norm);

        // NOT actual forces
        //Puts the forces in an array, and multiplies it with the thruster setup
        forces << force_world(0),
                  force_world(1),
                  force_world(2),
                  orientation_output(1),
                  orientation_output(0),
                  orientation_output(2);

        set_thrust(forces);

        //Get errors for logging
        Eigen::Vector3d position_error(
            PID_x.get_error_float(),
            PID_y.get_error_float(),
            PID_z.get_error_float()
        );
        Eigen::Vector3d orientation_error = PID_orientation.get_error_quat();
        log_errors_to_csv(position_error, orientation_error, current_time);
    }

    void set_thrust(const Eigen::VectorXd &forces)
    {
        Eigen::VectorXd gain = thrust_map_matrix*forces;
        gain = gain.cwiseMax(-10.0).cwiseMin(10.0);             //Forces the output between pluss, minus 10 to prevent to high power consumption, and making SLAM easier
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = std::vector<double>({gain(0), gain(1), gain(2), gain(3), gain(4), gain(5), gain(6), gain(7)});       //Publishes to thrusters
        thrust_publisher_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), 
        //    "Thruster - T1: %.2f, T2: %.2f, T3: %.2f, T4: %.2f, T5: %.2f, T6: %.2f, T7: %.2f, T8: %.2f", 
        //    gain(0), gain(1), gain(2), gain(3), gain(4), gain(5), gain(6), gain(7));
    }

    void log_errors_to_csv(const Eigen::Vector3d& position_error, const Eigen::Vector3d& orientation_error, double timestamp)
    {
        //Logs stuff to CSV, similare to same function in sensor_handler.
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
    //Defines stuff
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr orientation_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr object_position_subscriber_;
    rclcpp::Subscription<controller_msgs::msg::ControllerState>::SharedPtr controller_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_publisher_;
    Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_position_ = Eigen::Vector3d(-1.0, 6, -1.7);
    Eigen::Vector3d object_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond current_orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond target_orientation_ = Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *   // pitch
                                                            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *   // roll
                                                            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())     // yaw
                                                        );
    Eigen::Quaterniond target_object_direction_ = Eigen::Quaterniond::Identity();
    bool found_object = false;
    bool running_ = true;
    bool auto_ = false;
    bool manual_alt_mode_ = false;
    bool prev_ps_ = false;
    bool prev_l1_ = false;
    bool prev_r1_ = false;
    bool prev_dpad_up_ = false;
    bool prev_dpad_down_ = false;
    bool prev_dpad_left_ = false;
    bool prev_dpad_right_ = false;
    double speed_percentage_ = 0.25;
    double light_percentage_ = 0.5;

    Eigen::Quaterniond current_direction_target = target_orientation_;
    std::thread processing_thread_;
    //Creates PID controllers
    PID_controller PID_x;
    PID_controller PID_y;
    PID_controller PID_z;
    PID_controller PID_orientation;

    //Thruster allocation (x, y, z, roll, pitch, yaw)
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
