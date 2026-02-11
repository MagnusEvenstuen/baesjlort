#ifndef SYS_ID_COMMANDS_HPP
#define SYS_ID_COMMANDS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <array>
#include <vector>
#include <chrono>
#include <random>
#include <fstream>
#include <iomanip>
#include <string>
#include <filesystem>
#include "../../sensor_read_and_interpret/include/structs.hpp"

class sys_id_commands : public rclcpp::Node
{
public:
    sys_id_commands() : Node("sys_id_commands"), 
        setup_complete_(false),
        going_down_complete_(false)
    {
        thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gbr/thrusters",
            10
        );
        avg_gyro_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/average_gyro",
            10,
            std::bind(&sys_id_commands::avg_gyro_callback, this, std::placeholders::_1)
        );
        avg_acc_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/average_acceleration",
            10,
            std::bind(&sys_id_commands::avg_acc_callback, this, std::placeholders::_1)
        );

        //Using timer to prevent blocking while
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&sys_id_commands::control_loop, this)
        );

        std::random_device rd;
        gen_ = std::mt19937(rd());
        duration_dist_ = std::uniform_int_distribution<>(20, 10000);
        power_dist_ = std::uniform_int_distribution<>(0, 20);
        command_dist_ = std::uniform_int_distribution<>(0, 11);

        start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "System identification node started");
    }

    ~sys_id_commands()
    {
        //Close CSV if open
        if (csv_file_.is_open())
        {
            csv_file_.close();
            RCLCPP_INFO(this->get_logger(), "CSV file closed");
        }
    }

    void run_system_identification()
    {
        static std::chrono::steady_clock::time_point command_start_time;
        static std::array<float, 8> current_command;
        static int command_duration = 0;

        auto now = std::chrono::steady_clock::now();

        if (now >= command_start_time + std::chrono::milliseconds(command_duration))
        {
            int command_index = command_dist_(gen_);
            command_duration = duration_dist_(gen_);
            float power = static_cast<float>(power_dist_(gen_));

            switch (command_index)
            {
                case 0:
                    current_command = multiply_array(forward_, power);
                    break;
                case 1:
                    current_command = multiply_array(backward_, power);
                    break;
                case 2:
                    current_command = multiply_array(left_, power);
                    break;
                case 3:
                    current_command = multiply_array(right_, power);
                    break;
                case 4:
                    current_command = multiply_array(up_, power);
                    break;
                case 5:
                    current_command = multiply_array(down_, power);
                    break;
                case 6:
                    current_command = multiply_array(pitch_down_, power);
                    break;
                case 7:
                    current_command = multiply_array(pritch_up_, power);
                    break;
                case 8:
                    current_command = multiply_array(roll_left_, power);
                    break;
                case 9:
                    current_command = multiply_array(roll_right_, power);
                    break;
                case 10:
                    current_command = multiply_array(yaw_left_, power);
                    break;
                case 11:
                    current_command = multiply_array(yaw_right_, power);
                    break;
                default:
                    current_command = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                    break;
            }

            command_start_time = now;
        }

        send_thrust_command(current_command);
        log_to_csv(current_command);
    }


private:
    //Function made iterativley with ChatGPT
    void control_loop()
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();

        if (elapsed < 10000)
        {
            static bool logged = false;
            if (!logged && elapsed > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for initialization... %lld ms", elapsed);
                logged = true;
            }
            return;
        }

        if (elapsed < 180000)
        {
            if (!going_down_complete_)
            {
                RCLCPP_INFO(this->get_logger(), "Going down... %lld ms", elapsed - 10000);
                send_thrust_command(multiply_array(down_, 100.0));
                
                static int interval = 3000;
                static int duration = 250;
                
                if ((elapsed % interval) < duration) {
                    send_thrust_command(multiply_array(pitch_down_, 100.0));
                }
            }
            return;
        }

        if (!setup_complete_)
        {
            RCLCPP_INFO(this->get_logger(), "Starting system identification");
            going_down_complete_ = true;
            setup_csv_file();
            sys_id_start_time_ = now;
            setup_complete_ = true;
        }

        auto sys_id_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - sys_id_start_time_).count();

        if (sys_id_elapsed < 900000)  // 15 minutter
        {
            run_system_identification();
        }
        else
        {
            send_thrust_command({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "System identification complete");
        }
    }


    void avg_gyro_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        gyro_.x = msg->data[0];
        gyro_.y = msg->data[1];
        gyro_.z = msg->data[2];
    }

    void avg_acc_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        acc_.x = msg->data[0];
        acc_.y = msg->data[1];
        acc_.z = msg->data[2];
    }

    std::array<float, 8> multiply_array(const std::array<float, 8>& arr, float scalar) const
    {
        std::array<float, 8> result;
        for (size_t i = 0; i < arr.size(); ++i) {
            result[i] = arr[i] * scalar;
        }
        return result;
    }

    void send_thrust_command(const std::array<float, 8>& thrust_values)
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = std::vector<double>(thrust_values.begin(), thrust_values.end());
        thrust_publisher_->publish(msg);
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Sent thrust command: " 
            << thrust_values[0] << ", " << thrust_values[1] << ", " 
            << thrust_values[2] << ", " << thrust_values[3] << ", "
            << thrust_values[4] << ", " << thrust_values[5] << ", "
            << thrust_values[6] << ", " << thrust_values[7]);
    }

    void setup_csv_file()
    {
        std::filesystem::path dir_path = "/home/gud/Skole/baesjlort/src/sys_id/data_files";
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream filename;
        filename << dir_path.string() << "/thrust_commands_"
                 << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".csv";
        
        csv_file_.open(filename.str(), std::ios::out);
        if (csv_file_.is_open())
        {
            csv_file_ << "timestamp_ms,thruster0,thruster1,thruster2,thruster3,"
                     << "thruster4,thruster5,thruster6,thruster7,acc_x,acc_y,acc_z,"
                     << "gyro_x,gyro_y,gyro_z\n";
            csv_file_.flush();
        }
    }

    void log_to_csv(const std::array<float, 8>& thrust_values)
    {
        if (csv_file_.is_open())
        {
            auto current_time = std::chrono::steady_clock::now();
            auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time.time_since_epoch()).count();
            
            csv_file_ << timestamp_ms << ","
                     << std::fixed << std::setprecision(2) 
                     << thrust_values[0] << "," << thrust_values[1] << "," 
                     << thrust_values[2] << "," << thrust_values[3] << ","
                     << thrust_values[4] << "," << thrust_values[5] << ","
                     << thrust_values[6] << "," << thrust_values[7] << "," 
                     << acc_.x << "," << acc_.y << "," << acc_.z << ","
                     << gyro_.x << "," << gyro_.y << "," << gyro_.z << "\n";
            
            static int flush_counter = 0;
            flush_counter++;
            if (flush_counter % 100 == 0)
            {
                csv_file_.flush();
            }
        }
    }


private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr avg_gyro_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr avg_acc_subscriber_;
    std::ofstream csv_file_;
    bool setup_complete_;
    bool going_down_complete_;

    Vector3 acc_ = {0, 0, 0};
    Vector3 gyro_ = {0, 0, 0};
    //Possible command for system identification with one as value to be multiplied with thrust factor
    std::array<float, 8> forward_ = {1.0, 1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 8> backward_ = {-1.0, -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 8> left_ = {1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 8> right_ = {-1.0, 1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 8> down_ = {0.0, 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, -1.0};
    std::array<float, 8> up_ = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
    std::array<float, 8> pitch_down_ = {0.0, 0.0, 0.0, 0.0, -1.0, -1.0, 1.0, 1.0};
    std::array<float, 8> pritch_up_ = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, -1.0, -1.0};
    std::array<float, 8> roll_left_ = {0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0};
    std::array<float, 8> roll_right_ = {0.0, 0.0, 0.0, 0.0, -1.0, 1.0, -1.0, 1.0};
    std::array<float, 8> yaw_left_ = {1.0, -1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 8> yaw_right_ = {-1.0, 1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0};
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point sys_id_start_time_;
    
    std::mt19937 gen_;
    std::uniform_int_distribution<> duration_dist_;
    std::uniform_int_distribution<> power_dist_;
    std::uniform_int_distribution<> command_dist_;
};

#endif // SYS_ID_COMMANDS_HPP
