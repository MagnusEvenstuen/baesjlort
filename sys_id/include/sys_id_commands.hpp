#ifndef SYS_ID_COMMANDS_HPP
#define SYS_ID_COMMANDS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <array>
#include <vector>
#include <chrono>
#include <random>
#include <fstream>
#include <iomanip>
#include <string>
#include <filesystem>

class sys_id_commands : public rclcpp::Node
{
public:
    sys_id_commands() : Node("sys_id_commands")
    {
        thrust_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gbr/thrusters",
            10
        );
        //Start with delay to allow other nodes to initialize and calibration to be done
        start_time_ = std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1);
        while (std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1) - start_time_ < 10000)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for other nodes to initialize...");
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
        } 
        while (std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1) - start_time_ < 40000)
        {
            RCLCPP_INFO(this->get_logger(), "Going down deep for system identification...");
            send_thrust_command(multiply_array(down_, 100.0));
        }
        setup_csv_file();
        random_signal_sys_id(120000); //Run system identification for 2 minutes
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

    void random_signal_sys_id(const unsigned int milliseonds_of_sys_id)
    {
        unsigned long sys_id_start_time = std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> duration_dist(20, 4000);
        std::uniform_int_distribution<> power_dist(0, 20);
        std::uniform_int_distribution<> command_dist(0, 11);
        while (std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1) - sys_id_start_time < static_cast<long long>(milliseonds_of_sys_id))
        {
            int command_index = command_dist(gen);
            int command_duration = duration_dist(gen);
            float power = static_cast<float>(power_dist(gen));
            
            auto command_start_time = std::chrono::steady_clock::now();
            
            std::array<float, 8> command;
            switch (command_index)
            {
                case 0:
                    command = multiply_array(forward_, power);
                    break;
                case 1:
                    command = multiply_array(backward_, power);
                    break;
                case 2:
                    command = multiply_array(left_, power);
                    break;
                case 3:
                    command = multiply_array(right_, power);
                    break;
                case 4:
                    command = multiply_array(up_, power);
                    break;
                case 5:
                    command = multiply_array(down_, power);
                    break;
                case 6:
                    command = multiply_array(pitch_down_, power);
                    break;
                case 7:
                    command = multiply_array(pritch_up_, power);
                    break;
                case 8:
                    command = multiply_array(roll_left_, power);
                    break;
                case 9:
                    command = multiply_array(roll_right_, power);
                    break;
                case 10:
                    command = multiply_array(yaw_left_, power);
                    break;
                case 11:
                    command = multiply_array(yaw_right_, power);
                    break;
                default:
                    command = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                    break;
            }
            
            auto end_time = command_start_time + std::chrono::milliseconds(command_duration);
            while (std::chrono::steady_clock::now() < end_time)
            {
                send_thrust_command(command);
                log_to_csv(command);
                rclcpp::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }


private:
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
        std::filesystem::path dir_path = "/home/gud/Skole/baesjlort/sys_id/data_files";
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream filename;
        filename << dir_path.string() << "/thrust_commands_"
                 << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".csv";
        
        csv_file_.open(filename.str(), std::ios::out);
        if (csv_file_.is_open())
        {
            csv_file_ << "timestamp_ms,thruster0,thruster1,thruster2,thruster3,"
                     << "thruster4,thruster5,thruster6,thruster7\n";
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
                     << thrust_values[6] << "," << thrust_values[7] << "\n";
            
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
    std::ofstream csv_file_;

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
    unsigned long start_time_;
};

#endif // SYS_ID_COMMANDS_HPP