#ifndef SYS_ID_COMMANDS_HPP
#define SYS_ID_COMMANDS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <array>
#include <vector>
#include <chrono>

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
        if (std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1) - start_time_ < 10000)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for other nodes to initialize...");
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
        } else if (std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1) - start_time_ < 40000)
        {
            RCLCPP_INFO(this->get_logger(), "Going down for system identification...");
            send_thrust_command(down_*40.0);
            rclcpp::sleep_for(std::chrono::milliseconds(4800));
            send_thrust_command(pitch_down_*20.0);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        random_signal_sys_id(120000.0); //Run system identification for 2 minutes
    }

    void random_signal_sys_id(const float milliseonds_of_sys_id)
    {
        unsigned long sys_id_start_time = std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> duration_dist(20, 4000);
        std::uniform_int_distribution<> power_dist(0, 20);
        std::uniform_int_distribution<> command_dist(0, 11);
        while (std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1) - sys_id_start_time < milliseonds_of_sys_id)
        {
            int command_index = command_dist(gen);
            int milliseconds_duration = duration_dist(gen);
            int power = power_dist(gen);
            switch (command_index)
            {
                case 0:
                    send_thrust_command(forward_*power);
                    break;
                case 1:
                    send_thrust_command(backward_*power);
                    break;
                case 2:
                    send_thrust_command(left_*power);
                    break;
                case 3:
                    send_thrust_command(right_*power);
                    break;
                case 4:
                    send_thrust_command(up_*power);
                    break;
                case 5:
                    send_thrust_command(down_*power);
                    break;
                case 6:
                    send_thrust_command(pitch_down_*power);
                    break;
                case 7:
                    send_thrust_command(pritch_up_*power);
                    break;
                case 8:
                    send_thrust_command(roll_left_*power);
                    break;
                case 9:
                    send_thrust_command(roll_right_*power);
                    break;
                case 10:
                    send_thrust_command(yaw_left_*power);
                    break;
                case 11:
                    send_thrust_command(yaw_right_*power);
                    break;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(milliseconds_duration));
        }
    }


private:
    void send_thrust_command(const std::array<float, 8>& thrust_values)
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = std::vector<double>(thrust_values.begin(), thrust_values.end());
        thrust_publisher_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Sent thrust command");
    }


private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thrust_publisher_;

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