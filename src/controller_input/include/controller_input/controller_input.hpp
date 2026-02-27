#ifndef CONTROLLER_INPUT_HPP
#define CONTROLLER_INPUT_HPP

#include "rclcpp/rclcpp.hpp"
#include "controller_msgs/msg/controller_state.hpp"

using controller_msgs::msg::ControllerState;

class ControllerInput : public rclcpp::Node
{
public:
    // Ps3 Controller
    enum Button { Cross, Circle, Triangle, Square, L1, R1,
        L2, R2, Select, Start, PS, L3, R3, Up, Down, Left, Right };
public:
    ControllerInput();

    void timerCallback() const;

private:
    rclcpp::Publisher<ControllerState>::SharedPtr controller_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // CONTROLLER_INPUT_HPP
