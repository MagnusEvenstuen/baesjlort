#include "controller_input/controller_input.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<ControllerInput>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
