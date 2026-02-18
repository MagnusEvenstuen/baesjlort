#include "ROV_controller/ROV_controller.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROV_controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}