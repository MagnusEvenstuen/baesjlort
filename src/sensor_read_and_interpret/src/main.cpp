#include "sensor_subscriber.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sensor_subscriber>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}