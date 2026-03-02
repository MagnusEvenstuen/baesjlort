#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include "controller_msgs/msg/controller_state.hpp"

using controller_msgs::msg::ControllerState;

class ControllerLogger : public rclcpp::Node
{
public:
	ControllerLogger(const std::string &topic, std::filesystem::path path);

    void controllerSubscriptionCallback(ControllerState::UniquePtr msg);

private:
	std::ofstream log_file_;
    rclcpp::Subscription<ControllerState>::SharedPtr controller_subscriber_;
};
