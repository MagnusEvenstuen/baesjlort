#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include "sensor_msgs/msg/fluid_pressure.hpp"

using sensor_msgs::msg::FluidPressure;

class PressureLogger : public rclcpp::Node
{
public:
	PressureLogger(const std::string &topic, std::filesystem::path path);

    void pressureSubscriptionCallback(FluidPressure::UniquePtr msg);

private:
	std::ofstream log_file_;
    rclcpp::Subscription<FluidPressure>::SharedPtr pressure_subscriber_;
};
