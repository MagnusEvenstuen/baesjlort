#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include "sensor_msgs/msg/imu.hpp"

using sensor_msgs::msg::Imu;

class ImuLogger : public rclcpp::Node
{
public:
	ImuLogger(std::filesystem::path path);

    void imuSubscriptionCallback(Imu::UniquePtr msg);

private:
	std::ofstream log_file_;
    rclcpp::Subscription<Imu>::SharedPtr imu_subscriber_;
};
