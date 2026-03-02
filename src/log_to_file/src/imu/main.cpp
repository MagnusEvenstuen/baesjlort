#include "log_to_file/imu/imu_logger.hpp"
#include <filesystem>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
    auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() <= 2)
    {
        std::filesystem::path path;
        if (args.size() == 2)
        {
            path = args[1];
        }
        auto logger = std::make_shared<ImuLogger>(path);
        rclcpp::spin(logger);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("imu_logger"), "Invalid number of user arguments. Got %li, expected 1", args.size() - 1);
        for (unsigned i = 1; i < args.size(); i++)
        {
            RCLCPP_ERROR(rclcpp::get_logger("imu_logger"), "%s", args[i].c_str());
        }
    }

	rclcpp::shutdown();
}
