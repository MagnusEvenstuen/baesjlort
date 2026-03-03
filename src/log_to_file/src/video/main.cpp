#include "log_to_file/video/video_logger.hpp"
#include <filesystem>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
    auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() <= 3)
    {
        std::filesystem::path path;
        if (args.size() == 3)
        {
            path = args[2];
        }
        auto logger = std::make_shared<VideoLogger>(args[1], path);
        rclcpp::spin(logger);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("video_logger"), "Invalid number of user arguments. Got %li, expected 1", args.size() - 1);
        for (unsigned i = 1; i < args.size(); i++)
        {
            RCLCPP_ERROR(rclcpp::get_logger("video_logger"), "%s", args[i].c_str());
        }
    }

	rclcpp::shutdown();
}
