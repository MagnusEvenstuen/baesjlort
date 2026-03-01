#include "log_to_file/log_to_file.hpp"
#include <ctime>
#include <chrono>
#include <filesystem>

FileLogger::FileLogger(std::filesystem::path path)
	: rclcpp::Node("file_logger")
{
    if (path.empty())
    {
        path = "log.csv";
    }

    RCLCPP_INFO(get_logger(), "Logging to %s", std::filesystem::absolute(path).c_str());

	if (std::filesystem::exists(path))
	{
		log_file_.open(path, std::ios::app);
	}
	else
	{
		log_file_.open(path);
		// Make csv headers
		log_file_ << "timestamp,other header" << std::endl;
	}

	timer_ = create_wall_timer(std::chrono::milliseconds(200),
			std::bind(&FileLogger::timerCallback, this));

	// TODO: Create subscribers
}

void FileLogger::timerCallback()
{
	log_file_ << std::chrono::system_clock::now().time_since_epoch().count() << "," << "noge" << std::endl;
}
