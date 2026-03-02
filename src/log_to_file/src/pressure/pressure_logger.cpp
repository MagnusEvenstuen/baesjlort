#include "log_to_file/pressure/pressure_logger.hpp"
#include <filesystem>

PressureLogger::PressureLogger(const std::string &topic, std::filesystem::path path)
	: rclcpp::Node("controller_logger")
{
    if (path.empty())
    {
        std::string name = topic.substr(1);
        std::replace(name.begin(), name.end(), '/', '_');
        path = name + ".csv";
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
		log_file_ << "timestamp,frame_id,"
            << "fluid_pressure,variance" << std::endl;
	}

    pressure_subscriber_ = create_subscription<FluidPressure>(topic, 10,
            std::bind(&PressureLogger::pressureSubscriptionCallback, this, std::placeholders::_1));
}

void PressureLogger::pressureSubscriptionCallback(FluidPressure::UniquePtr msg)
{
    // Concatenate seconds and nanoseconds
    log_file_ << msg->header.stamp.sec << msg->header.stamp.nanosec
        << ',' << msg->header.frame_id
        << ',' << msg->fluid_pressure
        << ',' << msg->variance << std::endl;
}
