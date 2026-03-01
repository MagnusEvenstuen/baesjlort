#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <filesystem>

class FileLogger : public rclcpp::Node
{
public:
	FileLogger(std::filesystem::path path);

	void timerCallback();

private:
	std::ofstream log_file_;
	rclcpp::TimerBase::SharedPtr timer_;
	// TODO: create variables to store, unless I find a better way
};
