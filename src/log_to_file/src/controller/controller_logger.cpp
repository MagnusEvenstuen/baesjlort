#include "log_to_file/controller/controller_logger.hpp"
#include <filesystem>

ControllerLogger::ControllerLogger(const std::string &topic, std::filesystem::path path)
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
		log_file_ << "timestamp,frame_id,lx,ly,rx,ry,"
            << "cross,circle,triangle,square,"
            << "l1,r1,l2,r2,select,start,ps,l3,r3,"
            << "up,down,left,right"
            << std::endl;
	}

    controller_subscriber_ = create_subscription<ControllerState>(topic, 10,
            std::bind(&ControllerLogger::controllerSubscriptionCallback, this, std::placeholders::_1));
}

void ControllerLogger::controllerSubscriptionCallback(ControllerState::UniquePtr msg)
{
    // Concatenate seconds and nanoseconds
    log_file_ << msg->header.stamp.sec << msg->header.stamp.nanosec
        << ',' << msg->header.frame_id << ',' << msg->lx
        << ',' << msg->ly << ',' << msg->rx << ',' << msg->ry
        << ',' << msg->cross << ',' << msg->circle
        << ',' << msg->triangle << ',' << msg->square
        << ',' << msg->l1 << ',' << msg->r1 << ',' << msg->l2
        << ',' << msg->r2 << ',' << msg->select << ',' << msg->start
        << ',' << msg->ps << ',' << msg->l3 << ',' << msg->r3
        << ',' << msg->up << ',' << msg->down << ',' << msg->left
        << ',' << msg->right << std::endl;
}
