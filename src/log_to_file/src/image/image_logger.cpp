#include "log_to_file/image/image_logger.hpp"
#include <ctime>
#include <filesystem>
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/imgcodecs.hpp>
#include <sstream>

ImageLogger::ImageLogger(const std::string &topic, std::filesystem::path path)
	: rclcpp::Node("image_logger")
{
    if (path.empty())
    {
        std::string name = topic.substr(1);
        std::replace(name.begin(), name.end(), '/', '_');
        path = name + "_image_log";
    }
    image_directory_ = path;

    RCLCPP_INFO(get_logger(), "Logging to %s", std::filesystem::absolute(path).c_str());

	if (!std::filesystem::exists(path))
	{
        std::filesystem::create_directory(image_directory_);
	}

	// TODO: Create subscribers
    image_subscriber_ = create_subscription<Image>(topic, 10,
            std::bind(&ImageLogger::imageSubscriptionCallback, this, std::placeholders::_1));
}

void ImageLogger::imageSubscriptionCallback(Image::UniquePtr msg)
{
    std::string frame_id = msg->header.frame_id;
    std::replace(frame_id.begin(), frame_id.end(), '/', '_');

    std::ostringstream oss;
    oss << frame_id << "_" << msg->header.stamp.sec <<
        msg->header.stamp.nanosec << ".jpg";
    std::filesystem::path imgPath = image_directory_/oss.str();

    // RCLCPP_INFO(get_logger(), "Writing image to path: %s", std::filesystem::absolute(imgPath).c_str());
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(*msg, msg->encoding);
    cv::imwrite(imgPath.string(), img->image);
}
