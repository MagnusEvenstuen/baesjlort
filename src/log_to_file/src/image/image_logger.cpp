#include "log_to_file/image/image_logger.hpp"
#include <filesystem>
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <algorithm>

ImageLogger::ImageLogger(const std::string &topic, std::filesystem::path path)
	: rclcpp::Node("image_logger"), topic_(topic)
{
    if (path.empty())
    {
        std::string name = topic.substr(1);
        std::replace(name.begin(), name.end(), '/', '_');
        path = name + "_image_log";
    }
    image_directory_ = path;

    RCLCPP_INFO(get_logger(), "Logging to directory %s/", std::filesystem::absolute(path).c_str());

	if (!std::filesystem::exists(path))
	{
        std::filesystem::create_directory(image_directory_);
	}

    safe_topic_ = topic_;
    std::replace(safe_topic_.begin(), safe_topic_.end(), '/', '_');

    image_subscriber_ = create_subscription<Image>(topic, 10,
            std::bind(&ImageLogger::imageSubscriptionCallback, this, std::placeholders::_1));
}

void ImageLogger::imageSubscriptionCallback(Image::UniquePtr msg)
{
    static int count = 0;
    if (count++ % 100 != 0)
    {
        return;
    }
    std::string frame_id = msg->header.frame_id;
    std::replace(frame_id.begin(), frame_id.end(), '/', '_');

    std::ostringstream oss;
    oss << safe_topic_ << "_" << msg->header.stamp.sec <<
        msg->header.stamp.nanosec << ".jpg";
    std::filesystem::path imgPath = image_directory_/oss.str();

    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(*msg, msg->encoding);
    cv::Mat bgr_image;
    cv::cvtColor(img->image, bgr_image, cv::COLOR_YUV2BGR_YUY2);
    cv::imwrite(imgPath.string(), bgr_image);
}
