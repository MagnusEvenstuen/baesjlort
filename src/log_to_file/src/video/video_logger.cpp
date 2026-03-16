#include "log_to_file/video/video_logger.hpp"
#include <filesystem>
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/imgcodecs.hpp>

VideoLogger::VideoLogger(const std::string &topic, std::filesystem::path path)
	: rclcpp::Node("video_logger")
{
    if (path.empty())
    {
        std::string name = topic.substr(1) + "_" + std::to_string(get_clock()->now().nanoseconds());
        std::replace(name.begin(), name.end(), '/', '_');
        path = name + ".mp4";
    }
    video_path_ = path;

    RCLCPP_INFO(get_logger(), "Logging to %s", std::filesystem::absolute(path).c_str());

    video_subscriber_ = create_subscription<Image>(topic, 10,
            std::bind(&VideoLogger::imageSubscriptionCallback, this, std::placeholders::_1));
}

VideoLogger::~VideoLogger()
{
    video_writer_.release();
}

void VideoLogger::imageSubscriptionCallback(Image::UniquePtr msg)
{
    if (!video_writer_.isOpened())
    {
        cv::Size size = { (int)msg->width, (int)msg->height };
        video_writer_.open(video_path_.string(), cv::VideoWriter::fourcc('H', '2', '6', '4'), 30, size);
    }

    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(*msg, "bgr8");
    //cv::imwrite(imgPath.string(), img->image);
    video_writer_.write(img->image);
}
