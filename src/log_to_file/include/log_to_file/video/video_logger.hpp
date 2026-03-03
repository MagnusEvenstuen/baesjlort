#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/videoio.hpp>

using sensor_msgs::msg::Image;

class VideoLogger : public rclcpp::Node
{
public:
	VideoLogger(const std::string &topic, std::filesystem::path path);
    ~VideoLogger();

    void imageSubscriptionCallback(Image::UniquePtr msg);

private:
    std::filesystem::path video_path_;
    rclcpp::Subscription<Image>::SharedPtr video_subscriber_;
    cv::VideoWriter video_writer_;
};
