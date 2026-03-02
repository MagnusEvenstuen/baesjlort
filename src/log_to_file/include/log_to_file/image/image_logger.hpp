#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include "sensor_msgs/msg/image.hpp"

using sensor_msgs::msg::Image;

class ImageLogger : public rclcpp::Node
{
public:
	ImageLogger(std::filesystem::path path);

    void imageSubscriptionCallback(Image::UniquePtr msg);

private:
    std::filesystem::path image_directory_;
    rclcpp::Subscription<Image>::SharedPtr image_subscriber_;
};
