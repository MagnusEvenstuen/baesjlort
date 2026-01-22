#ifndef IMAGE_DISPLAY_AND_HANDLE_HPP
#define IMAGE_DISPLAY_AND_HANDLE_HPP

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>


class image_display_and_handle
{
public:
    image_display_and_handle() = default;
    ~image_display_and_handle() = default;

    void display_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg, const std::string& window_name, rclcpp::Logger logger)
    {
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, msg->encoding)->image;
        cv::imshow(window_name, cv_image);
        cv::waitKey(1);  // 1ms delay to allow event processing
    }

private:
    
};

#endif // DISPLAY_IMAGES_HPP