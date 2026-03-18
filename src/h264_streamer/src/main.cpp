#include "h264_streamer/h264_streamer.hpp"

int main(int argc, char **argv)
{
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);
    auto streamer = std::make_shared<H264Streamer>();
    rclcpp::spin(streamer);
    rclcpp::shutdown();
    return 0;
}
