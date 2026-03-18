#include "h264_receiver/h264_receiver.hpp"

int main(int argc, char **argv)
{
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);
    auto receiver = std::make_shared<H264Receiver>();
    rclcpp::spin(receiver);
    rclcpp::shutdown();
    return 0;
}
