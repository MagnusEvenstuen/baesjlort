#include "h264_receiver/h264_receiver.hpp"

int main(int argc, char **argv)
{
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);
    auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() == 2)
    {
        auto streamer = std::make_shared<H264Receiver>(args.back());
        rclcpp::spin(streamer);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("h264_streamer"),
                "Got %li arguments, expected 2", args.size());
    
        for (const auto &arg : args)
        {
            RCLCPP_ERROR(rclcpp::get_logger("h264_streamer"), "%s", arg.c_str());
        }
    }
    rclcpp::shutdown();
    return 0;
}
