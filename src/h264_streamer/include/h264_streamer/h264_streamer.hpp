#ifndef H264_STREAMER_HPP
#define H264_STREAMER_HPP

#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <thread>
#include <sensor_msgs/msg/compressed_image.hpp>

using sensor_msgs::msg::CompressedImage;

struct PipelineDeleter
{
    void operator()(GstElement *gst_element)
    {
        gst_element_set_state(gst_element, GST_STATE_NULL);
        gst_object_unref(gst_element);
    }
};

using UniquePipeline = std::unique_ptr<GstElement, PipelineDeleter>;

class H264Streamer : public rclcpp::Node
{
public:
    H264Streamer();
    ~H264Streamer();

    void image_capture(std::stop_token st);

private:
    rclcpp::Publisher<CompressedImage>::SharedPtr image_publisher_;

    std::unique_ptr<std::jthread> thread_;
    std::mutex mutex_;

    GstElement *camera_;
    GstElement *capsfilter_;
    // Just for testing
    GstElement *encoder_;
    GstElement *sink_;
    UniquePipeline pipeline_;
};

#endif // H264_STREAMER_HPP
