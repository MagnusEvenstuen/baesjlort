#ifndef H264_RECEIVER_HPP
#define H264_RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

struct PipelineDeleter
{
    void operator()(GstElement *gst_element)
    {
        gst_element_set_state(gst_element, GST_STATE_NULL);
        gst_object_unref(gst_element);
    }
};

using UniquePipeline = std::unique_ptr<GstElement, PipelineDeleter>;

class H264Receiver : public rclcpp::Node
{
public:
    H264Receiver();
    ~H264Receiver();

    void image_received_callback(CompressedImage::UniquePtr msg);
    static void image_decoded_callback(GstElement *sink, H264Receiver *data);

    void publish_image(const Image &msg);

private:
    rclcpp::Subscription<CompressedImage>::SharedPtr image_subscriber_;
    rclcpp::Publisher<Image>::SharedPtr image_publisher_;

    GstElement *src_;
    GstElement *decoder_;
    GstElement *converter_;
    GstElement *capsfilter_;
    GstElement *sink_;
    UniquePipeline pipeline_;
};

#endif // H264_RECEIVER_HPP
