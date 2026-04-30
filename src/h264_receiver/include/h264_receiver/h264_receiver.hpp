#ifndef H264_RECEIVER_HPP
#define H264_RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using camera_info_manager::CameraInfoManager;

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
    H264Receiver(const std::string &image_topic);

    void image_received_callback(CompressedImage::UniquePtr msg);
    static void image_decoded_callback(GstElement *sink, H264Receiver *data);

    void publish_image(const Image &msg);
    void publish_info();

private:
    rclcpp::Subscription<CompressedImage>::SharedPtr image_subscriber_;
    rclcpp::Publisher<Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<CameraInfo>::SharedPtr camera_info_publisher_;
    std::unique_ptr<CameraInfoManager> cim;

    GstElement *src_;
    GstElement *decoder_;
    GstElement *parser_;
    GstElement *converter_;
    GstElement *capsfilter_;
    GstElement *sink_;
    UniquePipeline pipeline_;

    CameraInfo camera_info_;
};

#endif // H264_RECEIVER_HPP
