#include "h264_streamer/h264_streamer.hpp"
#include <string>

H264Streamer::H264Streamer()
    : rclcpp::Node("h264_streamer_node")
{
    this->declare_parameter<std::string>("device", "/dev/video0");
    std::string device = this->get_parameter("device").as_string();

    this->declare_parameter<std::string>("topic_name", "/image_compressed");
    std::string topic_name = this->get_parameter("topic_name").as_string();

    camera_ = gst_element_factory_make("v4l2src", "camera");
    g_object_set(camera_, "device", device.c_str(), nullptr);

    capsfilter_ = gst_element_factory_make("capsfilter", "capsfilter");
    convert_    = gst_element_factory_make("videoconvert", "convert");
    encoder_    = gst_element_factory_make("x264enc", "encoder");
    sink_       = gst_element_factory_make("appsink", "sink");

    pipeline_ = UniquePipeline(gst_pipeline_new("h264_stream"));

    GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",
        "width", G_TYPE_INT, 640,
        "height", G_TYPE_INT, 480,
        "framerate", GST_TYPE_FRACTION, 30, 1,
        nullptr);
    g_object_set(capsfilter_, "caps", caps, nullptr);
    gst_caps_unref(caps);

    g_object_set(encoder_,
        "tune", 4,
        "speed-preset", 1,
        "key-int-max", 1,
        "bitrate", 2000,
        nullptr);

    g_object_set(sink_,
        "sync", FALSE,
        "max-buffers", 1,
        "drop", TRUE,
        nullptr);

    gst_bin_add_many(GST_BIN(pipeline_.get()), camera_,
                     capsfilter_, convert_, encoder_, sink_, nullptr);

    if (gst_element_link_many(camera_, capsfilter_, convert_,
                              encoder_, sink_, nullptr) != TRUE)
    {
        std::cerr << "Failed to link gstreamer elements" << std::endl;
        std::exit(-1);
    }

    gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);

    image_publisher_ = create_publisher<CompressedImage>(topic_name, 1);
    thread_ = std::make_unique<std::jthread>(
            std::bind(&H264Streamer::image_capture, this, std::placeholders::_1));
}

H264Streamer::~H264Streamer()
{
    thread_.reset();
}

void H264Streamer::image_capture(std::stop_token st)
{
    // for (int i = 0; i < 1000; i++)
    while (!st.stop_requested())
    {
        std::lock_guard<std::mutex> lock(mutex_);
        GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
        if (!sample)
        {
            std::cout << "No samples" << std::endl;
            continue;
        }
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer)
        {
            std::cout << "No buffer" << std::endl;
            continue;
        }

        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_READ) != TRUE)
        {
            std::cout << "No map" << std::endl;
            continue;
        }
        CompressedImage msg;
        std::copy(map.data, map.data + map.size, std::back_inserter(msg.data));
        msg.header.stamp = rclcpp::Time(buffer->dts);
        msg.format = "h264";
        image_publisher_->publish(msg);
        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
    }
}
