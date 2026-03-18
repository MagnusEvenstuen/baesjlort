#include "h264_receiver/h264_receiver.hpp"

H264Receiver::H264Receiver(const std::string &image_topic)
    : rclcpp::Node("h264_receiver_node")
{
    src_ = gst_element_factory_make("appsrc", "src");
    decoder_ = gst_element_factory_make("avdec_h264", "decoder");
    converter_ = gst_element_factory_make("videoconvert", "converter");
    capsfilter_ = gst_element_factory_make("capsfilter", "capsfilter");
    sink_ = gst_element_factory_make("appsink", "sink");

    pipeline_ = UniquePipeline(gst_pipeline_new("h264_stream"));

    GstCaps *caps = gst_caps_new_simple(
            "video/x-h264",
            "stream-format", G_TYPE_STRING, "byte-stream",
            "alignment", G_TYPE_STRING, "au",
            nullptr);

    g_object_set(src_,
            "caps", caps,
            "format", GST_FORMAT_TIME,
            nullptr);

    gst_caps_unref(caps);
    
    caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "RGB",
        // "width", G_TYPE_INT, 640,
        // "height", G_TYPE_INT, 480,
        // "framerate", GST_TYPE_FRACTION, 30, 1,
        nullptr);

    g_object_set(capsfilter_,
            "caps", caps,
            nullptr);

    gst_caps_unref(caps);

    gst_bin_add_many(GST_BIN(pipeline_.get()), src_, decoder_,
            converter_, capsfilter_, sink_, nullptr);

    if (gst_element_link_many(src_, decoder_, converter_,
                capsfilter_, sink_, nullptr) != TRUE)
    {
        std::cerr << "Failed to link gstreamer elements" << std::endl;
        std::exit(-1);
    }

    g_object_set(sink_, "emit-signals", TRUE, nullptr);
    g_signal_connect(sink_, "new-sample",
            G_CALLBACK(&H264Receiver::image_decoded_callback), this);

    gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);

    RCLCPP_INFO(get_logger(), "Subscribing to topic '%s'", image_topic.c_str()); 

    image_subscriber_ = create_subscription<CompressedImage>(image_topic, 1,
            std::bind(&H264Receiver::image_received_callback, this, std::placeholders::_1));
    image_publisher_ = create_publisher<Image>("image_raw", 1);
}

void H264Receiver::image_received_callback(CompressedImage::UniquePtr msg)
{
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    std::copy(msg->data.begin(), msg->data.end(), map.data);
    map.size = msg->data.size();
    gst_buffer_unmap(buffer, &map);

    // Transfers the ownership of buffer, no need for cleanup
    if (gst_app_src_push_buffer(GST_APP_SRC(src_), buffer) != GST_FLOW_OK)
    {
        std::cout << "Failed to push buffer" << std::endl;
        return;
    }
}

void H264Receiver::image_decoded_callback(GstElement *sink, H264Receiver *data)
{
    std::cout << "Entering decoded callback" << std::endl;
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample)
    {
        std::cout << "No samples" << std::endl;
        return;
    }
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer)
    {
        std::cout << "No buffer" << std::endl;
        return;
    }

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ) != TRUE)
    {
        std::cout << "No map" << std::endl;
        return;
    }
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *s = gst_caps_get_structure(caps, 0);

    int width, height;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);
    const char *format = gst_structure_get_string(s, "format");
    std::cout << "Format: " << format << ", Size: ("
        << width << ", " << height << ')' << std::endl;

    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(buffer->dts);
    cv::Mat img(height, width, CV_8UC3, (char*)map.data);
    
    auto pub_msg = cv_bridge::CvImage(header, "rgb8", img).toImageMsg();

    data->publish_image(*pub_msg);
    
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
}

void H264Receiver::publish_image(const Image &msg)
{
    image_publisher_->publish(msg);
}
