#include "h264_receiver/h264_receiver.hpp"

H264Receiver::H264Receiver(const std::string &image_topic)
    : rclcpp::Node("h264_receiver_node")
{

    this->declare_parameter<std::string>("topic_out", "image_raw");
    std::string topic_out = this->get_parameter("topic_out").as_string();
    this->declare_parameter<bool>("use_nvidia", false);
    bool use_nvidia = this->get_parameter("use_nvidia").as_bool();
    this->declare_parameter<std::string>("cam_info_path", "");
    std::string cam_info_path = this->get_parameter("cam_info_path").as_string();

    src_ = gst_element_factory_make("appsrc", "src");
    parser_ = gst_element_factory_make("h264parse", "parser");
    if (use_nvidia)
    {
        decoder_ = gst_element_factory_make("nvh264dec", "decoder");
        converter_ = gst_element_factory_make("cudaconvert", "converter");
        downloader_ = gst_element_factory_make("cudadownload", "downloader");
        RCLCPP_INFO(get_logger(), "Using nvidia decoder and converter");
    }
    else
    {
        decoder_ = gst_element_factory_make("avdec_h264", "decoder");
        converter_ = gst_element_factory_make("videoconvert", "converter");
        downloader_ = gst_element_factory_make("identity", "dummy");
        RCLCPP_INFO(get_logger(), "Using software decoder and converter");
    }
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
        "is-live", TRUE,
        "do-timestamp", TRUE,
        "block", TRUE,
        nullptr);

    gst_caps_unref(caps);
    
    caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        // "width", G_TYPE_INT, 640,
        // "height", G_TYPE_INT, 480,
        // "framerate", GST_TYPE_FRACTION, 30, 1,
        nullptr);

    g_object_set(capsfilter_,
            "caps", caps,
            nullptr);

    gst_caps_unref(caps);

    gst_bin_add_many(GST_BIN(pipeline_.get()), src_, parser_,
            decoder_, converter_, downloader_,
            capsfilter_, sink_, nullptr);

    if (gst_element_link_many(src_, parser_, decoder_, converter_,
                downloader_, capsfilter_, sink_, nullptr) != TRUE)
    {
        std::cerr << "Failed to link gstreamer elements" << std::endl;
        std::exit(-1);
    }

    g_object_set(sink_,
        "emit-signals", TRUE,
        "sync", FALSE,
        "max-buffers", 1,
        "drop", TRUE,
        nullptr);
    g_signal_connect(sink_, "new-sample",
            G_CALLBACK(&H264Receiver::image_decoded_callback), this);

    gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);

    RCLCPP_INFO(get_logger(), "Subscribing to topic '%s/%s'", get_namespace(), image_topic.c_str()); 
    image_subscriber_ = create_subscription<CompressedImage>(image_topic, rclcpp::QoS(1).best_effort(),
            std::bind(&H264Receiver::image_received_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Publishing to topic '%s/%s'", get_namespace(), topic_out.c_str()); 
    image_publisher_ = create_publisher<Image>(topic_out, rclcpp::QoS(1).best_effort());

    if (!cam_info_path.empty())
    {
        std::string cam_name = get_namespace();
        std::replace(cam_name.begin(), cam_name.end(), '/', '_');
        cim = std::make_unique<CameraInfoManager>(this, cam_name, "file://" + cam_info_path);
        camera_info_ = cim->getCameraInfo();
        if (cim->isCalibrated())
        {
            RCLCPP_INFO(get_logger(), "Camera is calibrated");
            camera_info_publisher_ = create_publisher<CameraInfo>("camera_info", 10);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Camera is NOT calibrated");
            RCLCPP_WARN(get_logger(), "Camera info path was: %s", cam_info_path.c_str());
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "'cam_info_path' is not set");
    }
}

void H264Receiver::image_received_callback(CompressedImage::UniquePtr msg)
{
    // RCLCPP_INFO(get_logger(), "Img received callback");
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    std::copy(msg->data.begin(), msg->data.end(), map.data);
    map.size = msg->data.size();
    gst_buffer_unmap(buffer, &map);

    // Transfers the ownership of buffer, no need for cleanup
    if (gst_app_src_push_buffer(GST_APP_SRC(src_), buffer) != GST_FLOW_OK)
    {
        RCLCPP_ERROR(get_logger(), "Failed to push buffer");
        return;
    }
}

void H264Receiver::image_decoded_callback(GstElement *sink, H264Receiver *data)
{
    // RCLCPP_INFO(data->get_logger(), "Entering decoded callback");
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
    // const char *format = gst_structure_get_string(s, "format");
    // std::cout << "Format: " << format << ", Size: ("
    //     << width << ", " << height << ')' << std::endl;

    Image msg;
    // msg.header.stamp = rclcpp::Time(buffer->dts);
    msg.header.stamp = data->get_clock()->now();
    msg.width = width;
    msg.height = height;
    msg.data.assign(map.data, map.data + map.size);
    msg.encoding = "bgr8";
    msg.step = width * 3;

    data->publish_image(msg);
    if (data->cim->isCalibrated())
    {
        data->camera_info_.header.stamp = msg.header.stamp;
        data->publish_info();
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
}

void H264Receiver::publish_image(const Image &msg)
{
    image_publisher_->publish(msg);
}

void H264Receiver::publish_info()
{
    camera_info_publisher_->publish(camera_info_);
}
