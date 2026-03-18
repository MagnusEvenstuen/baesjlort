#include "h264_streamer/h264_streamer.hpp"

H264Streamer::H264Streamer()
    : rclcpp::Node("h264_streamer_node")
{
    camera_ = gst_element_factory_make("videotestsrc", "camera");
    capsfilter_ = gst_element_factory_make("capsfilter", "capsfilter");
    encoder_ = gst_element_factory_make("x264enc", "encoder");
    sink_ = gst_element_factory_make("appsink", "sink");

    pipeline_ = UniquePipeline(gst_pipeline_new("h264_stream"));

    gst_bin_add_many(GST_BIN(pipeline_.get()), camera_,
            capsfilter_, encoder_, sink_, nullptr);

    if (gst_element_link_many(camera_, capsfilter_, encoder_, sink_, nullptr) != TRUE)
    {
        std::cerr << "Failed to link gstreamer elements" << std::endl;
        std::exit(-1);
    }

    gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);

    image_publisher_ = create_publisher<CompressedImage>("/image_compressed", 1);
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
