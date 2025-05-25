#include "camerastream.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/imgproc/imgproc.hpp> // For cv::cvtColor
#include <sstream> // For std::stringstream

// Initialize GStreamer (must be called before any GStreamer functions)
// Doing it globally or in main before node creation
class GstInitializer {
public:
    GstInitializer() {
        gst_init(nullptr, nullptr);
    }
};
static GstInitializer gst_initializer;


GstCameraPublisherNode::GstCameraPublisherNode()
    : Node("gst_camera_publisher"),
      pipeline_(nullptr),
      appsink_(nullptr),
      main_loop_(nullptr),
      frame_count_(0),
      running_(true)
{
    declare_parameters();
    get_parameters();

    if (!initialize_gstreamer()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GStreamer pipeline. Node will not function.");
        // Consider throwing an exception or setting a flag to prevent further operations
        return;
    }

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos_profile);

    fps_log_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&GstCameraPublisherNode::log_fps, this));

    last_fps_log_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "GstCameraPublisherNode started.");
}

GstCameraPublisherNode::~GstCameraPublisherNode() {
    shutdown();
}

void GstCameraPublisherNode::shutdown() {
    RCLCPP_INFO(this->get_logger(), "Shutting down GstCameraPublisherNode...");
    running_ = false;

    if (fps_log_timer_) {
        fps_log_timer_->cancel();
    }

    if (pipeline_) {
        RCLCPP_INFO(this->get_logger(), "Setting GStreamer pipeline to NULL state...");
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }

    if (main_loop_) {
        if (g_main_loop_is_running(main_loop_)) {
            RCLCPP_INFO(this->get_logger(), "Quitting GStreamer main loop...");
            g_main_loop_quit(main_loop_);
        }
    }

    if (gst_main_loop_thread_.joinable()) {
        RCLCPP_INFO(this->get_logger(), "Joining GStreamer main loop thread...");
        gst_main_loop_thread_.join();
        RCLCPP_INFO(this->get_logger(), "GStreamer main loop thread joined.");
    }
    
    if (appsink_) {
        gst_object_unref(appsink_); // appsink_ is owned by pipeline_, unref if taken with gst_bin_get_by_name
        appsink_ = nullptr;
    }
    if (pipeline_) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
    if (main_loop_) {
        g_main_loop_unref(main_loop_);
        main_loop_ = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "GStreamer resources released.");
}

void GstCameraPublisherNode::declare_parameters() {
    this->declare_parameter<int>("image_width", 320);
    this->declare_parameter<int>("image_height", 240);
    this->declare_parameter<int>("framerate", 25);
}

void GstCameraPublisherNode::get_parameters() {
    this->get_parameter("image_width", image_width_);
    this->get_parameter("image_height", image_height_);
    this->get_parameter("framerate", framerate_);
    RCLCPP_INFO(this->get_logger(), "Parameters: Width=%d, Height=%d, FPS=%d",
                image_width_, image_height_, framerate_);
}

bool GstCameraPublisherNode::initialize_gstreamer() {
    std::stringstream ss;
    ss << "libcamerasrc ! video/x-raw,format=BGRx,width=" << image_width_
       << ",height=" << image_height_ << ",framerate=" << framerate_
       << "/1 ! videoconvert ! appsink name=mysink drop=true max-buffers=1 emit-signals=true";
    std::string gst_pipeline_str = ss.str();

    RCLCPP_INFO(this->get_logger(), "Using GStreamer pipeline: %s", gst_pipeline_str.c_str());

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(gst_pipeline_str.c_str(), &error);

    if (error) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse GStreamer pipeline: %s", error->message);
        g_error_free(error);
        return false;
    }
    if (!pipeline_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer pipeline (gst_parse_launch returned NULL).");
        return false;
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysink");
    if (!appsink_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get appsink element from pipeline.");
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }

    // Set the appsink properties
    g_object_set(G_OBJECT(appsink_),
                 "emit-signals", TRUE,
                 "max-buffers", 1, // Keep small buffer
                 "drop", TRUE,    // Drop old buffers if not consumed
                 nullptr);

    // Connect the "new-sample" signal
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(GstCameraPublisherNode::on_new_sample_from_sink), this);

    // Create and run the GStreamer main loop in a separate thread
    main_loop_ = g_main_loop_new(nullptr, FALSE);
    
    GstBus *bus = gst_element_get_bus(pipeline_);
    gst_bus_add_watch(bus, GstCameraPublisherNode::on_gst_bus_message, this);
    gst_object_unref(bus);


    // Start playing the pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(this->get_logger(), "Unable to set the pipeline to the playing state.");
        gst_object_unref(appsink_); // unref appsink before pipeline
        appsink_ = nullptr;
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        g_main_loop_unref(main_loop_);
        main_loop_ = nullptr;
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "GStreamer pipeline is PLAYING.");

    gst_main_loop_thread_ = std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "GStreamer main loop thread started.");
        g_main_loop_run(main_loop_); // This blocks until g_main_loop_quit() is called
        RCLCPP_INFO(this->get_logger(), "GStreamer main loop thread finished.");
    });
    
    return true;
}


GstFlowReturn GstCameraPublisherNode::on_new_sample_from_sink(GstElement *sink, gpointer data) {
    GstCameraPublisherNode *node = static_cast<GstCameraPublisherNode*>(data);
    if (!node || !node->running_.load()) {
        return GST_FLOW_ERROR; // Or some other appropriate error/EOS code
    }

    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) {
        RCLCPP_WARN(node->get_logger(), "gst_app_sink_pull_sample returned null");
        return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        RCLCPP_WARN(node->get_logger(), "gst_sample_get_buffer returned null");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        RCLCPP_WARN(node->get_logger(), "gst_buffer_map failed");
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Create cv::Mat from GStreamer buffer (BGRx format)
    // BGRx is 4 channels (B, G, R, Alpha/unused)
    cv::Mat frame_bgra(cv::Size(node->image_width_, node->image_height_), CV_8UC4, map.data);
    
    // Convert BGRx (BGRA) to BGR for cv_bridge if needed, or use "bgra8"
    cv::Mat frame_bgr;
    cv::cvtColor(frame_bgra, frame_bgr, cv::COLOR_BGRA2BGR);

    gst_buffer_unmap(buffer, &map);


    std_msgs::msg::Header header;
    header.stamp = node->get_clock()->now();
    header.frame_id = "camera_frame";

    sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(header, "bgr8", frame_bgr).toImageMsg();
    
    if (node->image_publisher_ && rclcpp::ok()) {
         node->image_publisher_->publish(*image_msg);
    }

    node->frame_count_++;
    gst_sample_unref(sample); // Release the sample
    return GST_FLOW_OK;
}

gboolean GstCameraPublisherNode::on_gst_bus_message(GstBus *bus, GstMessage *message, gpointer data) {
    GstCameraPublisherNode *node = static_cast<GstCameraPublisherNode*>(data);
    if (!node || !node->running_.load()) {
        return TRUE; // Remove message from bus
    }

    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar *debug_info;
            gst_message_parse_error(message, &err, &debug_info);
            RCLCPP_ERROR(node->get_logger(), "GStreamer Error: %s. Debug Info: %s", err->message, debug_info ? debug_info : "none");
            g_error_free(err);
            g_free(debug_info);
            // Optionally, trigger a shutdown or restart of the pipeline
            if (node->main_loop_ && g_main_loop_is_running(node->main_loop_)) {
                 g_main_loop_quit(node->main_loop_);
            }
            break;
        }
        case GST_MESSAGE_EOS:
            RCLCPP_INFO(node->get_logger(), "GStreamer: End-Of-Stream reached.");
            if (node->main_loop_ && g_main_loop_is_running(node->main_loop_)) {
                 g_main_loop_quit(node->main_loop_); // Stop the GStreamer main loop
            }
            break;
        case GST_MESSAGE_STATE_CHANGED: {
             if (GST_MESSAGE_SRC(message) == GST_OBJECT(node->pipeline_)) {
                GstState old_state, new_state, pending_state;
                gst_message_parse_state_changed(message, &old_state, &new_state, &pending_state);
                RCLCPP_DEBUG(node->get_logger(), "Pipeline state changed from %s to %s.",
                   gst_element_state_get_name(old_state), gst_element_state_get_name(new_state));
            }
            break;
        }
        default:
            // Unhandled message
            break;
    }
    // We want to keep the watcher active
    return TRUE;
}


void GstCameraPublisherNode::log_fps() {
    rclcpp::Time current_time = this->get_clock()->now();
    double time_diff_sec = (current_time - last_fps_log_time_).seconds();
    long current_frame_count = frame_count_.exchange(0); // Reset frame count atomically

    if (time_diff_sec > 0) {
        double fps = static_cast<double>(current_frame_count) / time_diff_sec;
        RCLCPP_INFO(this->get_logger(), "FPS: %.2f (Frames: %ld, Time: %.2fs)", fps, current_frame_count, time_diff_sec);
    }
    last_fps_log_time_ = current_time;
}
// Main function removed as this will be compiled as a library
