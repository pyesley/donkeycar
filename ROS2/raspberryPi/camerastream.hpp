#ifndef GST_CAMERA_PUBLISHER_NODE_HPP_
#define GST_CAMERA_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <string>
#include <memory> // For std::shared_ptr
#include <atomic> // For std::atomic_long, std::atomic_bool

class GstCameraPublisherNode : public rclcpp::Node {
public:
    GstCameraPublisherNode();
    ~GstCameraPublisherNode();
    void shutdown();

private:
    // Parameters
    int image_width_;
    int image_height_;
    int framerate_;

    // ROS Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr fps_log_timer_;

    // GStreamer
    GstElement *pipeline_;
    GstElement *appsink_;
    GMainLoop *main_loop_; // GStreamer main loop
    std::thread gst_main_loop_thread_; // Thread for GStreamer main loop

    // Frame counting for FPS
    std::atomic_long frame_count_;
    rclcpp::Time last_fps_log_time_;
    std::atomic_bool running_;


    // Methods
    void declare_parameters();
    void get_parameters();
    bool initialize_gstreamer();
    void publish_frame(GstSample *sample);
    void log_fps();
    void gst_bus_callback_wrapper(GstBus *bus, GstMessage *message);


    // Static callback for GStreamer appsink new-sample signal
    static GstFlowReturn on_new_sample_from_sink(GstElement *sink, gpointer data);
    // Static callback for GStreamer bus messages
    static gboolean on_gst_bus_message(GstBus *bus, GstMessage *message, gpointer data);
};

#endif // GST_CAMERA_PUBLISHER_NODE_HPP_
