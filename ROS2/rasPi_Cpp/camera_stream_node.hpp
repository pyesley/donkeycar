#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class CameraStreamNode : public rclcpp::Node {
public:
    explicit CameraStreamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~CameraStreamNode();

private:
    void declare_parameters();
    void load_parameters();
    void build_gstreamer_pipeline();
    void timer_callback();
    void release_camera();

    // Parameters
    int img_width_;
    int img_height_;
    int framerate_;
    int jpeg_quality_;
    std::string camera_frame_id_;
    std::string gst_pipeline_string_;
    bool publish_compressed_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;

    // Timer and capture
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    // FPS logging
    int frame_count_;
    rclcpp::Time last_log_time_;
    double log_interval_sec_;

    // Defaults
    static constexpr int IMG_WIDTH_DEFAULT = 640;
    static constexpr int IMG_HEIGHT_DEFAULT = 480;
    static constexpr int FRAMERATE_DEFAULT = 30;
    static constexpr int JPEG_QUALITY_DEFAULT = 70;
};
