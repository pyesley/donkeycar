#ifndef CAMERA_STREAM_NODE_HPP_
#define CAMERA_STREAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>

// Configuration (can be made parameters later if needed)
const int IMG_WIDTH_DEFAULT = 640;
const int IMG_HEIGHT_DEFAULT = 480;
const int FRAMERATE_DEFAULT = 25; // FPS

class CameraStreamNode : public rclcpp::Node {
public:
    CameraStreamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~CameraStreamNode();

private:
    // Parameters
    int img_width_;
    int img_height_;
    int framerate_;
    std::string camera_frame_id_;
    std::string gst_pipeline_string_;


    // ROS 2 interfaces
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // OpenCV and GStreamer
    cv::VideoCapture cap_;
    cv_bridge::CvImage cv_image_;

    // Frame rate logging
    int frame_count_;
    rclcpp::Time last_log_time_;
    double log_interval_sec_;

    // Methods
    void declare_parameters();
    void load_parameters();
    void build_gstreamer_pipeline();
    void timer_callback();
    void release_camera();
};

#endif // CAMERA_STREAM_NODE_HPP_
