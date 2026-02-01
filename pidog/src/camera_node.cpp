#include "pidog_ros/camera_node.hpp"
#include <rclcpp/qos.hpp>
#include <sstream>

namespace pidog_ros {

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
    : Node("pidog_camera_node", options), frame_count_(0), log_interval_sec_(5.0) {

    declare_parameters();
    load_parameters();
    build_gstreamer_pipeline();

    // QoS profile for sensor data (camera images)
    auto qos_profile_sensor_data = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .durability_volatile();

    // Publish to pidog/camera/image_raw topic (uncompressed)
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "pidog/camera/image_raw", qos_profile_sensor_data);

    // Publish compressed images for efficient WiFi transmission
    compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "pidog/camera/image_raw/compressed", qos_profile_sensor_data);

    double timer_period_sec = 1.0 / static_cast<double>(framerate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_sec),
        std::bind(&CameraNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "CameraNode: Attempting to open GStreamer pipeline: %s",
                gst_pipeline_string_.c_str());

    // Open GStreamer pipeline
    cap_.open(gst_pipeline_string_, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "CameraNode: Failed to open GStreamer pipeline.");
        throw std::runtime_error("Failed to open GStreamer pipeline in CameraNode");
    }

    RCLCPP_INFO(this->get_logger(), "CameraNode: GStreamer pipeline opened successfully.");
    RCLCPP_INFO(this->get_logger(), "CameraNode: Publishing to pidog/camera/image_raw");
    RCLCPP_INFO(this->get_logger(), "CameraNode: Publishing compressed to pidog/camera/image_raw/compressed (JPEG quality: %d)", jpeg_quality_);
    last_log_time_ = this->get_clock()->now();
}

CameraNode::~CameraNode() {
    RCLCPP_INFO(this->get_logger(), "CameraNode shutting down...");
    if (timer_) {
        timer_->cancel();
    }
    release_camera();
    RCLCPP_INFO(this->get_logger(), "CameraNode shutdown complete.");
}

void CameraNode::declare_parameters() {
    this->declare_parameter<int>("camera.width", IMG_WIDTH_DEFAULT);
    this->declare_parameter<int>("camera.height", IMG_HEIGHT_DEFAULT);
    this->declare_parameter<int>("camera.framerate", FRAMERATE_DEFAULT);
    this->declare_parameter<std::string>("camera.frame_id", "pidog_camera_frame");
    this->declare_parameter<std::string>("camera.gst_pipeline_override", "");
    this->declare_parameter<int>("camera.jpeg_quality", 80);
}

void CameraNode::load_parameters() {
    this->get_parameter("camera.width", img_width_);
    this->get_parameter("camera.height", img_height_);
    this->get_parameter("camera.framerate", framerate_);
    this->get_parameter("camera.frame_id", camera_frame_id_);
    this->get_parameter("camera.gst_pipeline_override", gst_pipeline_string_);
    this->get_parameter("camera.jpeg_quality", jpeg_quality_);
}

void CameraNode::build_gstreamer_pipeline() {
    // If a full pipeline string is provided as a parameter, use it directly
    if (!gst_pipeline_string_.empty()) {
        RCLCPP_INFO(this->get_logger(), "CameraNode: Using overridden GStreamer pipeline: %s",
                    gst_pipeline_string_.c_str());
        return;
    }

    // Construct the default pipeline for Raspberry Pi 5 with libcamera
    std::stringstream ss;
    ss << "libcamerasrc ! "
       << "videoconvert ! "
       << "videoscale ! "
       << "video/x-raw,format=BGR,width=" << img_width_
       << ",height=" << img_height_ << " ! "
       << "appsink drop=true max-buffers=1";
    gst_pipeline_string_ = ss.str();
    RCLCPP_INFO(this->get_logger(), "CameraNode: Constructed GStreamer pipeline: %s",
                gst_pipeline_string_.c_str());
}

void CameraNode::timer_callback() {
    if (!cap_.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "CameraNode: Pipeline not open. Skipping frame capture.");
        return;
    }

    cv::Mat frame;
    if (!cap_.read(frame)) {
        RCLCPP_WARN(this->get_logger(), "CameraNode: Failed to capture frame.");
        return;
    }

    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "CameraNode: Captured empty frame.");
        return;
    }

    try {
        // Convert BGRA (4 channels) to BGR (3 channels) if needed
        if (frame.channels() == 4) {
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
        }

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = camera_frame_id_;

        // Publish uncompressed image (for local use)
        sensor_msgs::msg::Image::SharedPtr img_msg =
            cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_publisher_->publish(*img_msg);

        // Publish compressed image (for WiFi/laptop viewing)
        std::vector<uchar> jpeg_buffer;
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(jpeg_quality_);

        if (cv::imencode(".jpg", frame, jpeg_buffer, compression_params)) {
            auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
            compressed_msg->header = header;
            compressed_msg->format = "jpeg";
            compressed_msg->data = jpeg_buffer;
            compressed_publisher_->publish(*compressed_msg);
        }

        frame_count_++;
        rclcpp::Time current_time = this->get_clock()->now();
        double elapsed_time_for_log = (current_time - last_log_time_).seconds();

        if (elapsed_time_for_log >= log_interval_sec_) {
            double fps = static_cast<double>(frame_count_) / elapsed_time_for_log;
            RCLCPP_INFO(this->get_logger(), "CameraNode: Publishing frames (~%.1f FPS)", fps);
            frame_count_ = 0;
            last_log_time_ = current_time;
        }

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CameraNode: cv_bridge exception: %s", e.what());
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CameraNode: OpenCV exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CameraNode: Exception: %s", e.what());
    }
}

void CameraNode::release_camera() {
    RCLCPP_INFO(this->get_logger(), "CameraNode: Releasing camera capture...");
    if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "CameraNode: cv::VideoCapture released.");
    }
}

}  // namespace pidog_ros
