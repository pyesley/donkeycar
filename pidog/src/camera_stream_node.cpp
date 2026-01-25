#include "pidog_camera/camera_stream_node.hpp"
#include <rclcpp/qos.hpp>
#include <sstream>

CameraStreamNode::CameraStreamNode(const rclcpp::NodeOptions & options)
 : Node("pidog_camera_stream_node", options), frame_count_(0), log_interval_sec_(5.0) {

    declare_parameters();
    load_parameters();
    build_gstreamer_pipeline();

    // QoS profile for sensor data (camera images)
    // Using RELIABLE for guaranteed delivery over WiFi
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
        std::bind(&CameraStreamNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "PiDog Camera: Attempting to open GStreamer pipeline: %s",
                gst_pipeline_string_.c_str());

    // Open GStreamer pipeline
    cap_.open(gst_pipeline_string_, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "!!! Failed to open GStreamer pipeline.");
        throw std::runtime_error("Failed to open GStreamer pipeline in PiDog CameraStreamNode");
    }

    RCLCPP_INFO(this->get_logger(), "PiDog Camera: GStreamer pipeline opened successfully.");
    RCLCPP_INFO(this->get_logger(), "Publishing uncompressed to: pidog/camera/image_raw");
    RCLCPP_INFO(this->get_logger(), "Publishing compressed to: pidog/camera/image_raw/compressed (JPEG quality: %d)", jpeg_quality_);
    last_log_time_ = this->get_clock()->now();
}

CameraStreamNode::~CameraStreamNode() {
    RCLCPP_INFO(this->get_logger(), "PiDog CameraStreamNode shutting down...");
    if (timer_) {
        timer_->cancel();
    }
    release_camera();
    RCLCPP_INFO(this->get_logger(), "PiDog CameraStreamNode shutdown complete.");
}

void CameraStreamNode::declare_parameters() {
    this->declare_parameter<int>("img_width", IMG_WIDTH_DEFAULT);
    this->declare_parameter<int>("img_height", IMG_HEIGHT_DEFAULT);
    this->declare_parameter<int>("framerate", FRAMERATE_DEFAULT);
    this->declare_parameter<std::string>("camera_frame_id", "pidog_camera_frame");
    this->declare_parameter<std::string>("gst_pipeline_override", "");
    this->declare_parameter<int>("jpeg_quality", 80);  // 0-100, higher = better quality
}

void CameraStreamNode::load_parameters() {
    this->get_parameter("img_width", img_width_);
    this->get_parameter("img_height", img_height_);
    this->get_parameter("framerate", framerate_);
    this->get_parameter("camera_frame_id", camera_frame_id_);
    this->get_parameter("gst_pipeline_override", gst_pipeline_string_);
    this->get_parameter("jpeg_quality", jpeg_quality_);
}

void CameraStreamNode::build_gstreamer_pipeline() {
    // If a full pipeline string is provided as a parameter, use it directly
    if (!gst_pipeline_string_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Using overridden GStreamer pipeline: %s",
                    gst_pipeline_string_.c_str());
        return;
    }

    // Construct the default pipeline for Raspberry Pi 5 with libcamera
    // Let libcamerasrc choose its format, then convert and resize
    std::stringstream ss;
    ss << "libcamerasrc ! "
       << "videoconvert ! "
       << "videoscale ! "
       << "video/x-raw,format=BGR,width=" << img_width_
       << ",height=" << img_height_ << " ! "
       << "appsink drop=true max-buffers=1";
    gst_pipeline_string_ = ss.str();
    RCLCPP_INFO(this->get_logger(), "Constructed GStreamer pipeline: %s",
                gst_pipeline_string_.c_str());
}

void CameraStreamNode::timer_callback() {
    if (!cap_.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "GStreamer pipeline is not open. Skipping frame capture.");
        return;
    }

    cv::Mat frame;
    if (!cap_.read(frame)) {
        RCLCPP_WARN(this->get_logger(), "Failed to capture frame from GStreamer pipeline.");
        return;
    }

    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Captured empty frame.");
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
            RCLCPP_INFO(this->get_logger(), "PiDog Camera: Publishing frames (~%.1f FPS)", fps);
            frame_count_ = 0;
            last_log_time_ = current_time;
        }

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception during frame processing.");
    }
}

void CameraStreamNode::release_camera() {
    RCLCPP_INFO(this->get_logger(), "Releasing camera capture in PiDog CameraStreamNode...");
    if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "cv::VideoCapture released.");
    }
}
