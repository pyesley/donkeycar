#include "rpi_ros2_cpp_nodes/camera_stream_node.hpp"
#include <rclcpp/qos.hpp>
#include <sstream>

CameraStreamNode::CameraStreamNode(const rclcpp::NodeOptions & options)
    : Node("camera_stream_node", options), frame_count_(0), log_interval_sec_(5.0) {

    declare_parameters();
    load_parameters();
    build_gstreamer_pipeline();

    // QoS profile for sensor data
    auto qos_profile_sensor_data = rclcpp::QoS(rclcpp::KeepLast(1))
                                       .best_effort()
                                       .durability_volatile();

    // Always create compressed publisher (main output)
    compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "camera/image_raw/compressed", qos_profile_sensor_data);

    // Optionally also publish raw (for local debugging)
    if (!publish_compressed_) {
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera/image_raw", qos_profile_sensor_data);
    }

    double timer_period_sec = 1.0 / static_cast<double>(framerate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_sec),
        std::bind(&CameraStreamNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Opening GStreamer pipeline: %s", gst_pipeline_string_.c_str());
    RCLCPP_INFO(this->get_logger(), "JPEG quality: %d, Resolution: %dx%d @ %d fps",
                jpeg_quality_, img_width_, img_height_, framerate_);

    cap_.open(gst_pipeline_string_, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline!");
        throw std::runtime_error("Failed to open GStreamer pipeline in CameraStreamNode");
    }

    RCLCPP_INFO(this->get_logger(), "Camera streaming started (JPEG compressed).");
    last_log_time_ = this->get_clock()->now();
}

CameraStreamNode::~CameraStreamNode() {
    RCLCPP_INFO(this->get_logger(), "CameraStreamNode shutting down...");
    if (timer_) {
        timer_->cancel();
    }
    release_camera();
}

void CameraStreamNode::declare_parameters() {
    this->declare_parameter<int>("img_width", IMG_WIDTH_DEFAULT);
    this->declare_parameter<int>("img_height", IMG_HEIGHT_DEFAULT);
    this->declare_parameter<int>("framerate", FRAMERATE_DEFAULT);
    this->declare_parameter<int>("jpeg_quality", JPEG_QUALITY_DEFAULT);
    this->declare_parameter<std::string>("camera_frame_id", "camera_frame");
    this->declare_parameter<std::string>("gst_pipeline_override", "");
    this->declare_parameter<bool>("publish_compressed_only", true);
}

void CameraStreamNode::load_parameters() {
    this->get_parameter("img_width", img_width_);
    this->get_parameter("img_height", img_height_);
    this->get_parameter("framerate", framerate_);
    this->get_parameter("jpeg_quality", jpeg_quality_);
    this->get_parameter("camera_frame_id", camera_frame_id_);
    this->get_parameter("gst_pipeline_override", gst_pipeline_string_);
    this->get_parameter("publish_compressed_only", publish_compressed_);
    
    // Clamp JPEG quality
    jpeg_quality_ = std::max(1, std::min(100, jpeg_quality_));
}

void CameraStreamNode::build_gstreamer_pipeline() {
    if (!gst_pipeline_string_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Using override pipeline: %s", gst_pipeline_string_.c_str());
        return;
    }

    // Default pipeline - outputs BGR for OpenCV
    std::stringstream ss;
    ss << "libcamerasrc ! "
       << "video/x-raw,format=BGRx,width=" << img_width_
       << ",height=" << img_height_
       << ",framerate=" << framerate_ << "/1 ! "
       << "videoconvert ! "
       << "video/x-raw,format=BGR ! "
       << "appsink drop=true max-buffers=1 emit-signals=true";
    gst_pipeline_string_ = ss.str();
}

void CameraStreamNode::timer_callback() {
    if (!cap_.isOpened()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Camera not open, skipping frame.");
        return;
    }

    cv::Mat frame;
    if (!cap_.read(frame)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Failed to capture frame.");
        return;
    }

    if (frame.empty()) {
        return;
    }

    try {
        // Handle BGRx (4 channel) -> BGR (3 channel)
        cv::Mat bgr_frame;
        if (frame.channels() == 4) {
            cv::cvtColor(frame, bgr_frame, cv::COLOR_BGRA2BGR);
        } else {
            bgr_frame = frame;
        }

        auto now = this->get_clock()->now();

        // === PUBLISH COMPRESSED JPEG ===
        {
            // Encode to JPEG
            std::vector<uchar> jpeg_buffer;
            std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
            
            if (!cv::imencode(".jpg", bgr_frame, jpeg_buffer, encode_params)) {
                RCLCPP_ERROR(this->get_logger(), "JPEG encoding failed!");
                return;
            }

            // Create compressed message
            sensor_msgs::msg::CompressedImage::SharedPtr compressed_msg =
                std::make_shared<sensor_msgs::msg::CompressedImage>();
            
            compressed_msg->header.stamp = now;
            compressed_msg->header.frame_id = camera_frame_id_;
            compressed_msg->format = "jpeg";
            compressed_msg->data = std::move(jpeg_buffer);

            compressed_publisher_->publish(*compressed_msg);
        }

        // === OPTIONALLY PUBLISH RAW (for local use) ===
        if (image_publisher_) {
            std_msgs::msg::Header header;
            header.stamp = now;
            header.frame_id = camera_frame_id_;
            
            sensor_msgs::msg::Image::SharedPtr img_msg =
                cv_bridge::CvImage(header, "bgr8", bgr_frame).toImageMsg();
            image_publisher_->publish(*img_msg);
        }

        // FPS logging
        frame_count_++;
        double elapsed = (now - last_log_time_).seconds();
        if (elapsed >= log_interval_sec_) {
            double fps = frame_count_ / elapsed;
            RCLCPP_INFO(this->get_logger(), "Publishing at ~%.1f FPS (JPEG %d%%, %zu bytes avg)",
                        fps, jpeg_quality_,
                        compressed_publisher_->get_subscription_count() > 0 ? 
                        static_cast<size_t>(img_width_ * img_height_ * 3 * jpeg_quality_ / 800) : 0);
            frame_count_ = 0;
            last_log_time_ = now;
        }

    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
}

void CameraStreamNode::release_camera() {
    if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "Camera released.");
    }
}
