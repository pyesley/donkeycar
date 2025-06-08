#include "rpi_ros2_cpp_nodes/camera_stream_node.hpp"
#include <rclcpp/qos.hpp> // For QoS profiles
#include <sstream> // For std::stringstream

CameraStreamNode::CameraStreamNode(const rclcpp::NodeOptions & options)
    : Node("camera_stream_node", options), frame_count_(0), log_interval_sec_(5.0) {

    declare_parameters();
    load_parameters();
    build_gstreamer_pipeline();

    // QoS profile for sensor data (camera images)
    auto qos_profile_sensor_data = rclcpp::QoS(rclcpp::KeepLast(1))
                                       .best_effort()
                                       .durability_volatile();

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos_profile_sensor_data);

    double timer_period_sec = 1.0 / static_cast<double>(framerate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_sec),
        std::bind(&CameraStreamNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Attempting to open GStreamer pipeline: %s", gst_pipeline_string_.c_str());

    // Open GStreamer pipeline
    // The third argument cv::CAP_GSTREAMER is crucial
    cap_.open(gst_pipeline_string_, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "!!! Failed to open GStreamer pipeline.");
        // This is a critical error, consider throwing or exiting if node cannot function
        throw std::runtime_error("Failed to open GStreamer pipeline in CameraStreamNode");
    }

    RCLCPP_INFO(this->get_logger(), "GStreamer pipeline opened successfully. Starting publishing.");
    last_log_time_ = this->get_clock()->now();
}

CameraStreamNode::~CameraStreamNode() {
    RCLCPP_INFO(this->get_logger(), "CameraStreamNode shutting down...");
    if (timer_) {
        timer_->cancel();
    }
    release_camera();
    RCLCPP_INFO(this->get_logger(), "CameraStreamNode shutdown complete.");
}

void CameraStreamNode::declare_parameters() {
    this->declare_parameter<int>("img_width", IMG_WIDTH_DEFAULT);
    this->declare_parameter<int>("img_height", IMG_HEIGHT_DEFAULT);
    this->declare_parameter<int>("framerate", FRAMERATE_DEFAULT);
    this->declare_parameter<std::string>("camera_frame_id", "camera_frame");
    // Allow overriding the full pipeline string if needed, otherwise it's constructed
    this->declare_parameter<std::string>("gst_pipeline_override", "");
}

void CameraStreamNode::load_parameters() {
    this->get_parameter("img_width", img_width_);
    this->get_parameter("img_height", img_height_);
    this->get_parameter("framerate", framerate_);
    this->get_parameter("camera_frame_id", camera_frame_id_);
    this->get_parameter("gst_pipeline_override", gst_pipeline_string_);
}

void CameraStreamNode::build_gstreamer_pipeline() {
    // If a full pipeline string is provided as a parameter, use it directly.
    if (!gst_pipeline_string_.empty()) {
        RCLCPP_INFO(this->get_logger(), "Using overridden GStreamer pipeline: %s", gst_pipeline_string_.c_str());
        return;
    }

    // Otherwise, construct the default pipeline
    std::stringstream ss;
    ss << "libcamerasrc ! "
       << "video/x-raw,format=BGRx,width=" << img_width_
       << ",height=" << img_height_
       << ",framerate=" << framerate_ << "/1 ! "
       << "videoconvert ! "
       << "appsink drop=true max-buffers=1 emit-signals=true";
    gst_pipeline_string_ = ss.str();
    RCLCPP_INFO(this->get_logger(), "Constructed GStreamer pipeline: %s", gst_pipeline_string_.c_str());
}


void CameraStreamNode::timer_callback() {
    if (!cap_.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "GStreamer pipeline is not open. Skipping frame capture.");
        // Attempt to reopen? For now, just warn.
        // cap_.open(gst_pipeline_string_, cv::CAP_GSTREAMER); // Could try this
        // if (!cap_.isOpened()) return;
        return;
    }

    cv::Mat frame;
    if (!cap_.read(frame)) { // cap.read(frame) is equivalent to cap >> frame;
        RCLCPP_WARN(this->get_logger(), "Failed to capture frame from GStreamer pipeline. cap.read() returned false.");
        return;
    }

    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Captured empty frame.");
        return;
    }

    try {
        // Convert OpenCV Mat to ROS Image message
        // Ensure encoding is correct. Python used 'bgr8'. GStreamer pipeline uses BGRx.
        // cv_bridge will handle conversion if needed, but specifying 'bgr8' is common.
        // If GStreamer outputs BGRx, OpenCV might read it as BGRA or BGR.
        // Check frame.channels() if issues arise. For BGRx, it's often 4 channels.
        // If it's BGRA, cv_bridge might need sensor_msgs::image_encodings::BGRA8.
        // Forcing BGR8 if input is 4 channels (like BGRx or BGRA)
        if (frame.channels() == 4) {
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
        }

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = camera_frame_id_;

        sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_publisher_->publish(*img_msg);

        frame_count_++;
        rclcpp::Time current_time = this->get_clock()->now();
        double elapsed_time_for_log = (current_time - last_log_time_).seconds();

        if (elapsed_time_for_log >= log_interval_sec_) {
            double fps = static_cast<double>(frame_count_) / elapsed_time_for_log;
            RCLCPP_INFO(this->get_logger(), "Publishing image... (~%.1f FPS)", fps);
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
        RCLCPP_ERROR(this->get_logger(), "Unknown exception during frame processing or publishing.");
    }
}

void CameraStreamNode::release_camera() {
    RCLCPP_INFO(this->get_logger(), "Releasing camera capture in CameraStreamNode...");
    if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "cv::VideoCapture released.");
    }
}

