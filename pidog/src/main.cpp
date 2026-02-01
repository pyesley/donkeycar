/**
 * @file main.cpp
 * @brief PiDog ROS2 Node - Main entry point
 *
 * This is the unified PiDog ROS2 node that runs multiple sub-nodes:
 * - CameraNode: Camera streaming (GStreamer + OpenCV)
 * - AudioNode: Bidirectional audio streaming (ALSA)
 * - IMUNode: SH3001 6-axis IMU (accelerometer + gyroscope)
 * - ServoNode: Servo control (future)
 *
 * All nodes run in a MultiThreadedExecutor for parallel processing.
 *
 * Usage:
 *   ros2 run pidog_ros pidog_node
 *
 * Topics Published:
 *   - pidog/camera/image_raw (sensor_msgs/Image)
 *   - pidog/camera/image_raw/compressed (sensor_msgs/CompressedImage)
 *   - pidog/audio/capture (std_msgs/UInt8MultiArray)
 *   - pidog/audio/status (std_msgs/String)
 *   - pidog/imu (sensor_msgs/Imu)
 *   - pidog/imu/status (std_msgs/String)
 *
 * Topics Subscribed:
 *   - pidog/audio/playback (std_msgs/UInt8MultiArray)
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "pidog_ros/camera_node.hpp"
#include "pidog_ros/audio_node.hpp"
#include "pidog_ros/imu_node.hpp"
// #include "pidog_ros/servo_node.hpp"  // Future
#include <memory>
#include <csignal>

// Global flag for graceful shutdown
static std::atomic<bool> g_running{true};

void signal_handler(int signum) {
    (void)signum;
    g_running = false;
}

int main(int argc, char * argv[]) {
    // Set up signal handlers for graceful shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);

    // Create multi-threaded executor for parallel node processing
    rclcpp::executors::MultiThreadedExecutor executor;

    // Node pointers
    std::shared_ptr<pidog_ros::CameraNode> camera_node = nullptr;
    std::shared_ptr<pidog_ros::AudioNode> audio_node = nullptr;
    std::shared_ptr<pidog_ros::IMUNode> imu_node = nullptr;

    try {
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "========================================");
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "PiDog ROS2 Node Starting...");
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "========================================");

        // Initialize Camera Node
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "Initializing CameraNode...");
        try {
            camera_node = std::make_shared<pidog_ros::CameraNode>();
            executor.add_node(camera_node);
            RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "CameraNode added to executor.");
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("pidog_main"),
                        "CameraNode initialization failed: %s (continuing without camera)", e.what());
        }

        // Initialize Audio Node
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "Initializing AudioNode...");
        try {
            audio_node = std::make_shared<pidog_ros::AudioNode>();
            executor.add_node(audio_node);
            RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "AudioNode added to executor.");
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("pidog_main"),
                        "AudioNode initialization failed: %s (continuing without audio)", e.what());
        }

        // Initialize IMU Node
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "Initializing IMUNode...");
        try {
            imu_node = std::make_shared<pidog_ros::IMUNode>();
            executor.add_node(imu_node);
            RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "IMUNode added to executor.");
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("pidog_main"),
                        "IMUNode initialization failed: %s (continuing without IMU)", e.what());
        }

        // Future: Servo Node
        // RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "Initializing ServoNode...");
        // servo_node = std::make_shared<pidog_ros::ServoNode>();
        // executor.add_node(servo_node);

        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "========================================");
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "PiDog ROS2 Node Running");
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "Press Ctrl+C to stop");
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "========================================");

        // Spin the executor
        while (rclcpp::ok() && g_running) {
            executor.spin_some(std::chrono::milliseconds(100));
        }

    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("pidog_main"), "Unhandled exception: %s", e.what());
    }

    RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "========================================");
    RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "PiDog ROS2 Node Shutting Down...");
    RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "========================================");

    // Clean shutdown
    camera_node.reset();
    audio_node.reset();
    imu_node.reset();

    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "PiDog shutdown complete.");
    return 0;
}
