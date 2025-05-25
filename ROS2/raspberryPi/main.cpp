#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "arduino_ros_bridge.hpp"
#include "camerastream.hpp" // Assuming GstCameraPublisherNode is in camerastream.hpp
#include "IMU_6050.hpp"     // Assuming MPU6050PublisherNode is in IMU_6050.hpp

#include <memory> // For std::make_shared
#include <vector> // For std::vector of nodes, if needed, though direct add_node is fine

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // Using a generic logger for main, or could get one from a temporary node.
    auto logger = rclcpp::get_logger("unified_runner_main");

    std::shared_ptr<ArduinoBridgeNode> arduino_node = nullptr;
    std::shared_ptr<GstCameraPublisherNode> camera_node = nullptr;
    std::shared_ptr<MPU6050PublisherNode> imu_node = nullptr;
    rclcpp::executors::MultiThreadedExecutor executor;

    try {
        RCLCPP_INFO(logger, "Creating ArduinoBridgeNode...");
        arduino_node = std::make_shared<ArduinoBridgeNode>();
        RCLCPP_INFO(logger, "ArduinoBridgeNode created.");

        RCLCPP_INFO(logger, "Creating GstCameraPublisherNode...");
        camera_node = std::make_shared<GstCameraPublisherNode>();
        if (camera_node->pipeline_ == nullptr) { // Check if GStreamer initialization failed
            RCLCPP_FATAL(logger, "GStreamer pipeline failed to initialize in GstCameraPublisherNode. Shutting down.");
            // arduino_node->shutdown(); // Manually call shutdown if needed before early exit.
                                     // Destructors should handle this when nodes go out of scope.
            rclcpp::shutdown();
            return -1;
        }
        RCLCPP_INFO(logger, "GstCameraPublisherNode created.");

        RCLCPP_INFO(logger, "Creating MPU6050PublisherNode...");
        imu_node = std::make_shared<MPU6050PublisherNode>();
        RCLCPP_INFO(logger, "MPU6050PublisherNode created.");

        RCLCPP_INFO(logger, "Adding nodes to MultiThreadedExecutor.");
        executor.add_node(arduino_node);
        executor.add_node(camera_node);
        executor.add_node(imu_node);

        RCLCPP_INFO(logger, "All nodes added. Spinning the executor...");
        executor.spin();

    } catch (const std::exception & e) {
        RCLCPP_FATAL(logger, "Unhandled exception in main: %s", e.what());
        // Nodes' destructors will be called as shared_ptrs go out of scope.
        // Executor might also need explicit handling if it was already spinning.
    } catch (...) {
        RCLCPP_FATAL(logger, "Unknown unhandled exception in main.");
    }

    RCLCPP_INFO(logger, "Executor has finished or an exception occurred. Shutting down ROS.");
    
    // Explicitly remove nodes from executor before shutdown to control deallocation order
    // or let destructors handle it. For simplicity, we'll rely on destructors.
    // If executor.spin() threw, nodes are still valid shared_ptr and will be cleaned up.
    // If executor.spin() returned normally (e.g. rclcpp::shutdown() called elsewhere),
    // nodes are still managed.

    // Call shutdown on nodes explicitly if their destructors aren't enough
    // or if specific shutdown order is needed before rclcpp::shutdown().
    // This is generally good practice for complex nodes with threads/hardware.
    // The current nodes have shutdown methods called by their destructors.
    // arduino_node->shutdown();
    // camera_node->shutdown();
    // imu_node->shutdown();


    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    RCLCPP_INFO(logger, "ROS 2 shutdown procedures complete for unified_runner_node.");
    return 0;
}
