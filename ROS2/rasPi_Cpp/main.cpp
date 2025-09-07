#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rpi_ros2_cpp_nodes/arduino_ros_bridge_node.hpp"
#include "rpi_ros2_cpp_nodes/camera_stream_node.hpp"
#include "rpi_ros2_cpp_nodes/imu_bmi088_node.hpp"
#include "rpi_ros2_cpp_nodes/rplidar_node.hpp" // <-- NEW INCLUDE
#include <memory>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    std::shared_ptr<ArduinoBridgeNode> arduino_node = nullptr;
    std::shared_ptr<CameraStreamNode> camera_node = nullptr;
    std::shared_ptr<ImuBmi088Node> imu_node = nullptr;
    std::shared_ptr<RplidarNode> rplidar_node = nullptr; // <-- NEW NODE

    try {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing ArduinoBridgeNode...");
        arduino_node = std::make_shared<ArduinoBridgeNode>();
        executor.add_node(arduino_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "ArduinoBridgeNode added to executor.");

        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing CameraStreamNode...");
        camera_node = std::make_shared<CameraStreamNode>();
        executor.add_node(camera_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "CameraStreamNode added to executor.");

        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing ImuBmi088Node...");
        imu_node = std::make_shared<ImuBmi088Node>();
        executor.add_node(imu_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "ImuBmi088Node added to executor.");

        // --- NEW RPLIDAR NODE INITIALIZATION ---
        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing RplidarNode...");
        rplidar_node = std::make_shared<RplidarNode>();
        executor.add_node(rplidar_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "RplidarNode added to executor.");
        // --- END OF NEW ADDITIONS ---

        RCLCPP_INFO(rclcpp::get_logger("main"), "All nodes added to executor. Spinning...");
        executor.spin();

    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unhandled exception in main: %s", e.what());
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down.");
    rclcpp::shutdown();
    return 0;
}