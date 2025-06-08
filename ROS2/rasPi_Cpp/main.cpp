#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rpi_ros2_cpp_nodes/arduino_ros_bridge_node.hpp"
#include "rpi_ros2_cpp_nodes/camera_stream_node.hpp"
#include "rpi_ros2_cpp_nodes/imu_6050_node.hpp"
#include <memory> // For std::make_shared

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    // Use MultiThreadedExecutor to allow multiple nodes/callbacks to run concurrently
    rclcpp::executors::MultiThreadedExecutor executor;

    std::shared_ptr<ArduinoBridgeNode> arduino_node = nullptr;
    std::shared_ptr<CameraStreamNode> camera_node = nullptr;
    std::shared_ptr<Imu6050Node> imu_node = nullptr;

    try {

        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing ArduinoBridgeNode...");
        arduino_node = std::make_shared<ArduinoBridgeNode>();
        executor.add_node(arduino_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "ArduinoBridgeNode added to executor.");

        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing CameraStreamNode...");
        camera_node = std::make_shared<CameraStreamNode>();
        executor.add_node(camera_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "CameraStreamNode added to executor.");

        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing Imu6050Node...");
        imu_node = std::make_shared<Imu6050Node>();
        executor.add_node(imu_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Imu6050Node added to executor.");

        RCLCPP_INFO(rclcpp::get_logger("main"), "Nodes added to executor. Spinning...");
        executor.spin();

    } catch (const rclcpp::exceptions::RCLError & e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "RCLCPP error: %s", e.what());
    } catch (const std::runtime_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Runtime error during node initialization or execution: %s", e.what());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unhandled C++ exception: %s", e.what());
    } catch (...) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unknown exception caught in main.");
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down executor...");
    // Note: MultiThreadedExecutor doesn't have a shutdown() method
    // The executor will stop when spin() returns or when rclcpp::shutdown() is called

    // Node destruction will be handled by shared_ptr going out of scope
    // or can be explicitly reset.
    // arduino_node.reset();
    // camera_node.reset();
    // imu_node.reset();
    // RCLCPP_INFO(rclcpp::get_logger("main"), "Nodes destroyed (or will be by shared_ptr).");

    if (rclcpp::ok()) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down RCLCPP.");
        rclcpp::shutdown();
    } else {
        RCLCPP_WARN(rclcpp::get_logger("main"), "RCLCPP was already shut down or not OK.");
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Main program finished.");
    return 0;
}