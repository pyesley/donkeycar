#include "rclcpp/rclcpp.hpp"
#include "pidog_camera/camera_stream_node.hpp"
#include <memory>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<CameraStreamNode> camera_node = nullptr;

    try {
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "Initializing PiDog CameraStreamNode...");
        camera_node = std::make_shared<CameraStreamNode>();
        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "PiDog CameraStreamNode initialized successfully.");

        RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "PiDog camera node spinning...");
        rclcpp::spin(camera_node);

    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("pidog_main"), "Unhandled exception in PiDog main: %s", e.what());
    }

    RCLCPP_INFO(rclcpp::get_logger("pidog_main"), "PiDog shutting down.");
    rclcpp::shutdown();
    return 0;
}
