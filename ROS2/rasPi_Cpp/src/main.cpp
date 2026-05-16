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

    auto param_node = std::make_shared<rclcpp::Node>("rpi_main_params");
    param_node->declare_parameter<bool>("use_yolo", false);
    const bool use_yolo = param_node->get_parameter("use_yolo").as_bool();
    RCLCPP_INFO(rclcpp::get_logger("main"), "use_yolo = %s", use_yolo ? "true" : "false");

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
        rclcpp::NodeOptions camera_opts;
        if (use_yolo) {
            // YOLO consumes raw sensor_msgs/Image on /camera/image_raw,
            // so force the camera node to publish raw frames too.
            camera_opts.parameter_overrides({
                rclcpp::Parameter("publish_compressed_only", false)
            });
        }
        camera_node = std::make_shared<CameraStreamNode>(camera_opts);
        executor.add_node(camera_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "CameraStreamNode added to executor.");

        if (use_yolo) {
            RCLCPP_INFO(rclcpp::get_logger("main"),
                "YOLO mode ON. In another terminal run:\n"
                "  source ~/ros2_jazzy/install/setup.bash\n"
                "  source ~/ros2_ws/install/setup.bash\n"
                "  ros2 launch yolo_bringup yolov8.launch.py \\\n"
                "      model:=yolov8n.pt input_image_topic:=/camera/image_raw");
        }

        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing ImuBmi088Node...");
        imu_node = std::make_shared<ImuBmi088Node>();
        executor.add_node(imu_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "ImuBmi088Node added to executor.");

        
        RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing RplidarNode...");
        rplidar_node = std::make_shared<RplidarNode>();
        executor.add_node(rplidar_node);
        RCLCPP_INFO(rclcpp::get_logger("main"), "RplidarNode added to executor.");
        

        RCLCPP_INFO(rclcpp::get_logger("main"), "All nodes added to executor. Spinning...");
        executor.spin();

    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unhandled exception in main: %s", e.what());
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down.");
    rclcpp::shutdown();
    return 0;
}