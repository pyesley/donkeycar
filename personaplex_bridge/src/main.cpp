/**
 * @file main.cpp
 * @brief PersonaPlex Bridge Node - Main entry point
 *
 * This node bridges audio between PiDog and NVIDIA PersonaPlex.
 * Run this on the desktop machine with the RTX 5090.
 *
 * Prerequisites:
 *   1. PersonaPlex server running: python -m moshi.server --ssl "$SSL_DIR"
 *   2. PiDog node running: ros2 run pidog_ros pidog_node
 *   3. Both machines on same ROS2 network (same ROS_DOMAIN_ID)
 *
 * Usage:
 *   ros2 run personaplex_bridge personaplex_bridge_node
 *
 * Topics:
 *   Subscribes: /pidog/audio/capture (from PiDog mic)
 *   Publishes:  /pidog/audio/playback (to PiDog speaker)
 *   Publishes:  /personaplex/status (connection status)
 */

#include "rclcpp/rclcpp.hpp"
#include "personaplex_bridge/personaplex_bridge_node.hpp"
#include <memory>
#include <csignal>

// Global flag for graceful shutdown
static std::atomic<bool> g_running{true};

void signal_handler(int signum) {
    (void)signum;
    g_running = false;
}

int main(int argc, char* argv[]) {
    // Set up signal handlers
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("personaplex_main"), "========================================");
    RCLCPP_INFO(rclcpp::get_logger("personaplex_main"), "PersonaPlex Bridge Starting...");
    RCLCPP_INFO(rclcpp::get_logger("personaplex_main"), "========================================");

    try {
        auto node = std::make_shared<personaplex_bridge::PersonaPlexBridgeNode>();

        RCLCPP_INFO(rclcpp::get_logger("personaplex_main"), "PersonaPlex Bridge Running");
        RCLCPP_INFO(rclcpp::get_logger("personaplex_main"), "Press Ctrl+C to stop");
        RCLCPP_INFO(rclcpp::get_logger("personaplex_main"), "========================================");

        while (rclcpp::ok() && g_running) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        node.reset();

    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("personaplex_main"), "Unhandled exception: %s", e.what());
    }

    RCLCPP_INFO(rclcpp::get_logger("personaplex_main"), "========================================");
    RCLCPP_INFO(rclcpp::get_logger("personaplex_main"), "PersonaPlex Bridge Shutting Down...");
    RCLCPP_INFO(rclcpp::get_logger("personaplex_main"), "========================================");

    rclcpp::shutdown();

    return 0;
}
