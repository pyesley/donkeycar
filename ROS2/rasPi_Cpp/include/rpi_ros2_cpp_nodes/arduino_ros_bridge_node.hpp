#ifndef ARDUINO_ROS_BRIDGE_NODE_HPP_
#define ARDUINO_ROS_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include <termios.h> // For serial port communication
#include <fcntl.h>   // For file control options
#include <unistd.h>  // For read/write/close
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <iomanip> // For std::hex

// Command packet definitions
const uint8_t CMD_START_BYTE_1 = 0xA5;
const uint8_t CMD_START_BYTE_2 = 0x5A;
// Format: <hh (2 short integers)
struct CmdPacket {
    int16_t steering_pwm;
    int16_t throttle_pwm;
};
const size_t CMD_PACKET_SIZE = sizeof(CmdPacket);

// Sensor packet definitions
const uint8_t SENSOR_START_BYTE_1 = 0xB6;
const uint8_t SENSOR_START_BYTE_2 = 0x6B;
// Format: <i (1 integer)
struct SensorPacket {
    int32_t encoder_ticks;
};
const size_t SENSOR_PACKET_SIZE = sizeof(SensorPacket);

const uint8_t ARDUINO_TO_PI_HEARTBEAT_PING = 0xC1;
const uint8_t PI_TO_ARDUINO_HEARTBEAT_PONG = 0xD1;
// const std::string ARDUINO_READY_MESSAGE = "ARDUINO_READY"; // Not directly used in C++ byte stream processing

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);

class ArduinoBridgeNode : public rclcpp::Node {
public:
    ArduinoBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~ArduinoBridgeNode();

private:
    // Parameters
    std::string serial_port_name_;
    int baud_rate_;
    double throttle_scale_;
    int steering_max_pwm_left_;
    int steering_max_pwm_right_;
    double steering_max_rad_per_sec_;
    double encoder_m_per_tick_;
    double loop_rate_hz_;
    double send_rate_hz_;
    std::string odom_frame_id_;
    std::string odom_child_frame_id_;
    bool use_arduino_ready_signal_; // Note: Arduino ready signal logic might need C++ specific handling if it's more than just a log
    double arduino_heartbeat_timeout_sec_;

    // Derived steering params
    double pwm_center_;
    double pwm_range_;
    double rad_to_pwm_scale_;

    // ROS 2 interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_check_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;


    // Serial port handling
    int serial_fd_ = -1; // File descriptor for the serial port
    std::mutex serial_mutex_;
    std::vector<uint8_t> serial_buffer_;
    std::thread serial_read_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> arduino_is_connected_;
    std::atomic<bool> connection_attempt_pending_;
    rclcpp::Time last_arduino_ping_time_;


    // Command and Odometry data
    geometry_msgs::msg::Twist last_twist_msg_;
    std::mutex command_mutex_;
    double x_pos_, y_pos_, theta_pos_;
    int32_t last_enc_ticks_;
    rclcpp::Time last_enc_time_;
    bool first_encoder_msg_after_connect_;


    // Methods
    void declare_parameters();
    void load_parameters();
    void update_steering_params();
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);

    bool connect_serial();
    void close_serial();
    void handle_serial_error();
    void serial_read_loop();
    void process_byte(uint8_t byte);
    uint8_t calculate_checksum(const uint8_t* data_bytes, size_t len);
    void process_sensor_packet(const std::vector<uint8_t>& data_bytes);

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void send_command_callback();
    void check_arduino_heartbeat();
    void reset_odometry_for_new_connection();

    // Helper to configure serial port
    bool configure_serial_port(int fd, int baud);

};

#endif // ARDUINO_ROS_BRIDGE_NODE_HPP_
