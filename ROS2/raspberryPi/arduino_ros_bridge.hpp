#ifndef ARDUINO_ROS_BRIDGE_HPP_
#define ARDUINO_ROS_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>
#include <deque>
#include <atomic>

// Packet definitions
const uint8_t CMD_START_BYTE_1 = 0xA5;
const uint8_t CMD_START_BYTE_2 = 0x5A;
const uint8_t SENSOR_START_BYTE_1 = 0xB6;
const uint8_t SENSOR_START_BYTE_2 = 0x6B;
const uint8_t ARDUINO_TO_PI_HEARTBEAT_PING = 0xC1;
const uint8_t PI_TO_ARDUINO_HEARTBEAT_PONG = 0xD1;
const std::string ARDUINO_READY_MESSAGE = "ARDUINO_READY";


#pragma pack(push, 1)
struct CommandPayload {
    int16_t steering_pwm;
    int16_t throttle_pwm;
};
struct SensorPayload {
    int32_t encoder_ticks;
};
#pragma pack(pop)

// Size constants for packets
const size_t CMD_PACKET_PAYLOAD_SIZE = sizeof(CommandPayload);
const size_t SENSOR_PACKET_PAYLOAD_SIZE = sizeof(SensorPayload);


geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);

class ArduinoBridgeNode : public rclcpp::Node {
public:
    ArduinoBridgeNode();
    ~ArduinoBridgeNode();
    void shutdown();

private:
    // Parameters
    std::string serial_port_name_;
    int baud_rate_;
    double throttle_axis_scale_;
    int steering_max_pwm_left_;
    int steering_max_pwm_right_;
    double steering_max_rad_per_sec_;
    double encoder_meters_per_tick_;
    double loop_rate_hz_;
    double send_rate_hz_;
    std::string odom_frame_id_;
    std::string odom_child_frame_id_;
    bool use_arduino_ready_signal_;
    double arduino_heartbeat_timeout_sec_;

    // Steering calculation params
    double pwm_center_;
    double pwm_range_;
    double rad_to_pwm_scale_;

    // Publishers and Subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    // Timers
    rclcpp::TimerBase::SharedPtr send_command_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_check_timer_;

    // Serial communication
    boost::asio::io_service io_service_;
    std::shared_ptr<boost::asio::serial_port> serial_port_;
    std::mutex serial_mutex_;
    std::deque<uint8_t> serial_buffer_; // Using deque for efficient front removal
    std::thread serial_read_thread_;
    std::atomic<bool> running_;
    bool arduino_is_connected_;
    bool connection_attempt_pending_;
    rclcpp::Rate loop_rate_limiter_;


    // Command related
    geometry_msgs::msg::Twist last_twist_msg_;
    std::mutex command_mutex_;

    // Odometry
    double x_pos_, y_pos_, theta_pos_;
    int32_t last_enc_ticks_;
    rclcpp::Time last_enc_time_;
    bool first_encoder_msg_after_connect_;

    // Heartbeat
    rclcpp::Time last_arduino_ping_time_;


    // Parameter callback
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Methods
    void declare_parameters();
    void get_parameters();
    void update_steering_params();
    void reset_odometry_for_new_connection();
    bool connect_serial();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void send_command_callback();
    uint8_t calculate_checksum(const uint8_t *data, size_t size);
    void handle_serial_error();
    void check_arduino_heartbeat();
    void serial_read_loop();
    void process_sensor_packet(const std::vector<uint8_t>& payload);
};

#endif // ARDUINO_ROS_BRIDGE_HPP_
