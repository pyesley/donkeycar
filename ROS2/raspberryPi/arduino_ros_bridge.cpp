#include "arduino_ros_bridge.hpp"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For toMsg and fromMsg
#include <iostream>
#include <algorithm> // For std::clamp
#include <vector>

// Helper function (as in Python)
geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw) {
    tf2::Quaternion q_tf;
    q_tf.setRPY(0, 0, yaw);
    return tf2::toMsg(q_tf);
}

ArduinoBridgeNode::ArduinoBridgeNode() : Node("arduino_bridge"), 
                                         loop_rate_limiter_(1.0) // Placeholder, will be updated
{
    declare_parameters();
    get_parameters();
    update_steering_params();

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ArduinoBridgeNode::parameters_callback, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ArduinoBridgeNode::cmd_vel_callback, this, std::placeholders::_1));

    last_twist_msg_ = geometry_msgs::msg::Twist(); // Initialize to zeros

    x_pos_ = 0.0;
    y_pos_ = 0.0;
    theta_pos_ = 0.0;
    last_enc_ticks_ = 0;
    last_enc_time_ = this->get_clock()->now();
    first_encoder_msg_after_connect_ = true;

    arduino_is_connected_ = false;
    connection_attempt_pending_ = false;
    running_ = true;

    loop_rate_limiter_ = rclcpp::Rate(loop_rate_hz_);

    RCLCPP_INFO(this->get_logger(), "Arduino Bridge node starting. Port: %s@%d", serial_port_name_.c_str(), baud_rate_);
    
    // Timers must be created after node is fully initialized
    send_command_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / send_rate_hz_),
        std::bind(&ArduinoBridgeNode::send_command_callback, this));
    
    heartbeat_check_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ArduinoBridgeNode::check_arduino_heartbeat, this));

    serial_read_thread_ = std::thread(&ArduinoBridgeNode::serial_read_loop, this);
}

ArduinoBridgeNode::~ArduinoBridgeNode() {
    shutdown();
}

void ArduinoBridgeNode::shutdown() {
    RCLCPP_INFO(this->get_logger(), "Arduino Bridge node shutting down...");
    running_ = false;

    if (send_command_timer_) send_command_timer_->cancel();
    if (heartbeat_check_timer_) heartbeat_check_timer_->cancel();

    if (serial_read_thread_.joinable()) {
        serial_read_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Serial read thread joined.");
    }

    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_port_ && serial_port_->is_open()) {
        try {
            serial_port_->close();
            RCLCPP_INFO(this->get_logger(), "Closed serial port %s", serial_port_name_.c_str());
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error closing serial port during shutdown: %s", e.what());
        }
    }
    serial_port_.reset(); 
    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
}

void ArduinoBridgeNode::declare_parameters() {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    this->declare_parameter<int>("baud_rate", 57600);
    this->declare_parameter<double>("throttle_axis_scale", 255.0);
    this->declare_parameter<int>("steering_max_pwm_left", 0);
    this.declare_parameter<int>("steering_max_pwm_right", 80);
    this->declare_parameter<double>("steering_max_rad_per_sec", 1.0);
    this->declare_parameter<double>("encoder_meters_per_tick", 0.0003115);
    this->declare_parameter<double>("loop_rate_hz", 50.0);
    this->declare_parameter<double>("send_rate_hz", 20.0);
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("odom_child_frame_id", "base_link");
    this->declare_parameter<bool>("use_arduino_ready_signal", false);
    this->declare_parameter<double>("arduino_heartbeat_timeout_sec", 5.0);
}

void ArduinoBridgeNode::get_parameters() {
    this->get_parameter("serial_port", serial_port_name_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("throttle_axis_scale", throttle_axis_scale_);
    this->get_parameter("steering_max_pwm_left", steering_max_pwm_left_);
    this->get_parameter("steering_max_pwm_right", steering_max_pwm_right_);
    this->get_parameter("steering_max_rad_per_sec", steering_max_rad_per_sec_);
    this->get_parameter("encoder_meters_per_tick", encoder_meters_per_tick_);
    this->get_parameter("loop_rate_hz", loop_rate_hz_);
    this->get_parameter("send_rate_hz", send_rate_hz_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("odom_child_frame_id", odom_child_frame_id_);
    this->get_parameter("use_arduino_ready_signal", use_arduino_ready_signal_);
    this->get_parameter("arduino_heartbeat_timeout_sec", arduino_heartbeat_timeout_sec_);
}

rcl_interfaces::msg::SetParametersResult ArduinoBridgeNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool steering_changed = false;

    for (const auto &param : parameters) {
        if (param.get_name() == "steering_max_pwm_left" || 
            param.get_name() == "steering_max_pwm_right" ||
            param.get_name() == "steering_max_rad_per_sec") {
            steering_changed = true;
        } else if (param.get_name() == "encoder_meters_per_tick") {
            if (param.as_double() <= 0) {
                RCLCPP_WARN(this->get_logger(), "Invalid encoder_meters_per_tick (%f). Keeping old value.", param.as_double());
                result.successful = false; // Or just keep old value without marking as unsuccessful
            } else {
                encoder_meters_per_tick_ = param.as_double();
            }
        } else if (param.get_name() == "arduino_heartbeat_timeout_sec") {
            arduino_heartbeat_timeout_sec_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Arduino heartbeat timeout updated to: %.2fs", arduino_heartbeat_timeout_sec_);
        } else if (param.get_name() == "baud_rate") {
            int new_baud_rate = param.as_int();
            if (new_baud_rate != baud_rate_) {
                RCLCPP_INFO(this->get_logger(), "Baud rate parameter changed to: %d. Serial port will be reset.", new_baud_rate);
                baud_rate_ = new_baud_rate;
                handle_serial_error(); // This will close the port and trigger a reconnect.
            } else {
                 RCLCPP_INFO(this->get_logger(), "Baud rate parameter set to current value: %d. No change.", new_baud_rate);
            }
        }
    }

    if (steering_changed) {
        get_parameters(); // Re-fetch all steering related params
        update_steering_params();
        RCLCPP_INFO(this->get_logger(), "Steering params updated. PWM: L=%d, R=%d, C=%.1f", 
                    steering_max_pwm_left_, steering_max_pwm_right_, pwm_center_);
    }
    return result;
}

void ArduinoBridgeNode::update_steering_params() {
    // Ensure latest params are used
    this->get_parameter("steering_max_pwm_left", steering_max_pwm_left_);
    this->get_parameter("steering_max_pwm_right", steering_max_pwm_right_);
    this->get_parameter("steering_max_rad_per_sec", steering_max_rad_per_sec_);

    pwm_center_ = (steering_max_pwm_right_ + steering_max_pwm_left_) / 2.0;
    pwm_range_ = std::abs(steering_max_pwm_right_ - pwm_center_);
    if (std::abs(steering_max_rad_per_sec_) <= 1e-6) {
        rad_to_pwm_scale_ = 0.0;
    } else {
        rad_to_pwm_scale_ = pwm_range_ / steering_max_rad_per_sec_;
    }
}

void ArduinoBridgeNode::reset_odometry_for_new_connection() {
    RCLCPP_INFO(this->get_logger(), "Resetting odometry state for new/re-established connection.");
    x_pos_ = 0.0;
    y_pos_ = 0.0;
    theta_pos_ = 0.0;
    last_enc_ticks_ = 0;
    first_encoder_msg_after_connect_ = true;
    last_enc_time_ = this->get_clock()->now();
}

bool ArduinoBridgeNode::connect_serial() {
    if (serial_port_ && serial_port_->is_open()) {
        return true;
    }
    if (connection_attempt_pending_) {
        return false;
    }

    connection_attempt_pending_ = true;
    RCLCPP_INFO(this->get_logger(), "Attempting to connect to Arduino on %s at %d baud...",
                serial_port_name_.c_str(), baud_rate_);
    
    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_port_ && serial_port_->is_open()) {
            serial_port_->close();
        }
        serial_port_ = std::make_shared<boost::asio::serial_port>(io_service_);
        serial_port_->open(serial_port_name_);
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        RCLCPP_INFO(this->get_logger(), "Successfully opened serial port %s.", serial_port_name_.c_str());
        // Boost.Asio doesn't have a direct 'reset_input_buffer'. Reading available data can help clear it.
        // Or, if needed, platform-specific tcflush could be used. For now, assume opening fresh is enough.

        if (use_arduino_ready_signal_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for '%s' signal from Arduino (not implemented in C++ yet).", ARDUINO_READY_MESSAGE.c_str());
            // This part needs careful implementation if strict ready signal waiting is required.
            // For now, we'll rely on heartbeat.
            std::this_thread::sleep_for(std::chrono::seconds(1)); 
        }

        arduino_is_connected_ = false; // Will be set true on first PING
        last_arduino_ping_time_ = this->get_clock()->now();
        reset_odometry_for_new_connection();
        connection_attempt_pending_ = false;
        return true;

    } catch (const boost::system::system_error& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to connect to %s: %s. Will retry.", serial_port_name_.c_str(), e.what());
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_port_) serial_port_->close(); // Ensure it's closed
        serial_port_.reset();
        arduino_is_connected_ = false;
        connection_attempt_pending_ = false;
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Unexpected error during serial connection: %s", e.what());
        std::lock_guard<std::mutex> lock(serial_mutex_);
         if (serial_port_) serial_port_->close();
        serial_port_.reset();
        arduino_is_connected_ = false;
        connection_attempt_pending_ = false;
        return false;
    }
}

void ArduinoBridgeNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_twist_msg_ = *msg;
}

void ArduinoBridgeNode::send_command_callback() {
    if (!arduino_is_connected_) {
        return;
    }

    geometry_msgs::msg::Twist twist_to_send;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        twist_to_send = last_twist_msg_;
    }

    double clamped_angular_z = std::clamp(twist_to_send.angular.z, -steering_max_rad_per_sec_, steering_max_rad_per_sec_);
    double pwm_deviation = -clamped_angular_z * rad_to_pwm_scale_;
    double target_pwm = pwm_center_ + pwm_deviation;
    
    int16_t steering_pwm = static_cast<int16_t>(std::clamp(target_pwm, 
        static_cast<double>(std::min(steering_max_pwm_left_, steering_max_pwm_right_)), 
        static_cast<double>(std::max(steering_max_pwm_left_, steering_max_pwm_right_))));

    int16_t throttle_pwm = static_cast<int16_t>(std::clamp(twist_to_send.linear.x, -1.0, 1.0) * throttle_axis_scale_);

    CommandPayload payload;
    payload.steering_pwm = steering_pwm;
    payload.throttle_pwm = throttle_pwm;

    std::vector<uint8_t> packet_bytes;
    packet_bytes.push_back(CMD_START_BYTE_1);
    packet_bytes.push_back(CMD_START_BYTE_2);
    
    const uint8_t* payload_ptr = reinterpret_cast<const uint8_t*>(&payload);
    packet_bytes.insert(packet_bytes.end(), payload_ptr, payload_ptr + CMD_PACKET_PAYLOAD_SIZE);
    
    uint8_t checksum = calculate_checksum(payload_ptr, CMD_PACKET_PAYLOAD_SIZE);
    packet_bytes.push_back(checksum);

    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_port_ && serial_port_->is_open()) {
            boost::asio::write(*serial_port_, boost::asio::buffer(packet_bytes));
        } else {
            RCLCPP_WARN(this->get_logger(), "Attempted to send command, but serial port not open.");
            handle_serial_error();
        }
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial write error for command: %s", e.what());
        handle_serial_error();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Unexpected error sending command: %s", e.what());
        handle_serial_error();
    }
}

uint8_t ArduinoBridgeNode::calculate_checksum(const uint8_t *data, size_t size) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < size; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

void ArduinoBridgeNode::handle_serial_error() {
    RCLCPP_INFO(this->get_logger(), "Serial error or PING timeout. Closing port and marking Arduino as disconnected.");
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_port_ && serial_port_->is_open()) {
        try {
            serial_port_->close();
        } catch (const boost::system::system_error& e_close) {
            RCLCPP_ERROR(this->get_logger(), "Exception while closing serial port: %s", e_close.what());
        }
    }
    serial_port_.reset(); // Release the shared_ptr
    arduino_is_connected_ = false;
}

void ArduinoBridgeNode::check_arduino_heartbeat() {
    if (serial_port_ && serial_port_->is_open()) { // Check if port object exists and is open
        if (arduino_is_connected_) {
            if ((this->get_clock()->now() - last_arduino_ping_time_).seconds() > arduino_heartbeat_timeout_sec_) {
                RCLCPP_WARN(this->get_logger(),
                    "Arduino PING timeout (last PING %.2fs ago). Assuming disconnection.",
                    (this->get_clock()->now() - last_arduino_ping_time_).seconds());
                handle_serial_error();
            }
        }
    } else if (arduino_is_connected_) { // If it thinks it's connected but port is not open/valid
        RCLCPP_WARN(this->get_logger(), "Arduino marked connected, but serial port is not valid. Handling error.");
        handle_serial_error();
    }
}

void ArduinoBridgeNode::serial_read_loop() {
    RCLCPP_INFO(this->get_logger(), "Serial read loop started.");
    
    enum class ParserState {
        WAITING_FOR_START_1,
        WAITING_FOR_START_2,
        READING_PAYLOAD,
        READING_CHECKSUM
    };
    ParserState current_parser_state = ParserState::WAITING_FOR_START_1;
    std::vector<uint8_t> current_sensor_payload_buffer;

    while (rclcpp::ok() && running_.load()) {
        if (!serial_port_ || !serial_port_->is_open()) {
            if (!connection_attempt_pending_) {
                if (!connect_serial()) {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }

        try {
            uint8_t byte_read;
            size_t bytes_transferred = 0;
            { // Scope for serial_mutex_
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if (!serial_port_ || !serial_port_->is_open()) continue; // Re-check after acquiring lock
                // Non-blocking read of one byte
                bytes_transferred = serial_port_->read_some(boost::asio::buffer(&byte_read, 1));
            }


            if (bytes_transferred > 0) {
                serial_buffer_.push_back(byte_read);
            } else { // No data read, let's process what's in buffer and then sleep
                 if (serial_buffer_.empty()) { // Only sleep if buffer is also empty
                    loop_rate_limiter_.sleep(); // Use the main loop rate for read attempts
                    continue; 
                 }
            }
            
            // Process the internal buffer
            size_t processed_count = 0;
            for (size_t k = 0; k < serial_buffer_.size(); ++k) {
                uint8_t current_byte = serial_buffer_[k];
                processed_count++;

                if (current_byte == ARDUINO_TO_PI_HEARTBEAT_PING) {
                    if (!arduino_is_connected_) {
                        RCLCPP_INFO(this->get_logger(), "Arduino PING received. Connection ESTABLISHED.");
                        reset_odometry_for_new_connection();
                    }
                    last_arduino_ping_time_ = this->get_clock()->now();
                    arduino_is_connected_ = true;
                    try {
                        uint8_t pong_byte = PI_TO_ARDUINO_HEARTBEAT_PONG;
                        std::lock_guard<std::mutex> lock(serial_mutex_);
                        if (serial_port_ && serial_port_->is_open()) {
                            boost::asio::write(*serial_port_, boost::asio::buffer(&pong_byte, 1));
                        }
                    } catch (const boost::system::system_error& e) {
                        RCLCPP_ERROR(this->get_logger(), "Serial write error for PONG: %s", e.what());
                        handle_serial_error();
                        break; // Exit processing loop, will attempt reconnect
                    }
                    current_parser_state = ParserState::WAITING_FOR_START_1;
                    current_sensor_payload_buffer.clear();
                    continue; 
                }

                switch (current_parser_state) {
                    case ParserState::WAITING_FOR_START_1:
                        if (current_byte == SENSOR_START_BYTE_1) {
                            current_parser_state = ParserState::WAITING_FOR_START_2;
                        }
                        break;
                    case ParserState::WAITING_FOR_START_2:
                        if (current_byte == SENSOR_START_BYTE_2) {
                            current_parser_state = ParserState::READING_PAYLOAD;
                            current_sensor_payload_buffer.clear();
                        } else if (current_byte == SENSOR_START_BYTE_1) {
                            // Stay in this state, got another start_1
                        } else {
                            current_parser_state = ParserState::WAITING_FOR_START_1;
                        }
                        break;
                    case ParserState::READING_PAYLOAD:
                        current_sensor_payload_buffer.push_back(current_byte);
                        if (current_sensor_payload_buffer.size() == SENSOR_PACKET_PAYLOAD_SIZE) {
                            current_parser_state = ParserState::READING_CHECKSUM;
                        }
                        break;
                    case ParserState::READING_CHECKSUM:
                        uint8_t received_checksum = current_byte;
                        uint8_t calculated_checksum = calculate_checksum(current_sensor_payload_buffer.data(), current_sensor_payload_buffer.size());
                        if (received_checksum == calculated_checksum) {
                            if (arduino_is_connected_) {
                                process_sensor_packet(current_sensor_payload_buffer);
                            }
                        } else {
                            RCLCPP_WARN(this->get_logger(),
                                "Sensor packet checksum mismatch! Got: %02X, Calc: %02X. Discarding.",
                                received_checksum, calculated_checksum);
                        }
                        current_parser_state = ParserState::WAITING_FOR_START_1;
                        current_sensor_payload_buffer.clear();
                        break;
                }
            }
            // Remove processed bytes from the front of the deque
            serial_buffer_.erase(serial_buffer_.begin(), serial_buffer_.begin() + processed_count);


        } catch (const boost::system::system_error& e) {
            if (e.code() == boost::asio::error::operation_aborted || e.code() == boost::asio::error::eof || e.code() == boost::asio::error::bad_descriptor) {
                 RCLCPP_INFO(this->get_logger(), "Serial port closed or operation aborted in read loop: %s", e.what());
            } else {
                 RCLCPP_ERROR(this->get_logger(), "Serial read error: %s. Code: %d", e.what(), e.code().value());
            }
            handle_serial_error();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Unexpected error in serial read loop: %s", e.what());
            current_parser_state = ParserState::WAITING_FOR_START_1;
            current_sensor_payload_buffer.clear();
            serial_buffer_.clear(); // Clear buffer on unexpected error
            handle_serial_error();
        }
        // If no data was read and buffer is empty, sleep. Otherwise, loop immediately to process more.
        if (bytes_transferred == 0 && serial_buffer_.empty()) {
             loop_rate_limiter_.sleep();
        }
    }
    RCLCPP_INFO(this->get_logger(), "Serial read loop finished.");
}


void ArduinoBridgeNode::process_sensor_packet(const std::vector<uint8_t>& payload) {
    rclcpp::Time current_time_rclpy = this->get_clock()->now();
    if (payload.size() != SENSOR_PACKET_PAYLOAD_SIZE) {
        RCLCPP_WARN(this->get_logger(), "Sensor packet payload size mismatch. Expected %zu, got %zu.", SENSOR_PACKET_PAYLOAD_SIZE, payload.size());
        return;
    }

    SensorPayload sensor_data;
    std::memcpy(&sensor_data, payload.data(), SENSOR_PACKET_PAYLOAD_SIZE);
    int32_t current_enc_ticks = sensor_data.encoder_ticks;

    if (first_encoder_msg_after_connect_) {
        last_enc_ticks_ = current_enc_ticks;
        last_enc_time_ = current_time_rclpy;
        first_encoder_msg_after_connect_ = false;
        RCLCPP_INFO(this->get_logger(), "First encoder message after (re)connect. Initial ticks: %d", current_enc_ticks);
        return;
    }

    double delta_time_sec = (current_time_rclpy - last_enc_time_).seconds();

    if (delta_time_sec <= 1e-6) { // approx 1 microsecond
        if (current_enc_ticks != last_enc_ticks_) {
            RCLCPP_WARN(this->get_logger(), "Encoder ticks changed (%d -> %d) but delta_time is too small (%.6fs). Resetting odom 'first_encoder_msg' flag.",
                        last_enc_ticks_, current_enc_ticks, delta_time_sec);
            first_encoder_msg_after_connect_ = true; // Reset to re-initialize on next valid packet
        }
        return;
    }

    int32_t delta_ticks = current_enc_ticks - last_enc_ticks_;
    double delta_distance = static_cast<double>(delta_ticks) * encoder_meters_per_tick_;
    double vx = delta_distance / delta_time_sec;

    double vth;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        vth = last_twist_msg_.angular.z;
    }

    double delta_x = vx * std::cos(theta_pos_ + (vth * delta_time_sec / 2.0)) * delta_time_sec;
    double delta_y = vx * std::sin(theta_pos_ + (vth * delta_time_sec / 2.0)) * delta_time_sec;
    double delta_theta = vth * delta_time_sec;

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    theta_pos_ += delta_theta;
    theta_pos_ = std::atan2(std::sin(theta_pos_), std::cos(theta_pos_));

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_rclpy;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = odom_child_frame_id_;
    odom_msg.pose.pose.position.x = x_pos_;
    odom_msg.pose.pose.position.y = y_pos_;
    odom_msg.pose.pose.orientation = quaternion_from_yaw(theta_pos_);
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.angular.z = vth; // Use commanded vth for twist component
    odom_publisher_->publish(odom_msg);

    last_enc_ticks_ = current_enc_ticks;
    last_enc_time_ = current_time_rclpy;
}
// Main function removed as this will be compiled as a library
