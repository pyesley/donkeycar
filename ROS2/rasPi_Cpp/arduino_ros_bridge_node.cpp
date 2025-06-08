#include "rpi_ros2_cpp_nodes/arduino_ros_bridge_node.hpp"
#include <chrono>
#include <vector>
#include <algorithm> // For std::min, std::max
#include <cstring> // For memcpy

using namespace std::chrono_literals;

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    q.w = std::cos(yaw / 2.0);
    return q;
}

ArduinoBridgeNode::ArduinoBridgeNode(const rclcpp::NodeOptions & options)
    : Node("arduino_bridge_node", options),
      running_(true),
      arduino_is_connected_(false),
      connection_attempt_pending_(false),
      x_pos_(0.0), y_pos_(0.0), theta_pos_(0.0),
      last_enc_ticks_(0),
      first_encoder_msg_after_connect_(true) {

    declare_parameters();
    load_parameters();
    update_steering_params();

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ArduinoBridgeNode::parameters_callback, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ArduinoBridgeNode::cmd_vel_callback, this, std::placeholders::_1));

    send_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / send_rate_hz_),
        std::bind(&ArduinoBridgeNode::send_command_callback, this));

    heartbeat_check_timer_ = this->create_wall_timer(
        1s, std::bind(&ArduinoBridgeNode::check_arduino_heartbeat, this));

    last_enc_time_ = this->get_clock()->now();
    last_arduino_ping_time_ = this->get_clock()->now(); // Initialize to avoid immediate timeout

    RCLCPP_INFO(this->get_logger(), "Arduino Bridge node starting. Port: %s@%d", serial_port_name_.c_str(), baud_rate_);

    // Start the serial reading thread
    serial_read_thread_ = std::thread(&ArduinoBridgeNode::serial_read_loop, this);
}

ArduinoBridgeNode::~ArduinoBridgeNode() {
    RCLCPP_INFO(this->get_logger(), "Arduino Bridge node shutting down...");
    running_ = false;

    if (send_timer_) send_timer_->cancel();
    if (heartbeat_check_timer_) heartbeat_check_timer_->cancel();

    if (serial_read_thread_.joinable()) {
        serial_read_thread_.join();
    }
    close_serial();
    RCLCPP_INFO(this->get_logger(), "Shutdown complete.");
}

void ArduinoBridgeNode::declare_parameters() {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
    this->declare_parameter<int>("baud_rate", 57600);
    this->declare_parameter<double>("throttle_axis_scale", 255.0);
    this->declare_parameter<int>("steering_max_pwm_left", 0);
    this->declare_parameter<int>("steering_max_pwm_right", 80);
    this->declare_parameter<double>("steering_max_rad_per_sec", 1.0);
    this->declare_parameter<double>("encoder_meters_per_tick", 0.0003115);
    this->declare_parameter<double>("loop_rate_hz", 50.0); // Used for serial read loop timing
    this->declare_parameter<double>("send_rate_hz", 20.0);
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("odom_child_frame_id", "base_link");
    this->declare_parameter<bool>("use_arduino_ready_signal", false);
    this->declare_parameter<double>("arduino_heartbeat_timeout_sec", 5.0);
}

void ArduinoBridgeNode::load_parameters() {
    this->get_parameter("serial_port", serial_port_name_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("throttle_axis_scale", throttle_scale_);
    this->get_parameter("steering_max_pwm_left", steering_max_pwm_left_);
    this->get_parameter("steering_max_pwm_right", steering_max_pwm_right_);
    this->get_parameter("steering_max_rad_per_sec", steering_max_rad_per_sec_);
    this->get_parameter("encoder_meters_per_tick", encoder_m_per_tick_);
    this->get_parameter("loop_rate_hz", loop_rate_hz_);
    this->get_parameter("send_rate_hz", send_rate_hz_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("odom_child_frame_id", odom_child_frame_id_);
    this->get_parameter("use_arduino_ready_signal", use_arduino_ready_signal_);
    this->get_parameter("arduino_heartbeat_timeout_sec", arduino_heartbeat_timeout_sec_);
}

void ArduinoBridgeNode::update_steering_params() {
    pwm_center_ = (static_cast<double>(steering_max_pwm_right_) + steering_max_pwm_left_) / 2.0;
    pwm_range_ = std::abs(steering_max_pwm_right_ - pwm_center_);
    if (steering_max_rad_per_sec_ <= 1e-6) {
        rad_to_pwm_scale_ = 0.0;
    } else {
        rad_to_pwm_scale_ = pwm_range_ / steering_max_rad_per_sec_;
    }
}

rcl_interfaces::msg::SetParametersResult ArduinoBridgeNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool steering_changed = false;

    for (const auto &param : parameters) {
        if (param.get_name() == "serial_port") {
            serial_port_name_ = param.as_string();
            RCLCPP_INFO(this->get_logger(), "Serial port parameter changed to: %s. Serial port will be reset.", serial_port_name_.c_str());
            handle_serial_error();
        } else if (param.get_name() == "baud_rate") {
            int new_baud_rate = param.as_int();
            if (new_baud_rate != baud_rate_) {
                 RCLCPP_INFO(this->get_logger(), "Baud rate parameter changed to: %d. Serial port will be reset.", new_baud_rate);
                baud_rate_ = new_baud_rate;
                handle_serial_error();
            }
        } else if (param.get_name() == "throttle_axis_scale") {
            throttle_scale_ = param.as_double();
        } else if (param.get_name() == "steering_max_pwm_left") {
            steering_max_pwm_left_ = param.as_int();
            steering_changed = true;
        } else if (param.get_name() == "steering_max_pwm_right") {
            steering_max_pwm_right_ = param.as_int();
            steering_changed = true;
        } else if (param.get_name() == "steering_max_rad_per_sec") {
            steering_max_rad_per_sec_ = param.as_double();
            steering_changed = true;
        } else if (param.get_name() == "encoder_meters_per_tick") {
            if (param.as_double() <= 0) {
                RCLCPP_WARN(this->get_logger(), "Invalid encoder_meters_per_tick (%f). Keeping old value.", param.as_double());
                result.successful = false; // Or just warn and keep old
            } else {
                encoder_m_per_tick_ = param.as_double();
            }
        } else if (param.get_name() == "arduino_heartbeat_timeout_sec") {
            arduino_heartbeat_timeout_sec_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Arduino heartbeat timeout updated to: %.2fs", arduino_heartbeat_timeout_sec_);
        }
        // loop_rate_hz, send_rate_hz, odom_frame_id, odom_child_frame_id, use_arduino_ready_signal
        // are typically not changed at runtime or require more complex handling (e.g. recreating timers)
    }

    if (steering_changed) {
        update_steering_params();
        RCLCPP_INFO(this->get_logger(), "Steering params updated. PWM: L=%d, R=%d, C=%.1f",
                    steering_max_pwm_left_, steering_max_pwm_right_, pwm_center_);
    }
    return result;
}


bool ArduinoBridgeNode::configure_serial_port(int fd, int baud) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
        return false;
    }

    // Set baud rate
    speed_t speed;
    switch (baud) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported baud rate: %d", baud);
            return false;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo,
                                                    // no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls,
                                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
        return false;
    }
    return true;
}


bool ArduinoBridgeNode::connect_serial() {
    if (serial_fd_ != -1) { // Already open
        return true;
    }
    if (connection_attempt_pending_.load()) {
        return false;
    }

    connection_attempt_pending_ = true;
    RCLCPP_INFO(this->get_logger(), "Attempting to connect to Arduino on %s at %d baud...",
                serial_port_name_.c_str(), baud_rate_);

    serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to open serial port %s: %s. Will retry.",
                    serial_port_name_.c_str(), strerror(errno));
        serial_fd_ = -1; // Ensure it's marked as closed
        arduino_is_connected_ = false;
        connection_attempt_pending_ = false;
        return false;
    }

    fcntl(serial_fd_, F_SETFL, 0); // Clear O_NDELAY for blocking reads (or use FNDELAY for non-blocking)
                                   // For this threaded model, non-blocking with select/poll or small timeouts is better.
                                   // The current termios setting VMIN=0, VTIME=5 makes read non-blocking with timeout.

    if (!configure_serial_port(serial_fd_, baud_rate_)) {
        close(serial_fd_);
        serial_fd_ = -1;
        arduino_is_connected_ = false;
        connection_attempt_pending_ = false;
        return false;
    }

    // Clear any existing data in RX/TX buffers
    tcflush(serial_fd_, TCIOFLUSH);

    RCLCPP_INFO(this->get_logger(), "Successfully opened serial port %s.", serial_port_name_.c_str());

    // Arduino ready signal logic (if any beyond logging) would go here.
    // For now, we rely on the PING message.

    arduino_is_connected_ = false; // Will be set true upon receiving first PING
    last_arduino_ping_time_ = this->get_clock()->now();
    reset_odometry_for_new_connection();
    connection_attempt_pending_ = false;
    return true;
}

void ArduinoBridgeNode::close_serial() {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ != -1) {
        RCLCPP_INFO(this->get_logger(), "Closing serial port %s.", serial_port_name_.c_str());
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

void ArduinoBridgeNode::handle_serial_error() {
    RCLCPP_INFO(this->get_logger(), "Serial error or PING timeout. Closing port and marking Arduino as disconnected.");
    close_serial(); // This handles the lock
    arduino_is_connected_ = false;
    // Connection will be re-attempted by serial_read_loop
}


void ArduinoBridgeNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_twist_msg_ = *msg;
    // RCLCPP_DEBUG(this->get_logger(), "Received Twist: LX: %.4f, AZ: %.4f", msg->linear.x, msg->angular.z);
}

void ArduinoBridgeNode::send_command_callback() {
    if (!arduino_is_connected_.load()) {
        return;
    }

    geometry_msgs::msg::Twist twist_to_send;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        twist_to_send = last_twist_msg_;
    }

    double clamped_angular_z = std::max(-steering_max_rad_per_sec_, std::min(twist_to_send.angular.z, steering_max_rad_per_sec_));
    double pwm_deviation = -clamped_angular_z * rad_to_pwm_scale_;
    double target_pwm_double = pwm_center_ + pwm_deviation;

    int16_t steering_pwm = static_cast<int16_t>(
        std::max(static_cast<double>(std::min(steering_max_pwm_left_, steering_max_pwm_right_)),
                 std::min(target_pwm_double, static_cast<double>(std::max(steering_max_pwm_left_, steering_max_pwm_right_))))
    );

    int16_t throttle_pwm = static_cast<int16_t>(
        std::max(-1.0, std::min(twist_to_send.linear.x, 1.0)) * throttle_scale_
    );

    // RCLCPP_DEBUG(this->get_logger(), "Sending to Arduino: Steering PWM: %d, Throttle PWM: %d (from LX: %.4f)",
    //             steering_pwm, throttle_pwm, twist_to_send.linear.x);

    CmdPacket cmd_payload;
    cmd_payload.steering_pwm = steering_pwm;
    cmd_payload.throttle_pwm = throttle_pwm;

    std::vector<uint8_t> packet_buffer;
    packet_buffer.push_back(CMD_START_BYTE_1);
    packet_buffer.push_back(CMD_START_BYTE_2);

    uint8_t payload_bytes[CMD_PACKET_SIZE];
    memcpy(payload_bytes, &cmd_payload, CMD_PACKET_SIZE); // Assumes little-endian, like Python's '<'

    for(size_t i = 0; i < CMD_PACKET_SIZE; ++i) {
        packet_buffer.push_back(payload_bytes[i]);
    }

    uint8_t checksum = calculate_checksum(payload_bytes, CMD_PACKET_SIZE);
    packet_buffer.push_back(checksum);

    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ != -1) {
        ssize_t written = write(serial_fd_, packet_buffer.data(), packet_buffer.size());
        if (written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial write error for command: %s", strerror(errno));
            handle_serial_error();
        } else if (static_cast<size_t>(written) != packet_buffer.size()) {
            RCLCPP_WARN(this->get_logger(), "Serial write incomplete for command. Wrote %zd of %zu bytes.", written, packet_buffer.size());
            // Potentially handle_serial_error() here too, or retry logic
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Attempted to send command, but serial port not open.");
        handle_serial_error(); // Trigger reconnect
    }
}

uint8_t ArduinoBridgeNode::calculate_checksum(const uint8_t* data_bytes, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum ^= data_bytes[i];
    }
    return checksum;
}

void ArduinoBridgeNode::check_arduino_heartbeat() {
    if (serial_fd_ != -1 && arduino_is_connected_.load()) {
        double time_since_last_ping_sec = (this->get_clock()->now() - last_arduino_ping_time_).seconds();
        if (time_since_last_ping_sec > arduino_heartbeat_timeout_sec_) {
            RCLCPP_WARN(this->get_logger(),
                        "Arduino PING timeout (last PING %.2fs ago). Assuming disconnection.",
                        time_since_last_ping_sec);
            handle_serial_error();
        }
    }
}

void ArduinoBridgeNode::reset_odometry_for_new_connection() {
    RCLCPP_INFO(this->get_logger(), "Resetting odometry state for new/re-established connection.");
    x_pos_ = 0.0;
    y_pos_ = 0.0;
    theta_pos_ = 0.0;
    last_enc_ticks_ = 0; // Or should be set to first received value? Python code sets to 0 then updates on first msg.
    first_encoder_msg_after_connect_ = true;
    last_enc_time_ = this->get_clock()->now();
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
    std::vector<uint8_t> current_sensor_payload;
    current_sensor_payload.reserve(SENSOR_PACKET_SIZE);

    rclcpp::WallRate loop_rate(std::chrono::duration<double>(1.0 / loop_rate_hz_));

    while (rclcpp::ok() && running_.load()) {
        if (serial_fd_ == -1) {
            if (!connection_attempt_pending_.load()) {
                if (!connect_serial()) {
                    // Sleep for a bit before retrying connection
                    std::this_thread::sleep_for(1s);
                }
            } else {
                 std::this_thread::sleep_for(100ms); // Waiting for pending attempt
            }
            continue;
        }

        uint8_t read_buf[128];
        ssize_t num_bytes = 0;
        {
            std::lock_guard<std::mutex> lock(serial_mutex_); // Lock before accessing serial_fd_
            if (serial_fd_ != -1) { // Check again after acquiring lock
                 // Non-blocking read, termios VMIN=0, VTIME=X makes it timeout after X/10 seconds
                num_bytes = read(serial_fd_, read_buf, sizeof(read_buf));
            }
        }


        if (num_bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available, perfectly normal for non-blocking
            } else {
                RCLCPP_ERROR(this->get_logger(), "Serial read error: %s.", strerror(errno));
                handle_serial_error(); // This will close the port
                current_parser_state = ParserState::WAITING_FOR_START_1; // Reset parser
                current_sensor_payload.clear();
                serial_buffer_.clear(); // Clear any partially parsed data
            }
        } else if (num_bytes > 0) {
            serial_buffer_.insert(serial_buffer_.end(), read_buf, read_buf + num_bytes);
        }

        size_t processed_up_to = 0;
        for (size_t i = 0; i < serial_buffer_.size(); ++i) {
            uint8_t byte = serial_buffer_[i];
            processed_up_to = i;

            if (byte == ARDUINO_TO_PI_HEARTBEAT_PING) {
                if (!arduino_is_connected_.load()) {
                    RCLCPP_INFO(this->get_logger(), "Arduino PING received. Connection ESTABLISHED.");
                    reset_odometry_for_new_connection();
                }
                last_arduino_ping_time_ = this->get_clock()->now();
                arduino_is_connected_ = true;

                uint8_t pong_msg[] = {PI_TO_ARDUINO_HEARTBEAT_PONG};
                {
                    std::lock_guard<std::mutex> lock(serial_mutex_);
                    if (serial_fd_ != -1) {
                        if (write(serial_fd_, pong_msg, 1) < 0) {
                             RCLCPP_ERROR(this->get_logger(), "Serial write error for PONG: %s", strerror(errno));
                             handle_serial_error();
                             // Break from this inner loop as serial is now bad
                             current_parser_state = ParserState::WAITING_FOR_START_1;
                             current_sensor_payload.clear();
                             break;
                        }
                    }
                }
                current_parser_state = ParserState::WAITING_FOR_START_1;
                current_sensor_payload.clear();
                continue; // Move to next byte in buffer
            }

            switch (current_parser_state) {
                case ParserState::WAITING_FOR_START_1:
                    if (byte == SENSOR_START_BYTE_1) {
                        current_parser_state = ParserState::WAITING_FOR_START_2;
                    }
                    break;
                case ParserState::WAITING_FOR_START_2:
                    if (byte == SENSOR_START_BYTE_2) {
                        current_parser_state = ParserState::READING_PAYLOAD;
                        current_sensor_payload.clear();
                    } else if (byte == SENSOR_START_BYTE_1) {
                        // Stay in this state, got another start byte 1
                    } else {
                        current_parser_state = ParserState::WAITING_FOR_START_1; // Invalid sequence
                    }
                    break;
                case ParserState::READING_PAYLOAD:
                    current_sensor_payload.push_back(byte);
                    if (current_sensor_payload.size() == SENSOR_PACKET_SIZE) {
                        current_parser_state = ParserState::READING_CHECKSUM;
                    }
                    break;
                case ParserState::READING_CHECKSUM:
                    uint8_t received_checksum = byte;
                    uint8_t calculated_checksum = calculate_checksum(current_sensor_payload.data(), current_sensor_payload.size());
                    if (received_checksum == calculated_checksum) {
                        if (arduino_is_connected_.load()) {
                            process_sensor_packet(current_sensor_payload);
                        }
                    } else {
                        std::stringstream ss;
                        for(uint8_t b : current_sensor_payload) {
                            ss << std::hex << std::setw(2) << std::setfill('0') << (int)b;
                        }
                        RCLCPP_WARN(this->get_logger(),
                                    "Sensor packet checksum mismatch! Got: %02X, Calc: %02X, Data: %s. Discarding.",
                                    received_checksum, calculated_checksum, ss.str().c_str());
                    }
                    current_parser_state = ParserState::WAITING_FOR_START_1; // Reset for next packet
                    current_sensor_payload.clear();
                    break;
            }
        }
        if (processed_up_to > 0 || serial_buffer_.size() > 0) { // if processed_up_to is 0 but buffer has data, means first byte was processed.
             serial_buffer_.erase(serial_buffer_.begin(), serial_buffer_.begin() + processed_up_to + (serial_buffer_.empty() ? 0 : 1) );
        }


        loop_rate.sleep(); // Control loop frequency
    }
    RCLCPP_INFO(this->get_logger(), "Serial read loop stopped.");
}


void ArduinoBridgeNode::process_sensor_packet(const std::vector<uint8_t>& data_bytes) {
    if (data_bytes.size() != SENSOR_PACKET_SIZE) {
        RCLCPP_ERROR(this->get_logger(), "Invalid sensor packet size: %zu, expected %zu", data_bytes.size(), SENSOR_PACKET_SIZE);
        return;
    }

    SensorPacket sensor_data;
    // Assuming little-endian, matching Python's '<i'
    memcpy(&sensor_data, data_bytes.data(), SENSOR_PACKET_SIZE);
    int32_t current_enc_ticks = sensor_data.encoder_ticks;

    rclcpp::Time current_time_rclpy = this->get_clock()->now();

    if (first_encoder_msg_after_connect_) {
        last_enc_ticks_ = current_enc_ticks;
        last_enc_time_ = current_time_rclpy;
        first_encoder_msg_after_connect_ = false;
        RCLCPP_INFO(this->get_logger(), "First encoder message after (re)connect. Initial ticks: %d", current_enc_ticks);
        return;
    }

    double delta_time_sec = (current_time_rclpy - last_enc_time_).seconds();

    if (delta_time_sec <= 1e-6) { // Threshold for minimal time diff
        if (current_enc_ticks != last_enc_ticks_) {
             RCLCPP_WARN(this->get_logger(), "Encoder ticks changed (%d -> %d) but delta_time is too small (%.6fs). Resetting odom 'first_encoder_msg' flag.",
                         last_enc_ticks_, current_enc_ticks, delta_time_sec);
             first_encoder_msg_after_connect_ = true; // Force re-initialization of odometry baseline
        }
        return;
    }

    int32_t delta_ticks = current_enc_ticks - last_enc_ticks_;
    double delta_distance = static_cast<double>(delta_ticks) * encoder_m_per_tick_;
    double vx = delta_distance / delta_time_sec;

    double vth;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        vth = last_twist_msg_.angular.z; // Use the last commanded angular velocity
    }

    // Odometry calculation (same as Python)
    double delta_x = vx * std::cos(theta_pos_ + (vth * delta_time_sec / 2.0)) * delta_time_sec;
    double delta_y = vx * std::sin(theta_pos_ + (vth * delta_time_sec / 2.0)) * delta_time_sec;
    double delta_theta = vth * delta_time_sec;

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    theta_pos_ += delta_theta;
    theta_pos_ = std::atan2(std::sin(theta_pos_), std::cos(theta_pos_)); // Normalize angle

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_rclpy;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = odom_child_frame_id_;

    odom_msg.pose.pose.position.x = x_pos_;
    odom_msg.pose.pose.position.y = y_pos_;
    odom_msg.pose.pose.position.z = 0.0; // Assuming 2D
    odom_msg.pose.pose.orientation = quaternion_from_yaw(theta_pos_);

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = vth;

    // Set covariance (example values, adjust as needed)
    // Pose covariance
    odom_msg.pose.covariance[0] = 0.1; // x
    odom_msg.pose.covariance[7] = 0.1; // y
    odom_msg.pose.covariance[14] = 1e6; // z (large for 2D)
    odom_msg.pose.covariance[21] = 1e6; // roll
    odom_msg.pose.covariance[28] = 1e6; // pitch
    odom_msg.pose.covariance[35] = 0.1; // yaw
    // Twist covariance
    odom_msg.twist.covariance[0] = 0.1; // vx
    odom_msg.twist.covariance[7] = 1e6; // vy
    odom_msg.twist.covariance[14] = 1e6; // vz
    odom_msg.twist.covariance[21] = 1e6; // wx
    odom_msg.twist.covariance[28] = 1e6; // wy
    odom_msg.twist.covariance[35] = 0.1; // wz

    odom_publisher_->publish(odom_msg);

    last_enc_ticks_ = current_enc_ticks;
    last_enc_time_ = current_time_rclpy;
}

