#include "rpi_ros2_cpp_nodes/rplidar_node.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include <algorithm>
#include <vector>
#include <cmath>

RplidarNode::RplidarNode(const rclcpp::NodeOptions & options)
    : Node("rplidar_node", options), scanning_(false), should_stop_(false) {

    declare_parameters();
    load_parameters();

    laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    
    // Create service to start/stop lidar
    start_stop_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_stop_lidar",
        std::bind(&RplidarNode::start_stop_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    double timer_period_sec = 1.0 / scan_frequency_;
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_sec),
        std::bind(&RplidarNode::publish_scan, this));

    RCLCPP_INFO(this->get_logger(), "RPLiDAR node starting on port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Auto-start scanning: %s", auto_start_ ? "enabled" : "disabled");
    
    // Initialize device connection
    if (!initialize_device()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize device. Node will attempt to reconnect.");
        return;
    }
    
    if (auto_start_) {
        if (start_scan()) {
            RCLCPP_INFO(this->get_logger(), "Auto-started scanning");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to auto-start scanning");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "RPLiDAR initialized. Use 'ros2 service call /start_stop_lidar std_srvs/srv/SetBool \"{data: true}\"' to start scanning");
    }
}

RplidarNode::~RplidarNode() {
    RCLCPP_INFO(this->get_logger(), "RPLiDAR node shutting down...");
    
    should_stop_ = true;
    
    if (scanning_) {
        stop_scan();
    }
    
    if (scan_thread_.joinable()) {
        scan_thread_.join();
    }
    
    if (timer_) {
        timer_->cancel();
    }
    
    close_serial_port();
    RCLCPP_INFO(this->get_logger(), "RPLiDAR node shutdown complete.");
}

void RplidarNode::declare_parameters() {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("serial_baudrate", 115200);
    this->declare_parameter<std::string>("frame_id", "laser");
    this->declare_parameter<bool>("inverted", false);
    this->declare_parameter<double>("angle_compensate", 0.0);
    this->declare_parameter<double>("scan_frequency", 10.0);
    this->declare_parameter<double>("range_min", 0.15);
    this->declare_parameter<double>("range_max", 12.0);
    this->declare_parameter<bool>("auto_start", false);
}

void RplidarNode::load_parameters() {
    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("serial_baudrate", serial_baudrate_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("inverted", inverted_);
    this->get_parameter("angle_compensate", angle_compensate_);
    this->get_parameter("scan_frequency", scan_frequency_);
    this->get_parameter("range_min", range_min_);
    this->get_parameter("range_max", range_max_);
    this->get_parameter("auto_start", auto_start_);
}

bool RplidarNode::initialize_device() {
    // Open serial port
    if (!open_serial_port()) {
        return false;
    }
    
    // Reset the device first
    if (!reset_device()) {
        RCLCPP_WARN(this->get_logger(), "Failed to reset device, continuing anyway");
    }
    
    // Try to get device info with smart baud rate detection
    bool handshake_ok = get_device_info() && get_device_health();
    
    if (!handshake_ok) {
        RCLCPP_INFO(this->get_logger(), "Trying different baud rates...");
        std::vector<int> try_bauds = {115200, 256000, 230400, 460800, 57600};
        
        for (int baud : try_bauds) {
            if (baud == serial_baudrate_) continue; // already tried
            
            RCLCPP_INFO(this->get_logger(), "Trying baud rate: %d", baud);
            close_serial_port();
            serial_baudrate_ = baud;
            
            if (!open_serial_port()) {
                continue;
            }
            
            // Reset at new baud rate
            reset_device();
            
            if (get_device_info() && get_device_health()) {
                RCLCPP_INFO(this->get_logger(), "Successfully connected at baud rate: %d", baud);
                handshake_ok = true;
                break;
            }
        }
    }
    
    return handshake_ok;
}

bool RplidarNode::reset_device() {
    RCLCPP_INFO(this->get_logger(), "Resetting RPLiDAR...");
    
    // Send reset command
    if (!send_command(RPLIDAR_CMD_RESET)) {
        return false;
    }
    
    // Wait for device to reset
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Clear any pending data
    tcflush(serial_fd_, TCIOFLUSH);
    
    return true;
}

void RplidarNode::start_stop_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                     std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        // Start scanning
        if (serial_fd_ < 0) {
            if (!initialize_device()) {
                response->success = false;
                response->message = "Failed to initialize device";
                return;
            }
        }
        
        if (scanning_) {
            response->success = true;
            response->message = "Already scanning";
            return;
        }
        
        if (start_scan()) {
            response->success = true;
            response->message = "Scanning started successfully";
            RCLCPP_INFO(this->get_logger(), "Scanning started via service call");
        } else {
            response->success = false;
            response->message = "Failed to start scanning";
        }
    } else {
        // Stop scanning
        if (!scanning_) {
            response->success = true;
            response->message = "Already stopped";
            return;
        }
        
        if (stop_scan()) {
            response->success = true;
            response->message = "Scanning stopped successfully";
            RCLCPP_INFO(this->get_logger(), "Scanning stopped via service call");
        } else {
            response->success = false;
            response->message = "Failed to stop scanning";
        }
    }
}

bool RplidarNode::open_serial_port() {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s: %s", 
                     serial_port_.c_str(), strerror(errno));
        return false;
    }

    // Make it blocking again
    fcntl(serial_fd_, F_SETFL, 0);

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting terminal attributes: %s", strerror(errno));
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Set baud rate
    speed_t baud_flag = B115200;
    switch(serial_baudrate_) {
        case 57600: baud_flag = B57600; break;
        case 115200: baud_flag = B115200; break;
        case 230400: baud_flag = B230400; break;
        case 460800: baud_flag = B460800; break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unsupported baud rate %d, using 115200", serial_baudrate_);
            serial_baudrate_ = 115200;
            baud_flag = B115200;
    }
    
    cfsetospeed(&tty, baud_flag);
    cfsetispeed(&tty, baud_flag);

    // 8N1
    tty.c_cflag &= ~PARENB;  // No parity
    tty.c_cflag &= ~CSTOPB;  // One stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable reading and ignore control lines

    // Raw mode
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_lflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // 1 second timeout

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting terminal attributes: %s", strerror(errno));
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Clear any existing data
    tcflush(serial_fd_, TCIOFLUSH);
    
    // Control motor via DTR (clear DTR = motor on for USB adapter)
    int dtr_flag = TIOCM_DTR;
    if (ioctl(serial_fd_, TIOCMBIC, &dtr_flag) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to control DTR: %s", strerror(errno));
    }
    
    // Wait for port to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", serial_port_.c_str(), serial_baudrate_);
    return true;
}

void RplidarNode::close_serial_port() {
    if (serial_fd_ >= 0) {
        RCLCPP_INFO(this->get_logger(), "Closing serial port");
        
        // Send stop command
        send_command(RPLIDAR_CMD_STOP);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Stop motor (set DTR)
        int dtr_flag = TIOCM_DTR;
        ioctl(serial_fd_, TIOCMBIS, &dtr_flag);
        
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool RplidarNode::serial_write(const uint8_t* data, size_t length) {
    if (serial_fd_ < 0) return false;
    
    size_t written = 0;
    while (written < length) {
        ssize_t n = write(serial_fd_, data + written, length - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", strerror(errno));
            return false;
        }
        written += n;
    }
    
    tcdrain(serial_fd_);
    return true;
}

int RplidarNode::serial_read(uint8_t* buffer, size_t length) {
    if (serial_fd_ < 0) return -1;
    
    ssize_t bytes_read = read(serial_fd_, buffer, length);
    if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN(this->get_logger(), "Serial read failed: %s", strerror(errno));
        return -1;
    }
    return bytes_read;
}

int RplidarNode::serial_read_timeout(uint8_t* buffer, size_t length, int timeout_ms) {
    if (serial_fd_ < 0) return -1;

    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(serial_fd_, &readfds);
    
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    
    int select_result = select(serial_fd_ + 1, &readfds, nullptr, nullptr, &timeout);
    
    if (select_result > 0 && FD_ISSET(serial_fd_, &readfds)) {
        return serial_read(buffer, length);
    } else if (select_result == 0) {
        return 0; // Timeout
    } else {
        RCLCPP_WARN(this->get_logger(), "Select error: %s", strerror(errno));
        return -1;
    }
}

bool RplidarNode::send_command(uint8_t cmd) {
    if (serial_fd_ < 0) return false;
    
    uint8_t pkt_header[2] = {0xA5, cmd};
    bool result = serial_write(pkt_header, 2);
    
    if (result) {
        RCLCPP_DEBUG(this->get_logger(), "Sent command: 0x%02X", cmd);
    }
    
    return result;
}

bool RplidarNode::wait_response_header(uint8_t response_type, uint32_t& response_length) {
    uint8_t response_header[7];
    int total_read = 0;
    int retry_count = 0;
    
    // Try to sync to header
    while (retry_count < 10) {
        // Read first byte
        int bytes = serial_read_timeout(&response_header[0], 1, 2000);
        if (bytes <= 0) {
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for response header");
            return false;
        }
        
        if (response_header[0] != 0xA5) {
            retry_count++;
            continue;
        }
        
        // Read second byte
        bytes = serial_read_timeout(&response_header[1], 1, 1000);
        if (bytes <= 0) continue;
        
        if (response_header[1] != 0x5A) {
            retry_count++;
            continue;
        }
        
        // Read rest of header
        total_read = 2;
        while (total_read < 7) {
            bytes = serial_read_timeout(&response_header[total_read], 7 - total_read, 1000);
            if (bytes <= 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to read complete header");
                return false;
            }
            total_read += bytes;
        }
        
        break;
    }
    
    if (retry_count >= 10) {
        RCLCPP_ERROR(this->get_logger(), "Failed to sync to response header");
        return false;
    }
    
    // Extract response length (30-bit value)
    response_length = (response_header[2] & 0x3F) | 
                     (static_cast<uint32_t>(response_header[3]) << 6) |
                     (static_cast<uint32_t>(response_header[4]) << 14) |
                     (static_cast<uint32_t>(response_header[5]) << 22);
    
    uint8_t received_type = response_header[6];
    if (received_type != response_type) {
        RCLCPP_WARN(this->get_logger(), "Unexpected response type: 0x%02X, expected: 0x%02X", 
                    received_type, response_type);
        return false;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Valid response header received, type: 0x%02X, length: %u", 
                 received_type, response_length);
    return true;
}

bool RplidarNode::get_device_info() {
    // Clear any pending data
    tcflush(serial_fd_, TCIOFLUSH);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    if (!send_command(RPLIDAR_CMD_GET_INFO)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send GET_INFO command");
        return false;
    }
    
    uint32_t response_length;
    if (!wait_response_header(RPLIDAR_ANS_TYPE_DEVINFO, response_length)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive device info header");
        return false;
    }
    
    if (response_length != 20) {
        RCLCPP_WARN(this->get_logger(), "Unexpected device info length: %u", response_length);
    }
    
    uint8_t info_data[20];
    int total_read = 0;
    
    while (total_read < 20) {
        int bytes_read = serial_read_timeout(&info_data[total_read], 20 - total_read, 1000);
        if (bytes_read <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read device info data");
            return false;
        }
        total_read += bytes_read;
    }
    
    uint8_t model = info_data[0];
    uint16_t firmware_version = (info_data[2] << 8) | info_data[1];
    uint8_t hardware_version = info_data[3];
    
    RCLCPP_INFO(this->get_logger(), "RPLiDAR Info - Model: %u, FW: %u.%02u, HW: %u", 
                model, firmware_version >> 8, firmware_version & 0xFF, hardware_version);
    
    return true;
}

bool RplidarNode::get_device_health() {
    if (!send_command(RPLIDAR_CMD_GET_HEALTH)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send GET_HEALTH command");
        return false;
    }
    
    uint32_t response_length;
    if (!wait_response_header(RPLIDAR_ANS_TYPE_DEVHEALTH, response_length)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive health header");
        return false;
    }
    
    uint8_t health_data[3];
    int total_read = 0;
    
    while (total_read < 3) {
        int bytes_read = serial_read_timeout(&health_data[total_read], 3 - total_read, 1000);
        if (bytes_read <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read health data");
            return false;
        }
        total_read += bytes_read;
    }
    
    uint8_t status = health_data[0];
    uint16_t error_code = (health_data[2] << 8) | health_data[1];
    
    if (status == 0) {
        RCLCPP_INFO(this->get_logger(), "RPLiDAR health: Good");
        return true;
    } else if (status == 1) {
        RCLCPP_WARN(this->get_logger(), "RPLiDAR health: Warning (error_code: %u)", error_code);
        return true; // Warning is acceptable
    } else {
        RCLCPP_ERROR(this->get_logger(), "RPLiDAR health: Error (status: %u, error_code: %u)", 
                     status, error_code);
        return false;
    }
}

bool RplidarNode::start_scan() {
    if (scanning_) {
        RCLCPP_WARN(this->get_logger(), "Already scanning");
        return true;
    }
    
    // Stop any previous scan and clear buffer
    send_command(RPLIDAR_CMD_STOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    tcflush(serial_fd_, TCIOFLUSH);
    
    // Start motor (clear DTR)
    int dtr_flag = TIOCM_DTR;
    ioctl(serial_fd_, TIOCMBIC, &dtr_flag);
    
    // Wait for motor to spin up
    RCLCPP_INFO(this->get_logger(), "Starting motor...");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Send scan command
    if (!send_command(RPLIDAR_CMD_SCAN)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send SCAN command");
        return false;
    }
    
    // Wait for response header
    uint32_t response_length;
    if (!wait_response_header(RPLIDAR_ANS_TYPE_MEASUREMENT, response_length)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive scan header");
        // Stop motor on failure
        ioctl(serial_fd_, TIOCMBIS, &dtr_flag);
        return false;
    }
    
    scanning_ = true;
    scan_thread_ = std::thread(&RplidarNode::scan_thread_func, this);
    
    RCLCPP_INFO(this->get_logger(), "Scan started successfully");
    return true;
}

bool RplidarNode::stop_scan() {
    if (!scanning_) {
        return true;
    }
    
    scanning_ = false;
    
    // Wait for scan thread to finish
    if (scan_thread_.joinable()) {
        scan_thread_.join();
    }
    
    // Send stop command
    send_command(RPLIDAR_CMD_STOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Stop motor (set DTR)
    int dtr_flag = TIOCM_DTR;
    ioctl(serial_fd_, TIOCMBIS, &dtr_flag);
    
    // Clear any remaining data
    tcflush(serial_fd_, TCIOFLUSH);
    
    RCLCPP_INFO(this->get_logger(), "Scan stopped");
    return true;
}

void RplidarNode::scan_thread_func() {
    RCLCPP_INFO(this->get_logger(), "Scan thread started");
    
    while (scanning_ && !should_stop_) {
        double angle, distance;
        uint8_t quality;
        
        if (read_measurement(angle, distance, quality)) {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            scan_points_.push_back({angle, distance, quality});
            
            // Keep only recent points
            if (scan_points_.size() > 3600) {
                scan_points_.erase(scan_points_.begin(), scan_points_.begin() + 1800);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Scan thread terminated");
}

bool RplidarNode::read_measurement(double& angle, double& distance, uint8_t& quality) {
    uint8_t measurement_data[5];
    int total_read = 0;
    
    // Read 5-byte measurement packet
    while (total_read < 5) {
        int bytes_read = serial_read_timeout(&measurement_data[total_read], 5 - total_read, 50);
        if (bytes_read <= 0) {
            return false;
        }
        total_read += bytes_read;
    }
    
    // Parse measurement packet
    uint8_t sync_quality = measurement_data[0];
    bool sync_bit = (sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) != 0;
    quality = sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
    
    uint8_t check_angle_low = measurement_data[1];
    bool check_bit = (check_angle_low & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) != 0;
    
    if (!check_bit) {
        return false; // Invalid packet
    }
    
    uint16_t angle_raw = ((measurement_data[2] << 8) | check_angle_low) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
    uint16_t distance_raw = (measurement_data[4] << 8) | measurement_data[3];
    
    // Convert to physical units
    angle = static_cast<double>(angle_raw) / 64.0; // Convert to degrees
    distance = static_cast<double>(distance_raw) / 4000.0; // Convert to meters
    
    // Apply angle compensation
    angle += angle_compensate_;
    if (angle < 0.0) angle += 360.0;
    if (angle >= 360.0) angle -= 360.0;
    
    // Apply inversion if requested
    if (inverted_) {
        angle = 360.0 - angle;
    }
    
    return true;
}

sensor_msgs::msg::LaserScan RplidarNode::create_laser_scan_msg() {
    sensor_msgs::msg::LaserScan scan_msg;
    
    scan_msg.header.stamp = this->get_clock()->now();
    scan_msg.header.frame_id = frame_id_;
    
    scan_msg.angle_min = 0.0;
    scan_msg.angle_max = 2.0 * M_PI;
    scan_msg.angle_increment = M_PI / 180.0; // 1 degree resolution
    scan_msg.time_increment = 1.0 / (scan_frequency_ * 360.0);
    scan_msg.scan_time = 1.0 / scan_frequency_;
    scan_msg.range_min = range_min_;
    scan_msg.range_max = range_max_;
    
    // Initialize ranges array
    scan_msg.ranges.resize(360, std::numeric_limits<float>::infinity());
    scan_msg.intensities.resize(360, 0.0);
    
    std::lock_guard<std::mutex> lock(scan_mutex_);
    
    for (const auto& point : scan_points_) {
        if (point.distance >= range_min_ && point.distance <= range_max_ && point.quality > 0) {
            int angle_index = static_cast<int>(std::round(point.angle));
            if (angle_index >= 0 && angle_index < 360) {
                // Use closest distance if multiple readings for same angle
                if (std::isinf(scan_msg.ranges[angle_index]) || 
                    point.distance < scan_msg.ranges[angle_index]) {
                    scan_msg.ranges[angle_index] = static_cast<float>(point.distance);
                    scan_msg.intensities[angle_index] = static_cast<float>(point.quality);
                }
            }
        }
    }
    
    // Clear processed points
    scan_points_.clear();
    
    return scan_msg;
}

void RplidarNode::publish_scan() {
    if (!scanning_) {
        return;
    }
    
    auto scan_msg = create_laser_scan_msg();
    
    // Only publish if we have some valid data
    bool has_valid_data = false;
    for (const auto& range : scan_msg.ranges) {
        if (!std::isinf(range)) {
            has_valid_data = true;
            break;
        }
    }
    
    if (has_valid_data) {
        laser_publisher_->publish(scan_msg);
    }
}