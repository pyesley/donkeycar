#include "rpi_ros2_cpp_nodes/rplidar_node.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>


// Update the constructor with more aggressive reset
RplidarNode::RplidarNode(const rclcpp::NodeOptions & options)
    : Node("rplidar_node", options), scanning_(false), should_stop_(false) {

    declare_parameters();
    load_parameters();

    laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    
    start_stop_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_stop_lidar",
        std::bind(&RplidarNode::start_stop_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    double timer_period_sec = 1.0 / scan_frequency_;
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_sec),
        std::bind(&RplidarNode::publish_scan, this));

    RCLCPP_INFO(this->get_logger(), "RPLiDAR node starting on port: %s", serial_port_.c_str());
    
    // Try to initialize with retries
    bool initialized = false;
    for (int retry = 0; retry < 3 && !initialized; retry++) {
        if (retry > 0) {
            RCLCPP_WARN(this->get_logger(), "Retry %d/3...", retry);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        
        if (open_serial_port()) {
            RCLCPP_INFO(this->get_logger(), "Port opened. Attempting handshake...");
            
            // More aggressive reset sequence
            // Send multiple stop commands to ensure clean state
            for (int i = 0; i < 3; i++) {
                send_command(RPLIDAR_CMD_STOP);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            
            // Flush all buffers
            tcflush(serial_fd_, TCIOFLUSH);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Now try handshake
            if (get_device_info()) {
                RCLCPP_INFO(this->get_logger(), "Device info retrieved successfully!");
                
                if (get_device_health()) {
                    RCLCPP_INFO(this->get_logger(), "LiDAR health check passed!");
                    initialized = true;
                    if (auto_start_) {
                        start_scan();
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Device health check failed");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get device info");
                close_serial_port();
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        }

        if (auto_start_) {
            if (start_scan()) {
                RCLCPP_INFO(this->get_logger(), "Auto-started scanning");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to auto-start scan");
            }
        }

    }
    
    if (!initialized) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize after 3 retries.");
        RCLCPP_INFO(this->get_logger(), "You can try starting manually with: ros2 service call /start_stop_lidar std_srvs/srv/SetBool '{data: true}'");
    }
}

RplidarNode::~RplidarNode() {
    should_stop_ = true;
    if (scanning_) stop_scan();
    if (scan_thread_.joinable()) scan_thread_.join();
    close_serial_port();
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
    this->declare_parameter<bool>("auto_start", true);  // Changed to true
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

void RplidarNode::start_stop_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                     std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        // REQUEST: START
        if (scanning_) {
            response->success = true;
            response->message = "Already scanning";
            return;
        }

        // If port is closed, try to open it first
        if (serial_fd_ < 0) {
            if (!open_serial_port()) {
                response->success = false;
                response->message = "Failed to open serial port";
                return;
            }
        }

        // Try to start scanning
        if (start_scan()) {
            response->success = true;
            response->message = "Scanning started";
        } else {
            response->success = false;
            response->message = "Failed to send scan command";
        }
    } else {
        // REQUEST: STOP
        if (!scanning_) {
            response->success = true;
            response->message = "Already stopped";
            return;
        }
        
        stop_scan();
        response->success = true;
        response->message = "Scanning stopped";
    }
}

bool RplidarNode::open_serial_port() {
    if (serial_fd_ >= 0) close(serial_fd_);

    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s", serial_port_.c_str(), strerror(errno));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
        close(serial_fd_);
        return false;
    }

    speed_t speed = B115200;
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(ICRNL | INLCR);
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        close(serial_fd_);
        return false;
    }

    // CORRECTED DTR Control for RPLiDAR A1:
    // DTR LOW = Motor ON, DTR HIGH = Motor OFF
    int dtr_flag = TIOCM_DTR;
    
    // First set DTR high to stop motor (reset state)
    ioctl(serial_fd_, TIOCMBIS, &dtr_flag);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Then clear DTR (set low) to start motor
    ioctl(serial_fd_, TIOCMBIC, &dtr_flag);  
    RCLCPP_INFO(this->get_logger(), "Motor enabled (DTR cleared/low)");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    tcflush(serial_fd_, TCIOFLUSH);
    return true;
}

void RplidarNode::close_serial_port() {
    if (serial_fd_ >= 0) {
        // Send stop command
        uint8_t pkt[2] = {0xA5, RPLIDAR_CMD_STOP};
        write(serial_fd_, pkt, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Set DTR high to stop motor
        int dtr_flag = TIOCM_DTR;
        ioctl(serial_fd_, TIOCMBIS, &dtr_flag);  // Set high = motor off
        
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

// Replace the current read_exact function with this improved version
int RplidarNode::read_exact(uint8_t* buffer, size_t length, int timeout_ms) {
    if (serial_fd_ < 0) return -1;
    
    size_t total_read = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    // Temporarily set to non-blocking for timeout control
    int flags = fcntl(serial_fd_, F_GETFL, 0);
    fcntl(serial_fd_, F_SETFL, flags | O_NONBLOCK);
    
    while (total_read < length) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        
        if (elapsed > timeout_ms) {
            RCLCPP_DEBUG(this->get_logger(), "Read timeout after %ld ms, got %zu/%zu bytes", 
                        elapsed, total_read, length);
            break;
        }
        
        ssize_t n = read(serial_fd_, buffer + total_read, length - total_read);
        if (n > 0) {
            total_read += n;
        } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
            // Actual error, not just "no data available"
            RCLCPP_DEBUG(this->get_logger(), "Read error: %s", strerror(errno));
            break;
        } else {
            // No data available, wait a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    // Restore blocking mode
    fcntl(serial_fd_, F_SETFL, flags);
    
    return total_read;
}

bool RplidarNode::serial_write(const uint8_t* data, size_t length) {
    if (serial_fd_ < 0) return false;
    return write(serial_fd_, data, length) == (ssize_t)length;
}

bool RplidarNode::send_command(uint8_t cmd) {
    uint8_t pkt[2] = {0xA5, cmd};
    return serial_write(pkt, 2);
}


bool RplidarNode::wait_response_header(uint8_t response_type, uint32_t& response_length) {
    // Read exactly 7 bytes for the header
    uint8_t header[7];
    
    // Use blocking read with timeout
    size_t total_read = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    while (total_read < 7) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        
        if (elapsed > 2000) { // 2 second timeout
            RCLCPP_WARN(this->get_logger(), "Timeout reading header: got %zu/7 bytes", total_read);
            if (total_read > 0) {
                std::string hex_str;
                for (size_t i = 0; i < total_read; i++) {
                    char buf[4];
                    snprintf(buf, sizeof(buf), "%02X ", header[i]);
                    hex_str += buf;
                }
                RCLCPP_DEBUG(this->get_logger(), "Partial header: %s", hex_str.c_str());
            }
            return false;
        }
        
        ssize_t n = read(serial_fd_, header + total_read, 7 - total_read);
        if (n > 0) {
            total_read += n;
        } else if (n == 0) {
            // No data, wait a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "Read error: %s", strerror(errno));
            return false;
        }
    }
    
    // Log the full header received
    std::string hex_str;
    for (int i = 0; i < 7; i++) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", header[i]);
        hex_str += buf;
    }
    RCLCPP_DEBUG(this->get_logger(), "Received header: %s", hex_str.c_str());
    
    // Check sync bytes
    if (header[0] != 0xA5 || header[1] != 0x5A) {
        RCLCPP_WARN(this->get_logger(), "Invalid header sync: %02X %02X (expected A5 5A)", 
                    header[0], header[1]);
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "SUCCESS: Valid LiDAR Header found!");
    
    // Parse length (matching Python struct.unpack("<I", header[2:6])[0] & 0x3FFFFFFF)
    response_length = (static_cast<uint32_t>(header[2]) |
                      (static_cast<uint32_t>(header[3]) << 8) |
                      (static_cast<uint32_t>(header[4]) << 16) |
                      (static_cast<uint32_t>(header[5]) << 24)) & 0x3FFFFFFF;
    
    uint8_t recv_type = header[6];
    
    RCLCPP_INFO(this->get_logger(), "Response Type: 0x%02X, Length: %u", recv_type, response_length);
    
    if (recv_type != response_type) {
        RCLCPP_WARN(this->get_logger(), "Type mismatch but continuing (got 0x%02X, expected 0x%02X)", 
                    recv_type, response_type);
    }
    
    return true;
}

bool RplidarNode::get_device_info() {
    RCLCPP_DEBUG(this->get_logger(), "Sending GET_INFO command...");
    
    // Clear input buffer
    tcflush(serial_fd_, TCIFLUSH);
    
    // Send GET_INFO command (A5 50)
    uint8_t cmd[2] = {0xA5, 0x50};
    RCLCPP_INFO(this->get_logger(), "Sending GET_INFO: %02X %02X", cmd[0], cmd[1]);
    
    if (write(serial_fd_, cmd, 2) != 2) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send GET_INFO command");
        return false;
    }
    
    // Read response header
    uint32_t len;
    if (!wait_response_header(RPLIDAR_ANS_TYPE_DEVINFO, len)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive device info header");
        return false;
    }
    
    // Read payload
    if (len != 20) {
        RCLCPP_WARN(this->get_logger(), "Unexpected payload length: %u (expected 20)", len);
    }
    
    uint8_t data[20];
    size_t bytes_to_read = (len < 20) ? len : 20;
    size_t bytes_read = 0;
    
    auto start_time = std::chrono::steady_clock::now();
    while (bytes_read < bytes_to_read) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > 2000) break;
        
        ssize_t n = read(serial_fd_, data + bytes_read, bytes_to_read - bytes_read);
        if (n > 0) bytes_read += n;
        else std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (bytes_read >= 4) {
        RCLCPP_INFO(this->get_logger(), "RPLiDAR Device Info:");
        RCLCPP_INFO(this->get_logger(), "  Model: 0x%02X", data[0]);
        RCLCPP_INFO(this->get_logger(), "  Firmware: %d.%02d", data[2], data[1]);
        RCLCPP_INFO(this->get_logger(), "  Hardware: %d", data[3]);
        return true;
    }
    
    return false;
}

bool RplidarNode::get_device_health() {
    tcflush(serial_fd_, TCIOFLUSH);
    if (!send_command(RPLIDAR_CMD_GET_HEALTH)) return false;
    
    uint32_t len;
    if (!wait_response_header(RPLIDAR_ANS_TYPE_DEVHEALTH, len)) return false;
    
    uint8_t data[3];
    if (read_exact(data, 3, 1000) != 3) return false;
    
    return (data[0] == 0);
}

bool RplidarNode::start_scan() {
    if (scanning_) {
        RCLCPP_WARN(this->get_logger(), "Already scanning");
        return true;
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting scan...");
    
    // Clear any pending data
    tcflush(serial_fd_, TCIOFLUSH);
    
    // Send SCAN command
    if (!send_command(RPLIDAR_CMD_SCAN)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send SCAN command");
        return false;
    }
    
    // Wait for scan response header
    uint32_t len;
    if (!wait_response_header(RPLIDAR_ANS_TYPE_MEASUREMENT, len)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get scan response header");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Scan started successfully! Motor should be spinning.");
    
    // Start the scan thread
    scanning_ = true;
    should_stop_ = false;
    scan_thread_ = std::thread(&RplidarNode::scan_thread_func, this);
    
    return true;
}

bool RplidarNode::stop_scan() {
    scanning_ = false;
    if (scan_thread_.joinable()) scan_thread_.join();
    
    send_command(RPLIDAR_CMD_STOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Keep motor spinning even when not scanning
    // (DTR should remain low)
    
    tcflush(serial_fd_, TCIOFLUSH);
    RCLCPP_INFO(this->get_logger(), "Scan stopped (motor still running)");
    return true;
}

void RplidarNode::scan_thread_func() {
    RCLCPP_INFO(this->get_logger(), "Scan thread started");
    size_t point_count = 0;
    
    while (scanning_ && !should_stop_) {
        uint8_t node[5];
        int n = read_exact(node, 5, 1000);
        
        if (n == 5) {
            uint8_t quality = node[0] >> 2;
            bool check_bit = node[1] & 0x01;
            
            if (check_bit) {
                uint16_t angle_q6 = ((node[1] >> 1) | (node[2] << 7));
                uint16_t dist_q2 = (node[3] | (node[4] << 8));
                
                double angle = angle_q6 / 64.0;
                double dist = dist_q2 / 4.0 / 1000.0;
                
                if (inverted_) angle = 360.0 - angle;
                angle = angle * M_PI / 180.0;
                if (angle < 0) angle += 2*M_PI;
                if (angle >= 2*M_PI) angle -= 2*M_PI;

                std::lock_guard<std::mutex> lock(scan_mutex_);
                scan_points_.push_back({angle, dist, quality});
                point_count++;
                
                // Log every 360 points (roughly one full rotation)
                if (point_count % 360 == 0) {
                    RCLCPP_DEBUG(this->get_logger(), "Received %zu scan points", point_count);
                }
                
                if (scan_points_.size() > 3600) {
                    scan_points_.erase(scan_points_.begin(), scan_points_.begin() + 1800);
                }
            }
        } else if (n > 0) {
            RCLCPP_WARN(this->get_logger(), "Incomplete scan data: got %d/5 bytes", n);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Scan thread stopped");
}

sensor_msgs::msg::LaserScan RplidarNode::create_laser_scan_msg() {
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = this->get_clock()->now();
    scan_msg.header.frame_id = frame_id_;
    scan_msg.angle_min = 0.0;
    scan_msg.angle_max = 2.0 * M_PI;
    scan_msg.angle_increment = M_PI / 180.0;
    scan_msg.range_min = range_min_;
    scan_msg.range_max = range_max_;
    scan_msg.ranges.resize(360, std::numeric_limits<float>::infinity());
    scan_msg.intensities.resize(360, 0.0);
    
    std::lock_guard<std::mutex> lock(scan_mutex_);
    for (const auto& p : scan_points_) {
        if (p.distance >= range_min_ && p.distance <= range_max_) {
            int idx = static_cast<int>(p.angle * 180.0 / M_PI);
            if (idx >= 0 && idx < 360) {
                scan_msg.ranges[idx] = p.distance;
                scan_msg.intensities[idx] = p.quality;
            }
        }
    }
    scan_points_.clear();
    return scan_msg;
}

void RplidarNode::publish_scan() {
    if (scanning_) laser_publisher_->publish(create_laser_scan_msg());
}







// Update the open_serial_port function - REVERSE the DTR logic:



// Also update close_serial_port to stop the motor properly:



// And for the stop_scan function:

