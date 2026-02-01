#include "pidog_ros/imu_node.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>

namespace pidog_ros {

IMUNode::IMUNode(const rclcpp::NodeOptions& options)
    : Node("pidog_imu_node", options),
      i2c_fd_(-1),
      initialized_(false),
      gyro_offset_x_(0.0),
      gyro_offset_y_(0.0),
      gyro_offset_z_(0.0),
      samples_published_(0),
      read_errors_(0) {

    declare_parameters();
    load_parameters();

    // Create publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "pidog/imu", rclcpp::SensorDataQoS());

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "pidog/imu/status", rclcpp::QoS(10));

    // Initialize I2C
    if (!init_i2c()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize I2C - IMU will not function");
        publish_status("ERROR: Failed to initialize I2C");
        return;
    }

    // Verify chip ID
    if (!verify_chip_id()) {
        RCLCPP_ERROR(this->get_logger(), "SH3001 chip ID verification failed");
        publish_status("ERROR: SH3001 not detected");
        close_i2c();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "SH3001 IMU detected successfully");

    // Calibrate gyroscope (measure offsets while stationary)
    RCLCPP_INFO(this->get_logger(), "Calibrating gyroscope (keep IMU still)...");
    calibrate_gyro(100);
    RCLCPP_INFO(this->get_logger(), "Gyro offsets: X=%.2f, Y=%.2f, Z=%.2f",
                gyro_offset_x_, gyro_offset_y_, gyro_offset_z_);

    initialized_ = true;
    start_time_ = this->get_clock()->now();

    // Create sample timer based on configured rate
    auto sample_period = std::chrono::microseconds(1000000 / sample_rate_hz_);
    sample_timer_ = this->create_wall_timer(
        sample_period,
        std::bind(&IMUNode::sample_timer_callback, this));

    // Status timer every 10 seconds
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&IMUNode::status_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "IMUNode initialized successfully");
    RCLCPP_INFO(this->get_logger(), "  I2C bus: /dev/i2c-%d", i2c_bus_);
    RCLCPP_INFO(this->get_logger(), "  Sample rate: %d Hz", sample_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publishing to: pidog/imu");

    publish_status("IMU initialized");
}

IMUNode::~IMUNode() {
    RCLCPP_INFO(this->get_logger(), "IMUNode shutting down...");

    if (sample_timer_) {
        sample_timer_->cancel();
    }
    if (status_timer_) {
        status_timer_->cancel();
    }

    close_i2c();

    RCLCPP_INFO(this->get_logger(), "IMUNode shutdown complete. Published %lu samples, %lu errors",
                samples_published_.load(), read_errors_.load());
}

void IMUNode::declare_parameters() {
    this->declare_parameter<int>("imu.i2c_bus", DEFAULT_I2C_BUS);
    this->declare_parameter<int>("imu.sample_rate_hz", 100);
    this->declare_parameter<std::string>("imu.frame_id", "pidog_imu_link");
    this->declare_parameter<std::string>("imu.acc_range", "8G");
    this->declare_parameter<std::string>("imu.gyro_range", "2000");
}

void IMUNode::load_parameters() {
    this->get_parameter("imu.i2c_bus", i2c_bus_);
    this->get_parameter("imu.sample_rate_hz", sample_rate_hz_);
    this->get_parameter("imu.frame_id", frame_id_);

    // Parse accelerometer range
    std::string acc_range;
    this->get_parameter("imu.acc_range", acc_range);
    if (acc_range == "2G") {
        acc_scale_ = ACC_SCALE_2G;
    } else if (acc_range == "4G") {
        acc_scale_ = ACC_SCALE_4G;
    } else if (acc_range == "16G") {
        acc_scale_ = ACC_SCALE_16G;
    } else {
        acc_scale_ = ACC_SCALE_8G;  // Default
    }

    // Parse gyroscope range
    std::string gyro_range;
    this->get_parameter("imu.gyro_range", gyro_range);
    if (gyro_range == "125") {
        gyro_scale_ = GYRO_SCALE_125;
    } else if (gyro_range == "250") {
        gyro_scale_ = GYRO_SCALE_250;
    } else if (gyro_range == "500") {
        gyro_scale_ = GYRO_SCALE_500;
    } else if (gyro_range == "1000") {
        gyro_scale_ = GYRO_SCALE_1000;
    } else {
        gyro_scale_ = GYRO_SCALE_2000;  // Default
    }
}

bool IMUNode::init_i2c() {
    std::string device = "/dev/i2c-" + std::to_string(i2c_bus_);

    i2c_fd_ = open(device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device %s: %s",
                     device.c_str(), strerror(errno));
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, SH3001_I2C_ADDR) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set I2C slave address 0x%02X: %s",
                     SH3001_I2C_ADDR, strerror(errno));
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "I2C initialized: %s @ 0x%02X",
                device.c_str(), SH3001_I2C_ADDR);
    return true;
}

void IMUNode::close_i2c() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool IMUNode::verify_chip_id() {
    uint8_t chip_id = read_register(REG_CHIP_ID);
    RCLCPP_INFO(this->get_logger(), "Read chip ID: 0x%02X (expected 0x%02X)",
                chip_id, SH3001_CHIP_ID);
    return chip_id == SH3001_CHIP_ID;
}

uint8_t IMUNode::read_register(uint8_t reg) {
    if (i2c_fd_ < 0) return 0;

    uint8_t value = 0;
    if (write(i2c_fd_, &reg, 1) != 1) {
        read_errors_++;
        return 0;
    }
    if (read(i2c_fd_, &value, 1) != 1) {
        read_errors_++;
        return 0;
    }
    return value;
}

bool IMUNode::read_registers(uint8_t start_reg, uint8_t* buffer, size_t length) {
    if (i2c_fd_ < 0) return false;

    if (write(i2c_fd_, &start_reg, 1) != 1) {
        read_errors_++;
        return false;
    }
    if (read(i2c_fd_, buffer, length) != static_cast<ssize_t>(length)) {
        read_errors_++;
        return false;
    }
    return true;
}

int16_t IMUNode::read_int16(uint8_t low_reg, uint8_t high_reg) {
    uint8_t low = read_register(low_reg);
    uint8_t high = read_register(high_reg);
    return static_cast<int16_t>((high << 8) | low);
}

void IMUNode::sample_timer_callback() {
    if (!initialized_ || i2c_fd_ < 0) {
        return;
    }

    // Read all 12 bytes of sensor data in one transaction for efficiency
    uint8_t data[12];
    if (!read_registers(REG_ACC_X_LOW, data, 12)) {
        // Already logged error in read_registers
        return;
    }

    // Parse raw data (little-endian 16-bit signed)
    int16_t acc_x_raw = static_cast<int16_t>((data[1] << 8) | data[0]);
    int16_t acc_y_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
    int16_t acc_z_raw = static_cast<int16_t>((data[5] << 8) | data[4]);
    int16_t gyro_x_raw = static_cast<int16_t>((data[7] << 8) | data[6]);
    int16_t gyro_y_raw = static_cast<int16_t>((data[9] << 8) | data[8]);
    int16_t gyro_z_raw = static_cast<int16_t>((data[11] << 8) | data[10]);

    // Convert accelerometer to m/s^2
    double acc_x = acc_x_raw * acc_scale_ * G_TO_MS2;
    double acc_y = acc_y_raw * acc_scale_ * G_TO_MS2;
    double acc_z = acc_z_raw * acc_scale_ * G_TO_MS2;

    // Convert gyroscope to rad/s (with calibration offset)
    double gyro_x = (gyro_x_raw - gyro_offset_x_) * gyro_scale_ * DEG_TO_RAD;
    double gyro_y = (gyro_y_raw - gyro_offset_y_) * gyro_scale_ * DEG_TO_RAD;
    double gyro_z = (gyro_z_raw - gyro_offset_z_) * gyro_scale_ * DEG_TO_RAD;

    // Create and publish IMU message
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;

    // Linear acceleration (m/s^2)
    msg.linear_acceleration.x = acc_x;
    msg.linear_acceleration.y = acc_y;
    msg.linear_acceleration.z = acc_z;

    // Angular velocity (rad/s)
    msg.angular_velocity.x = gyro_x;
    msg.angular_velocity.y = gyro_y;
    msg.angular_velocity.z = gyro_z;

    // Orientation is not provided by SH3001 (no magnetometer)
    // Set covariance to -1 to indicate orientation is unknown
    msg.orientation_covariance[0] = -1.0;

    // Set covariances for acceleration and angular velocity
    // These are approximate values - adjust based on datasheet
    // Diagonal elements represent variance for x, y, z
    double acc_variance = 0.01;  // (m/s^2)^2
    double gyro_variance = 0.001;  // (rad/s)^2

    msg.linear_acceleration_covariance[0] = acc_variance;
    msg.linear_acceleration_covariance[4] = acc_variance;
    msg.linear_acceleration_covariance[8] = acc_variance;

    msg.angular_velocity_covariance[0] = gyro_variance;
    msg.angular_velocity_covariance[4] = gyro_variance;
    msg.angular_velocity_covariance[8] = gyro_variance;

    imu_pub_->publish(msg);
    samples_published_++;
}

void IMUNode::status_timer_callback() {
    auto now = this->get_clock()->now();
    double elapsed = (now - start_time_).seconds();
    double rate = samples_published_ / elapsed;

    std::stringstream ss;
    ss << "IMU Status: "
       << "samples=" << samples_published_
       << ", rate=" << std::fixed << std::setprecision(1) << rate << " Hz"
       << ", errors=" << read_errors_;

    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    publish_status(ss.str());
}

void IMUNode::publish_status(const std::string& status) {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub_->publish(msg);
}

void IMUNode::calibrate_gyro(int num_samples) {
    if (i2c_fd_ < 0) return;

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    int valid_samples = 0;

    for (int i = 0; i < num_samples; i++) {
        uint8_t data[6];
        if (read_registers(REG_GYRO_X_LOW, data, 6)) {
            int16_t gyro_x_raw = static_cast<int16_t>((data[1] << 8) | data[0]);
            int16_t gyro_y_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
            int16_t gyro_z_raw = static_cast<int16_t>((data[5] << 8) | data[4]);

            sum_x += gyro_x_raw;
            sum_y += gyro_y_raw;
            sum_z += gyro_z_raw;
            valid_samples++;
        }

        // Small delay between samples
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (valid_samples > 0) {
        gyro_offset_x_ = sum_x / valid_samples;
        gyro_offset_y_ = sum_y / valid_samples;
        gyro_offset_z_ = sum_z / valid_samples;
    }
}

}  // namespace pidog_ros
