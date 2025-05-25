#include "IMU_6050.hpp"

#include <fcntl.h>      // For O_RDWR
#include <unistd.h>     // For open, close, read, write
#include <sys/ioctl.h>  // For ioctl
#include <linux/i2c-dev.h> // For I2C_SLAVE
#include <linux/i2c.h>  // For struct i2c_msg, I2C_M_RD
#include <cmath>        // For M_PI, NAN
#include <cstring>      // For strerror
#include <cerrno>       // For errno
#include <chrono>       // For std::chrono::milliseconds


MPU6050PublisherNode::MPU6050PublisherNode()
    : Node("mpu6050_publisher"),
      i2c_fd_(-1),
      mpu6050_initialized_(false),
      shutdown_requested_(false)
{
    declare_parameters();
    get_parameters();

    i2c_device_name_ = "/dev/i2c-" + std::to_string(i2c_bus_number_);

    if (!initialize_mpu6050()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6050 on startup. Will retry periodically.");
        // Timer will attempt to re-initialize
    }

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw_mpu6050", 10);
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        std::bind(&MPU6050PublisherNode::publish_imu_data, this));

    RCLCPP_INFO(this->get_logger(), "MPU6050 Publisher Node started. Device: %s, Address: 0x%X",
                i2c_device_name_.c_str(), mpu6050_address_);
}

MPU6050PublisherNode::~MPU6050PublisherNode() {
    shutdown();
}

void MPU6050PublisherNode::shutdown() {
    shutdown_requested_ = true;
    RCLCPP_INFO(this->get_logger(), "Shutting down MPU6050 Publisher Node...");
    if (publish_timer_) {
        publish_timer_->cancel();
    }
    close_mpu6050();
    RCLCPP_INFO(this->get_logger(), "MPU6050 Publisher Node shutdown complete.");
}

void MPU6050PublisherNode::declare_parameters() {
    this->declare_parameter<int>("i2c_bus", 1);
    this->declare_parameter<int>("mpu6050_address", 0x68);
    this->declare_parameter<double>("publish_rate_hz", 200.0);
    this->declare_parameter<std::string>("imu_frame_id", "imu_link_mpu6050");
    this->declare_parameter<double>("accel_scale_factor", 16384.0); // LSB/g for +/-2g range
    this->declare_parameter<double>("gyro_scale_factor", 131.0);   // LSB/(deg/s) for +/-250deg/s range
}

void MPU6050PublisherNode::get_parameters() {
    this->get_parameter("i2c_bus", i2c_bus_number_);
    this->get_parameter("mpu6050_address", mpu6050_address_);
    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    this->get_parameter("imu_frame_id", imu_frame_id_);
    this->get_parameter("accel_scale_factor", accel_scale_factor_);
    this->get_parameter("gyro_scale_factor", gyro_scale_factor_);
}

bool MPU6050PublisherNode::initialize_mpu6050() {
    if (mpu6050_initialized_.load()) {
        return true;
    }
    if (shutdown_requested_.load()) return false;

    RCLCPP_INFO(this->get_logger(), "Attempting to initialize MPU6050 on %s address 0x%X...",
                i2c_device_name_.c_str(), mpu6050_address_);

    close_mpu6050(); // Close if already open, for re-initialization attempts

    i2c_fd_ = open(i2c_device_name_.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus %s: %s", i2c_device_name_.c_str(), strerror(errno));
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, mpu6050_address_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set I2C slave address 0x%X: %s", mpu6050_address_, strerror(errno));
        close_mpu6050();
        return false;
    }

    // Wake up MPU6050 by writing 0 to PWR_MGMT_1 register
    if (!write_byte_to_mpu6050(PWR_MGMT_1, 0x00)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to wake up MPU6050 (write to PWR_MGMT_1 failed).");
        close_mpu6050();
        return false;
    }
    
    // Optional: Could add more configuration here, e.g., setting gyroscope/accelerometer ranges
    // For now, assume default ranges which match typical scale factors.

    mpu6050_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "MPU6050 initialized successfully.");
    return true;
}

void MPU6050PublisherNode::close_mpu6050() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
        RCLCPP_INFO(this->get_logger(), "Closed I2C device %s.", i2c_device_name_.c_str());
    }
    mpu6050_initialized_ = false;
}

bool MPU6050PublisherNode::write_byte_to_mpu6050(uint8_t reg_addr, uint8_t data) {
    if (i2c_fd_ < 0) {
        RCLCPP_WARN(this->get_logger(), "Write attempt while I2C fd is not valid.");
        return false;
    }
    uint8_t buffer[2] = {reg_addr, data};
    if (::write(i2c_fd_, buffer, 2) != 2) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to MPU6050 register 0x%X: %s", reg_addr, strerror(errno));
        return false;
    }
    return true;
}

bool MPU6050PublisherNode::read_bytes_from_mpu6050(uint8_t reg_addr, uint8_t* buffer, int length) {
     if (i2c_fd_ < 0) {
        RCLCPP_WARN(this->get_logger(), "Read attempt while I2C fd is not valid.");
        return false;
    }

    // Send the register address to read from
    if (::write(i2c_fd_, &reg_addr, 1) != 1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write register address 0x%X for reading: %s", reg_addr, strerror(errno));
        return false;
    }

    // Read the data
    if (::read(i2c_fd_, buffer, length) != length) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read %d bytes from MPU6050 register 0x%X: %s", length, reg_addr, strerror(errno));
        return false;
    }
    return true;
}


int16_t MPU6050PublisherNode::read_word_2c_from_mpu6050(uint8_t reg_addr, bool& success) {
    uint8_t buffer[2];
    success = false;
    if (read_bytes_from_mpu6050(reg_addr, buffer, 2)) {
        int16_t val = (buffer[0] << 8) | buffer[1]; // Big-endian: MSB first
        success = true;
        return val;
    }
    return 0; // Return 0 on failure
}


MPU6050PublisherNode::RawImuData MPU6050PublisherNode::get_mpu6050_data_raw() {
    RawImuData raw_data = {0,0,0,0,0,0, false};
    bool read_ok;

    if (!mpu6050_initialized_.load()) {
        RCLCPP_WARN(this->get_logger(), "Attempting to read data, but MPU6050 not initialized. Trying to re-initialize...");
        if (!initialize_mpu6050()) {
            RCLCPP_ERROR(this->get_logger(), "Re-initialization failed. Cannot read data.");
            return raw_data;
        }
    }

    raw_data.accel_x = read_word_2c_from_mpu6050(ACCEL_XOUT_H, read_ok);
    if (!read_ok) { mpu6050_initialized_ = false; return raw_data; }
    raw_data.accel_y = read_word_2c_from_mpu6050(ACCEL_XOUT_H + 2, read_ok);
    if (!read_ok) { mpu6050_initialized_ = false; return raw_data; }
    raw_data.accel_z = read_word_2c_from_mpu6050(ACCEL_XOUT_H + 4, read_ok);
    if (!read_ok) { mpu6050_initialized_ = false; return raw_data; }

    raw_data.gyro_x = read_word_2c_from_mpu6050(GYRO_XOUT_H, read_ok);
    if (!read_ok) { mpu6050_initialized_ = false; return raw_data; }
    raw_data.gyro_y = read_word_2c_from_mpu6050(GYRO_XOUT_H + 2, read_ok);
    if (!read_ok) { mpu6050_initialized_ = false; return raw_data; }
    raw_data.gyro_z = read_word_2c_from_mpu6050(GYRO_XOUT_H + 4, read_ok);
    if (!read_ok) { mpu6050_initialized_ = false; return raw_data; }
    
    raw_data.success = true;
    return raw_data;
}

void MPU6050PublisherNode::publish_imu_data() {
    if (shutdown_requested_.load()) return;

    RawImuData raw_data = get_mpu6050_data_raw();

    if (!raw_data.success) {
        RCLCPP_WARN(this->get_logger(), "Failed to get raw IMU data. Skipping publish cycle.");
        // Attempt to re-initialize on next cycle due to mpu6050_initialized_ being false
        return;
    }

    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = this->get_clock()->now();
    imu_msg->header.frame_id = imu_frame_id_;

    // Convert raw accelerometer data to m/s^2
    imu_msg->linear_acceleration.x = (static_cast<double>(raw_data.accel_x) / accel_scale_factor_) * STANDARD_GRAVITY;
    imu_msg->linear_acceleration.y = (static_cast<double>(raw_data.accel_y) / accel_scale_factor_) * STANDARD_GRAVITY;
    imu_msg->linear_acceleration.z = (static_cast<double>(raw_data.accel_z) / accel_scale_factor_) * STANDARD_GRAVITY;

    // Convert raw gyroscope data to rad/s
    imu_msg->angular_velocity.x = (static_cast<double>(raw_data.gyro_x) / gyro_scale_factor_) * (M_PI / 180.0);
    imu_msg->angular_velocity.y = (static_cast<double>(raw_data.gyro_y) / gyro_scale_factor_) * (M_PI / 180.0);
    imu_msg->angular_velocity.z = (static_cast<double>(raw_data.gyro_z) / gyro_scale_factor_) * (M_PI / 180.0);

    // Orientation is not provided by MPU6050 directly without sensor fusion
    imu_msg->orientation.x = 0.0;
    imu_msg->orientation.y = 0.0;
    imu_msg->orientation.z = 0.0;
    imu_msg->orientation.w = 1.0; // Representing unknown orientation
    imu_msg->orientation_covariance[0] = -1.0; // Mark orientation as not available

    // Placeholder covariances (as in Python script)
    // Linear acceleration covariance
    imu_msg->linear_acceleration_covariance[0] = 0.01; // Example: 0.01 m^2/s^4 variance on X
    imu_msg->linear_acceleration_covariance[4] = 0.01; // Y
    imu_msg->linear_acceleration_covariance[8] = 0.01; // Z
    // Angular velocity covariance
    imu_msg->angular_velocity_covariance[0] = 0.0025; // Example: 0.0025 rad^2/s^2 variance on X
    imu_msg->angular_velocity_covariance[4] = 0.0025; // Y
    imu_msg->angular_velocity_covariance[8] = 0.0025; // Z

    imu_publisher_->publish(std::move(imu_msg));
}
// Main function removed as this will be compiled as a library
