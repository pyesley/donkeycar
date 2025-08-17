#include "rpi_ros2_cpp_nodes/imu_bmi088_node.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include <chrono>
#include <thread>

// --- Accelerometer Registers (from Python script) ---
#define ACC_CHIP_ID     0x00
#define ACC_X_LSB       0x12
#define ACC_CONF        0x40
#define ACC_RANGE       0x41
#define ACC_PWR_CTRL    0x7D
#define ACC_SOFTRESET   0x7E

// --- Gyroscope Registers (from Python script) ---
#define GYR_CHIP_ID     0x00
#define GYR_RATE_X_LSB  0x02
#define GYR_RANGE       0x0F
#define GYR_BANDWIDTH   0x10
#define GYR_LPM1        0x11

// --- Configuration Constants (from Python script) ---
// Accel: 100 Hz, "Normal" bandwidth (bwp=0x0A), ODR=0x08 -> 0xA8 total
#define ACC_CONF_VALUE 0xA8
// Accel Range: ±6g
#define ACC_RANGE_SEL 0x01
// Gyro: 100 Hz ODR, ~32 Hz BW
#define GYR_BW_VALUE 0x07
// Gyro Range: ±2000 dps
#define GYR_RANGE_SEL 0x00


ImuBmi088Node::ImuBmi088Node(const rclcpp::NodeOptions & options)
    : Node("imu_bmi088_node", options) {

    declare_parameters();
    load_parameters();

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw_bmi088", 10);

    double timer_period_sec = 1.0 / publish_rate_hz_;
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_sec),
        std::bind(&ImuBmi088Node::publish_imu_data, this));

    RCLCPP_INFO(this->get_logger(), "BMI088 Publisher node starting. I2C Bus: %d", i2c_bus_num_);

    if (!init_bmi088_sensor()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize BMI088 sensor. Node will attempt to re-initialize periodically.");
    }
}

ImuBmi088Node::~ImuBmi088Node() {
    RCLCPP_INFO(this->get_logger(), "BMI088 Publisher node shutting down...");
    if (timer_) {
        timer_->cancel();
    }
    close_i2c();
    RCLCPP_INFO(this->get_logger(), "BMI088 node shutdown complete.");
}

void ImuBmi088Node::declare_parameters() {
    this->declare_parameter<int>("i2c_bus", 1);
    this->declare_parameter<double>("publish_rate_hz", 100.0);
    this->declare_parameter<std::string>("imu_frame_id", "imu_link_bmi088");
    // Default scaling factors from Python script
    this->declare_parameter<double>("accel_lsb_per_g", 5460.0);   // for ±6g
    this->declare_parameter<double>("gyro_lsb_per_dps", 16.384); // for ±2000 dps
    this->declare_parameter<double>("gyro_bias_measurement_time_sec", 2.0);
}

void ImuBmi088Node::load_parameters() {
    this->get_parameter("i2c_bus", i2c_bus_num_);
    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    this->get_parameter("imu_frame_id", imu_frame_id_);
    this->get_parameter("accel_lsb_per_g", accel_lsb_per_g_);
    this->get_parameter("gyro_lsb_per_dps", gyro_lsb_per_dps_);
    this->get_parameter("gyro_bias_measurement_time_sec", gyro_bias_measurement_time_sec_);
}

bool ImuBmi088Node::autodetect_addresses() {
    // Probe for Accelerometer at 0x18 and 0x19
    accel_addr_ = -1;
    for (int addr : {0x18, 0x19}) {
        uint8_t chip_id;
        if (i2c_read_bytes(addr, ACC_CHIP_ID, &chip_id, 1)) {
            accel_addr_ = addr;
            RCLCPP_INFO(this->get_logger(), "Found BMI088 Accelerometer at 0x%X (Chip ID: 0x%X)", addr, chip_id);
            break;
        }
    }
    if (accel_addr_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "BMI088 Accelerometer not found at 0x18 or 0x19.");
        return false;
    }

    // Probe for Gyroscope at 0x68 and 0x69
    gyro_addr_ = -1;
    for (int addr : {0x68, 0x69}) {
        uint8_t chip_id;
        if (i2c_read_bytes(addr, GYR_CHIP_ID, &chip_id, 1)) {
            gyro_addr_ = addr;
            RCLCPP_INFO(this->get_logger(), "Found BMI088 Gyroscope at 0x%X (Chip ID: 0x%X)", addr, chip_id);
            break;
        }
    }
    if (gyro_addr_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "BMI088 Gyroscope not found at 0x68 or 0x69.");
        return false;
    }
    return true;
}

bool ImuBmi088Node::init_accel() {
    // Soft reset, then wait
    if (!i2c_write_byte(accel_addr_, ACC_SOFTRESET, 0xB6)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // Enable accel
    if (!i2c_write_byte(accel_addr_, ACC_PWR_CTRL, 0x04)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    // Set range and config (ODR/BW)
    if (!i2c_write_byte(accel_addr_, ACC_RANGE, ACC_RANGE_SEL)) return false;
    if (!i2c_write_byte(accel_addr_, ACC_CONF, ACC_CONF_VALUE)) return false;
    
    RCLCPP_INFO(this->get_logger(), "Accelerometer configured successfully.");
    return true;
}

bool ImuBmi088Node::init_gyro() {
    // Wake to normal mode
    if (!i2c_write_byte(gyro_addr_, GYR_LPM1, 0x00)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Set bandwidth and range
    if (!i2c_write_byte(gyro_addr_, GYR_BANDWIDTH, GYR_BW_VALUE)) return false;
    if (!i2c_write_byte(gyro_addr_, GYR_RANGE, GYR_RANGE_SEL)) return false;
    
    RCLCPP_INFO(this->get_logger(), "Gyroscope configured successfully.");
    return true;
}

void ImuBmi088Node::measure_gyro_bias() {
    RCLCPP_INFO(this->get_logger(), "Measuring gyro bias... keep sensor still for %.1f seconds.", gyro_bias_measurement_time_sec_);
    
    long long samples = static_cast<long long>(gyro_bias_measurement_time_sec_ * publish_rate_hz_);
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    
    if (samples <= 0) {
        RCLCPP_WARN(this->get_logger(), "Invalid sample count for bias measurement. Skipping.");
        return;
    }

    for (long long i = 0; i < samples; ++i) {
        int16_t gx_raw = read_word_le(gyro_addr_, GYR_RATE_X_LSB);
        int16_t gy_raw = read_word_le(gyro_addr_, GYR_RATE_X_LSB + 2);
        int16_t gz_raw = read_word_le(gyro_addr_, GYR_RATE_X_LSB + 4);
        
        sum_x += static_cast<double>(gx_raw);
        sum_y += static_cast<double>(gy_raw);
        sum_z += static_cast<double>(gz_raw);
        
        std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / publish_rate_hz_));
    }
    
    gyro_bias_x_ = sum_x / samples;
    gyro_bias_y_ = sum_y / samples;
    gyro_bias_z_ = sum_z / samples;
    
    RCLCPP_INFO(this->get_logger(), "Gyro bias (raw LSB): [x: %.2f, y: %.2f, z: %.2f]", gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
}


bool ImuBmi088Node::init_bmi088_sensor() {
    std::string i2c_bus_filename = "/dev/i2c-" + std::to_string(i2c_bus_num_);
    i2c_fd_ = open(i2c_bus_filename.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus %s: %s", i2c_bus_filename.c_str(), strerror(errno));
        return false;
    }

    // Give the I2C bus and sensor a moment to stabilize on startup.
    // This can help prevent "Remote I/O error" on the first attempt.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!autodetect_addresses()) {
        close_i2c();
        return false;
    }

    if (!init_accel()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize accelerometer.");
        close_i2c();
        return false;
    }

    if (!init_gyro()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize gyroscope.");
        close_i2c();
        return false;
    }
    
    measure_gyro_bias();

    bmi088_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "BMI088 initialized successfully.");
    return true;
}

void ImuBmi088Node::close_i2c() {
    if (i2c_fd_ != -1) {
        RCLCPP_INFO(this->get_logger(), "Closing I2C bus.");
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
    bmi088_initialized_ = false;
}

bool ImuBmi088Node::i2c_write_byte(int device_addr, uint8_t reg_addr, uint8_t data) {
    if (i2c_fd_ < 0) return false;
    if (ioctl(i2c_fd_, I2C_SLAVE, device_addr) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to set I2C slave address 0x%X: %s", device_addr, strerror(errno));
        return false;
    }
    uint8_t buffer[2] = {reg_addr, data};
    if (write(i2c_fd_, buffer, 2) != 2) {
        RCLCPP_WARN(this->get_logger(), "I2C write to addr 0x%X, reg 0x%X failed: %s", device_addr, reg_addr, strerror(errno));
        return false;
    }
    return true;
}

bool ImuBmi088Node::i2c_read_bytes(int device_addr, uint8_t reg_addr, uint8_t* read_buffer, uint8_t length) {
    if (i2c_fd_ < 0) return false;
    if (ioctl(i2c_fd_, I2C_SLAVE, device_addr) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to set I2C slave address 0x%X: %s", device_addr, strerror(errno));
        return false;
    }

    if (write(i2c_fd_, &reg_addr, 1) != 1) {
        RCLCPP_WARN(this->get_logger(), "I2C write of reg addr 0x%X for read failed: %s", reg_addr, strerror(errno));
        return false;
    }
    if (read(i2c_fd_, read_buffer, length) != length) {
        RCLCPP_WARN(this->get_logger(), "I2C read from addr 0x%X, reg 0x%X failed: %s", device_addr, reg_addr, strerror(errno));
        return false;
    }
    return true;
}

int16_t ImuBmi088Node::read_word_le(int device_addr, uint8_t reg_addr) {
    uint8_t buffer[2];
    if (!i2c_read_bytes(device_addr, reg_addr, buffer, 2)) {
        return 0; // Error already logged
    }
    // Data is little-endian: LSB first, then MSB
    return static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
}

bool ImuBmi088Node::get_bmi088_data_raw(double& ax, double& ay, double& az,
                                        double& gx, double& gy, double& gz) {
    if (!bmi088_initialized_ || i2c_fd_ < 0) {
        return false;
    }

    int16_t ax_raw = read_word_le(accel_addr_, ACC_X_LSB);
    int16_t ay_raw = read_word_le(accel_addr_, ACC_X_LSB + 2);
    int16_t az_raw = read_word_le(accel_addr_, ACC_X_LSB + 4);

    int16_t gx_raw = read_word_le(gyro_addr_, GYR_RATE_X_LSB);
    int16_t gy_raw = read_word_le(gyro_addr_, GYR_RATE_X_LSB + 2);
    int16_t gz_raw = read_word_le(gyro_addr_, GYR_RATE_X_LSB + 4);

    // Convert to physical units
    ax = (static_cast<double>(ax_raw) / accel_lsb_per_g_) * STANDARD_GRAVITY;
    ay = (static_cast<double>(ay_raw) / accel_lsb_per_g_) * STANDARD_GRAVITY;
    az = (static_cast<double>(az_raw) / accel_lsb_per_g_) * STANDARD_GRAVITY;

    // Apply bias correction and convert to rad/s
    gx = ((static_cast<double>(gx_raw) - gyro_bias_x_) / gyro_lsb_per_dps_) * (M_PI / 180.0);
    gy = ((static_cast<double>(gy_raw) - gyro_bias_y_) / gyro_lsb_per_dps_) * (M_PI / 180.0);
    gz = ((static_cast<double>(gz_raw) - gyro_bias_z_) / gyro_lsb_per_dps_) * (M_PI / 180.0);

    return true;
}

void ImuBmi088Node::publish_imu_data() {
    if (!bmi088_initialized_) {
        // Try to reinitialize if it failed before or bus closed
        if (i2c_fd_ < 0) {
            if (!init_bmi088_sensor()){
                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Re-initialization attempt failed. Will try again later.");
            }
        }
        return;
    }

    double ax, ay, az, gx, gy, gz;
    if (get_bmi088_data_raw(ax, ay, az, gx, gy, gz)) {
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
        imu_msg->header.stamp = this->get_clock()->now();
        imu_msg->header.frame_id = imu_frame_id_;

        imu_msg->linear_acceleration.x = ax;
        imu_msg->linear_acceleration.y = ay;
        imu_msg->linear_acceleration.z = az;
        imu_msg->linear_acceleration_covariance[0] = 0.01;
        imu_msg->linear_acceleration_covariance[4] = 0.01;
        imu_msg->linear_acceleration_covariance[8] = 0.01;

        imu_msg->angular_velocity.x = gx;
        imu_msg->angular_velocity.y = gy;
        imu_msg->angular_velocity.z = gz;
        imu_msg->angular_velocity_covariance[0] = 0.0025;
        imu_msg->angular_velocity_covariance[4] = 0.0025;
        imu_msg->angular_velocity_covariance[8] = 0.0025;

        // Set orientation_covariance[0] to -1 to indicate orientation is not available
        imu_msg->orientation.w = 1.0;
        imu_msg->orientation_covariance[0] = -1.0;

        imu_publisher_->publish(std::move(imu_msg));
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to get BMI088 data. Attempting to re-init.");
        close_i2c();
    }
}
