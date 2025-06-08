#include "rpi_ros2_cpp_nodes/imu_6050_node.hpp"
#include <fcntl.h>      // For O_RDWR
#include <unistd.h>     // For open, close, read, write
#include <sys/ioctl.h>  // For ioctl
#include <linux/i2c-dev.h> // For I2C_SLAVE
#include <linux/i2c.h>
#include <string.h>     // For strerror
#include <errno.h>      // For errno
#include <vector>

Imu6050Node::Imu6050Node(const rclcpp::NodeOptions & options)
    : Node("imu_6050_node", options), mpu6050_initialized_(false) {

    declare_parameters();
    load_parameters();

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw_mpu6050", 10);

    double timer_period_sec = 1.0 / publish_rate_hz_;
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_sec),
        std::bind(&Imu6050Node::publish_imu_data, this));

    RCLCPP_INFO(this->get_logger(), "MPU-6050 Publisher node starting. I2C Bus: %d, Address: 0x%X",
                i2c_bus_num_, mpu6050_address_);

    if (!init_mpu6050_sensor()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU-6050 sensor. Node may not function correctly.");
        // Depending on severity, could throw or set a flag to prevent publishing
    }
}

Imu6050Node::~Imu6050Node() {
    RCLCPP_INFO(this->get_logger(), "MPU-6050 Publisher node shutting down...");
    if (timer_) {
        timer_->cancel();
    }
    close_i2c();
    RCLCPP_INFO(this->get_logger(), "MPU-6050 node shutdown complete.");
}

void Imu6050Node::declare_parameters() {
    this->declare_parameter<int>("i2c_bus", 1);
    this->declare_parameter<int>("mpu6050_address", 0x68);
    this->declare_parameter<double>("publish_rate_hz", 200.0);
    this->declare_parameter<std::string>("imu_frame_id", "imu_link_mpu6050");
    this->declare_parameter<double>("accel_scale_factor", ACCEL_SCALE_FACTOR_DEFAULT);
    this->declare_parameter<double>("gyro_scale_factor", GYRO_SCALE_FACTOR_DEFAULT);
}

void Imu6050Node::load_parameters() {
    this->get_parameter("i2c_bus", i2c_bus_num_);
    this->get_parameter("mpu6050_address", mpu6050_address_);
    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    this->get_parameter("imu_frame_id", imu_frame_id_);
    this->get_parameter("accel_scale_factor", accel_scale_factor_);
    this->get_parameter("gyro_scale_factor", gyro_scale_factor_);
}


bool Imu6050Node::init_mpu6050_sensor() {
    std::string i2c_bus_filename = "/dev/i2c-" + std::to_string(i2c_bus_num_);
    i2c_fd_ = open(i2c_bus_filename.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus %s: %s", i2c_bus_filename.c_str(), strerror(errno));
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, mpu6050_address_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set I2C slave address 0x%X: %s", mpu6050_address_, strerror(errno));
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // Wake up MPU-6050 (write 0 to PWR_MGMT_1 register)
    if (!i2c_write_byte(MPU6050_PWR_MGMT_1, 0x00)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to wake up MPU-6050 (write to PWR_MGMT_1).");
        close_i2c(); // This will set i2c_fd_ to -1
        return false;
    }

    mpu6050_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "MPU-6050 initialized successfully on I2C bus %d, address 0x%X.", i2c_bus_num_, mpu6050_address_);
    return true;
}

void Imu6050Node::close_i2c() {
    if (i2c_fd_ != -1) {
        RCLCPP_INFO(this->get_logger(), "Closing I2C bus.");
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
    mpu6050_initialized_ = false;
}

bool Imu6050Node::i2c_write_byte(uint8_t reg_addr, uint8_t data) {
    if (i2c_fd_ < 0) return false;
    uint8_t buffer[2] = {reg_addr, data};
    if (write(i2c_fd_, buffer, 2) != 2) {
        RCLCPP_WARN(this->get_logger(), "I2C write to reg 0x%X failed: %s", reg_addr, strerror(errno));
        return false;
    }
    return true;
}

bool Imu6050Node::i2c_read_bytes(uint8_t reg_addr, uint8_t* read_buffer, uint8_t length) {
    if (i2c_fd_ < 0) return false;

    // Write the register address to start reading from
    if (write(i2c_fd_, &reg_addr, 1) != 1) {
        RCLCPP_WARN(this->get_logger(), "I2C write of reg addr 0x%X for read failed: %s", reg_addr, strerror(errno));
        return false;
    }
    // Read bytes
    if (read(i2c_fd_, read_buffer, length) != length) {
        RCLCPP_WARN(this->get_logger(), "I2C read from reg 0x%X failed: %s", reg_addr, strerror(errno));
        return false;
    }
    return true;
}


int16_t Imu6050Node::read_word_2c(uint8_t reg_addr) {
    uint8_t high_byte, low_byte;
    if (!i2c_read_bytes(reg_addr, &high_byte, 1) || !i2c_read_bytes(reg_addr + 1, &low_byte, 1)) {
        // Error already logged by i2c_read_bytes
        // Return a value indicating error, or handle more gracefully
        // For now, let's assume it might return 0 if read fails, which is not ideal.
        // A robust implementation would throw or return an optional/status.
        return 0;
    }

    int16_t val = (static_cast<int16_t>(high_byte) << 8) | low_byte;
    // No need for 2's complement manual conversion if val is already int16_t
    return val;
}


bool Imu6050Node::get_mpu6050_data_raw(double& ax, double& ay, double& az,
                                        double& gx, double& gy, double& gz) {
    if (!mpu6050_initialized_ || i2c_fd_ < 0) {
        RCLCPP_DEBUG(this->get_logger(), "MPU-6050 not initialized or I2C FD invalid, cannot read data.");
        return false;
    }

    // Re-set slave address before each transaction block if using raw ioctl,
    // but for basic read/write on an opened fd, it should persist.
    // However, some complex I2C interactions might require it.
    // For this case, it's likely fine.
    // if (ioctl(i2c_fd_, I2C_SLAVE, mpu6050_address_) < 0) {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to re-set I2C slave address 0x%X: %s", mpu6050_address_, strerror(errno));
    //     return false;
    // }

    uint8_t accel_data[6];
    uint8_t gyro_data[6];

    if (!i2c_read_bytes(MPU6050_ACCEL_XOUT_H, accel_data, 6)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read accelerometer data.");
        // Attempt to re-initialize in case of I2C error
        close_i2c();
        init_mpu6050_sensor();
        return false;
    }
    if (!i2c_read_bytes(MPU6050_GYRO_XOUT_H, gyro_data, 6)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read gyroscope data.");
        close_i2c();
        init_mpu6050_sensor();
        return false;
    }

    int16_t accel_x_raw = (static_cast<int16_t>(accel_data[0]) << 8) | accel_data[1];
    int16_t accel_y_raw = (static_cast<int16_t>(accel_data[2]) << 8) | accel_data[3];
    int16_t accel_z_raw = (static_cast<int16_t>(accel_data[4]) << 8) | accel_data[5];

    int16_t gyro_x_raw = (static_cast<int16_t>(gyro_data[0]) << 8) | gyro_data[1];
    int16_t gyro_y_raw = (static_cast<int16_t>(gyro_data[2]) << 8) | gyro_data[3];
    int16_t gyro_z_raw = (static_cast<int16_t>(gyro_data[4]) << 8) | gyro_data[5];

    // Convert to physical units
    ax = (static_cast<double>(accel_x_raw) / accel_scale_factor_) * STANDARD_GRAVITY;
    ay = (static_cast<double>(accel_y_raw) / accel_scale_factor_) * STANDARD_GRAVITY;
    az = (static_cast<double>(accel_z_raw) / accel_scale_factor_) * STANDARD_GRAVITY;

    gx = (static_cast<double>(gyro_x_raw) / gyro_scale_factor_) * (M_PI / 180.0);
    gy = (static_cast<double>(gyro_y_raw) / gyro_scale_factor_) * (M_PI / 180.0);
    gz = (static_cast<double>(gyro_z_raw) / gyro_scale_factor_) * (M_PI / 180.0);

    return true;
}

void Imu6050Node::publish_imu_data() {
    if (!mpu6050_initialized_) {
        RCLCPP_DEBUG(this->get_logger(), "MPU-6050 not initialized. Skipping data publish.");
        // Try to reinitialize if it failed before or bus closed
        if (i2c_fd_ < 0) {
            if (!init_mpu6050_sensor()){ // Try to re-init
                 RCLCPP_WARN(this->get_logger(), "Re-initialization attempt failed. Will try again later.");
            }
        }
        return;
    }

    double ax, ay, az, gx, gy, gz;
    if (get_mpu6050_data_raw(ax, ay, az, gx, gy, gz)) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = imu_frame_id_;

        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        // Example covariances (variances on diagonal)
        imu_msg.linear_acceleration_covariance[0] = 0.01; // X
        imu_msg.linear_acceleration_covariance[4] = 0.01; // Y
        imu_msg.linear_acceleration_covariance[8] = 0.01; // Z (Indices: 0, 4, 8 for xx, yy, zz)
        // Other covariance elements (off-diagonal) are 0 if not specified

        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;
        imu_msg.angular_velocity_covariance[0] = 0.0025; // X (e.g., (0.05 rad/s)^2)
        imu_msg.angular_velocity_covariance[4] = 0.0025; // Y
        imu_msg.angular_velocity_covariance[8] = 0.0025; // Z

        // Orientation is not directly provided by MPU-6050 raw data without fusion
        // Set orientation_covariance[0] to -1 to indicate orientation is not available
        imu_msg.orientation.x = 0.0; // Or NAN
        imu_msg.orientation.y = 0.0; // Or NAN
        imu_msg.orientation.z = 0.0; // Or NAN
        imu_msg.orientation.w = 1.0; // Or NAN (Unit quaternion for unknown)
        imu_msg.orientation_covariance[0] = -1.0;

        imu_publisher_->publish(imu_msg);
        // RCLCPP_DEBUG(this->get_logger(), "Published IMU: A=(%.2f,%.2f,%.2f), G=(%.2f,%.2f,%.2f)", ax,ay,az,gx,gy,gz);
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to get MPU-6050 data. Skipping publish.");
    }
}
