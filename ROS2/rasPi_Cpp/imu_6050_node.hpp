#ifndef IMU_6050_NODE_HPP_
#define IMU_6050_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <chrono>
#include <cmath> // For M_PI, NAN
#include <cstdint> // For int16_t

// MPU-6050 Register Addresses
const uint8_t MPU6050_PWR_MGMT_1   = 0x6B;
const uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_GYRO_XOUT_H  = 0x43;

// Default scale factors
const double ACCEL_SCALE_FACTOR_DEFAULT = 16384.0; // LSB/g for +/- 2g
const double GYRO_SCALE_FACTOR_DEFAULT  = 131.0;   // LSB/deg/s for +/- 250 deg/s
const double STANDARD_GRAVITY = 9.80665;           // m/s^2

class Imu6050Node : public rclcpp::Node {
public:
    Imu6050Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~Imu6050Node();

private:
    // Parameters
    int i2c_bus_num_;
    int mpu6050_address_;
    double publish_rate_hz_;
    std::string imu_frame_id_;
    double accel_scale_factor_;
    double gyro_scale_factor_;

    // ROS 2 interfaces
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // I2C handling
    int i2c_fd_ = -1; // File descriptor for I2C bus
    bool mpu6050_initialized_;

    // Methods
    void declare_parameters();
    void load_parameters();

    bool init_mpu6050_sensor();
    void close_i2c();

    bool i2c_write_byte(uint8_t reg_addr, uint8_t data);
    bool i2c_read_bytes(uint8_t reg_addr, uint8_t* buffer, uint8_t length);
    int16_t read_word_2c(uint8_t reg_addr);

    void publish_imu_data();
    bool get_mpu6050_data_raw(double& ax, double& ay, double& az,
                                double& gx, double& gy, double& gz);
};

#endif // IMU_6050_NODE_HPP_
