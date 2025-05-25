#ifndef MPU6050_PUBLISHER_NODE_HPP_
#define MPU6050_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector> // For std::vector in read/write operations if needed
#include <atomic> // For std::atomic_bool

// MPU6050 Register Addresses
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B; // Base address for accelerometer data
const uint8_t GYRO_XOUT_H = 0x43;  // Base address for gyroscope data

const double STANDARD_GRAVITY = 9.80665;

class MPU6050PublisherNode : public rclcpp::Node {
public:
    MPU6050PublisherNode();
    ~MPU6050PublisherNode();
    void shutdown();

private:
    // Parameters
    int i2c_bus_number_;
    int mpu6050_address_;
    double publish_rate_hz_;
    std::string imu_frame_id_;
    double accel_scale_factor_;
    double gyro_scale_factor_;

    // ROS Publisher
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // I2C
    int i2c_fd_; // File descriptor for I2C bus
    std::string i2c_device_name_;
    std::atomic_bool mpu6050_initialized_;
    std::atomic_bool shutdown_requested_;


    // Methods
    void declare_parameters();
    void get_parameters();
    bool initialize_mpu6050();
    void close_mpu6050();
    bool read_bytes_from_mpu6050(uint8_t reg_addr, uint8_t* buffer, int length);
    bool write_byte_to_mpu6050(uint8_t reg_addr, uint8_t data);
    int16_t read_word_2c_from_mpu6050(uint8_t reg_addr, bool& success);
    
    struct RawImuData {
        int16_t accel_x, accel_y, accel_z;
        int16_t gyro_x, gyro_y, gyro_z;
        bool success;
    };
    RawImuData get_mpu6050_data_raw();
    void publish_imu_data();
};

#endif // MPU6050_PUBLISHER_NODE_HPP_
