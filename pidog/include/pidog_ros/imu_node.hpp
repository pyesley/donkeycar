#ifndef PIDOG_ROS__IMU_NODE_HPP_
#define PIDOG_ROS__IMU_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>
#include <atomic>
#include <mutex>

namespace pidog_ros {

/**
 * @brief IMUNode - ROS2 node for SH3001 6-axis IMU on Fusion HAT+
 *
 * Publishes:
 *   - pidog/imu (sensor_msgs/Imu) - IMU data at configured rate
 *   - pidog/imu/status (std_msgs/String) - Status messages
 *
 * The SH3001 provides:
 *   - 3-axis accelerometer (±2G to ±16G)
 *   - 3-axis gyroscope (±125 to ±2000 deg/s)
 *
 * Default configuration:
 *   - Accelerometer: ±8G range
 *   - Gyroscope: ±2000 deg/s range
 *   - Sample rate: 100 Hz
 */
class IMUNode : public rclcpp::Node {
public:
    explicit IMUNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~IMUNode();

private:
    // SH3001 constants
    static constexpr int DEFAULT_I2C_BUS = 1;
    static constexpr int SH3001_I2C_ADDR = 0x36;
    static constexpr uint8_t SH3001_CHIP_ID = 0x61;

    // Register addresses
    static constexpr uint8_t REG_ACC_X_LOW = 0x00;
    static constexpr uint8_t REG_ACC_X_HIGH = 0x01;
    static constexpr uint8_t REG_ACC_Y_LOW = 0x02;
    static constexpr uint8_t REG_ACC_Y_HIGH = 0x03;
    static constexpr uint8_t REG_ACC_Z_LOW = 0x04;
    static constexpr uint8_t REG_ACC_Z_HIGH = 0x05;
    static constexpr uint8_t REG_GYRO_X_LOW = 0x06;
    static constexpr uint8_t REG_GYRO_X_HIGH = 0x07;
    static constexpr uint8_t REG_GYRO_Y_LOW = 0x08;
    static constexpr uint8_t REG_GYRO_Y_HIGH = 0x09;
    static constexpr uint8_t REG_GYRO_Z_LOW = 0x0A;
    static constexpr uint8_t REG_GYRO_Z_HIGH = 0x0B;
    static constexpr uint8_t REG_CHIP_ID = 0x0F;

    // Accelerometer range settings (register 0x21)
    // Range | Sensitivity (LSB/g) | Scale (g/LSB)
    // ±2G   | 16384              | 0.00006103515625
    // ±4G   | 8192               | 0.0001220703125
    // ±8G   | 4096               | 0.000244140625
    // ±16G  | 2048               | 0.00048828125
    static constexpr double ACC_SCALE_2G = 0.00006103515625;
    static constexpr double ACC_SCALE_4G = 0.0001220703125;
    static constexpr double ACC_SCALE_8G = 0.000244140625;
    static constexpr double ACC_SCALE_16G = 0.00048828125;

    // Gyroscope range settings (register 0x22)
    // Range      | Sensitivity (LSB/deg/s) | Scale (deg/s/LSB)
    // ±125 dps   | 262.144                 | 0.003814697265625
    // ±250 dps   | 131.072                 | 0.00762939453125
    // ±500 dps   | 65.536                  | 0.0152587890625
    // ±1000 dps  | 32.768                  | 0.030517578125
    // ±2000 dps  | 16.384                  | 0.06103515625
    static constexpr double GYRO_SCALE_125 = 0.003814697265625;
    static constexpr double GYRO_SCALE_250 = 0.00762939453125;
    static constexpr double GYRO_SCALE_500 = 0.0152587890625;
    static constexpr double GYRO_SCALE_1000 = 0.030517578125;
    static constexpr double GYRO_SCALE_2000 = 0.06103515625;

    // Conversion constants
    static constexpr double G_TO_MS2 = 9.80665;  // m/s^2 per g
    static constexpr double DEG_TO_RAD = 0.017453292519943295;  // radians per degree

    // Parameters
    int i2c_bus_;
    int sample_rate_hz_;
    std::string frame_id_;
    double acc_scale_;  // g per LSB
    double gyro_scale_;  // deg/s per LSB

    // Gyroscope calibration offsets (in raw units)
    double gyro_offset_x_;
    double gyro_offset_y_;
    double gyro_offset_z_;

    // I2C file descriptor
    int i2c_fd_;
    std::atomic<bool> initialized_;

    // ROS2 interfaces
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr sample_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Statistics
    std::atomic<uint64_t> samples_published_;
    std::atomic<uint64_t> read_errors_;
    rclcpp::Time start_time_;

    // Methods
    void declare_parameters();
    void load_parameters();
    bool init_i2c();
    void close_i2c();
    bool verify_chip_id();

    uint8_t read_register(uint8_t reg);
    bool read_registers(uint8_t start_reg, uint8_t* buffer, size_t length);
    int16_t read_int16(uint8_t low_reg, uint8_t high_reg);

    void sample_timer_callback();
    void status_timer_callback();
    void publish_status(const std::string& status);

    void calibrate_gyro(int num_samples = 100);
};

}  // namespace pidog_ros

#endif  // PIDOG_ROS__IMU_NODE_HPP_
