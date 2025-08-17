#ifndef IMU_BMI088_NODE_HPP_
#define IMU_BMI088_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <memory>
#include <string>
#include <chrono>
#include <cmath>

// Standard Gravity for unit conversion
constexpr double STANDARD_GRAVITY = 9.80665;

class ImuBmi088Node : public rclcpp::Node {
public:
    /**
     * @brief Construct a new ImuBmi088Node object
     * @param options Node options for rclcpp
     */
    explicit ImuBmi088Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Destroy the ImuBmi088Node object
     */
    virtual ~ImuBmi088Node();

private:
    // ROS2 specific members
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    int i2c_bus_num_;
    int accel_addr_;
    int gyro_addr_;
    double publish_rate_hz_;
    std::string imu_frame_id_;
    double accel_lsb_per_g_;
    double gyro_lsb_per_dps_;
    double gyro_bias_measurement_time_sec_;

    // I2C and sensor state
    int i2c_fd_ = -1;
    bool bmi088_initialized_ = false;
    double gyro_bias_x_ = 0.0;
    double gyro_bias_y_ = 0.0;
    double gyro_bias_z_ = 0.0;

    // --- Methods ---

    /**
     * @brief Declares ROS2 parameters for the node.
     */
    void declare_parameters();

    /**
     * @brief Loads ROS2 parameters into member variables.
     */
    void load_parameters();

    /**
     * @brief Timer callback to read from the sensor and publish the IMU message.
     */
    void publish_imu_data();

    /**
     * @brief Initializes the BMI088 sensor by configuring both accelerometer and gyroscope.
     * @return true if initialization is successful, false otherwise.
     */
    bool init_bmi088_sensor();

    /**
     * @brief Initializes the accelerometer part of the BMI088.
     * @return true on success, false on failure.
     */
    bool init_accel();

    /**
     * @brief Initializes the gyroscope part of the BMI088.
     * @return true on success, false on failure.
     */
    bool init_gyro();

    /**
     * @brief Auto-detects the I2C addresses for the accelerometer and gyroscope.
     * @return true if both are found, false otherwise.
     */
    bool autodetect_addresses();
    
    /**
     * @brief Measures the gyroscope bias by averaging readings over a short period.
     */
    void measure_gyro_bias();

    /**
     * @brief Reads raw data from the sensor and converts it to physical units.
     * @param ax Output for acceleration in X (m/s^2).
     * @param ay Output for acceleration in Y (m/s^2).
     * @param az Output for acceleration in Z (m/s^2).
     * @param gx Output for angular velocity in X (rad/s).
     * @param gy Output for angular velocity in Y (rad/s).
     * @param gz Output for angular velocity in Z (rad/s).
     * @return true on successful read, false on failure.
     */
    bool get_bmi088_data_raw(double& ax, double& ay, double& az,
                               double& gx, double& gy, double& gz);

    /**
     * @brief Closes the I2C file descriptor.
     */
    void close_i2c();

    // --- I2C Helper Methods ---

    /**
     * @brief Writes a single byte to a specific register on an I2C device.
     * @param device_addr The I2C address of the target device.
     * @param reg_addr The register to write to.
     * @param data The byte of data to write.
     * @return true on success, false on failure.
     */
    bool i2c_write_byte(int device_addr, uint8_t reg_addr, uint8_t data);

    /**
     * @brief Reads a block of bytes from a specific register on an I2C device.
     * @param device_addr The I2C address of the target device.
     * @param reg_addr The starting register to read from.
     * @param read_buffer Pointer to the buffer to store read data.
     * @param length The number of bytes to read.
     * @return true on success, false on failure.
     */
    bool i2c_read_bytes(int device_addr, uint8_t reg_addr, uint8_t* read_buffer, uint8_t length);

    /**
     * @brief Reads a 16-bit signed integer from the device (little-endian).
     * @param device_addr The I2C address of the target device.
     * @param reg_addr The starting register address (LSB).
     * @return The 16-bit signed value.
     */
    int16_t read_word_le(int device_addr, uint8_t reg_addr);
};

#endif // IMU_BMI088_NODE_HPP_
