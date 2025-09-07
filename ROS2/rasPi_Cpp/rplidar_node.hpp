#ifndef RPLIDAR_NODE_HPP_
#define RPLIDAR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

// RPLiDAR communication constants
#define RPLIDAR_CMD_STOP                0x25
#define RPLIDAR_CMD_RESET               0x40
#define RPLIDAR_CMD_SCAN                0x20
#define RPLIDAR_CMD_EXPRESS_SCAN        0x82
#define RPLIDAR_CMD_GET_INFO            0x50
#define RPLIDAR_CMD_GET_HEALTH          0x52

#define RPLIDAR_ANS_TYPE_MEASUREMENT    0x81
#define RPLIDAR_ANS_TYPE_DEVINFO        0x04
#define RPLIDAR_ANS_TYPE_DEVHEALTH      0x06

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

class RplidarNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new RplidarNode object
     * @param options Node options for rclcpp
     */
    explicit RplidarNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Destroy the RplidarNode object
     */
    virtual ~RplidarNode();

private:
    // ROS2 specific members
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;

    // Parameters
    std::string serial_port_;
    int serial_baudrate_;
    std::string frame_id_;
    bool inverted_;
    double angle_compensate_;
    double scan_frequency_;
    double range_min_;
    double range_max_;
    bool auto_start_;

    // Serial communication
    int serial_fd_ = -1;
    std::atomic<bool> scanning_;
    std::atomic<bool> should_stop_;
    std::thread scan_thread_;
    
    // Scan data
    struct ScanPoint {
        double angle;
        double distance;
        uint8_t quality;
    };
    
    std::vector<ScanPoint> scan_points_;
    std::mutex scan_mutex_;
    rclcpp::Time last_scan_time_;

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
     * @brief Service callback to start/stop the lidar scanning.
     * @param request Service request with data field (true=start, false=stop).
     * @param response Service response with success status and message.
     */
    void start_stop_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /**
     * @brief Timer callback to publish laser scan data.
     */
    void publish_scan();

    /**
     * @brief Initializes the RPLiDAR by opening serial port and checking device info.
     * @return true if initialization is successful, false otherwise.
     */
    bool init_rplidar();

    /**
     * @brief Opens the serial port with specified settings.
     * @return true on success, false on failure.
     */
    bool open_serial_port();

    /**
     * @brief Closes the serial port.
     */
    void close_serial_port();

    /**
     * @brief Sends a command to the RPLiDAR.
     * @param cmd Command byte to send.
     * @return true on success, false on failure.
     */
    bool send_command(uint8_t cmd);

    /**
     * @brief Waits for and reads the response header.
     * @param response_type Expected response type.
     * @param response_length Output for response data length.
     * @return true if valid header received, false otherwise.
     */
    bool wait_response_header(uint8_t response_type, uint32_t& response_length);

    /**
     * @brief Gets device information from RPLiDAR.
     * @return true on success, false on failure.
     */
    bool get_device_info();

    /**
     * @brief Gets device health status from RPLiDAR.
     * @return true if device is healthy, false otherwise.
     */
    bool get_device_health();

    /**
     * @brief Starts scanning mode.
     * @return true on success, false on failure.
     */
    bool start_scan();

    /**
     * @brief Stops scanning mode.
     * @return true on success, false on failure.
     */
    bool stop_scan();

    /**
     * @brief Thread function that continuously reads scan data.
     */
    void scan_thread_func();

    /**
     * @brief Reads a single measurement packet from the RPLiDAR.
     * @param angle Output angle in degrees.
     * @param distance Output distance in meters.
     * @param quality Output signal quality.
     * @return true on successful read, false on failure.
     */
    bool read_measurement(double& angle, double& distance, uint8_t& quality);

    /**
     * @brief Converts accumulated scan points to a LaserScan message.
     * @return LaserScan message.
     */
    sensor_msgs::msg::LaserScan create_laser_scan_msg();

    // --- Serial Helper Methods ---

    /**
     * @brief Writes data to the serial port.
     * @param data Pointer to data buffer.
     * @param length Number of bytes to write.
     * @return true on success, false on failure.
     */
    bool serial_write(const uint8_t* data, size_t length);

    /**
     * @brief Reads data from the serial port.
     * @param buffer Pointer to buffer to store read data.
     * @param length Number of bytes to read.
     * @return Number of bytes actually read, -1 on error.
     */
    int serial_read(uint8_t* buffer, size_t length);

    /**
     * @brief Reads data from serial port with timeout.
     * @param buffer Pointer to buffer to store read data.
     * @param length Number of bytes to read.
     * @param timeout_ms Timeout in milliseconds.
     * @return Number of bytes actually read, -1 on error.
     */
    int serial_read_timeout(uint8_t* buffer, size_t length, int timeout_ms);
};

#endif // RPLIDAR_NODE_HPP_