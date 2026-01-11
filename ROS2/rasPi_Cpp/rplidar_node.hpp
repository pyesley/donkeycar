#ifndef RPI_ROS2_CPP_NODES__RPLIDAR_NODE_HPP_
#define RPI_ROS2_CPP_NODES__RPLIDAR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

// RPLiDAR Protocol Constants
#define RPLIDAR_CMD_STOP            0x25
#define RPLIDAR_CMD_SCAN            0x20
#define RPLIDAR_CMD_GET_INFO        0x50
#define RPLIDAR_CMD_GET_HEALTH      0x52

#define RPLIDAR_ANS_TYPE_MEASUREMENT      0x81
#define RPLIDAR_ANS_TYPE_DEVINFO          0x04
#define RPLIDAR_ANS_TYPE_DEVHEALTH        0x06

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

class RplidarNode : public rclcpp::Node {
public:
    explicit RplidarNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~RplidarNode();

private:
    // ROS Parameters
    std::string serial_port_;
    int serial_baudrate_;
    std::string frame_id_;
    bool inverted_;
    double angle_compensate_;
    double scan_frequency_;
    double range_min_;
    double range_max_;
    bool auto_start_;

    // ROS Communications
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Serial & Device State
    int serial_fd_ = -1;
    std::atomic<bool> scanning_;
    std::atomic<bool> should_stop_;

    // Data Storage
    struct ScanPoint {
        double angle;
        double distance;
        uint8_t quality;
    };
    std::vector<ScanPoint> scan_points_;
    std::mutex scan_mutex_;
    std::thread scan_thread_;

    // Methods
    void declare_parameters();
    void load_parameters();
    void start_stop_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    void publish_scan();
    sensor_msgs::msg::LaserScan create_laser_scan_msg();
    
    // Hardware Control
    bool open_serial_port();
    void close_serial_port();
    
    // Low-level Communication (Updated)
    bool serial_write(const uint8_t* data, size_t length);
    int read_exact(uint8_t* buffer, size_t length, int timeout_ms); // <--- NEW FUNCTION
    bool send_command(uint8_t cmd);
    bool wait_response_header(uint8_t response_type, uint32_t& response_length);
    
    // Logic
    bool get_device_info();
    bool get_device_health();
    bool start_scan();
    bool stop_scan();
    void scan_thread_func();
};

#endif // RPI_ROS2_CPP_NODES__RPLIDAR_NODE_HPP_