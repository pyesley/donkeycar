#ifndef PIDOG_ROS__MOTION_NODE_HPP_
#define PIDOG_ROS__MOTION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>
#include <vector>
#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <cmath>

namespace pidog_ros {

/**
 * @brief Servo class - Controls a single servo via Fusion HAT+ PWM sysfs
 */
class Servo {
public:
    static constexpr int MAX_PW = 2500;  // Max pulse width (microseconds)
    static constexpr int MIN_PW = 500;   // Min pulse width (microseconds)
    static constexpr int FREQ = 50;      // PWM frequency (Hz)
    static constexpr int PERIOD = 20000; // Period in microseconds (1/50Hz)

    explicit Servo(int channel);
    ~Servo();

    void set_angle(double angle);
    double get_angle() const { return current_angle_; }
    void set_offset(double offset) { offset_ = offset; }

private:
    int channel_;
    double offset_ = 0.0;
    double current_angle_ = 0.0;
    std::string pwm_path_;

    void enable(bool enable);
    void set_pulse_width(int pulse_width_us);
    static double constrain(double value, double min_val, double max_val);
    static double map_value(double value, double in_min, double in_max, double out_min, double out_max);
};

/**
 * @brief Walk gait generator - creates walking motion coordinates
 */
class WalkGait {
public:
    enum Direction { FORWARD = 1, BACKWARD = -1 };
    enum Turn { LEFT = -1, STRAIGHT = 0, RIGHT = 1 };

    static constexpr int SECTION_COUNT = 8;
    static constexpr int STEP_COUNT = 6;
    static constexpr double LEG_STEP_HEIGHT = 20.0;
    static constexpr double LEG_STEP_WIDTH = 80.0;
    static constexpr double CENTER_OF_GRAVITY = -15.0;
    static constexpr double Z_ORIGIN = 80.0;
    static constexpr double TURNING_RATE = 0.1;

    WalkGait(Direction fb, Turn lr);

    // Get all coordinates for one complete walk cycle
    std::vector<std::array<std::array<double, 2>, 4>> get_coords();

private:
    Direction fb_;
    Turn lr_;
    double y_offset_;
    std::array<double, 4> leg_step_width_;
    std::array<double, 4> section_length_;
    std::array<double, 4> step_down_length_;
    std::array<double, 4> leg_origin_;

    static constexpr std::array<int, 8> LEG_ORDER = {1, 0, 4, 0, 2, 0, 3, 0};
    static constexpr std::array<double, 4> LEG_POSITION_OFFSETS = {-10, -10, 20, 20};
    static constexpr std::array<int, 4> LEG_ORIGINAL_Y_TABLE = {0, 2, 3, 1};

    double step_y_func(int leg, int step);
    double step_z_func(int step);
};

/**
 * @brief Inverse kinematics for leg coordinates to servo angles
 */
class LegKinematics {
public:
    static constexpr double LEG_LENGTH = 42.0;   // Upper leg length (mm)
    static constexpr double FOOT_LENGTH = 76.0;  // Lower leg length (mm)

    // Convert (y, z) coordinate to (leg_angle, foot_angle) in degrees
    static std::pair<double, double> coord_to_polar(double y, double z);

    // Convert 4 leg coordinates to 8 servo angles
    static std::array<double, 8> legs_angle_calculation(
        const std::array<std::array<double, 2>, 4>& coords);
};

/**
 * @brief MotionNode - ROS2 node for PiDog motion control
 *
 * Subscribes:
 *   - /pidog/cmd_vel (geometry_msgs/Twist) - Velocity commands
 *   - /pidog/cmd (std_msgs/String) - Discrete commands
 *
 * Publishes:
 *   - /pidog/motion/status (std_msgs/String) - Motion status
 */
class MotionNode : public rclcpp::Node {
public:
    explicit MotionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MotionNode();

private:
    // Servo pin mappings (from pidog_fusion.py)
    static constexpr std::array<int, 8> LEGS_PINS = {2, 3, 7, 8, 0, 1, 10, 11};
    static constexpr std::array<int, 3> HEAD_PINS = {4, 6, 5};  // yaw, roll, pitch
    static constexpr int TAIL_PIN = 9;

    // Motion state
    enum class State { IDLE, STANDING, WALKING, SITTING, LYING };
    std::atomic<State> state_{State::IDLE};
    std::atomic<bool> is_moving_{false};
    std::mutex motion_mutex_;

    // Velocity commands
    double target_linear_x_ = 0.0;
    double target_angular_z_ = 0.0;
    std::mutex cmd_mutex_;
    rclcpp::Time last_cmd_time_;
    double cmd_timeout_ = 0.5;

    // Servos
    std::vector<std::unique_ptr<Servo>> leg_servos_;
    std::vector<std::unique_ptr<Servo>> head_servos_;
    std::unique_ptr<Servo> tail_servo_;
    std::array<double, 8> leg_offsets_{};
    std::array<double, 3> head_offsets_{};
    double tail_offset_ = 0.0;
    bool hardware_initialized_ = false;

    // Parameters
    int walk_speed_ = 50;
    int step_count_ = 2;

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr motion_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Initialization
    void declare_parameters();
    void load_parameters();
    bool init_hardware();
    void load_calibration();

    // Callbacks
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void cmd_callback(const std_msgs::msg::String::SharedPtr msg);
    void motion_loop();
    void status_timer_callback();

    // Motion commands
    void do_stand();
    void do_sit();
    void do_lie();
    void do_stretch();
    void do_wag();
    void do_wave();
    void do_shake();
    void do_pushup();
    void do_stop();

    void do_walk_forward(int steps);
    void do_walk_backward(int steps);
    void do_turn_left(int steps);
    void do_turn_right(int steps);
    void execute_walk(WalkGait::Direction fb, WalkGait::Turn lr, int steps);

    // Low-level servo control
    void set_leg_angles(const std::array<double, 8>& angles);
    void set_head_angles(double yaw, double roll, double pitch);
    void set_tail_angle(double angle);

    // Utility
    void publish_status(const std::string& status);
    static void sleep_ms(int ms);
};

}  // namespace pidog_ros

#endif  // PIDOG_ROS__MOTION_NODE_HPP_
