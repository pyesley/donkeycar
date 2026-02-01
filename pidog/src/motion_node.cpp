#include "pidog_ros/motion_node.hpp"

#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <filesystem>

namespace pidog_ros {

// ============================================================================
// Servo Implementation
// ============================================================================

Servo::Servo(int channel) : channel_(channel) {
    pwm_path_ = "/sys/class/fusion_hat/fusion_hat/pwm/pwm" + std::to_string(channel);
    enable(true);
}

Servo::~Servo() {
    try {
        enable(false);
    } catch (...) {}
}

void Servo::enable(bool en) {
    std::ofstream f(pwm_path_ + "/enable");
    if (f.is_open()) {
        f << (en ? "1" : "0");
    }
}

void Servo::set_pulse_width(int pulse_width_us) {
    std::ofstream f(pwm_path_ + "/duty_cycle");
    if (f.is_open()) {
        f << pulse_width_us;
    }
}

double Servo::constrain(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

double Servo::map_value(double value, double in_min, double in_max, double out_min, double out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Servo::set_angle(double angle) {
    angle = constrain(angle, -90.0, 90.0);
    current_angle_ = angle;

    double calibrated_angle = angle + offset_;
    calibrated_angle = constrain(calibrated_angle, -90.0, 90.0);

    int pulse_width = static_cast<int>(map_value(calibrated_angle, -90.0, 90.0, MIN_PW, MAX_PW));
    set_pulse_width(pulse_width);
}

// ============================================================================
// WalkGait Implementation
// ============================================================================

WalkGait::WalkGait(Direction fb, Turn lr) : fb_(fb), lr_(lr) {
    y_offset_ = CENTER_OF_GRAVITY;

    // Turning scales
    static constexpr std::array<std::array<double, 4>, 3> TURN_SCALES = {{
        {TURNING_RATE, 1.0, TURNING_RATE, 1.0},  // LEFT
        {1.0, 1.0, 1.0, 1.0},                     // STRAIGHT
        {1.0, TURNING_RATE, 1.0, TURNING_RATE}   // RIGHT
    }};

    int turn_idx = lr_ + 1;  // -1 -> 0, 0 -> 1, 1 -> 2

    for (int i = 0; i < 4; i++) {
        leg_step_width_[i] = LEG_STEP_WIDTH * TURN_SCALES[turn_idx][i];
        section_length_[i] = leg_step_width_[i] / (SECTION_COUNT - 1);
        step_down_length_[i] = section_length_[i] / STEP_COUNT;
        leg_origin_[i] = leg_step_width_[i] / 2.0 + y_offset_ +
                         (LEG_POSITION_OFFSETS[i] * TURN_SCALES[turn_idx][i]);
    }
}

double WalkGait::step_y_func(int leg, int step) {
    double theta = step * M_PI / (STEP_COUNT - 1);
    double temp = (leg_step_width_[leg] * (std::cos(theta) - fb_) / 2.0 * fb_);
    return leg_origin_[leg] + temp;
}

double WalkGait::step_z_func(int step) {
    return Z_ORIGIN - (LEG_STEP_HEIGHT * step / (STEP_COUNT - 1));
}

std::vector<std::array<std::array<double, 2>, 4>> WalkGait::get_coords() {
    // Initialize origin coordinates
    std::array<std::array<double, 2>, 4> origin_leg_coord;
    for (int i = 0; i < 4; i++) {
        origin_leg_coord[i][0] = leg_origin_[i] - LEG_ORIGINAL_Y_TABLE[i] * 2.0 * section_length_[i];
        origin_leg_coord[i][1] = Z_ORIGIN;
    }

    auto leg_coord = origin_leg_coord;
    std::vector<std::array<std::array<double, 2>, 4>> leg_coords;

    for (int section = 0; section < SECTION_COUNT; section++) {
        for (int step = 0; step < STEP_COUNT; step++) {
            int raise_leg;
            if (fb_ == FORWARD) {
                raise_leg = LEG_ORDER[section];
            } else {
                raise_leg = LEG_ORDER[SECTION_COUNT - section - 1];
            }

            for (int i = 0; i < 4; i++) {
                double y, z;
                if (raise_leg != 0 && i == raise_leg - 1) {
                    y = step_y_func(i, step);
                    z = step_z_func(step);
                } else {
                    y = leg_coord[i][0] + step_down_length_[i] * fb_;
                    z = Z_ORIGIN;
                }
                leg_coord[i][0] = y;
                leg_coord[i][1] = z;
            }
            leg_coords.push_back(leg_coord);
        }
    }

    leg_coords.push_back(origin_leg_coord);
    return leg_coords;
}

// ============================================================================
// LegKinematics Implementation
// ============================================================================

std::pair<double, double> LegKinematics::coord_to_polar(double y, double z) {
    double u = std::sqrt(y * y + z * z);

    // Clamp to valid range
    double cos_angle1 = (FOOT_LENGTH * FOOT_LENGTH + LEG_LENGTH * LEG_LENGTH - u * u) /
                        (2.0 * FOOT_LENGTH * LEG_LENGTH);
    cos_angle1 = std::max(-1.0, std::min(1.0, cos_angle1));
    double beta = std::acos(cos_angle1);

    double angle1 = std::atan2(y, z);
    double cos_angle2 = (LEG_LENGTH * LEG_LENGTH + u * u - FOOT_LENGTH * FOOT_LENGTH) /
                        (2.0 * LEG_LENGTH * u);
    cos_angle2 = std::max(-1.0, std::min(1.0, cos_angle2));
    double angle2 = std::acos(cos_angle2);
    double alpha = angle2 + angle1;

    // Convert to degrees
    alpha = alpha * 180.0 / M_PI;
    beta = beta * 180.0 / M_PI;

    return {alpha, beta};
}

std::array<double, 8> LegKinematics::legs_angle_calculation(
    const std::array<std::array<double, 2>, 4>& coords) {

    std::array<double, 8> angles;

    for (int i = 0; i < 4; i++) {
        auto [leg_angle, foot_angle] = coord_to_polar(coords[i][0], coords[i][1]);
        foot_angle = foot_angle - 90.0;

        // Left and right sides are opposite
        if (i % 2 != 0) {
            leg_angle = -leg_angle;
            foot_angle = -foot_angle;
        }

        angles[i * 2] = leg_angle;
        angles[i * 2 + 1] = foot_angle;
    }

    return angles;
}

// ============================================================================
// MotionNode Implementation
// ============================================================================

MotionNode::MotionNode(const rclcpp::NodeOptions& options)
    : Node("pidog_motion_node", options) {

    declare_parameters();
    load_parameters();

    // Initialize hardware
    if (!init_hardware()) {
        RCLCPP_WARN(get_logger(), "Hardware initialization failed, running in simulation mode");
    }

    // QoS
    auto qos_cmd = rclcpp::QoS(1).reliable();

    // Subscribers
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/pidog/cmd_vel", qos_cmd,
        std::bind(&MotionNode::cmd_vel_callback, this, std::placeholders::_1));

    cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "/pidog/cmd", qos_cmd,
        std::bind(&MotionNode::cmd_callback, this, std::placeholders::_1));

    // Publisher
    status_pub_ = create_publisher<std_msgs::msg::String>("/pidog/motion/status", 10);

    // Timers
    motion_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MotionNode::motion_loop, this));

    status_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MotionNode::status_timer_callback, this));

    last_cmd_time_ = get_clock()->now();

    // Start in standing position
    if (hardware_initialized_) {
        do_stand();
    }

    RCLCPP_INFO(get_logger(), "MotionNode initialized");
    RCLCPP_INFO(get_logger(), "  Hardware: %s", hardware_initialized_ ? "yes" : "simulation");
    RCLCPP_INFO(get_logger(), "  Subscribing to: /pidog/cmd_vel, /pidog/cmd");
    RCLCPP_INFO(get_logger(), "  Publishing to: /pidog/motion/status");
}

MotionNode::~MotionNode() {
    RCLCPP_INFO(get_logger(), "MotionNode shutting down...");
    if (hardware_initialized_) {
        do_stand();
    }
}

void MotionNode::declare_parameters() {
    declare_parameter<int>("motion.walk_speed", 50);
    declare_parameter<int>("motion.step_count", 2);
    declare_parameter<double>("motion.cmd_timeout", 0.5);
}

void MotionNode::load_parameters() {
    get_parameter("motion.walk_speed", walk_speed_);
    get_parameter("motion.step_count", step_count_);
    get_parameter("motion.cmd_timeout", cmd_timeout_);
}

bool MotionNode::init_hardware() {
    // Check if PWM sysfs exists
    if (!std::filesystem::exists("/sys/class/fusion_hat/fusion_hat/pwm/pwm0")) {
        RCLCPP_ERROR(get_logger(), "Fusion HAT+ PWM sysfs not found");
        return false;
    }

    try {
        // Initialize leg servos
        for (int i = 0; i < 8; i++) {
            leg_servos_.push_back(std::make_unique<Servo>(LEGS_PINS[i]));
        }

        // Initialize head servos
        for (int i = 0; i < 3; i++) {
            head_servos_.push_back(std::make_unique<Servo>(HEAD_PINS[i]));
        }

        // Initialize tail servo
        tail_servo_ = std::make_unique<Servo>(TAIL_PIN);

        // Load calibration offsets
        load_calibration();

        // Apply offsets
        for (int i = 0; i < 8; i++) {
            leg_servos_[i]->set_offset(leg_offsets_[i]);
        }
        for (int i = 0; i < 3; i++) {
            head_servos_[i]->set_offset(head_offsets_[i]);
        }
        tail_servo_->set_offset(tail_offset_);

        hardware_initialized_ = true;
        RCLCPP_INFO(get_logger(), "Hardware initialized successfully");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Hardware init error: %s", e.what());
        return false;
    }
}

void MotionNode::load_calibration() {
    // Try to load calibration from pidog_fusion_v2.conf
    std::string config_path = std::string(getenv("HOME")) + "/pidog/claude/config/pidog_fusion_v2.conf";

    std::ifstream file(config_path);
    if (!file.is_open()) {
        RCLCPP_WARN(get_logger(), "Calibration file not found, using defaults");
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Parse leg_zero_list and leg_offset_list
        if (line.find("leg_zero_list") != std::string::npos ||
            line.find("leg_offset_list") != std::string::npos) {
            // Extract values between [ and ]
            size_t start = line.find('[');
            size_t end = line.find(']');
            if (start != std::string::npos && end != std::string::npos) {
                std::string values = line.substr(start + 1, end - start - 1);
                std::istringstream iss(values);
                std::vector<double> vals;
                double val;
                char comma;
                while (iss >> val) {
                    vals.push_back(val);
                    iss >> comma;
                }

                if (vals.size() == 8) {
                    if (line.find("leg_zero_list") != std::string::npos) {
                        for (int i = 0; i < 8; i++) {
                            leg_offsets_[i] += vals[i];
                        }
                    } else {
                        for (int i = 0; i < 8; i++) {
                            leg_offsets_[i] += vals[i];
                        }
                    }
                }
            }
        }

        // Parse head offsets similarly
        if (line.find("head_zero_list") != std::string::npos ||
            line.find("head_offset_list") != std::string::npos) {
            size_t start = line.find('[');
            size_t end = line.find(']');
            if (start != std::string::npos && end != std::string::npos) {
                std::string values = line.substr(start + 1, end - start - 1);
                std::istringstream iss(values);
                std::vector<double> vals;
                double val;
                char comma;
                while (iss >> val) {
                    vals.push_back(val);
                    iss >> comma;
                }

                if (vals.size() == 3) {
                    for (int i = 0; i < 3; i++) {
                        head_offsets_[i] += vals[i];
                    }
                }
            }
        }
    }

    RCLCPP_INFO(get_logger(), "Loaded calibration from %s", config_path.c_str());
}

void MotionNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    target_linear_x_ = msg->linear.x;
    target_angular_z_ = msg->angular.z;
    last_cmd_time_ = get_clock()->now();
}

void MotionNode::cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string cmd = msg->data;
    // Convert to lowercase
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

    RCLCPP_INFO(get_logger(), "Received command: %s", cmd.c_str());

    // Execute in separate thread to avoid blocking
    std::thread([this, cmd]() {
        std::lock_guard<std::mutex> lock(motion_mutex_);

        if (cmd == "stand") do_stand();
        else if (cmd == "sit") do_sit();
        else if (cmd == "rest" || cmd == "lie") do_lie();
        else if (cmd == "stretch") do_stretch();
        else if (cmd == "wag" || cmd == "wag_tail") do_wag();
        else if (cmd == "wave") do_wave();
        else if (cmd == "shake") do_shake();
        else if (cmd == "pushup") do_pushup();
        else if (cmd == "stop") do_stop();
        else RCLCPP_WARN(get_logger(), "Unknown command: %s", cmd.c_str());
    }).detach();
}

void MotionNode::motion_loop() {
    double linear_x, angular_z;
    rclcpp::Time last_time;

    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        linear_x = target_linear_x_;
        angular_z = target_angular_z_;
        last_time = last_cmd_time_;
    }

    // Check timeout
    double elapsed = (get_clock()->now() - last_time).seconds();
    if (elapsed > cmd_timeout_) {
        if (state_ == State::WALKING) {
            std::lock_guard<std::mutex> lock(motion_mutex_);
            do_stop();
        }
        return;
    }

    // Don't interrupt ongoing motion
    if (is_moving_) return;

    // No significant motion command
    if (std::abs(linear_x) < 0.1 && std::abs(angular_z) < 0.1) return;

    // Execute motion in separate thread
    std::thread([this, linear_x, angular_z]() {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        is_moving_ = true;

        if (std::abs(linear_x) > std::abs(angular_z) * 0.5) {
            // Primarily forward/backward
            if (linear_x > 0) {
                do_walk_forward(step_count_);
            } else {
                do_walk_backward(step_count_);
            }
        } else {
            // Primarily turning
            if (angular_z > 0) {
                do_turn_left(step_count_);
            } else {
                do_turn_right(step_count_);
            }
        }

        is_moving_ = false;
    }).detach();
}

void MotionNode::status_timer_callback() {
    std::string state_str;
    switch (state_.load()) {
        case State::IDLE: state_str = "idle"; break;
        case State::STANDING: state_str = "standing"; break;
        case State::WALKING: state_str = "walking"; break;
        case State::SITTING: state_str = "sitting"; break;
        case State::LYING: state_str = "lying"; break;
    }

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    std::stringstream ss;
    ss << "state=" << state_str
       << ", linear=" << std::fixed << std::setprecision(2) << target_linear_x_
       << ", angular=" << target_angular_z_;

    publish_status(ss.str());
}

void MotionNode::publish_status(const std::string& status) {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub_->publish(msg);
}

void MotionNode::sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// ============================================================================
// Motion Commands
// ============================================================================

void MotionNode::set_leg_angles(const std::array<double, 8>& angles) {
    if (!hardware_initialized_) return;

    for (int i = 0; i < 8; i++) {
        leg_servos_[i]->set_angle(angles[i]);
    }
}

void MotionNode::set_head_angles(double yaw, double roll, double pitch) {
    if (!hardware_initialized_) return;

    head_servos_[0]->set_angle(yaw);
    head_servos_[1]->set_angle(roll);
    head_servos_[2]->set_angle(pitch);
}

void MotionNode::set_tail_angle(double angle) {
    if (!hardware_initialized_) return;
    tail_servo_->set_angle(angle);
}

void MotionNode::do_stand() {
    state_ = State::STANDING;
    RCLCPP_INFO(get_logger(), "Standing");

    std::array<double, 8> angles = {0, 0, 0, 0, 0, 0, 0, 0};
    set_leg_angles(angles);
    set_head_angles(0, 0, 0);
}

void MotionNode::do_sit() {
    state_ = State::SITTING;
    RCLCPP_INFO(get_logger(), "Sitting");

    // Front legs extended, back legs bent
    std::array<double, 8> angles = {0, 20, 0, -20, 60, 60, -60, -60};
    set_leg_angles(angles);
    set_head_angles(0, 0, 0);
}

void MotionNode::do_lie() {
    state_ = State::LYING;
    RCLCPP_INFO(get_logger(), "Lying down");

    std::array<double, 8> angles = {45, 45, -45, -45, 45, 45, -45, -45};
    set_leg_angles(angles);
}

void MotionNode::do_stretch() {
    RCLCPP_INFO(get_logger(), "Stretching");

    std::array<double, 8> angles = {-30, 45, 30, -45, 60, 45, -60, -45};
    set_leg_angles(angles);
}

void MotionNode::do_wag() {
    RCLCPP_INFO(get_logger(), "Wagging tail");

    for (int i = 0; i < 3; i++) {
        set_tail_angle(30);
        sleep_ms(200);
        set_tail_angle(-30);
        sleep_ms(200);
    }
    set_tail_angle(0);
}

void MotionNode::do_wave() {
    RCLCPP_INFO(get_logger(), "Waving");

    // Stand on 3 legs, lift front right
    std::array<double, 8> lift = {0, -45, 0, 0, 0, 45, 0, 0};
    set_leg_angles(lift);
    sleep_ms(300);

    // Wave
    for (int i = 0; i < 3; i++) {
        std::array<double, 8> wave1 = {0, -30, 0, 0, 0, 60, 0, 0};
        set_leg_angles(wave1);
        sleep_ms(200);
        std::array<double, 8> wave2 = {0, -60, 0, 0, 0, 30, 0, 0};
        set_leg_angles(wave2);
        sleep_ms(200);
    }

    do_stand();
}

void MotionNode::do_shake() {
    RCLCPP_INFO(get_logger(), "Shaking");

    for (int i = 0; i < 3; i++) {
        set_head_angles(20, 0, 0);
        sleep_ms(150);
        set_head_angles(-20, 0, 0);
        sleep_ms(150);
    }
    set_head_angles(0, 0, 0);
    do_wag();
}

void MotionNode::do_pushup() {
    RCLCPP_INFO(get_logger(), "Pushups");

    for (int i = 0; i < 3; i++) {
        // Down
        std::array<double, 8> down = {30, 30, -30, -30, 30, 30, -30, -30};
        set_leg_angles(down);
        sleep_ms(300);
        // Up
        std::array<double, 8> up = {0, 0, 0, 0, 0, 0, 0, 0};
        set_leg_angles(up);
        sleep_ms(300);
    }
}

void MotionNode::do_stop() {
    if (state_ != State::STANDING) {
        do_stand();
    }
}

void MotionNode::do_walk_forward(int steps) {
    execute_walk(WalkGait::FORWARD, WalkGait::STRAIGHT, steps);
}

void MotionNode::do_walk_backward(int steps) {
    execute_walk(WalkGait::BACKWARD, WalkGait::STRAIGHT, steps);
}

void MotionNode::do_turn_left(int steps) {
    execute_walk(WalkGait::FORWARD, WalkGait::LEFT, steps);
}

void MotionNode::do_turn_right(int steps) {
    execute_walk(WalkGait::FORWARD, WalkGait::RIGHT, steps);
}

void MotionNode::execute_walk(WalkGait::Direction fb, WalkGait::Turn lr, int steps) {
    state_ = State::WALKING;

    WalkGait gait(fb, lr);
    auto coords = gait.get_coords();

    // Calculate delay based on walk_speed_
    // Speed 0 = 50ms, Speed 100 = 5ms
    int delay_ms = static_cast<int>((100 - walk_speed_) / 100.0 * 45.0 + 5.0);

    for (int step = 0; step < steps; step++) {
        for (const auto& coord : coords) {
            auto angles = LegKinematics::legs_angle_calculation(coord);
            set_leg_angles(angles);
            sleep_ms(delay_ms);
        }
    }
}

}  // namespace pidog_ros
