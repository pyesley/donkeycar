#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h> // Include the Imu message type
#include <Arduino_LSM9DS1.h>     // Include the IMU library

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// --- micro-ROS Configuration ---
// Set the serial transport baud rate. MUST MATCH the agent command.
// Common values: 115200, 576000, 1000000, 2000000
// Higher rates reduce latency but can be less stable. Start with 115200 or 576000.
const long serial_baudrate = 576000;

// --- ROS Topic Configuration ---
const char* node_name = "nano33ble_imu_node";
const char* imu_topic_name = "/imu/data_raw"; // Standard topic name for raw IMU
const char* imu_frame_id = "imu_link"; // Important for TF later in SLAM

// --- micro-ROS Error Handling ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error loop
void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// --- Timer Callback ---
// This function is called periodically by the micro-ROS executor
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Read IMU data
    float ax, ay, az, gx, gy, gz;
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az); // Reads acceleration in g's
      IMU.readGyroscope(gx, gy, gz);    // Reads gyroscope in degrees/sec

      // Populate the IMU message
      // Header timestamp (use micro-ROS time)
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      imu_msg.header.stamp.sec = ts.tv_sec;
      imu_msg.header.stamp.nanosec = ts.tv_nsec;

      // Set the frame ID
      imu_msg.header.frame_id.data = (char*)imu_frame_id;
      imu_msg.header.frame_id.size = strlen(imu_frame_id);
      imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

      // Linear Acceleration (convert g's to m/s^2)
      imu_msg.linear_acceleration.x = ax * 9.80665;
      imu_msg.linear_acceleration.y = ay * 9.80665;
      imu_msg.linear_acceleration.z = az * 9.80665;
      // Set covariance if known, otherwise leave as 0s (unknown)
      // imu_msg.linear_acceleration_covariance[0] = ...

      // Angular Velocity (convert deg/sec to rad/sec)
      imu_msg.angular_velocity.x = gx * DEG_TO_RAD;
      imu_msg.angular_velocity.y = gy * DEG_TO_RAD;
      imu_msg.angular_velocity.z = gz * DEG_TO_RAD;
      // Set covariance if known, otherwise leave as 0s (unknown)
      // imu_msg.angular_velocity_covariance[0] = ...

      // Orientation (leave as 0s if not available/calculated)
      imu_msg.orientation.x = 0.0;
      imu_msg.orientation.y = 0.0;
      imu_msg.orientation.z = 0.0;
      imu_msg.orientation.w = 1.0; // Neutral orientation
      imu_msg.orientation_covariance[0] = -1; // Indicate orientation is not available

      // Publish the message
      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
  }
}

void setup() {
  // Configure serial transport
  set_microros_serial_transports(Serial1);
  Serial1.begin(serial_baudrate); // Initialize Serial Monitor AFTER setting transport

  // Initialize IMU sensor
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn LED on during setup
  if (!IMU.begin()) {
    // Serial.println("Failed to initialize IMU!"); // Can't use Serial Monitor easily with micro-ROS
    error_loop(); // Indicate error with blinking LED
  }
  // Set IMU ranges if needed (optional)
  // IMU.setAccelRange(LSM9DS1_ACCEL_RANGE_4G);
  // IMU.setGyroRange(LSM9DS1_GYRO_RANGE_500DPS);

  delay(2000); // Wait a bit for stability

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

  // create IMU publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    imu_topic_name));

  // create timer to publish data periodically (e.g., 10 Hz = 100 ms)
  const unsigned int timer_period = 100; // milliseconds
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_period),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // 1 handle = timer
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  digitalWrite(LED_BUILTIN, LOW); // Turn LED off when setup is done
}

void loop() {
  // Spin the micro-ROS executor
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10))); // Spin for 10ms max
  delay(10); // Small delay to prevent busy-waiting if needed
}