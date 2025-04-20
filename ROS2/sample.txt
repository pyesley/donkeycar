#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.time import Time # Import Time

import serial
import struct
import time
import threading
import numpy as np
import math # Import math for trig functions

from sensor_msgs.msg import Imu
# from std_msgs.msg import Int32 # No longer needed
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry # Import Odometry message type
# Optionally import tf2_ros to publish the odom->base_link transform
# import tf2_ros

# Protocol constants (Unchanged)
CMD_START_BYTE_1 = 0xA5
CMD_START_BYTE_2 = 0x5A
CMD_PACKET_FORMAT = '<hh'
CMD_PACKET_SIZE = struct.calcsize(CMD_PACKET_FORMAT)
SENSOR_START_BYTE_1 = 0xB6
SENSOR_START_BYTE_2 = 0x6B
SENSOR_PACKET_FORMAT = '<ffffffi'
SENSOR_PACKET_SIZE = struct.calcsize(SENSOR_PACKET_FORMAT)

# Helper function for quaternion from yaw
def quaternion_from_yaw(yaw):
    """Converts a yaw angle (in radians) to a Quaternion message."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Declare parameters (Added encoder_meters_per_tick)
        self.declare_parameter('serial_port', '/dev/ttyAMA0', ParameterDescriptor(description='Serial port connected to Arduino'))
        self.declare_parameter('baud_rate', 115200, ParameterDescriptor(description='Serial baud rate'))
        self.declare_parameter('throttle_axis_scale', 255.0, ParameterDescriptor(description='Scale factor for linear.x to throttle PWM (-255 to 255)'))
        self.declare_parameter('steering_max_pwm_left', 6, ParameterDescriptor(description='PWM value for maximum left steering (0-255)'))
        self.declare_parameter('steering_max_pwm_right', 82, ParameterDescriptor(description='PWM value for maximum right steering (0-255)'))
        self.declare_parameter('steering_max_rad_per_sec', 1.0, ParameterDescriptor(description='Absolute angular velocity (rad/s) that maps to max steering PWM deviation'))
        # --- NEW: Encoder parameter ---
        # Convert default 0.3115 cm/tick to 0.003115 m/tick
        self.declare_parameter('encoder_meters_per_tick', 0.003115, ParameterDescriptor(description='Distance in meters moved per encoder tick'))
        # ---
        self.declare_parameter('loop_rate_hz', 50.0, ParameterDescriptor(description='Rate for the main serial reading loop'))
        self.declare_parameter('send_rate_hz', 20.0, ParameterDescriptor(description='Rate at which to send commands (reduces serial traffic)'))
        self.declare_parameter('imu_frame_id', 'imu_link', ParameterDescriptor(description='Frame ID for the IMU message'))
        # --- NEW: Odometry parameters ---
        self.declare_parameter('odom_frame_id', 'odom', ParameterDescriptor(description='Frame ID for Odometry parent frame'))
        self.declare_parameter('odom_child_frame_id', 'base_link', ParameterDescriptor(description='Frame ID for Odometry child frame (robot base)'))
        # self.declare_parameter('publish_odom_tf', False, ParameterDescriptor(description='Publish the odom -> base_link transform via tf2')) # Optional TF publishing
        # ---
        self.declare_parameter('use_arduino_ready_signal', True, ParameterDescriptor(description='Wait for "READY" signal from Arduino on startup'))

        # Get parameters
        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.throttle_scale = self.get_parameter('throttle_axis_scale').value
        self.send_rate = self.get_parameter('send_rate_hz').value
        self.imu_frame = self.get_parameter('imu_frame_id').value
        self.encoder_m_per_tick = self.get_parameter('encoder_meters_per_tick').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.odom_child_frame = self.get_parameter('odom_child_frame_id').value
        # self.publish_tf = self.get_parameter('publish_odom_tf').value # Optional TF publishing
        self.wait_for_ready = self.get_parameter('use_arduino_ready_signal').value
        self.loop_rate = self.get_parameter('loop_rate_hz').value

        # Calculate initial steering params (Unchanged)
        self.update_steering_params()
        # Add parameter callback (Unchanged)
        self.add_on_set_parameters_callback(self.parameters_callback)

        # ROS Publishers (IMU + Odometry)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        # self.encoder_publisher = self.create_publisher(Int32, 'encoder_ticks', 10) # Removed
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10) # NEW Odom publisher

        # Optional TF Broadcaster
        # if self.publish_tf:
        #     self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ROS Subscribers (Unchanged)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Serial port object & Locks (Unchanged)
        self.serial_port = None
        self.last_twist_msg = Twist()
        self.command_lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.serial_buffer = bytearray()

        # --- Odometry State Variables ---
        self.last_enc_ticks = 0
        self.last_enc_time = self.get_clock().now()
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta_pos = 0.0
        self.first_encoder_msg = True # Flag to handle first message properly

        # Rate Limiter (Unchanged)
        self.loop_rate_limiter = self.create_rate(self.loop_rate)

        # Connect Serial & Start Threads (Unchanged)
        self.connect_serial()
        self.running = True
        self.thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.thread.start()
        self.send_timer = self.create_timer(1.0 / self.send_rate, self.send_command_callback)

        self.get_logger().info(f"Arduino Bridge node started. Publishing Odometry.")
        self.get_logger().info(f"Encoder meters/tick: {self.encoder_m_per_tick}")

    # Parameter callback (Modified to include encoder_meters_per_tick)
    def parameters_callback(self, params):
        success = True
        steering_changed = False
        for param in params:
            if param.name in ['steering_max_pwm_left', 'steering_max_pwm_right', 'steering_max_rad_per_sec']:
                steering_changed = True
            elif param.name == 'encoder_meters_per_tick':
                 # Basic validation: ensure it's positive
                 if param.value <= 0:
                      self.get_logger().warn(f"Invalid encoder_meters_per_tick ({param.value}), must be positive. Keeping old value.")
                      success = False # Indicate failure for this param? Or just warn? Let's warn.
                 else:
                      self.encoder_m_per_tick = param.value
            # Add checks for other parameters if necessary

        if steering_changed:
            self.update_steering_params()
            self.get_logger().info(f"Parameters updated. New Steering PWM Limits: Left={self.pwm_left}, Right={self.pwm_right}, Center={self.pwm_center:.1f}")
        if 'encoder_meters_per_tick' in [p.name for p in params if success]: # Only log if update was successful
            self.get_logger().info(f"Parameters updated. New encoder meters/tick: {self.encoder_m_per_tick}")

        return SetParametersResult(successful=success) # Report overall success

    # update_steering_params (Unchanged from previous version)
    def update_steering_params(self):
        self.pwm_left = self.get_parameter('steering_max_pwm_left').value
        self.pwm_right = self.get_parameter('steering_max_pwm_right').value
        self.max_rad_per_sec = self.get_parameter('steering_max_rad_per_sec').value
        if self.pwm_left >= self.pwm_right:
            self.get_logger().error("Steering PWM Left limit must be less than Right limit! Check parameters.")
        self.pwm_center = (self.pwm_right + self.pwm_left) / 2.0
        self.pwm_range = self.pwm_right - self.pwm_center
        if self.max_rad_per_sec <= 0:
             self.get_logger().warn("steering_max_rad_per_sec is zero or negative, steering will not respond to angular.z. Setting scale to 0.")
             self.rad_to_pwm_scale = 0.0
        else:
             self.rad_to_pwm_scale = self.pwm_range / self.max_rad_per_sec

    # connect_serial (Unchanged)
    def connect_serial(self):
        # ... (Serial connection logic - unchanged) ...
        while rclpy.ok() and self.serial_port is None:
            try:
                self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0.1)
                self.get_logger().info(f"Successfully connected to {self.serial_port_name}")
                if self.wait_for_ready:
                    self.get_logger().info("Waiting for READY signal from Arduino...")
                    #...(READY logic)...
                    ready_line = b""
                    got_ready = False
                    start_time = time.time()
                    while time.time() - start_time < 5.0:
                        if self.serial_port.in_waiting > 0:
                            byte = self.serial_port.read(1)
                            if byte == b'\n':
                                try:
                                    decoded_line = ready_line.decode('utf-8').strip()
                                    if "READY" in decoded_line:
                                        self.get_logger().info(f"Arduino reported READY ('{decoded_line}').")
                                        got_ready = True
                                    else:
                                            self.get_logger().warn(f"Received line, but not expected READY signal. Got: '{decoded_line}'.")
                                    ready_line = b""
                                    break
                                except UnicodeDecodeError:
                                        self.get_logger().warn(f"Received non-text data terminated by newline (bytes: {ready_line.hex()}).")
                                        ready_line = b""
                                        break
                            elif byte != b'\r':
                                ready_line += byte
                        else:
                            time.sleep(0.01)
                    if not got_ready and time.time() - start_time >= 5.0 :
                            self.get_logger().warn("Timeout waiting for READY signal from Arduino.")
                    elif not got_ready and len(ready_line) > 0 :
                            self.get_logger().warn(f"Finished waiting for READY, but signal not confirmed. Partial/Final buffer: {ready_line.hex()}")
                    self.serial_port.reset_input_buffer()
                    self.get_logger().info("Input buffer cleared. Proceeding to binary read loop.")
                break
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to {self.serial_port_name}: {e}. Retrying in 5 seconds...")
                self.serial_port = None; time.sleep(5)
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred during serial connection: {e}")
                self.serial_port = None; time.sleep(5)

    # cmd_vel_callback (Unchanged)
    def cmd_vel_callback(self, msg):
        with self.command_lock:
            self.last_twist_msg = msg

    # send_command_callback (Unchanged)
    def send_command_callback(self):
        with self.command_lock:
            twist_to_send = self.last_twist_msg
        self.send_command(twist_to_send)

    # send_command (Unchanged from previous version)
    def send_command(self, msg: Twist):
        # ... (PWM steering and throttle logic - unchanged) ...
        if self.serial_port is None or not self.serial_port.is_open: return
        throttle = int(np.clip(msg.linear.x, -1.0, 1.0) * self.throttle_scale)
        clamped_angular_z = np.clip(msg.angular.z, -self.max_rad_per_sec, self.max_rad_per_sec)
        pwm_deviation = -clamped_angular_z * self.rad_to_pwm_scale
        target_pwm = self.pwm_center + pwm_deviation
        steering_pwm = int(np.clip(target_pwm, self.pwm_left, self.pwm_right))
        try:
            data_bytes = struct.pack(CMD_PACKET_FORMAT, steering_pwm, throttle)
            checksum = self.calculate_checksum(data_bytes)
            packet = bytes([CMD_START_BYTE_1, CMD_START_BYTE_2]) + data_bytes + bytes([checksum])
            self.get_logger().debug(f"Sending: TargetPWM={target_pwm:.1f}, SteerPWM={steering_pwm}, Thr={throttle}, Packet={packet.hex()}")
            with self.serial_lock:
                self.serial_port.write(packet)
        except struct.error as e: self.get_logger().error(f"Struct packing error: {e}")
        except serial.SerialException as e: self.get_logger().error(f"Serial write error: {e}"); self.close_serial()
        except Exception as e: self.get_logger().error(f"Unexpected error sending command: {e}")


    # calculate_checksum (Unchanged)
    def calculate_checksum(self, data_bytes):
        # ... (Checksum logic - unchanged) ...
        checksum = 0
        for byte in data_bytes: checksum ^= byte
        return checksum

    # serial_read_loop (Unchanged)
    def serial_read_loop(self):
        # ... (Serial reading & packet parsing logic - unchanged) ...
        packet_state = 0; expected_data_len = SENSOR_PACKET_SIZE; data_buffer = bytearray()
        self.get_logger().info("Starting serial read loop...")
        while rclpy.ok() and self.running:
            if self.serial_port is None or not self.serial_port.is_open:
                self.connect_serial()
                if self.serial_port is None: time.sleep(1.0); continue
            try:
                if self.serial_port.in_waiting > 0:
                    with self.serial_lock: read_bytes = self.serial_port.read(self.serial_port.in_waiting)
                    self.serial_buffer.extend(read_bytes)
                while len(self.serial_buffer) > 0:
                    byte = self.serial_buffer.pop(0)
                    if packet_state == 0:
                        if byte == SENSOR_START_BYTE_1: packet_state = 1
                    elif packet_state == 1:
                        if byte == SENSOR_START_BYTE_2: packet_state = 2; data_buffer.clear()
                        else: packet_state = 0;
                        if byte == SENSOR_START_BYTE_1: packet_state = 1 # Handle back-to-back start bytes
                    elif packet_state == 2:
                        data_buffer.append(byte)
                        if len(data_buffer) == expected_data_len: packet_state = 3
                    elif packet_state == 3:
                        received_checksum = byte
                        calculated_checksum = self.calculate_checksum(data_buffer)
                        if received_checksum == calculated_checksum:
                            self.process_sensor_packet(data_buffer) # Process valid packet
                        else:
                            self.get_logger().warn(f"Checksum mismatch! Got: {received_checksum:02X}, Calc: {calculated_checksum:02X}")
                        packet_state = 0; data_buffer.clear(); break # Reset state machine
                    if (packet_state == 2 or packet_state == 3) and len(self.serial_buffer) == 0: break
            except serial.SerialException as e: self.get_logger().error(f"Serial read error: {e}. Closing port."); self.close_serial()
            except Exception as e: self.get_logger().error(f"Unexpected error in read loop processing: {e}"); packet_state=0; data_buffer.clear(); self.serial_buffer.clear()
            finally: self.loop_rate_limiter.sleep() # Ensure loop rate

    # --- MODIFIED: process_sensor_packet calculates and publishes odometry ---
    def process_sensor_packet(self, data_bytes):
        """Unpack sensor data, publish IMU, and calculate/publish Odometry."""
        current_time_rclpy = self.get_clock().now() # Get rclpy.Time object

        try:
            # Unpack data according to the defined format
            ax, ay, az, gx, gy, gz, current_enc_ticks = struct.unpack(SENSOR_PACKET_FORMAT, data_bytes)

            # --- Publish IMU Data (Unchanged) ---
            imu_msg = Imu()
            imu_msg.header.stamp = current_time_rclpy.to_msg()
            imu_msg.header.frame_id = self.imu_frame
            imu_msg.linear_acceleration.x = float(ax); imu_msg.linear_acceleration.y = float(ay); imu_msg.linear_acceleration.z = float(az)
            imu_msg.linear_acceleration_covariance[0] = 0.01; imu_msg.linear_acceleration_covariance[4] = 0.01; imu_msg.linear_acceleration_covariance[8] = 0.01
            imu_msg.angular_velocity.x = float(gx); imu_msg.angular_velocity.y = float(gy); imu_msg.angular_velocity.z = float(gz)
            imu_msg.angular_velocity_covariance[0] = 0.005; imu_msg.angular_velocity_covariance[4] = 0.005; imu_msg.angular_velocity_covariance[8] = 0.005
            imu_msg.orientation_covariance[0] = -1.0
            self.imu_publisher.publish(imu_msg)

            # --- Calculate and Publish Odometry ---
            # Handle first message received
            if self.first_encoder_msg:
                self.last_enc_ticks = current_enc_ticks
                self.last_enc_time = current_time_rclpy
                self.first_encoder_msg = False
                return # Cannot calculate deltas yet

            # Calculate time difference
            delta_time_sec = (current_time_rclpy - self.last_enc_time).nanoseconds / 1e9

            # Avoid division by zero or stale data
            if delta_time_sec <= 0.0:
                self.get_logger().warn(f"Delta time is zero or negative ({delta_time_sec:.4f}s), skipping odom calculation.")
                # Update time but not ticks, wait for next message with valid delta_t
                self.last_enc_time = current_time_rclpy
                return

            # Calculate change in ticks (consider potential rollover if using smaller int types, less likely with int32)
            delta_ticks = current_enc_ticks - self.last_enc_ticks
            # TODO: Add rollover handling if necessary (e.g., if Arduino sends uint16)

            # Calculate distance and linear velocity
            delta_distance = delta_ticks * self.encoder_m_per_tick
            vx = delta_distance / delta_time_sec

            # Estimate angular velocity from last commanded Twist
            # Lock access to ensure thread safety
            with self.command_lock:
                vth = self.last_twist_msg.angular.z

            # Calculate change in pose
            delta_x = vx * math.cos(self.theta_pos) * delta_time_sec
            delta_y = vx * math.sin(self.theta_pos) * delta_time_sec
            delta_theta = vth * delta_time_sec

            # Update pose
            self.x_pos += delta_x
            self.y_pos += delta_y
            self.theta_pos += delta_theta
            # Normalize theta between -pi and pi (optional but good practice)
            self.theta_pos = math.atan2(math.sin(self.theta_pos), math.cos(self.theta_pos))

            # Create Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time_rclpy.to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.odom_child_frame

            # Set the pose
            odom_msg.pose.pose.position.x = self.x_pos
            odom_msg.pose.pose.position.y = self.y_pos
            odom_msg.pose.pose.position.z = 0.0 # Assuming 2D motion
            odom_msg.pose.pose.orientation = quaternion_from_yaw(self.theta_pos)
            # TODO: Add covariance for pose if estimates are available

            # Set the twist (velocities)
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = 0.0 # Assuming no sideways motion
            odom_msg.twist.twist.angular.z = vth
            # TODO: Add covariance for twist if estimates are available

            self.odom_publisher.publish(odom_msg)

            # --- Optional: Publish TF Transform ---
            # if self.publish_tf:
            #     t = TransformStamped()
            #     t.header.stamp = current_time_rclpy.to_msg()
            #     t.header.frame_id = self.odom_frame
            #     t.child_frame_id = self.odom_child_frame
            #     t.transform.translation.x = self.x_pos
            #     t.transform.translation.y = self.y_pos
            #     t.transform.translation.z = 0.0
            #     t.transform.rotation = odom_msg.pose.pose.orientation # Use same quaternion
            #     self.tf_broadcaster.sendTransform(t)

            # Update last known values for next iteration
            self.last_enc_ticks = current_enc_ticks
            self.last_enc_time = current_time_rclpy

            # self.get_logger().debug(f"Odom: dt={delta_time_sec:.3f}, dx={vx:.3f}, dth={vth:.3f}, Pose({self.x_pos:.2f},{self.y_pos:.2f},{self.theta_pos:.2f})")

        except struct.error as e:
            self.get_logger().error(f"Failed to unpack sensor packet: {e}")
            self.first_encoder_msg = True # Reset state on error
        except Exception as e:
            self.get_logger().error(f"Error processing sensor data for odom: {e}")
            self.first_encoder_msg = True # Reset state on error


    # close_serial (Unchanged)
    def close_serial(self):
        # ... (Close logic - unchanged) ...
        if self.serial_port is not None and self.serial_port.is_open:
            try: self.serial_port.close(); self.get_logger().info(f"Closed serial port {self.serial_port_name}")
            except Exception as e: self.get_logger().error(f"Error closing serial port: {e}")
        self.serial_port = None

    # shutdown (Unchanged)
    def shutdown(self):
        # ... (Shutdown logic - unchanged) ...
        self.running = False
        if self.thread.is_alive(): self.thread.join(timeout=1.0)
        self.close_serial()
        self.get_logger().info("Arduino Bridge node shutting down.")

# main function (Unchanged)
def main(args=None):
    # ... (Main logic - unchanged) ...
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info("Keyboard interrupt detected.")
    except Exception as e: node.get_logger().error(f"Unhandled exception in spin: {e}")
    finally: node.shutdown(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()