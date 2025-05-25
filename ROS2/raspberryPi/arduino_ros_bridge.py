#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.time import Time

import serial
import struct
import time
import threading
import numpy as np
import math

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

# Protocol constants (matching Arduino)
CMD_START_BYTE_1 = 0xA5
CMD_START_BYTE_2 = 0x5A
CMD_PACKET_FORMAT = '<hh'  # steering_angle, throttle_speed
CMD_PACKET_SIZE = struct.calcsize(CMD_PACKET_FORMAT)

SENSOR_START_BYTE_1 = 0xB6
SENSOR_START_BYTE_2 = 0x6B
SENSOR_PACKET_FORMAT = '<ffffffi'  # ax, ay, az, gx, gy, gz, current_enc_ticks
SENSOR_PACKET_SIZE = struct.calcsize(SENSOR_PACKET_FORMAT)

# Heartbeat Protocol Constants (matching Arduino)
ARDUINO_TO_PI_HEARTBEAT_PING = 0xC1
PI_TO_ARDUINO_HEARTBEAT_PONG = 0xD1
ARDUINO_READY_MESSAGE = "ARDUINO_READY" # Optional: if Arduino sends a startup string

def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('throttle_axis_scale', 255.0)
        self.declare_parameter('steering_max_pwm_left', 6) # Example: 0-180 servo, 6 might be full left
        self.declare_parameter('steering_max_pwm_right', 82) # Example: 82 might be full right
        self.declare_parameter('steering_max_rad_per_sec', 1.0)
        self.declare_parameter('encoder_meters_per_tick', 0.003115)
        self.declare_parameter('loop_rate_hz', 120.0) # Rate for serial_read_loop attempts
        self.declare_parameter('send_rate_hz', 20.0) # Rate for sending cmd_vel commands
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('odom_child_frame_id', 'base_link')
        self.declare_parameter('use_arduino_ready_signal', False, # Set to True if Arduino sends ARDUINO_READY
                                ParameterDescriptor(description='Wait for initial signal from Arduino on startup'))
        self.declare_parameter('arduino_heartbeat_timeout_sec', 5.0, # How long to wait for an Arduino PING
                                ParameterDescriptor(description='Seconds to wait for Arduino heartbeat PING before assuming disconnection.'))


        # Get parameters
        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.throttle_scale = self.get_parameter('throttle_axis_scale').value
        self.send_rate = self.get_parameter('send_rate_hz').value
        self.imu_frame = self.get_parameter('imu_frame_id').value
        self.encoder_m_per_tick = self.get_parameter('encoder_meters_per_tick').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.odom_child_frame = self.get_parameter('odom_child_frame_id').value
        self.wait_for_ready_signal = self.get_parameter('use_arduino_ready_signal').value
        self.loop_rate_hz = self.get_parameter('loop_rate_hz').value
        self.arduino_heartbeat_timeout_duration = self.get_parameter('arduino_heartbeat_timeout_sec').value


        self.update_steering_params()
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Subscriber
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Serial port object
        self.serial_port = None
        self.serial_lock = threading.Lock() # For thread-safe access to serial_port
        self.serial_buffer = bytearray()    # Buffer for incoming serial data

        # Command handling
        self.last_twist_msg = Twist()
        self.command_lock = threading.Lock() # For thread-safe access to last_twist_msg

        # Odometry state
        self.x_pos, self.y_pos, self.theta_pos = 0.0, 0.0, 0.0
        self.last_enc_ticks = 0
        self.last_enc_time = self.get_clock().now()
        self.first_encoder_msg_after_connect = True # Reset odom on first valid encoder msg after (re)connect

        # Heartbeat and connection state
        self.last_arduino_ping_time = time.time() # Time when last PING was received from Arduino
        self.arduino_is_connected = False         # Tracks if Arduino is considered connected via heartbeat
        self.connection_attempt_pending = False   # To avoid spamming connection attempts

        # Main loop control
        self.running = True
        self.loop_rate_limiter = self.create_rate(self.loop_rate_hz)
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.send_timer = self.create_timer(1.0 / self.send_rate, self.send_command_callback)
        self.heartbeat_check_timer = self.create_timer(1.0, self.check_arduino_heartbeat) # Check Arduino heartbeat every 1 second

        self.get_logger().info(f"Arduino Bridge node starting. Port: {self.serial_port_name}@{self.baud_rate}")
        self.read_thread.start()


    def parameters_callback(self, params):
        # Standard parameter callback, update steering if relevant params change
        success = True
        steering_changed = False
        for param in params:
            if param.name in ['steering_max_pwm_left', 'steering_max_pwm_right', 'steering_max_rad_per_sec']:
                steering_changed = True
            elif param.name == 'encoder_meters_per_tick':
                if param.value <= 0:
                    self.get_logger().warn(f"Invalid encoder_meters_per_tick ({param.value}). Keeping old value.")
                else:
                    self.encoder_m_per_tick = param.value
            elif param.name == 'arduino_heartbeat_timeout_sec':
                 self.arduino_heartbeat_timeout_duration = param.value
                 self.get_logger().info(f"Arduino heartbeat timeout updated to: {self.arduino_heartbeat_timeout_duration}s")


        if steering_changed:
            self.update_steering_params()
            self.get_logger().info(
                f"Steering params updated. PWM: L={self.pwm_left}, R={self.pwm_right}, C={self.pwm_center:.1f}")
        return SetParametersResult(successful=success)

    def update_steering_params(self):
        self.pwm_left = self.get_parameter('steering_max_pwm_left').value
        self.pwm_right = self.get_parameter('steering_max_pwm_right').value
        self.max_rad_per_sec = self.get_parameter('steering_max_rad_per_sec').value
        self.pwm_center = (self.pwm_right + self.pwm_left) / 2.0
        self.pwm_range = self.pwm_right - self.pwm_center # Assumes symmetrical steering for this calculation
        if self.max_rad_per_sec <= 1e-6:
            self.rad_to_pwm_scale = 0.0
        else:
            self.rad_to_pwm_scale = self.pwm_range / self.max_rad_per_sec

    def reset_odometry_for_new_connection(self):
        self.get_logger().info("Resetting odometry state for new/re-established connection.")
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta_pos = 0.0
        self.last_enc_ticks = 0 # Will be set by first valid encoder message
        self.first_encoder_msg_after_connect = True
        self.last_enc_time = self.get_clock().now()

    def connect_serial(self):
        if self.serial_port is not None and self.serial_port.is_open:
            return True # Already connected
        if self.connection_attempt_pending:
            return False # Avoid rapid retries

        self.connection_attempt_pending = True
        self.get_logger().info(f"Attempting to connect to Arduino on {self.serial_port_name}...")
        try:
            with self.serial_lock: # Ensure exclusive access during port manipulation
                if self.serial_port and self.serial_port.is_open: # Double check
                    self.serial_port.close()
                self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0.05) # Short non-blocking timeout

            self.get_logger().info(f"Successfully connected to {self.serial_port_name}.")
            self.serial_port.reset_input_buffer() # Clear any stale data

            if self.wait_for_ready_signal:
                self.get_logger().info(f"Waiting for '{ARDUINO_READY_MESSAGE}' signal from Arduino...")
                # Simplified ready signal wait - heartbeat is primary connection monitor
                # For a more robust ready signal, implement a timeout and line-by-line read
                time.sleep(1.0) # Give Arduino a moment to send its ready signal if any
                # Actual check for ARDUINO_READY would involve reading lines here.
                # For now, we rely on the first heartbeat PING to confirm active connection.

            # Connection is physically open, but logical connection depends on heartbeat
            self.arduino_is_connected = False # Will be set to True upon first PING
            self.last_arduino_ping_time = time.time() # Reset PING timer
            self.reset_odometry_for_new_connection()
            self.connection_attempt_pending = False
            return True

        except serial.SerialException as e:
            self.get_logger().warn(f"Failed to connect to {self.serial_port_name}: {e}. Will retry.")
            with self.serial_lock:
                if self.serial_port: self.serial_port.close()
                self.serial_port = None
            self.arduino_is_connected = False
            self.connection_attempt_pending = False
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error during serial connection: {e}")
            with self.serial_lock:
                if self.serial_port: self.serial_port.close()
                self.serial_port = None
            self.arduino_is_connected = False
            self.connection_attempt_pending = False
            return False

    def cmd_vel_callback(self, msg):
        with self.command_lock:
            self.last_twist_msg = msg

    def send_command_callback(self):
        if not self.arduino_is_connected: # Only send if logically connected
            # self.get_logger().debug("Arduino not connected (heartbeat). Command not sent.")
            return

        with self.command_lock:
            twist_to_send = self.last_twist_msg

        # Steering calculation (same as your original logic)
        clamped_angular_z = np.clip(twist_to_send.angular.z, -self.max_rad_per_sec, self.max_rad_per_sec)
        pwm_deviation = -clamped_angular_z * self.rad_to_pwm_scale
        target_pwm = self.pwm_center + pwm_deviation
        steering_pwm = int(np.clip(target_pwm, min(self.pwm_left, self.pwm_right), max(self.pwm_left, self.pwm_right)))

        # Throttle calculation
        throttle_pwm = int(np.clip(twist_to_send.linear.x, -1.0, 1.0) * self.throttle_scale)

        try:
            payload_bytes = struct.pack(CMD_PACKET_FORMAT, steering_pwm, throttle_pwm)
            checksum = self.calculate_checksum(payload_bytes)
            packet = bytes([CMD_START_BYTE_1, CMD_START_BYTE_2]) + payload_bytes + bytes([checksum])

            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.write(packet)
                    # self.get_logger().debug(f"Sent CMD: Steer={steering_pwm}, Thr={throttle_pwm}")
                else:
                    # This case should ideally be caught by arduino_is_connected flag
                    self.get_logger().warn("Attempted to send command, but serial port not open.")
                    self.handle_serial_error()


        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error for command: {e}")
            self.handle_serial_error()
        except struct.error as e:
            self.get_logger().error(f"Struct packing error for command: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error sending command: {e}")
            self.handle_serial_error()


    def calculate_checksum(self, data_bytes):
        checksum = 0
        for byte_val in data_bytes:
            checksum ^= byte_val
        return checksum

    def handle_serial_error(self):
        """Common actions to take on serial communication errors."""
        self.get_logger().info("Serial error detected. Closing port and marking Arduino as disconnected.")
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception as e_close:
                    self.get_logger().error(f"Exception while closing serial port: {e_close}")
            self.serial_port = None
        self.arduino_is_connected = False
        # Connection attempt will be made by serial_read_loop or heartbeat_check

    def check_arduino_heartbeat(self):
        """Periodically checks if Arduino PINGs are being received."""
        if self.serial_port and self.serial_port.is_open: # Only check if port thinks it's open
            if self.arduino_is_connected: # If we currently think we are connected
                if (time.time() - self.last_arduino_ping_time) > self.arduino_heartbeat_timeout_duration:
                    self.get_logger().warn(
                        f"Arduino PING timeout (last PING {time.time() - self.last_arduino_ping_time:.2f}s ago). Assuming disconnection.")
                    self.handle_serial_error() # This will set arduino_is_connected to False and close port
        # If port is not open, serial_read_loop will attempt to reconnect.

    def serial_read_loop(self):
        packet_parser_state = 0  # 0: wait_sensor_start1, 1: wait_sensor_start2, 2: read_sensor_payload, 3: read_sensor_checksum
        sensor_data_buffer = bytearray()

        self.get_logger().info("Serial read loop started.")

        while rclpy.ok() and self.running:
            if self.serial_port is None or not self.serial_port.is_open:
                if not self.connection_attempt_pending: # Avoid spamming if an attempt is already underway
                    if not self.connect_serial():
                        time.sleep(1.0) # Wait before retrying connection
                else:
                    time.sleep(0.1) # Short sleep if connection attempt is pending
                continue # Loop back to check connection status or wait

            # At this point, serial_port should be non-None and (hopefully) open
            try:
                bytes_available = 0
                with self.serial_lock:
                    if self.serial_port and self.serial_port.is_open: # Re-check before reading
                         bytes_available = self.serial_port.in_waiting

                if bytes_available > 0:
                    with self.serial_lock:
                        # Read all available bytes to minimize read calls, up to a limit
                        read_bytes = self.serial_port.read(min(bytes_available, 1024))
                    self.serial_buffer.extend(read_bytes)

                # Process the buffered bytes
                i = 0
                while i < len(self.serial_buffer):
                    byte = self.serial_buffer[i]

                    # --- Heartbeat PING Handling (Highest Priority) ---
                    if byte == ARDUINO_TO_PI_HEARTBEAT_PING:
                        if not self.arduino_is_connected: # First PING after disconnect or startup
                             self.get_logger().info("Arduino PING received. Connection ESTABLISHED.")
                             self.reset_odometry_for_new_connection() # Reset odom state

                        self.last_arduino_ping_time = time.time()
                        self.arduino_is_connected = True

                        # Send PONG back
                        try:
                            with self.serial_lock:
                                if self.serial_port and self.serial_port.is_open:
                                    self.serial_port.write(bytes([PI_TO_ARDUINO_HEARTBEAT_PONG]))
                                    # self.get_logger().debug("Sent PONG to Arduino.")
                        except serial.SerialException as e:
                            self.get_logger().error(f"Serial write error for PONG: {e}")
                            self.handle_serial_error() # Marks disconnected, closes port
                            # Since we are iterating serial_buffer, break here to re-evaluate connection
                            # The rest of serial_buffer will be processed on next valid loop
                            self.serial_buffer = self.serial_buffer[i+1:] # Consume this PING byte
                            i = -1 # Restart loop over potentially modified buffer
                            break


                        # Consume this PING byte and reset sensor packet parser
                        packet_parser_state = 0
                        sensor_data_buffer.clear()
                        self.serial_buffer = self.serial_buffer[i+1:] # Remove processed byte
                        i = -1 # Restart loop from beginning of (now shorter) buffer
                        if len(self.serial_buffer) == 0: break # Buffer empty
                        continue # Next byte in buffer

                    # --- Sensor Packet Parsing State Machine ---
                    if packet_parser_state == 0:  # Wait for SENSOR_START_BYTE_1
                        if byte == SENSOR_START_BYTE_1:
                            packet_parser_state = 1
                    elif packet_parser_state == 1:  # Wait for SENSOR_START_BYTE_2
                        if byte == SENSOR_START_BYTE_2:
                            packet_parser_state = 2
                            sensor_data_buffer.clear()
                        elif byte == SENSOR_START_BYTE_1: # Got another SENSOR_START_BYTE_1
                            pass # Stay in state 1
                        else: # Unexpected byte, reset
                            # self.get_logger().debug(f"Sensor sync error. Expected SENSOR_START_2, got {byte:#02x}. Resetting parser.")
                            packet_parser_state = 0
                    elif packet_parser_state == 2:  # Reading SENSOR_PACKET_SIZE bytes
                        sensor_data_buffer.append(byte)
                        if len(sensor_data_buffer) == SENSOR_PACKET_SIZE:
                            packet_parser_state = 3  # Move to checksum state
                    elif packet_parser_state == 3:  # Reading checksum byte
                        received_checksum = byte
                        calculated_checksum = self.calculate_checksum(sensor_data_buffer)
                        if received_checksum == calculated_checksum:
                            if self.arduino_is_connected: # Process only if logically connected
                                self.process_sensor_packet(sensor_data_buffer)
                            # else:
                                # self.get_logger().debug("Sensor packet received but Arduino not connected (heartbeat). Discarding.")
                        else:
                            self.get_logger().warn(
                                f"Sensor packet checksum mismatch! Got: {received_checksum:02X}, Calc: {calculated_checksum:02X}. Discarding.")
                        packet_parser_state = 0 # Reset for next packet
                        sensor_data_buffer.clear()
                    i += 1 # Move to next byte in serial_buffer

                if i > 0 : # If we processed some bytes
                    self.serial_buffer = self.serial_buffer[i:] # Remove processed bytes

            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}.")
                self.handle_serial_error() # Marks disconnected, closes port
            except Exception as e:
                self.get_logger().error(f"Unexpected error in serial read loop: {e}", exc_info=True)
                packet_parser_state = 0 # Reset parser
                sensor_data_buffer.clear()
                self.serial_buffer.clear() # Clear buffer on unknown error
                self.handle_serial_error() # Attempt to recover connection state

            self.loop_rate_limiter.sleep()

    def process_sensor_packet(self, data_bytes):
        current_time_rclpy = self.get_clock().now()
        try:
            ax, ay, az, gx, gy, gz, current_enc_ticks = struct.unpack(SENSOR_PACKET_FORMAT, data_bytes)

            # IMU Message
            imu_msg = Imu()
            imu_msg.header.stamp = current_time_rclpy.to_msg()
            imu_msg.header.frame_id = self.imu_frame
            imu_msg.linear_acceleration.x = float(ax)
            imu_msg.linear_acceleration.y = float(ay)
            imu_msg.linear_acceleration.z = float(az)
            imu_msg.linear_acceleration_covariance[0] = 0.01 # Example
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01
            imu_msg.angular_velocity.x = float(gx)
            imu_msg.angular_velocity.y = float(gy)
            imu_msg.angular_velocity.z = float(gz)
            imu_msg.angular_velocity_covariance[0] = 0.005 # Example
            imu_msg.angular_velocity_covariance[4] = 0.005
            imu_msg.angular_velocity_covariance[8] = 0.005
            imu_msg.orientation_covariance[0] = -1.0  # Orientation not provided by this IMU packet
            self.imu_publisher.publish(imu_msg)

            # Odometry Calculation
            if self.first_encoder_msg_after_connect:
                self.last_enc_ticks = current_enc_ticks
                self.last_enc_time = current_time_rclpy
                self.first_encoder_msg_after_connect = False
                self.get_logger().info(f"First encoder message after (re)connect. Initial ticks: {current_enc_ticks}")
                return # Skip odom calculation for the very first message

            delta_time_nanosec = (current_time_rclpy - self.last_enc_time).nanoseconds
            if delta_time_nanosec <= 1000: # Avoid division by zero or tiny dt (e.g. < 1 microsecond)
                if current_enc_ticks != self.last_enc_ticks:
                     self.get_logger().warn(f"Encoder ticks changed ({self.last_enc_ticks} -> {current_enc_ticks}) but delta_time is too small ({delta_time_nanosec / 1e9:.6f}s). Resetting odom 'first_encoder_msg' flag.")
                     self.first_encoder_msg_after_connect = True # Re-init on next valid packet
                return

            delta_time_sec = delta_time_nanosec / 1e9
            delta_ticks = current_enc_ticks - self.last_enc_ticks
            delta_distance = delta_ticks * self.encoder_m_per_tick
            vx = delta_distance / delta_time_sec

            with self.command_lock: # Use last commanded angular velocity for odometry
                vth = self.last_twist_msg.angular.z

            # Pose integration (Euler)
            # Using midpoint for theta improves stability slightly for vth * dt
            delta_x = vx * math.cos(self.theta_pos + (vth * delta_time_sec / 2.0)) * delta_time_sec
            delta_y = vx * math.sin(self.theta_pos + (vth * delta_time_sec / 2.0)) * delta_time_sec
            delta_theta = vth * delta_time_sec

            self.x_pos += delta_x
            self.y_pos += delta_y
            self.theta_pos += delta_theta
            self.theta_pos = math.atan2(math.sin(self.theta_pos), math.cos(self.theta_pos)) # Normalize

            # Odometry Message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time_rclpy.to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.odom_child_frame
            odom_msg.pose.pose.position.x = self.x_pos
            odom_msg.pose.pose.position.y = self.y_pos
            odom_msg.pose.pose.orientation = quaternion_from_yaw(self.theta_pos)
            # TODO: Add covariance for pose
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.angular.z = vth
            # TODO: Add covariance for twist
            self.odom_publisher.publish(odom_msg)

            self.last_enc_ticks = current_enc_ticks
            self.last_enc_time = current_time_rclpy

        except struct.error as e:
            self.get_logger().error(f"Failed to unpack sensor packet: {e}")
            self.first_encoder_msg_after_connect = True # Reset on error
        except Exception as e:
            self.get_logger().error(f"Error processing sensor data: {e}", exc_info=True)
            self.first_encoder_msg_after_connect = True # Reset on error

    def shutdown(self):
        self.get_logger().info("Arduino Bridge node shutting down...")
        self.running = False
        if self.send_timer: self.send_timer.cancel()
        if self.heartbeat_check_timer: self.heartbeat_check_timer.cancel()

        # Wait for the read_thread to finish
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
            if self.read_thread.is_alive():
                self.get_logger().warn("Serial read thread did not terminate cleanly.")

        with self.serial_lock:
            if self.serial_port is not None and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                    self.get_logger().info(f"Closed serial port {self.serial_port_name}")
                except Exception as e:
                    self.get_logger().error(f"Error closing serial port during shutdown: {e}")
            self.serial_port = None
        self.get_logger().info("Shutdown complete.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArduinoBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("Keyboard interrupt. Shutting down.")
    except Exception as e:
        if node: node.get_logger().error(f"Unhandled exception in main: {e}", exc_info=True)
        else: print(f"Unhandled exception before node init: {e}")
    finally:
        if node:
            node.shutdown() # Custom shutdown logic
            if rclpy.ok(): node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 shutdown procedures complete.")

if __name__ == '__main__':
    main()
