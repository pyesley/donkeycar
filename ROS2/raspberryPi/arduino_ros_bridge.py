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

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

CMD_START_BYTE_1 = 0xA5
CMD_START_BYTE_2 = 0x5A
CMD_PACKET_FORMAT = '<hh'
CMD_PACKET_SIZE = struct.calcsize(CMD_PACKET_FORMAT)

SENSOR_START_BYTE_1 = 0xB6
SENSOR_START_BYTE_2 = 0x6B
SENSOR_PACKET_FORMAT = '<i'
SENSOR_PACKET_SIZE = struct.calcsize(SENSOR_PACKET_FORMAT)

ARDUINO_TO_PI_HEARTBEAT_PING = 0xC1
PI_TO_ARDUINO_HEARTBEAT_PONG = 0xD1
ARDUINO_READY_MESSAGE = "ARDUINO_READY"

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

        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 57600 ) # Baud rate corrected
        self.declare_parameter('throttle_axis_scale', 255.0)
        self.declare_parameter('steering_max_pwm_left', 0)
        self.declare_parameter('steering_max_pwm_right', 80)
        self.declare_parameter('steering_max_rad_per_sec', 1.0)
        self.declare_parameter('encoder_meters_per_tick', 0.0003115)
        self.declare_parameter('loop_rate_hz', 50.0)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('odom_child_frame_id', 'base_link')
        self.declare_parameter('use_arduino_ready_signal', False)
        self.declare_parameter('arduino_heartbeat_timeout_sec', 5.0)

        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.throttle_scale = self.get_parameter('throttle_axis_scale').value
        self.send_rate = self.get_parameter('send_rate_hz').value
        self.encoder_m_per_tick = self.get_parameter('encoder_meters_per_tick').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.odom_child_frame = self.get_parameter('odom_child_frame_id').value
        self.wait_for_ready_signal = self.get_parameter('use_arduino_ready_signal').value
        self.loop_rate_hz = self.get_parameter('loop_rate_hz').value
        self.arduino_heartbeat_timeout_duration = self.get_parameter('arduino_heartbeat_timeout_sec').value

        self.update_steering_params()
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.serial_buffer = bytearray()
        self.last_twist_msg = Twist() # Initialized to zeros
        self.command_lock = threading.Lock()

        self.x_pos, self.y_pos, self.theta_pos = 0.0, 0.0, 0.0
        self.last_enc_ticks = 0
        self.last_enc_time = self.get_clock().now()
        self.first_encoder_msg_after_connect = True

        self.last_arduino_ping_time = time.time()
        self.arduino_is_connected = False
        self.connection_attempt_pending = False

        self.running = True
        self.loop_rate_limiter = self.create_rate(self.loop_rate_hz)
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.send_timer = self.create_timer(1.0 / self.send_rate, self.send_command_callback)
        self.heartbeat_check_timer = self.create_timer(1.0, self.check_arduino_heartbeat)

        self.get_logger().info(f"Arduino Bridge node starting. Port: {self.serial_port_name}@{self.baud_rate}")
        self.read_thread.start()

    def parameters_callback(self, params): # From original file
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
            elif param.name == 'baud_rate':
                new_baud_rate = param.value
                if new_baud_rate != self.baud_rate:
                    self.get_logger().info(f"Baud rate parameter changed to: {new_baud_rate}. Serial port will be reset.")
                    self.baud_rate = new_baud_rate
                    self.handle_serial_error() # This will close the port and trigger a reconnect with the new baud rate.
                else:
                    self.get_logger().info(f"Baud rate parameter set to current value: {new_baud_rate}. No change.")


        if steering_changed:
            self.update_steering_params()
            self.get_logger().info(
                f"Steering params updated. PWM: L={self.pwm_left}, R={self.pwm_right}, C={self.pwm_center:.1f}")
        return SetParametersResult(successful=success)

    def update_steering_params(self): # From original file
        self.pwm_left = self.get_parameter('steering_max_pwm_left').value
        self.pwm_right = self.get_parameter('steering_max_pwm_right').value
        self.max_rad_per_sec = self.get_parameter('steering_max_rad_per_sec').value
        self.pwm_center = (self.pwm_right + self.pwm_left) / 2.0
        self.pwm_range = abs(self.pwm_right - self.pwm_center)
        if self.max_rad_per_sec <= 1e-6:
            self.rad_to_pwm_scale = 0.0
        else:
            self.rad_to_pwm_scale = self.pwm_range / self.max_rad_per_sec

    def reset_odometry_for_new_connection(self): # From original file
        self.get_logger().info("Resetting odometry state for new/re-established connection.")
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta_pos = 0.0
        self.last_enc_ticks = 0
        self.first_encoder_msg_after_connect = True
        self.last_enc_time = self.get_clock().now()

    def connect_serial(self): # From original file
        if self.serial_port is not None and self.serial_port.is_open:
            return True
        if self.connection_attempt_pending:
            return False

        self.connection_attempt_pending = True
        self.get_logger().info(f"Attempting to connect to Arduino on {self.serial_port_name} at {self.baud_rate} baud...")
        try:
            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
                self.serial_port = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0.05)

            self.get_logger().info(f"Successfully opened serial port {self.serial_port_name}.") # Corrected log message from log2.txt
            self.serial_port.reset_input_buffer()

            if self.wait_for_ready_signal:
                self.get_logger().info(f"Waiting for '{ARDUINO_READY_MESSAGE}' signal from Arduino...")
                time.sleep(1.0)

            self.arduino_is_connected = False
            self.last_arduino_ping_time = time.time()
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
        # --- MODIFIED LOGGING FOR HIGHER PRECISION ---
        #self.get_logger().info(f"Received Twist command: Linear X: {msg.linear.x:.8f}, Angular Z: {msg.angular.z:.4f}")

    def send_command_callback(self):
        if not self.arduino_is_connected:
            return

        with self.command_lock:
            twist_to_send = self.last_twist_msg

        clamped_angular_z = np.clip(twist_to_send.angular.z, -self.max_rad_per_sec, self.max_rad_per_sec)
        pwm_deviation = -clamped_angular_z * self.rad_to_pwm_scale
        target_pwm = self.pwm_center + pwm_deviation
        steering_pwm = int(np.clip(target_pwm, min(self.pwm_left, self.pwm_right), max(self.pwm_left, self.pwm_right)))

        throttle_pwm = int(np.clip(twist_to_send.linear.x, -1.0, 1.0) * self.throttle_scale)

        # --- ADDED LOGGING FOR PWM VALUES AND SOURCE LINEAR.X ---
        #self.get_logger().info(f"Preparing to send to Arduino: Steering PWM: {steering_pwm}, Throttle PWM: {throttle_pwm} (from linear.x: {twist_to_send.linear.x:.8f})")

        try:
            payload_bytes = struct.pack(CMD_PACKET_FORMAT, steering_pwm, throttle_pwm)
            checksum = self.calculate_checksum(payload_bytes)
            packet = bytes([CMD_START_BYTE_1, CMD_START_BYTE_2]) + payload_bytes + bytes([checksum])

            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.write(packet)
                else:
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

    def calculate_checksum(self, data_bytes): # From original file
        checksum = 0
        for byte_val in data_bytes:
            checksum ^= byte_val
        return checksum

    def handle_serial_error(self): # From original file
        self.get_logger().info("Serial error or PING timeout. Closing port and marking Arduino as disconnected.")
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except Exception as e_close:
                    self.get_logger().error(f"Exception while closing serial port: {e_close}")
            self.serial_port = None
        self.arduino_is_connected = False

    def check_arduino_heartbeat(self): # From original file
        if self.serial_port and self.serial_port.is_open:
            if self.arduino_is_connected:
                if (time.time() - self.last_arduino_ping_time) > self.arduino_heartbeat_timeout_duration:
                    self.get_logger().warn(
                        f"Arduino PING timeout (last PING {time.time() - self.last_arduino_ping_time:.2f}s ago). Assuming disconnection.")
                    self.handle_serial_error()

    def serial_read_loop(self): # From original file (with minor debug log adjustment)
        packet_parser_state = 0
        sensor_data_buffer = bytearray()
        self.get_logger().info("Serial read loop started.")
        while rclpy.ok() and self.running:
            if self.serial_port is None or not self.serial_port.is_open:
                if not self.connection_attempt_pending:
                    if not self.connect_serial():
                        time.sleep(1.0)
                else:
                    time.sleep(0.1)
                continue
            try:
                bytes_available = 0
                with self.serial_lock:
                    if self.serial_port and self.serial_port.is_open:
                         bytes_available = self.serial_port.in_waiting
                if bytes_available > 0:
                    with self.serial_lock:
                        read_bytes = self.serial_port.read(min(bytes_available, 128))
                    if read_bytes:
                        self.serial_buffer.extend(read_bytes)
                        # self.get_logger().debug(f"Read {len(read_bytes)} bytes. Buffer now: {self.serial_buffer.hex()}") # Can be very verbose

                i = 0
                processed_up_to = -1
                while i < len(self.serial_buffer):
                    byte = self.serial_buffer[i]
                    if byte == ARDUINO_TO_PI_HEARTBEAT_PING:
                        if not self.arduino_is_connected:
                             self.get_logger().info("Arduino PING received. Connection ESTABLISHED.")
                             self.reset_odometry_for_new_connection()
                        self.last_arduino_ping_time = time.time()
                        self.arduino_is_connected = True
                        try:
                            with self.serial_lock:
                                if self.serial_port and self.serial_port.is_open:
                                    self.serial_port.write(bytes([PI_TO_ARDUINO_HEARTBEAT_PONG]))
                        except serial.SerialException as e:
                            self.get_logger().error(f"Serial write error for PONG: {e}")
                            self.handle_serial_error()
                            processed_up_to = i
                            break
                        except Exception as e_pong:
                            self.get_logger().error(f"Unexpected error sending PONG: {e_pong}")
                            self.handle_serial_error()
                            processed_up_to = i
                            break
                        packet_parser_state = 0
                        sensor_data_buffer.clear()
                        processed_up_to = i
                        i += 1
                        continue

                    if packet_parser_state == 0:
                        if byte == SENSOR_START_BYTE_1:
                            packet_parser_state = 1
                    elif packet_parser_state == 1:
                        if byte == SENSOR_START_BYTE_2:
                            packet_parser_state = 2
                            sensor_data_buffer.clear()
                        elif byte == SENSOR_START_BYTE_1:
                            pass
                        else:
                            packet_parser_state = 0
                    elif packet_parser_state == 2:
                        sensor_data_buffer.append(byte)
                        if len(sensor_data_buffer) == SENSOR_PACKET_SIZE:
                            packet_parser_state = 3
                    elif packet_parser_state == 3:
                        received_checksum = byte
                        calculated_checksum = self.calculate_checksum(sensor_data_buffer)
                        if received_checksum == calculated_checksum:
                            if self.arduino_is_connected:
                                self.process_sensor_packet(bytes(sensor_data_buffer))
                        else:
                            self.get_logger().warn(
                                f"Sensor packet checksum mismatch! Got: {received_checksum:02X}, Calc: {calculated_checksum:02X}, Data: {sensor_data_buffer.hex()}. Discarding.")
                        packet_parser_state = 0
                        sensor_data_buffer.clear()
                    processed_up_to = i
                    i += 1
                if processed_up_to >= 0:
                    self.serial_buffer = self.serial_buffer[processed_up_to + 1:]
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}.")
                self.handle_serial_error()
            except Exception as e:
                self.get_logger().error(f"Unexpected error in serial read loop: {e}", exc_info=True)
                packet_parser_state = 0
                sensor_data_buffer.clear()
                self.serial_buffer.clear()
                self.handle_serial_error()
            self.loop_rate_limiter.sleep()

    def process_sensor_packet(self, data_bytes): # From original file
        current_time_rclpy = self.get_clock().now()
        try:
            (current_enc_ticks,) = struct.unpack(SENSOR_PACKET_FORMAT, data_bytes)
            if self.first_encoder_msg_after_connect:
                self.last_enc_ticks = current_enc_ticks
                self.last_enc_time = current_time_rclpy
                self.first_encoder_msg_after_connect = False
                self.get_logger().info(f"First encoder message after (re)connect. Initial ticks: {current_enc_ticks}")
                return

            delta_time_nanosec = (current_time_rclpy - self.last_enc_time).nanoseconds
            if delta_time_nanosec <= 1000:
                if current_enc_ticks != self.last_enc_ticks:
                     self.get_logger().warn(f"Encoder ticks changed ({self.last_enc_ticks} -> {current_enc_ticks}) but delta_time is too small ({delta_time_nanosec / 1e9:.6f}s). Resetting odom 'first_encoder_msg' flag.")
                     self.first_encoder_msg_after_connect = True
                return

            delta_time_sec = delta_time_nanosec / 1e9
            delta_ticks = current_enc_ticks - self.last_enc_ticks
            delta_distance = delta_ticks * self.encoder_m_per_tick
            vx = delta_distance / delta_time_sec

            with self.command_lock:
                vth = self.last_twist_msg.angular.z

            delta_x = vx * math.cos(self.theta_pos + (vth * delta_time_sec / 2.0)) * delta_time_sec
            delta_y = vx * math.sin(self.theta_pos + (vth * delta_time_sec / 2.0)) * delta_time_sec
            delta_theta = vth * delta_time_sec

            self.x_pos += delta_x
            self.y_pos += delta_y
            self.theta_pos += delta_theta
            self.theta_pos = math.atan2(math.sin(self.theta_pos), math.cos(self.theta_pos))

            odom_msg = Odometry()
            odom_msg.header.stamp = current_time_rclpy.to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.odom_child_frame
            odom_msg.pose.pose.position.x = self.x_pos
            odom_msg.pose.pose.position.y = self.y_pos
            odom_msg.pose.pose.orientation = quaternion_from_yaw(self.theta_pos)
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.angular.z = vth
            self.odom_publisher.publish(odom_msg)

            self.last_enc_ticks = current_enc_ticks
            self.last_enc_time = current_time_rclpy

        except struct.error as e:
            self.get_logger().error(f"Failed to unpack sensor packet: {e}. Data: {data_bytes.hex()}")
            self.first_encoder_msg_after_connect = True
        except Exception as e:
            self.get_logger().error(f"Error processing sensor data: {e}", exc_info=True)
            self.first_encoder_msg_after_connect = True

    def shutdown(self): # From original file
        self.get_logger().info("Arduino Bridge node shutting down...")
        self.running = False
        if self.send_timer: self.send_timer.cancel()
        if self.heartbeat_check_timer: self.heartbeat_check_timer.cancel()

        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
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

def main(args=None): # From original file
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
            node.shutdown()
            if rclpy.ok(): node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 shutdown procedures complete.")

if __name__ == '__main__':
    main()