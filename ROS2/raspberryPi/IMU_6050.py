#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import smbus2
import time
import math
from sensor_msgs.msg import Imu

# MPU-6050 Register Addresses (from MPC6050_test.py)
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# MPU-6050 Sensor sensitivity (default, from MPC6050_test.py)
ACCEL_SCALE_FACTOR_DEFAULT = 16384.0  # LSB/g for +/- 2g
GYRO_SCALE_FACTOR_DEFAULT = 131.0     # LSB/deg/s for +/- 250 deg/s

class MPU6050PublisherNode(Node):
    def __init__(self):
        super().__init__('mpu6050_publisher')

        # Declare parameters
        self.declare_parameter('i2c_bus', 1, ParameterDescriptor(description='I2C bus number for MPU-6050'))
        self.declare_parameter('mpu6050_address', 0x68, ParameterDescriptor(description='I2C address of MPU-6050'))
        self.declare_parameter('publish_rate_hz', 200.0, ParameterDescriptor(description='Rate to publish IMU data (Hz)'))
        self.declare_parameter('imu_frame_id', 'imu_link_mpu6050', ParameterDescriptor(description='Frame ID for the IMU data'))
        self.declare_parameter('accel_scale_factor', ACCEL_SCALE_FACTOR_DEFAULT, ParameterDescriptor(description='Accelerometer scale factor (LSB/g)'))
        self.declare_parameter('gyro_scale_factor', GYRO_SCALE_FACTOR_DEFAULT, ParameterDescriptor(description='Gyroscope scale factor (LSB/deg/s)'))
        # Standard gravity constant
        self.STANDARD_GRAVITY = 9.80665 # m/s^2

        # Get parameters
        self.i2c_bus_num = self.get_parameter('i2c_bus').value
        self.mpu6050_addr = self.get_parameter('mpu6050_address').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.imu_frame = self.get_parameter('imu_frame_id').value
        self.accel_sf = self.get_parameter('accel_scale_factor').value
        self.gyro_sf = self.get_parameter('gyro_scale_factor').value

        # I2C bus setup
        self.bus = None
        self.mpu6050_initialized = False

        # Publisher
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw_mpu6050', 10) # Using a distinct topic name

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu_data)

        self.get_logger().info(f"MPU-6050 Publisher node starting. I2C Bus: {self.i2c_bus_num}, Address: {hex(self.mpu6050_addr)}")
        self.init_mpu6050_sensor()

    def _read_word(self, reg):
        """Reads a 16-bit word from two 8-bit registers.""" #
        high = self.bus.read_byte_data(self.mpu6050_addr, reg)
        low = self.bus.read_byte_data(self.mpu6050_addr, reg + 1)
        value = (high << 8) + low
        return value

    def _read_word_2c(self, reg):
        """Reads a 16-bit signed word (2's complement).""" #
        val = self._read_word(reg)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def init_mpu6050_sensor(self):
        try:
            self.bus = smbus2.SMBus(self.i2c_bus_num)
            # Wake up MPU-6050 (set PWR_MGMT_1 to 0)
            self.bus.write_byte_data(self.mpu6050_addr, PWR_MGMT_1, 0)
            self.mpu6050_initialized = True
            self.get_logger().info("MPU-6050 initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Error initializing MPU-6050: {e}")
            self.get_logger().warn("Please check wiring, I2C address, and ensure 'i2c-tools' and 'python3-smbus2' are installed.")
            self.mpu6050_initialized = False

    def get_mpu6050_data_raw(self):
        """Reads accelerometer and gyroscope data from MPU-6050.""" #
        if not self.mpu6050_initialized:
            return None

        try:
            # Read accelerometer values (raw)
            accel_x_raw = self._read_word_2c(ACCEL_XOUT_H)
            accel_y_raw = self._read_word_2c(ACCEL_XOUT_H + 2)
            accel_z_raw = self._read_word_2c(ACCEL_XOUT_H + 4)

            # Read gyroscope values (raw)
            gyro_x_raw = self._read_word_2c(GYRO_XOUT_H)
            gyro_y_raw = self._read_word_2c(GYRO_XOUT_H + 2)
            gyro_z_raw = self._read_word_2c(GYRO_XOUT_H + 4)

            # Convert to physical units
            # Accelerometer: raw_value / scale_factor_g * standard_gravity = m/s^2
            ax = (accel_x_raw / self.accel_sf) * self.STANDARD_GRAVITY
            ay = (accel_y_raw / self.accel_sf) * self.STANDARD_GRAVITY
            az = (accel_z_raw / self.accel_sf) * self.STANDARD_GRAVITY

            # Gyroscope: raw_value / scale_factor_deg_s * (pi/180) = rad/s
            gx = (gyro_x_raw / self.gyro_sf) * (math.pi / 180.0)
            gy = (gyro_y_raw / self.gyro_sf) * (math.pi / 180.0)
            gz = (gyro_z_raw / self.gyro_sf) * (math.pi / 180.0)

            return ax, ay, az, gx, gy, gz
        except Exception as e:
            self.get_logger().error(f"Error reading MPU-6050 data: {e}")
            # Attempt to re-initialize in case of I2C error
            self.mpu6050_initialized = False
            self.init_mpu6050_sensor()
            return None

    def publish_imu_data(self):
        if not self.mpu6050_initialized:
            # self.get_logger().warn("MPU-6050 not initialized. Skipping data publish.")
            # Try to reinitialize if it failed before
            if not self.bus: # If bus was never opened
                 self.init_mpu6050_sensor()
            return

        data = self.get_mpu6050_data_raw()
        if data:
            ax, ay, az, gx, gy, gz = data

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame

            imu_msg.linear_acceleration.x = float(ax)
            imu_msg.linear_acceleration.y = float(ay)
            imu_msg.linear_acceleration.z = float(az)
            # Example covariances (dialogue with user suggests these are placeholders)
            imu_msg.linear_acceleration_covariance[0] = 0.01 # X
            imu_msg.linear_acceleration_covariance[4] = 0.01 # Y
            imu_msg.linear_acceleration_covariance[8] = 0.01 # Z

            imu_msg.angular_velocity.x = float(gx)
            imu_msg.angular_velocity.y = float(gy)
            imu_msg.angular_velocity.z = float(gz)
            imu_msg.angular_velocity_covariance[0] = 0.005 # X
            imu_msg.angular_velocity_covariance[4] = 0.005 # Y
            imu_msg.angular_velocity_covariance[8] = 0.005 # Z

            # Orientation is not directly provided by MPU-6050 raw data without fusion
            # Set orientation_covariance[0] to -1 to indicate orientation is not available
            imu_msg.orientation_covariance[0] = -1.0

            self.imu_publisher.publish(imu_msg)
            # self.get_logger().debug(f"Published IMU: A=({ax:.2f},{ay:.2f},{az:.2f}), G=({gx:.2f},{gy:.2f},{gz:.2f})")

    def shutdown(self):
        self.get_logger().info("MPU-6050 Publisher node shutting down...")
        if self.timer:
            self.timer.cancel()
        if self.bus:
            try:
                self.bus.close()
                self.get_logger().info("I2C bus closed.")
            except Exception as e:
                self.get_logger().error(f"Error closing I2C bus: {e}")

def main(args=None):
    rclpy.init(args=args)
    mpu6050_node = None
    try:
        mpu6050_node = MPU6050PublisherNode()
        rclpy.spin(mpu6050_node)
    except KeyboardInterrupt:
        if mpu6050_node: mpu6050_node.get_logger().info("Keyboard interrupt. Shutting down MPU-6050 node.")
    except Exception as e:
        if mpu6050_node: mpu6050_node.get_logger().error(f"Unhandled exception in MPU-6050 node: {e}", exc_info=True)
        else: print(f"Unhandled exception before MPU-6050 node init: {e}")
    finally:
        if mpu6050_node:
            mpu6050_node.shutdown()
            if rclpy.ok(): mpu6050_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("MPU-6050 ROS 2 node shutdown complete.")

if __name__ == '__main__':
    main()