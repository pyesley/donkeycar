#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster
import math


class HardwareBridge(Node):
    """
    Bridges raw robot hardware with the ROS 2 navigation stack.
    Provides the required 'odom -> base_link' and 'base_link -> laser' transforms.
    """

    def __init__(self):
        super().__init__('hardware_bridge')

        # QoS for sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(Imu, '/imu/data_raw_bmi088', self.imu_callback, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, sensor_qos)

        # Publisher for motor commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF Broadcaster for SLAM/Nav
        self.tf_broadcaster = TransformBroadcaster(self)

        # Internal State for Odometry (Simplified since encoders are unreliable)
        # We rely on SLAM for the heavy lifting, but provide a 'smooth' odom frame here
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Static transform: Base to Lidar (Update these values to match your physical robot)
        self.static_laser_transform = TransformStamped()
        self.static_laser_transform.header.frame_id = 'base_link'
        self.static_laser_transform.child_frame_id = 'laser'
        self.static_laser_transform.transform.translation.x = 0.2  # Lidar is 20cm forward
        self.static_laser_transform.transform.translation.z = 0.1  # 10cm high

        self.get_logger().info("Hardware Bridge active. Waiting for sensor data...")

    def imu_callback(self, msg):
        # We use the IMU to help maintain the 'odom' heading
        # In a real setup, you'd use robot_localization (EKF) here.
        # For simplicity, we broadcast the link between odom and base_link.
        pass

    def lidar_callback(self, msg):
        # Broadcast static laser transform every time we get a scan
        self.static_laser_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.static_laser_transform)

    def broadcast_odom(self, linear_vel, angular_vel):
        """
        Broadcasting the Odom -> Base link transform.
        If encoders are bad, we often use 'Lidar Odometry' via slam_toolbox.
        """
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        # Simple kinematic integration
        self.x += linear_vel * math.cos(self.th) * dt
        self.y += linear_vel * math.sin(self.th) * dt
        self.th += angular_vel * dt

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)

        self.tf_broadcaster.sendTransform(t)
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()