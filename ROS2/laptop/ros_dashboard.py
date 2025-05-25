#!/usr/bin/env python3
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

from PySide6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import numpy as np

# --- Configuration ---
MAX_PLOT_POINTS = 200  # Number of points for rolling plots
CAMERA_TOPIC = '/camera/image_raw'
IMU_TOPIC = '/imu/data_raw_mpu6050'
ENCODER_TOPIC = '/encoder_ticks'
CMD_VEL_TOPIC = '/cmd_vel'
PUBLISH_RATE_HZ = 10.0 # Rate to continuously publish commands

# Adjust these values based on your robot's capabilities
LINEAR_SPEED = 0.6  # m/s
ANGULAR_SPEED = 0.5 # rad/s

# --- ROS 2 Node ---
class RobotControlNode(Node, QtCore.QObject):
    # Qt Signals for thread-safe GUI updates
    new_image_data = QtCore.Signal(np.ndarray)
    new_imu_data = QtCore.Signal(Imu)
    new_encoder_data = QtCore.Signal(Int32)

    def __init__(self):
        Node.__init__(self, 'robot_control_dashboard_node')
        QtCore.QObject.__init__(self) # Initialize QObject base class for signals

        # QoS Profiles (unchanged)
        qos_profile_sensor_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_profile_camera_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_cmd_vel_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers (unchanged)
        self.image_subscription = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, qos_profile_camera_best_effort)
        self.imu_subscription = self.create_subscription(
            Imu, IMU_TOPIC, self.imu_callback, qos_profile_sensor_reliable)
        self.encoder_subscription = self.create_subscription(
            Int32, ENCODER_TOPIC, self.encoder_callback, qos_profile_sensor_reliable)

        # Publisher (unchanged)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, CMD_VEL_TOPIC, qos_profile_cmd_vel_reliable)

        self.bridge = CvBridge()

        # --- State for continuous publishing ---
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.command_lock = threading.Lock() # Protect access to target speeds

        # --- Timer for continuous command publishing ---
        self.publish_timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ, self.publish_current_command
        )

        self.get_logger().info(f"Robot Control Dashboard Node started.")
        self.get_logger().info(f"Listening for Image on '{CAMERA_TOPIC}'")
        self.get_logger().info(f"Listening for IMU on '{IMU_TOPIC}'")
        self.get_logger().info(f"Listening for Encoder on '{ENCODER_TOPIC}'")
        self.get_logger().info(f"Publishing commands to '{CMD_VEL_TOPIC}' at {PUBLISH_RATE_HZ} Hz")

        # REMOVED: self.stop_timer and related logic

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.new_image_data.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def imu_callback(self, msg):
        self.new_imu_data.emit(msg)

    def encoder_callback(self, msg):
        self.new_encoder_data.emit(msg)

    # --- NEW: Method to update the target speed ---
    def set_target_command(self, linear_x, angular_z):
        """Updates the target speeds to be published continuously."""
        with self.command_lock:
            self.target_linear_x = float(linear_x)
            self.target_angular_z = float(angular_z)
        # self.get_logger().debug(f"Set target command: lin_x={self.target_linear_x}, ang_z={self.target_angular_z}")

    # --- NEW: Method called by the timer to publish ---
    def publish_current_command(self):
        """Publishes the current target Twist message."""
        twist_msg = Twist()
        with self.command_lock:
            twist_msg.linear.x = self.target_linear_x
            twist_msg.angular.z = self.target_angular_z
        # Ensure other components are zero
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0

        self.cmd_vel_publisher.publish(twist_msg)

    # REMOVED: publish_stop_if_idle method

    def shutdown(self):
        self.get_logger().info("Shutting down ROS node.")
        # Publish a final stop command by setting target to zero
        self.set_target_command(0.0, 0.0)
        self.publish_current_command() # Send it immediately
        time.sleep(0.2) # Allow time for publish and timer to potentially cycle once more
        self.destroy_node()

# --- PyQtGraph/PySide6 GUI Window ---
class RobotControlWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("Robot Control Dashboard")
        self.setGeometry(50, 50, 1200, 800) # x, y, width, height

        # Data Storage (unchanged)
        self.timestamps = deque(maxlen=MAX_PLOT_POINTS)
        self.accel_x = deque(maxlen=MAX_PLOT_POINTS)
        self.accel_y = deque(maxlen=MAX_PLOT_POINTS)
        self.accel_z = deque(maxlen=MAX_PLOT_POINTS)
        self.gyro_x = deque(maxlen=MAX_PLOT_POINTS)
        self.gyro_y = deque(maxlen=MAX_PLOT_POINTS)
        self.gyro_z = deque(maxlen=MAX_PLOT_POINTS)
        self.encoder_ticks = deque(maxlen=MAX_PLOT_POINTS)
        self.sample_count = 0
        self.pressed_keys = set()

        # GUI Layout (unchanged, except maybe add publish rate to status/label)
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)
        left_pane_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_pane_widget)
        main_layout.addWidget(left_pane_widget, 1)
        self.camera_label = QtWidgets.QLabel("Waiting for camera feed...")
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setMinimumSize(320, 240)
        self.camera_label.setStyleSheet("border: 1px solid black; background-color: #333;")
        left_layout.addWidget(self.camera_label)
        self.command_label = QtWidgets.QLabel()
        self.command_label.setAlignment(QtCore.Qt.AlignCenter)
        self.command_label.setFont(QtGui.QFont("Monospace", 10))
        self.command_label.setText(self._get_command_text())
        left_layout.addWidget(self.command_label)
        self.statusBar = QtWidgets.QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Ready. Press keys to control. Press ESC to exit.")
        right_pane_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_pane_widget)
        main_layout.addWidget(right_pane_widget, 2)
        pg.setConfigOptions(antialias=True, background='w', foreground='k')
        self.accel_plot_widget = pg.PlotWidget()
        right_layout.addWidget(self.accel_plot_widget)
        self.accel_plot_widget.addLegend(offset=(-10, 10))
        self.accel_curve_x = self.accel_plot_widget.plot(pen=pg.mkPen('r', width=2), name="Accel X")
        self.accel_curve_y = self.accel_plot_widget.plot(pen=pg.mkPen('g', width=2), name="Accel Y")
        self.accel_curve_z = self.accel_plot_widget.plot(pen=pg.mkPen('b', width=2), name="Accel Z")
        self.setup_plot(self.accel_plot_widget, "Linear Acceleration", "m/s^2")
        self.gyro_plot_widget = pg.PlotWidget()
        right_layout.addWidget(self.gyro_plot_widget)
        self.gyro_plot_widget.addLegend(offset=(-10, 10))
        self.gyro_curve_x = self.gyro_plot_widget.plot(pen=pg.mkPen('r', width=2), name="Gyro X")
        self.gyro_curve_y = self.gyro_plot_widget.plot(pen=pg.mkPen('g', width=2), name="Gyro Y")
        self.gyro_curve_z = self.gyro_plot_widget.plot(pen=pg.mkPen('b', width=2), name="Gyro Z")
        self.setup_plot(self.gyro_plot_widget, "Angular Velocity", "rad/s")
        self.encoder_plot_widget = pg.PlotWidget()
        right_layout.addWidget(self.encoder_plot_widget)
        self.encoder_curve = self.encoder_plot_widget.plot(pen=pg.mkPen('m', width=2), name="Encoder Ticks")
        self.setup_plot(self.encoder_plot_widget, "Encoder Count", "Ticks")
        self.encoder_plot_widget.addLegend(offset=(-10, 10))

        # Connect ROS Signals (unchanged)
        self.ros_node.new_image_data.connect(self.update_camera_feed)
        self.ros_node.new_imu_data.connect(self.update_imu_data)
        self.ros_node.new_encoder_data.connect(self.update_encoder_data)

        self.setFocusPolicy(QtCore.Qt.StrongFocus)

    def _get_command_text(self): # Unchanged
        return """
Keyboard Controls:
------------------
       (FWD)
         W
      Q     E
(LEFT) A S D (RIGHT)
       (BWD)

 H: Steer Left (No Fwd/Bwd)
 J: Steer Right (No Fwd/Bwd)

 SPACE: STOP

 ESC: Exit Program
------------------
"""

    def setup_plot(self, plot_widget, title, ylabel): # Unchanged
        plot_widget.setTitle(title)
        plot_widget.setLabel('left', ylabel)
        plot_widget.setLabel('bottom', "Sample Number")
        plot_widget.showGrid(x=True, y=True)

    @QtCore.Slot(np.ndarray)
    def update_camera_feed(self, cv_image): # Unchanged
        try:
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(cv_image.data, w, h, bytes_per_line, QtGui.QImage.Format_BGR888)
            pixmap = QtGui.QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(self.camera_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            self.camera_label.setPixmap(scaled_pixmap)
        except Exception as e:
            self.get_ros_logger().error(f"Error updating camera feed display: {e}")

    @QtCore.Slot(Imu)
    def update_imu_data(self, msg): # Unchanged
        self.sample_count += 1
        self.timestamps.append(self.sample_count)
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)
        self.gyro_x.append(msg.angular_velocity.x)
        self.gyro_y.append(msg.angular_velocity.y)
        self.gyro_z.append(msg.angular_velocity.z)
        self.update_plots()

    @QtCore.Slot(Int32)
    def update_encoder_data(self, msg): # Unchanged
        if self.sample_count > len(self.encoder_ticks):
             self.encoder_ticks.append(msg.data)
             self.update_plots()

    def update_plots(self): # Unchanged
        time_axis = list(self.timestamps)
        self.accel_curve_x.setData(time_axis, list(self.accel_x))
        self.accel_curve_y.setData(time_axis, list(self.accel_y))
        self.accel_curve_z.setData(time_axis, list(self.accel_z))
        self.gyro_curve_x.setData(time_axis, list(self.gyro_x))
        self.gyro_curve_y.setData(time_axis, list(self.gyro_y))
        self.gyro_curve_z.setData(time_axis, list(self.gyro_z))
        encoder_time_axis = time_axis[:len(self.encoder_ticks)]
        self.encoder_curve.setData(encoder_time_axis, list(self.encoder_ticks))

    # Keyboard Event Handling (unchanged logic for adding/removing keys)
    def keyPressEvent(self, event):
        key = event.key()
        # --- Add this check: If the key is already pressed, do nothing ---
        if key in self.pressed_keys or event.isAutoRepeat():
             # Also ignore auto-repeat here explicitly
            return

        valid_key = True
        if key == QtCore.Qt.Key_Escape:
            self.get_ros_logger().info("Escape key pressed. Closing application.")
            self.close()
            return
        elif key == QtCore.Qt.Key_W: self.pressed_keys.add(QtCore.Qt.Key_W)
        elif key == QtCore.Qt.Key_S: self.pressed_keys.add(QtCore.Qt.Key_S)
        elif key == QtCore.Qt.Key_A: self.pressed_keys.add(QtCore.Qt.Key_A)
        elif key == QtCore.Qt.Key_D: self.pressed_keys.add(QtCore.Qt.Key_D)
        elif key == QtCore.Qt.Key_Q: self.pressed_keys.add(QtCore.Qt.Key_Q)
        elif key == QtCore.Qt.Key_E: self.pressed_keys.add(QtCore.Qt.Key_E)
        elif key == QtCore.Qt.Key_H: self.pressed_keys.add(QtCore.Qt.Key_H)
        elif key == QtCore.Qt.Key_J: self.pressed_keys.add(QtCore.Qt.Key_J)
        elif key == QtCore.Qt.Key_Space: self.pressed_keys.add(QtCore.Qt.Key_Space)
        else:
            valid_key = False
            super().keyPressEvent(event)

        if valid_key:
            # --- Process keys immediately on press ---
            self.process_keys()

    def keyReleaseEvent(self, event):
        key = event.key()
        if event.isAutoRepeat():
            return

        # --- Check if the key was actually in our set before removing ---
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            # --- Process keys immediately on release ---
            self.process_keys()
        else:
            super().keyReleaseEvent(event)

    # --- MODIFIED: process_keys now updates the *target* speed ---
    def process_keys(self):
        """Calculate TARGET Twist command based on currently pressed keys."""
        lin_x = 0.0
        ang_z = 0.0

        # Use
        # Qt.Key constants for checking the set
        key_map = {
            'q': QtCore.Qt.Key_Q, 'w': QtCore.Qt.Key_W, 'e': QtCore.Qt.Key_E,
            'a': QtCore.Qt.Key_A, 's': QtCore.Qt.Key_S, 'd': QtCore.Qt.Key_D,
            'h': QtCore.Qt.Key_H, 'j': QtCore.Qt.Key_J, 'space': QtCore.Qt.Key_Space
        }

        if key_map['space'] in self.pressed_keys:
            lin_x = 0.0
            ang_z = 0.0
            self.statusBar.showMessage("STOPPED (Spacebar)")
        # --- Swapped ANGULAR_SPEED sign for Q and E ---
        elif key_map['q'] in self.pressed_keys:
            lin_x = LINEAR_SPEED
            ang_z = ANGULAR_SPEED # Positive for left turn
            self.statusBar.showMessage("Forward-Left (Q)")
        elif key_map['w'] in self.pressed_keys:
            lin_x = LINEAR_SPEED
            ang_z = 0.0
            self.statusBar.showMessage("Forward (W)")
        elif key_map['e'] in self.pressed_keys:
            lin_x = LINEAR_SPEED
            ang_z = -ANGULAR_SPEED # Negative for right turn
            self.statusBar.showMessage("Forward-Right (E)")
        elif key_map['a'] in self.pressed_keys:
            lin_x = -LINEAR_SPEED
            ang_z = ANGULAR_SPEED # Positive for left turn
            self.statusBar.showMessage("Backward-Left (A)")
        elif key_map['s'] in self.pressed_keys:
            lin_x = -LINEAR_SPEED
            ang_z = 0.0
            self.statusBar.showMessage("Backward (S)")
        elif key_map['d'] in self.pressed_keys:
            lin_x = -LINEAR_SPEED
            ang_z = -ANGULAR_SPEED # Negative for right turn
            self.statusBar.showMessage("Backward-Right (D)")
        elif key_map['h'] in self.pressed_keys:
            lin_x = 0.0
            ang_z = ANGULAR_SPEED # Positive for left turn
            self.statusBar.showMessage("Steer Left (H)")
        elif key_map['j'] in self.pressed_keys:
            lin_x = 0.0
            ang_z = -ANGULAR_SPEED # Negative for right turn
            self.statusBar.showMessage("Steer Right (J)")
        else:
            # No movement keys pressed
            lin_x = 0.0
            ang_z = 0.0
            self.statusBar.showMessage("Ready. Press keys to control. Press ESC to exit.")

        # --- Update the target command in the ROS Node ---
        self.ros_node.set_target_command(lin_x, ang_z)

    def get_ros_logger(self): # Unchanged
        return self.ros_node.get_logger()

    def closeEvent(self, event): # Unchanged
        self.get_ros_logger().info("GUI window closing.")
        event.accept()


# --- Main Execution (unchanged) ---
def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    ros_node = None
    window = None
    ros_thread = None
    exit_code = 0

    try:
        ros_node = RobotControlNode()
        window = RobotControlWindow(ros_node)
        ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
        ros_thread.start()
        window.show()
        exit_code = app.exec()

    except Exception as e:
        if ros_node:
            ros_node.get_logger().fatal(f"Unhandled exception in GUI or setup: {e}")
        else:
            print(f"Unhandled exception before ROS node init: {e}")
        exit_code = 1
    finally:
        print("Application exiting...")
        if ros_node is not None and rclpy.ok():
            ros_node.shutdown()
            if ros_thread and ros_thread.is_alive():
                 ros_thread.join(timeout=0.5)
        if rclpy.ok():
            print("Shutting down rclpy...")
            rclpy.shutdown()
        if ros_thread and ros_thread.is_alive():
            print("ROS thread did not exit cleanly.")
        print("Exiting.")
        sys.exit(exit_code)

if __name__ == '__main__':
    main()