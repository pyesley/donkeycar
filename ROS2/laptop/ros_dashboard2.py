#!/usr/bin/env python3
import sys
import threading
import time
from collections import deque
import logging # Added for logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Int32 # Keep this if you still want raw encoder plot
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError # Import CvBridgeError

# --- NEW: Import LineFollowerController ---
from line_follower_controller import LineFollowerController
# ---

from PySide6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import numpy as np

# --- Configuration ---
MAX_PLOT_POINTS = 200
CAMERA_TOPIC = '/camera/image_raw'
IMU_TOPIC = '/imu/data_raw_mpu6050'
ENCODER_TOPIC = '/encoder_ticks' # Topic for raw ticks if still plotted
# ODOM_TOPIC = '/odom' # If you switched to publishing Odometry
CMD_VEL_TOPIC = '/cmd_vel'
PUBLISH_RATE_HZ = 10.0 # Rate for ROS Node command publishing

# Manual Control Speeds
MANUAL_LINEAR_SPEED = 0.6  # m/s (Adjust as needed)
MANUAL_ANGULAR_SPEED = 0.5 # rad/s (Adjust as needed)

# Line Follower Speed
LINE_FOLLOW_LINEAR_SPEED = 0.6 # Constant forward speed during line following


# --- ROS 2 Node (Largely Unchanged) ---
class RobotControlNode(Node, QtCore.QObject):
    # Qt Signals for thread-safe GUI updates
    # Send raw CV image AND ROS Image msg timestamp for timing PID
    new_image_data = QtCore.Signal(np.ndarray, float)
    new_imu_data = QtCore.Signal(Imu)
    new_encoder_data = QtCore.Signal(Int32) # Keep if plotting raw ticks
    # new_odom_data = QtCore.Signal(Odometry) # If plotting odom

    def __init__(self):
        Node.__init__(self, 'robot_control_dashboard_node')
        QtCore.QObject.__init__(self) # Initialize QObject base class for signals

        # QoS Profiles (Unchanged)
        qos_profile_sensor_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_profile_camera_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_profile_cmd_vel_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.image_subscription = self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, qos_profile_camera_best_effort)
        self.imu_subscription = self.create_subscription(Imu, IMU_TOPIC, self.imu_callback, qos_profile_sensor_reliable)
        # Choose one: Encoder Ticks OR Odometry
        self.encoder_subscription = self.create_subscription(Int32, ENCODER_TOPIC, self.encoder_callback, qos_profile_sensor_reliable)
        # self.odom_subscription = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, qos_profile_sensor_reliable)

        # Publisher (Unchanged)
        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos_profile_cmd_vel_reliable)

        self.bridge = CvBridge()

        # State for continuous publishing (Unchanged)
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.command_lock = threading.Lock()
        self.publish_timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_current_command)

        self.get_logger().info(f"Robot Control Dashboard Node started.")
        # ... other info logs ...

    def image_callback(self, msg):
        try:
            # Get image timestamp (use header stamp)
            timestamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Emit image and timestamp
            self.new_image_data.emit(cv_image, timestamp_sec)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
        except Exception as e:
             self.get_logger().error(f"Unexpected error in image callback: {e}", exc_info=True)


    def imu_callback(self, msg):
        self.new_imu_data.emit(msg)

    def encoder_callback(self, msg): # Keep if plotting raw ticks
        self.new_encoder_data.emit(msg)

    # def odom_callback(self, msg): # Use if plotting odom
    #    self.new_odom_data.emit(msg)

    # --- Control Methods (Unchanged) ---
    @QtCore.Slot(float, float) # Make this a slot to receive from GUI thread
    def set_target_command(self, linear_x, angular_z):
        with self.command_lock:
            self.target_linear_x = float(linear_x)
            self.target_angular_z = float(angular_z)

    def publish_current_command(self):
        twist_msg = Twist()
        with self.command_lock:
            twist_msg.linear.x = self.target_linear_x
            twist_msg.angular.z = self.target_angular_z
        twist_msg.linear.y = 0.0; twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0; twist_msg.angular.y = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    def shutdown(self):
        self.get_logger().info("Shutting down ROS node.")
        self.set_target_command(0.0, 0.0)
        self.publish_current_command()
        time.sleep(0.2)
        self.destroy_node()


# --- PyQtGraph/PySide6 GUI Window (Imports Controller) ---
class RobotControlWindow(QtWidgets.QMainWindow):
    # Signal to request command change from potentially different thread
    request_command = QtCore.Signal(float, float)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("Robot Control Dashboard")
        self.setGeometry(50, 50, 1200, 800)

        # Data Storage (unchanged)
        self.timestamps = deque(maxlen=MAX_PLOT_POINTS)
        self.accel_x = deque(maxlen=MAX_PLOT_POINTS); self.accel_y = deque(maxlen=MAX_PLOT_POINTS); self.accel_z = deque(maxlen=MAX_PLOT_POINTS)
        self.gyro_x = deque(maxlen=MAX_PLOT_POINTS); self.gyro_y = deque(maxlen=MAX_PLOT_POINTS); self.gyro_z = deque(maxlen=MAX_PLOT_POINTS)
        self.encoder_ticks = deque(maxlen=MAX_PLOT_POINTS)
        self.sample_count = 0

        # --- Control State ---
        self.pressed_keys = set()
        self.line_following_active = False # Start in manual mode
        self.last_image_time = 0.0
        self.last_valid_image_time = time.time() # Track when last good image arrived

        # --- Instantiate Line Follower ---
        # Pass the manual angular speed as the PID output limit
        self.line_follower = LineFollowerController(
            self.get_ros_logger(),
            pid_output_limit=MANUAL_ANGULAR_SPEED
        )
        self.latest_cv_image = None # Store latest image for processing

        # --- GUI Layout (Add Status Indicator for Mode) ---
        central_widget = QtWidgets.QWidget(); self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)
        left_pane_widget = QtWidgets.QWidget(); left_layout = QtWidgets.QVBoxLayout(left_pane_widget)
        main_layout.addWidget(left_pane_widget, 1)
        self.camera_label = QtWidgets.QLabel("Waiting for camera feed..."); self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setMinimumSize(320, 240); self.camera_label.setStyleSheet("border: 1px solid black; background-color: #333;")
        left_layout.addWidget(self.camera_label)
        self.command_label = QtWidgets.QLabel(); self.command_label.setAlignment(QtCore.Qt.AlignCenter)
        self.command_label.setFont(QtGui.QFont("Monospace", 10)); left_layout.addWidget(self.command_label)
        self.statusBar = QtWidgets.QStatusBar(); self.setStatusBar(self.statusBar)
        self.mode_label = QtWidgets.QLabel("Mode: MANUAL"); self.mode_label.setAlignment(QtCore.Qt.AlignCenter)
        font = self.mode_label.font(); font.setPointSize(12); font.setBold(True); self.mode_label.setFont(font)
        self.mode_label.setStyleSheet("color: blue; padding: 5px;"); left_layout.addWidget(self.mode_label)
        left_layout.addStretch()
        right_pane_widget = QtWidgets.QWidget(); right_layout = QtWidgets.QVBoxLayout(right_pane_widget)
        main_layout.addWidget(right_pane_widget, 2)
        pg.setConfigOptions(antialias=True, background='w', foreground='k')
        self.accel_plot_widget = pg.PlotWidget(); right_layout.addWidget(self.accel_plot_widget)
        self.accel_plot_widget.addLegend(offset=(-10, 10))
        self.accel_curve_x = self.accel_plot_widget.plot(pen=pg.mkPen('r', width=2), name="Accel X")
        self.accel_curve_y = self.accel_plot_widget.plot(pen=pg.mkPen('g', width=2), name="Accel Y")
        self.accel_curve_z = self.accel_plot_widget.plot(pen=pg.mkPen('b', width=2), name="Accel Z")
        self.setup_plot(self.accel_plot_widget, "Linear Acceleration", "m/s^2")
        self.gyro_plot_widget = pg.PlotWidget(); right_layout.addWidget(self.gyro_plot_widget)
        self.gyro_plot_widget.addLegend(offset=(-10, 10))
        self.gyro_curve_x = self.gyro_plot_widget.plot(pen=pg.mkPen('r', width=2), name="Gyro X")
        self.gyro_curve_y = self.gyro_plot_widget.plot(pen=pg.mkPen('g', width=2), name="Gyro Y")
        self.gyro_curve_z = self.gyro_plot_widget.plot(pen=pg.mkPen('b', width=2), name="Gyro Z")
        self.setup_plot(self.gyro_plot_widget, "Angular Velocity", "rad/s")
        self.encoder_plot_widget = pg.PlotWidget(); right_layout.addWidget(self.encoder_plot_widget)
        self.encoder_curve = self.encoder_plot_widget.plot(pen=pg.mkPen('m', width=2), name="Encoder Ticks")
        self.setup_plot(self.encoder_plot_widget, "Encoder Count", "Ticks")
        self.encoder_plot_widget.addLegend(offset=(-10, 10))

        # --- Connect Signals ---
        self.ros_node.new_image_data.connect(self.handle_new_image)
        self.ros_node.new_imu_data.connect(self.update_imu_data)
        self.ros_node.new_encoder_data.connect(self.update_encoder_data)
        self.request_command.connect(self.ros_node.set_target_command) # Connect signal to node's slot

        # Set focus and update command text
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.update_command_text()

    # --- Methods (mostly unchanged, but logic moved/added) ---

    def update_command_text(self):
        """Updates the command text display including line follower keys."""
        base_text = """
Keyboard Controls (Manual Mode):
--------------------------------
       (FWD)
         W
      Q     E
(LEFT) A S D (RIGHT)
       (BWD)

 H: Steer Left (No Fwd/Bwd)
 J: Steer Right (No Fwd/Bwd)

 SPACE: STOP

 ESC: Exit Program
--------------------------------
Autonomous Mode:
 Y: Activate Line Follow
 U: Deactivate (Manual Control)
--------------------------------"""
        self.command_label.setText(base_text)

    def setup_plot(self, plot_widget, title, ylabel): # Unchanged
        plot_widget.setTitle(title); plot_widget.setLabel('left', ylabel)
        plot_widget.setLabel('bottom', "Sample Number"); plot_widget.showGrid(x=True, y=True)

    @QtCore.Slot(np.ndarray, float)
    def handle_new_image(self, cv_image, timestamp_sec):
        """Handles incoming images, updates display, and runs line follower if active."""
        if cv_image is None:  # Add check for None image
            self.get_ros_logger().warn("GUI received None image.")
            return

        self.latest_cv_image = cv_image  # Store original image if needed elsewhere
        self.last_image_time = timestamp_sec
        self.last_valid_image_time = time.time()  # Record time of valid image arrival

        # Assume image for display starts as the input image
        display_image = cv_image

        if self.line_following_active:
            # Call find_line_center - it now returns the processed image
            # *************************************************************************** #
            # *** CRITICAL CHANGE HERE: Receive the image directly from find_line_center *** #
            line_x, confidence, processed_image_with_contour = self.line_follower.find_line_center(cv_image)
            # *************************************************************************** #

            # Use the returned image for display if it's not None
            if processed_image_with_contour is not None:
                display_image = processed_image_with_contour
            else:
                # If find_line_center returns None for the image (e.g., on initial error),
                # fall back to the original cv_image for display.
                display_image = cv_image

            if line_x is not None:
                angular_z = self.line_follower.compute_steering_angle(line_x, timestamp_sec)

                # --- ADD THIS LOGGING LINE ---
                self.get_ros_logger().info(f"Line Follower Calculated: line_x={line_x}, angular_z={angular_z:.4f}")
                # ---


                linear_x = LINE_FOLLOW_LINEAR_SPEED
                #linear_x = 0
                self.request_command.emit(linear_x, angular_z)
                self.statusBar.showMessage(
                    f"Line Following: LineX={line_x}, Steering={angular_z:.2f}, Conf={confidence:.0f}")
                # Visualization is now already done within find_line_center on 'display_image'

            else:
                # Line lost
                self.request_command.emit(0.0, 0.0)
                self.statusBar.showMessage(f"Line Following: Line Lost! (Conf={confidence:.0f}) Stopping.")
                # display_image already contains the drawing attempt from find_line_center

        # Update Camera Display Label always with the (potentially modified) display_image
        self.update_camera_display(display_image)

    def update_camera_display(self, img_to_display):
        """Updates the camera QLabel with the given image."""
        try:
            h, w, ch = img_to_display.shape
            qt_image = QtGui.QImage(img_to_display.data, w, h, ch * w, QtGui.QImage.Format_BGR888)
            pixmap = QtGui.QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(self.camera_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
            self.camera_label.setPixmap(scaled_pixmap)
        except Exception as e:
            self.get_ros_logger().error(f"Error updating camera feed display: {e}", exc_info=True)


    @QtCore.Slot(Imu)
    def update_imu_data(self, msg): # Unchanged
        self.sample_count += 1; self.timestamps.append(self.sample_count)
        self.accel_x.append(msg.linear_acceleration.x); self.accel_y.append(msg.linear_acceleration.y); self.accel_z.append(msg.linear_acceleration.z)
        self.gyro_x.append(msg.angular_velocity.x); self.gyro_y.append(msg.angular_velocity.y); self.gyro_z.append(msg.angular_velocity.z)
        self.update_plots()

    @QtCore.Slot(Int32)
    def update_encoder_data(self, msg):  # Keep if plotting raw ticks
        # Append encoder data regardless of timestamp count
        self.encoder_ticks.append(msg.data)
        # Ensure encoder_ticks doesn't grow indefinitely beyond MAX_PLOT_POINTS
        # Note: This might cause slight misalignment if IMU/Encoder rates differ
        while len(self.encoder_ticks) > MAX_PLOT_POINTS:
            self.encoder_ticks.popleft()
        # Call update_plots (which uses self.timestamps for x-axis)
        self.update_plots()

    def update_plots(self):  # Unchanged (add odom if needed)
        time_axis = list(self.timestamps)
        self.accel_curve_x.setData(time_axis, list(self.accel_x));
        self.accel_curve_y.setData(time_axis, list(self.accel_y));
        self.accel_curve_z.setData(time_axis, list(self.accel_z))
        self.gyro_curve_x.setData(time_axis, list(self.gyro_x));
        self.gyro_curve_y.setData(time_axis, list(self.gyro_y));
        self.gyro_curve_z.setData(time_axis, list(self.gyro_z))

        # Adjust the x-axis for the encoder plot to match the available encoder data length
        encoder_data_len = len(self.encoder_ticks)
        # Use the latest 'encoder_data_len' timestamps for the x-axis
        encoder_time_axis = time_axis[-encoder_data_len:]
        # Or, create a simple sample count axis if timestamps aren't aligned:
        # encoder_time_axis = list(range(self.sample_count - encoder_data_len + 1, self.sample_count + 1))[-encoder_data_len:]

        # Ensure we don't try to plot if lengths mismatch after adjustments
        if len(encoder_time_axis) == len(self.encoder_ticks):
            self.encoder_curve.setData(encoder_time_axis, list(self.encoder_ticks))
        else:
            # Handle the case where lengths still don't match (e.g., clear plot or log warning)
            self.encoder_curve.clear()
            # self.get_ros_logger().warn("Encoder plot skipped due to axis length mismatch.")

    def keyPressEvent(self, event):
        key = event.key()
        if key in self.pressed_keys or event.isAutoRepeat(): return

        # --- Mode Switching ---
        if key == QtCore.Qt.Key_Y:
            if not self.line_following_active:
                self.line_following_active = True; self.pressed_keys.clear()
                self.line_follower.pid.reset(); self.request_command.emit(0.0, 0.0)
                self.mode_label.setText("Mode: LINE FOLLOWING"); self.mode_label.setStyleSheet("color: green; padding: 5px;")
                self.statusBar.showMessage("Line Following Activated. Waiting for line..."); self.get_ros_logger().info("Line Following Activated")
            return
        elif key == QtCore.Qt.Key_U:
            if self.line_following_active:
                self.line_following_active = False; self.request_command.emit(0.0, 0.0)
                self.mode_label.setText("Mode: MANUAL"); self.mode_label.setStyleSheet("color: blue; padding: 5px;")
                self.statusBar.showMessage("Manual Mode Activated. Robot stopped."); self.get_ros_logger().info("Line Following Deactivated (Manual Mode)")
            return
        elif key == QtCore.Qt.Key_Escape:
             self.close(); return

        # --- Manual Control ---
        if not self.line_following_active:
            valid_key = True
            if   key == QtCore.Qt.Key_W: self.pressed_keys.add(QtCore.Qt.Key_W)
            elif key == QtCore.Qt.Key_S: self.pressed_keys.add(QtCore.Qt.Key_S)
            elif key == QtCore.Qt.Key_A: self.pressed_keys.add(QtCore.Qt.Key_A)
            elif key == QtCore.Qt.Key_D: self.pressed_keys.add(QtCore.Qt.Key_D)
            elif key == QtCore.Qt.Key_Q: self.pressed_keys.add(QtCore.Qt.Key_Q)
            elif key == QtCore.Qt.Key_E: self.pressed_keys.add(QtCore.Qt.Key_E)
            elif key == QtCore.Qt.Key_H: self.pressed_keys.add(QtCore.Qt.Key_H)
            elif key == QtCore.Qt.Key_J: self.pressed_keys.add(QtCore.Qt.Key_J)
            elif key == QtCore.Qt.Key_Space: self.pressed_keys.add(QtCore.Qt.Key_Space)
            else: valid_key = False; super().keyPressEvent(event)

            if valid_key: self.process_manual_keys()

    def keyReleaseEvent(self, event):
        key = event.key()
        if event.isAutoRepeat(): return
        if not self.line_following_active and key in self.pressed_keys:
            self.pressed_keys.remove(key); self.process_manual_keys()
        else: super().keyReleaseEvent(event)

    def process_manual_keys(self):
        if self.line_following_active: return # Safety check
        lin_x, ang_z, status = 0.0, 0.0, "Manual Mode: Ready" # Default to stopped
        key_map = { 'q': QtCore.Qt.Key_Q, 'w': QtCore.Qt.Key_W, 'e': QtCore.Qt.Key_E, 'a': QtCore.Qt.Key_A, 's': QtCore.Qt.Key_S, 'd': QtCore.Qt.Key_D, 'h': QtCore.Qt.Key_H, 'j': QtCore.Qt.Key_J, 'space': QtCore.Qt.Key_Space }
        if   key_map['space'] in self.pressed_keys: lin_x, ang_z, status = 0.0, 0.0, "STOPPED (Spacebar)"
        elif key_map['q'] in self.pressed_keys: lin_x, ang_z, status = MANUAL_LINEAR_SPEED, MANUAL_ANGULAR_SPEED, "Forward-Left (Q)"
        elif key_map['w'] in self.pressed_keys: lin_x, ang_z, status = MANUAL_LINEAR_SPEED, 0.0, "Forward (W)"
        elif key_map['e'] in self.pressed_keys: lin_x, ang_z, status = MANUAL_LINEAR_SPEED,-MANUAL_ANGULAR_SPEED, "Forward-Right (E)"
        elif key_map['a'] in self.pressed_keys: lin_x, ang_z, status = -MANUAL_LINEAR_SPEED, MANUAL_ANGULAR_SPEED, "Backward-Left (A)"
        elif key_map['s'] in self.pressed_keys: lin_x, ang_z, status = -MANUAL_LINEAR_SPEED, 0.0, "Backward (S)"
        elif key_map['d'] in self.pressed_keys: lin_x, ang_z, status = -MANUAL_LINEAR_SPEED,-MANUAL_ANGULAR_SPEED, "Backward-Right (D)"
        elif key_map['h'] in self.pressed_keys: lin_x, ang_z, status = 0.0,  MANUAL_ANGULAR_SPEED, "Steer Left (H)"
        elif key_map['j'] in self.pressed_keys: lin_x, ang_z, status = 0.0, -MANUAL_ANGULAR_SPEED, "Steer Right (J)"
        self.statusBar.showMessage(status)
        self.request_command.emit(lin_x, ang_z)

    def start_safety_timer(self):
         self.safety_timer = QtCore.QTimer(self); self.safety_timer.timeout.connect(self.check_image_staleness); self.safety_timer.start(500)

    def check_image_staleness(self):
        if self.line_following_active:
            if time.time() - self.last_valid_image_time > 1.0:
                self.get_ros_logger().warn("No image received for >1 sec during line following. Stopping robot!"); self.request_command.emit(0.0, 0.0)
                self.statusBar.showMessage("Line Following: Image Lost! STOPPED.")

    def get_ros_logger(self): return self.ros_node.get_logger()
    def closeEvent(self, event): self.get_ros_logger().info("GUI window closing."); event.accept()

# --- Main Execution ---
def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    ros_node = None; window = None; ros_thread = None; exit_code = 0
    try:
        ros_node = RobotControlNode()
        window = RobotControlWindow(ros_node)
        window.start_safety_timer() # Start the safety check
        ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True); ros_thread.start()
        window.show()
        exit_code = app.exec()
    except Exception as e:
        logger = rclpy.logging.get_logger("main_exception")
        logger.fatal(f"Unhandled exception in GUI or setup: {e}", exc_info=True)
        exit_code = 1
    finally:
        print("Application exiting...")
        if ros_node is not None and rclpy.ok(): ros_node.shutdown()
        if ros_thread and ros_thread.is_alive(): ros_thread.join(timeout=0.5)
        if rclpy.ok(): print("Shutting down rclpy..."); rclpy.shutdown()
        if ros_thread and ros_thread.is_alive(): print("ROS thread did not exit cleanly.")
        print("Exiting.")
        sys.exit(exit_code)

if __name__ == '__main__':
    main()