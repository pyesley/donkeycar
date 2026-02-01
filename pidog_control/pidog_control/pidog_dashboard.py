#!/usr/bin/env python3
"""
PiDog Control Dashboard - Desktop application for controlling PiDog.

Run this on your desktop machine (with RTX 5090).

Features:
    - Live camera feed from PiDog
    - IMU data visualization (accelerometer + gyroscope)
    - Keyboard control for motion
    - Discrete command buttons

Keyboard Controls:
    W - Walk forward
    S - Walk backward
    A - Turn left
    D - Turn right
    Q - Forward + left
    E - Forward + right
    Space - Stop

    1 - Stand
    2 - Sit
    3 - Rest
    4 - Wave
    5 - Stretch

    Escape - Quit

Topics:
    Subscribes:
        /pidog/camera/image_raw/compressed - Camera feed
        /pidog/imu - IMU data
        /pidog/motion/status - Motion node status

    Publishes:
        /pidog/cmd_vel - Velocity commands
        /pidog/cmd - Discrete commands
"""

import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage, Imu
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge
import numpy as np

from PySide6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

# ---------------- Configuration ----------------
MAX_PLOT_POINTS = 400

# Topics
CAMERA_TOPIC = '/pidog/camera/image_raw/compressed'
CAMERA_RAW_TOPIC = '/pidog/camera/image_raw'
IMU_TOPIC = '/pidog/imu'
CMD_VEL_TOPIC = '/pidog/cmd_vel'
CMD_TOPIC = '/pidog/cmd'
MOTION_STATUS_TOPIC = '/pidog/motion/status'

# UI update rates
PUBLISH_RATE_HZ = 10.0
UI_PLOT_FPS = 25.0
UI_CAMERA_FPS = 15.0

# Robot speeds
LINEAR_SPEED = 0.5   # m/s equivalent
ANGULAR_SPEED = 0.8  # rad/s equivalent


# ---------------- ROS 2 Node ----------------
class PiDogDashboardNode(Node, QtCore.QObject):
    """ROS2 node for the PiDog dashboard"""

    # Qt Signals for thread-safe GUI updates
    new_image_data = QtCore.Signal(object)  # np.ndarray
    new_compressed_image = QtCore.Signal(object)  # bytes
    new_imu_data = QtCore.Signal(object)  # Imu
    new_status_data = QtCore.Signal(str)  # status string

    def __init__(self):
        Node.__init__(self, 'pidog_dashboard_node')
        QtCore.QObject.__init__(self)

        # QoS profiles
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.create_subscription(
            CompressedImage, CAMERA_TOPIC, self.compressed_image_callback, qos_sensor)
        self.create_subscription(
            Image, CAMERA_RAW_TOPIC, self.image_callback, qos_sensor)
        self.create_subscription(
            Imu, IMU_TOPIC, self.imu_callback, qos_sensor)
        self.create_subscription(
            String, MOTION_STATUS_TOPIC, self.status_callback, qos_sensor)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, qos_reliable)
        self.cmd_pub = self.create_publisher(String, CMD_TOPIC, qos_reliable)

        self.bridge = CvBridge()

        # Motion state
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.command_lock = threading.Lock()

        # Timer for publishing velocity commands
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_cmd_vel)

        self.get_logger().info("PiDog Dashboard Node ready")
        self.get_logger().info(f"  Subscribing to: {CAMERA_TOPIC}, {IMU_TOPIC}")
        self.get_logger().info(f"  Publishing to: {CMD_VEL_TOPIC}, {CMD_TOPIC}")

    def compressed_image_callback(self, msg: CompressedImage):
        """Handle compressed camera images"""
        try:
            # Decode JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                self.new_image_data.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to decode compressed image: {e}")

    def image_callback(self, msg: Image):
        """Handle raw camera images (fallback if no compressed)"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.new_image_data.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def imu_callback(self, msg: Imu):
        """Handle IMU data"""
        self.new_imu_data.emit(msg)

    def status_callback(self, msg: String):
        """Handle motion status"""
        self.new_status_data.emit(msg.data)

    def set_target_command(self, linear_x: float, angular_z: float):
        """Set target velocity command"""
        with self.command_lock:
            self.target_linear_x = float(linear_x)
            self.target_angular_z = float(angular_z)

    def publish_cmd_vel(self):
        """Publish current velocity command"""
        msg = Twist()
        with self.command_lock:
            msg.linear.x = self.target_linear_x
            msg.angular.z = self.target_angular_z
        self.cmd_vel_pub.publish(msg)

    def send_command(self, cmd: str):
        """Send discrete command"""
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent command: {cmd}")

    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down dashboard node")
        self.set_target_command(0.0, 0.0)
        self.publish_cmd_vel()
        time.sleep(0.1)
        self.destroy_node()


# ---------------- GUI ----------------
class PiDogDashboardWindow(QtWidgets.QMainWindow):
    """Main dashboard window"""

    def __init__(self, ros_node: PiDogDashboardNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("PiDog Control Dashboard")
        self.resize(1400, 900)

        # Data storage for plots
        self.timestamps = deque(maxlen=MAX_PLOT_POINTS)
        self.accel_x = deque(maxlen=MAX_PLOT_POINTS)
        self.accel_y = deque(maxlen=MAX_PLOT_POINTS)
        self.accel_z = deque(maxlen=MAX_PLOT_POINTS)
        self.gyro_x = deque(maxlen=MAX_PLOT_POINTS)
        self.gyro_y = deque(maxlen=MAX_PLOT_POINTS)
        self.gyro_z = deque(maxlen=MAX_PLOT_POINTS)
        self.sample_count = 0
        self.pressed_keys = set()
        self._last_cam_ts = 0.0

        # Build UI
        self._build_ui()

        # Connect ROS signals to slots
        self.ros_node.new_image_data.connect(self._on_image)
        self.ros_node.new_imu_data.connect(self._on_imu)
        self.ros_node.new_status_data.connect(self._on_status)

        # Timers
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(int(1000.0 / UI_PLOT_FPS))

        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # Reduce OpenCV thread usage
        try:
            cv2.setNumThreads(1)
        except Exception:
            pass

    def _build_ui(self):
        """Build the user interface"""
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)

        # ===== Left Column: Camera + Controls =====
        left_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(left_layout, 1)

        # Camera feed
        camera_group = QtWidgets.QGroupBox("Camera Feed")
        camera_layout = QtWidgets.QVBoxLayout(camera_group)
        self.camera_label = QtWidgets.QLabel("Waiting for camera...")
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setMinimumSize(480, 360)
        self.camera_label.setStyleSheet(
            "border: 2px solid #555; background: #222; color: #bbb; font-size: 14px;")
        camera_layout.addWidget(self.camera_label)
        left_layout.addWidget(camera_group)

        # Control buttons
        ctrl_group = QtWidgets.QGroupBox("Commands")
        ctrl_layout = QtWidgets.QGridLayout(ctrl_group)

        # Row 1: Motion poses
        self.btn_stand = QtWidgets.QPushButton("Stand (1)")
        self.btn_stand.clicked.connect(lambda: self.ros_node.send_command('stand'))
        ctrl_layout.addWidget(self.btn_stand, 0, 0)

        self.btn_sit = QtWidgets.QPushButton("Sit (2)")
        self.btn_sit.clicked.connect(lambda: self.ros_node.send_command('sit'))
        ctrl_layout.addWidget(self.btn_sit, 0, 1)

        self.btn_rest = QtWidgets.QPushButton("Rest (3)")
        self.btn_rest.clicked.connect(lambda: self.ros_node.send_command('rest'))
        ctrl_layout.addWidget(self.btn_rest, 0, 2)

        # Row 2: Actions
        self.btn_wave = QtWidgets.QPushButton("Wave (4)")
        self.btn_wave.clicked.connect(lambda: self.ros_node.send_command('wave'))
        ctrl_layout.addWidget(self.btn_wave, 1, 0)

        self.btn_stretch = QtWidgets.QPushButton("Stretch (5)")
        self.btn_stretch.clicked.connect(lambda: self.ros_node.send_command('stretch'))
        ctrl_layout.addWidget(self.btn_stretch, 1, 1)

        self.btn_shake = QtWidgets.QPushButton("Shake (6)")
        self.btn_shake.clicked.connect(lambda: self.ros_node.send_command('shake'))
        ctrl_layout.addWidget(self.btn_shake, 1, 2)

        # Row 3: Sound commands
        self.btn_bark = QtWidgets.QPushButton("Bark (7)")
        self.btn_bark.clicked.connect(lambda: self.ros_node.send_command('bark'))
        ctrl_layout.addWidget(self.btn_bark, 2, 0)

        self.btn_howl = QtWidgets.QPushButton("Howl (8)")
        self.btn_howl.clicked.connect(lambda: self.ros_node.send_command('howl'))
        ctrl_layout.addWidget(self.btn_howl, 2, 1)

        self.btn_wag = QtWidgets.QPushButton("Wag (9)")
        self.btn_wag.clicked.connect(lambda: self.ros_node.send_command('wag'))
        ctrl_layout.addWidget(self.btn_wag, 2, 2)

        # Row 4: Stop button
        self.btn_stop = QtWidgets.QPushButton("STOP")
        self.btn_stop.setStyleSheet(
            "background-color: #cc3333; color: white; font-weight: bold; font-size: 16px; padding: 10px;")
        self.btn_stop.clicked.connect(self._emergency_stop)
        ctrl_layout.addWidget(self.btn_stop, 3, 0, 1, 3)

        left_layout.addWidget(ctrl_group)

        # Keyboard help
        help_group = QtWidgets.QGroupBox("Keyboard Controls")
        help_layout = QtWidgets.QVBoxLayout(help_group)
        help_text = QtWidgets.QLabel(
            "Motion:\n"
            "  W - Forward    S - Backward\n"
            "  A - Turn Left  D - Turn Right\n"
            "  Q - Fwd+Left   E - Fwd+Right\n"
            "  Space - Stop\n\n"
            "Commands:\n"
            "  1-Stand  2-Sit    3-Rest\n"
            "  4-Wave   5-Stretch 6-Shake\n"
            "  7-Bark   8-Howl   9-Wag\n\n"
            "  Esc - Quit"
        )
        help_text.setStyleSheet("font-family: monospace; font-size: 12px;")
        help_layout.addWidget(help_text)
        left_layout.addWidget(help_group)

        # Status
        self.status_label = QtWidgets.QLabel("Status: Waiting for connection...")
        self.status_label.setStyleSheet("font-size: 12px; color: #666;")
        left_layout.addWidget(self.status_label)

        left_layout.addStretch()

        # ===== Right Column: IMU Plots =====
        right_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(right_layout, 2)

        pg.setConfigOptions(antialias=True, background='w', foreground='k')

        # Accelerometer plot
        self.accel_plot = pg.PlotWidget()
        self._setup_plot(self.accel_plot, "Accelerometer", "m/s²")
        self.accel_plot.addLegend()
        self.accel_x_curve = self.accel_plot.plot(pen=pg.mkPen('r', width=2), name="X")
        self.accel_y_curve = self.accel_plot.plot(pen=pg.mkPen('g', width=2), name="Y")
        self.accel_z_curve = self.accel_plot.plot(pen=pg.mkPen('b', width=2), name="Z")
        right_layout.addWidget(self.accel_plot)

        # Gyroscope plot
        self.gyro_plot = pg.PlotWidget()
        self._setup_plot(self.gyro_plot, "Gyroscope", "rad/s")
        self.gyro_plot.addLegend()
        self.gyro_x_curve = self.gyro_plot.plot(pen=pg.mkPen('r', width=2), name="X")
        self.gyro_y_curve = self.gyro_plot.plot(pen=pg.mkPen('g', width=2), name="Y")
        self.gyro_z_curve = self.gyro_plot.plot(pen=pg.mkPen('b', width=2), name="Z")
        right_layout.addWidget(self.gyro_plot)

        # IMU values display
        imu_group = QtWidgets.QGroupBox("Current IMU Values")
        imu_layout = QtWidgets.QGridLayout(imu_group)

        self.accel_labels = {}
        self.gyro_labels = {}

        for i, axis in enumerate(['X', 'Y', 'Z']):
            # Accel
            imu_layout.addWidget(QtWidgets.QLabel(f"Accel {axis}:"), 0, i*2)
            self.accel_labels[axis] = QtWidgets.QLabel("--")
            self.accel_labels[axis].setStyleSheet("font-weight: bold;")
            imu_layout.addWidget(self.accel_labels[axis], 0, i*2+1)

            # Gyro
            imu_layout.addWidget(QtWidgets.QLabel(f"Gyro {axis}:"), 1, i*2)
            self.gyro_labels[axis] = QtWidgets.QLabel("--")
            self.gyro_labels[axis].setStyleSheet("font-weight: bold;")
            imu_layout.addWidget(self.gyro_labels[axis], 1, i*2+1)

        right_layout.addWidget(imu_group)

    def _setup_plot(self, plot_widget, title, ylabel):
        """Configure a plot widget"""
        plot_widget.setTitle(title)
        plot_widget.setLabel('left', ylabel)
        plot_widget.setLabel('bottom', "Sample #")
        plot_widget.showGrid(x=True, y=True)

    # ----- Slots -----
    @QtCore.Slot(object)
    def _on_image(self, cv_image):
        """Update camera display"""
        now = time.time()
        if now - self._last_cam_ts < (1.0 / UI_CAMERA_FPS):
            return
        self._last_cam_ts = now

        try:
            h, w = cv_image.shape[:2]
            # Scale to fit
            target_w = 640
            if w > target_w:
                scale = target_w / float(w)
                cv_image = cv2.resize(cv_image, (int(w * scale), int(h * scale)),
                                      interpolation=cv2.INTER_AREA)

            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(cv_image.data, w, h, bytes_per_line,
                                    QtGui.QImage.Format_BGR888)
            pixmap = QtGui.QPixmap.fromImage(qt_image)
            self.camera_label.setPixmap(pixmap)

        except Exception as e:
            self.ros_node.get_logger().error(f"Error updating camera: {e}")

    @QtCore.Slot(object)
    def _on_imu(self, msg: Imu):
        """Update IMU data"""
        self.sample_count += 1
        self.timestamps.append(self.sample_count)

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        self.accel_x.append(ax)
        self.accel_y.append(ay)
        self.accel_z.append(az)
        self.gyro_x.append(gx)
        self.gyro_y.append(gy)
        self.gyro_z.append(gz)

        # Update value labels
        self.accel_labels['X'].setText(f"{ax:.2f}")
        self.accel_labels['Y'].setText(f"{ay:.2f}")
        self.accel_labels['Z'].setText(f"{az:.2f}")
        self.gyro_labels['X'].setText(f"{gx:.3f}")
        self.gyro_labels['Y'].setText(f"{gy:.3f}")
        self.gyro_labels['Z'].setText(f"{gz:.3f}")

    @QtCore.Slot(str)
    def _on_status(self, status: str):
        """Update status display"""
        self.status_label.setText(f"Status: {status}")

    def _refresh_plots(self):
        """Refresh IMU plots"""
        t = list(self.timestamps)
        self.accel_x_curve.setData(t, list(self.accel_x))
        self.accel_y_curve.setData(t, list(self.accel_y))
        self.accel_z_curve.setData(t, list(self.accel_z))
        self.gyro_x_curve.setData(t, list(self.gyro_x))
        self.gyro_y_curve.setData(t, list(self.gyro_y))
        self.gyro_z_curve.setData(t, list(self.gyro_z))

    def _emergency_stop(self):
        """Emergency stop - all motion"""
        self.ros_node.set_target_command(0.0, 0.0)
        self.ros_node.send_command('stop')
        self.pressed_keys.clear()

    # ----- Keyboard handling -----
    def keyPressEvent(self, event):
        """Handle key press"""
        key = event.key()

        if key in self.pressed_keys or event.isAutoRepeat():
            return

        # Escape to quit
        if key == QtCore.Qt.Key_Escape:
            self.close()
            return

        # Number keys for commands
        if key == QtCore.Qt.Key_1:
            self.ros_node.send_command('stand')
            return
        if key == QtCore.Qt.Key_2:
            self.ros_node.send_command('sit')
            return
        if key == QtCore.Qt.Key_3:
            self.ros_node.send_command('rest')
            return
        if key == QtCore.Qt.Key_4:
            self.ros_node.send_command('wave')
            return
        if key == QtCore.Qt.Key_5:
            self.ros_node.send_command('stretch')
            return
        if key == QtCore.Qt.Key_6:
            self.ros_node.send_command('shake')
            return
        if key == QtCore.Qt.Key_7:
            self.ros_node.send_command('bark')
            return
        if key == QtCore.Qt.Key_8:
            self.ros_node.send_command('howl')
            return
        if key == QtCore.Qt.Key_9:
            self.ros_node.send_command('wag')
            return

        # Motion keys
        motion_keys = {
            QtCore.Qt.Key_W, QtCore.Qt.Key_S, QtCore.Qt.Key_A, QtCore.Qt.Key_D,
            QtCore.Qt.Key_Q, QtCore.Qt.Key_E, QtCore.Qt.Key_Space
        }
        if key in motion_keys:
            self.pressed_keys.add(key)
            self._process_motion_keys()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Handle key release"""
        key = event.key()
        if event.isAutoRepeat():
            return

        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            self._process_motion_keys()
        else:
            super().keyReleaseEvent(event)

    def _process_motion_keys(self):
        """Process currently pressed motion keys"""
        linear_x = 0.0
        angular_z = 0.0

        # Space = stop
        if QtCore.Qt.Key_Space in self.pressed_keys:
            pass  # Keep at 0, 0
        # Q = forward + left
        elif QtCore.Qt.Key_Q in self.pressed_keys:
            linear_x = LINEAR_SPEED
            angular_z = ANGULAR_SPEED
        # W = forward
        elif QtCore.Qt.Key_W in self.pressed_keys:
            linear_x = LINEAR_SPEED
        # E = forward + right
        elif QtCore.Qt.Key_E in self.pressed_keys:
            linear_x = LINEAR_SPEED
            angular_z = -ANGULAR_SPEED
        # A = turn left
        elif QtCore.Qt.Key_A in self.pressed_keys:
            angular_z = ANGULAR_SPEED
        # S = backward
        elif QtCore.Qt.Key_S in self.pressed_keys:
            linear_x = -LINEAR_SPEED
        # D = turn right
        elif QtCore.Qt.Key_D in self.pressed_keys:
            angular_z = -ANGULAR_SPEED

        self.ros_node.set_target_command(linear_x, angular_z)

    def closeEvent(self, event):
        """Handle window close"""
        self._emergency_stop()
        event.accept()


# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    node = PiDogDashboardNode()
    window = PiDogDashboardWindow(node)

    # Run ROS2 in separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    window.show()

    exit_code = 0
    try:
        exit_code = app.exec()
    finally:
        node.shutdown()
        if ros_thread.is_alive():
            ros_thread.join(timeout=0.5)
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
