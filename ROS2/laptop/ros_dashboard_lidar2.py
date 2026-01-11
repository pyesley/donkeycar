#!/usr/bin/env python3
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import cv2
from cv_bridge import CvBridge

from PySide6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import numpy as np

# ---------------- Configuration ----------------
MAX_PLOT_POINTS = 400
CAMERA_TOPIC = '/camera/image_raw'
IMU_TOPIC = '/imu/data_raw_bmi088'
ENCODER_TOPIC = '/encoder_ticks'
CMD_VEL_TOPIC = '/cmd_vel'
LIDAR_SCAN_TOPIC = '/scan'
LIDAR_SERVICE = '/start_stop_lidar'

# UI update rates
PUBLISH_RATE_HZ = 10.0
UI_PLOT_FPS = 25.0
UI_CAMERA_FPS = 10.0
LIDAR_DRAW_FPS = 20.0             # decoupled draw loop for lidar

# Robot speeds
LINEAR_SPEED = 0.6
ANGULAR_SPEED = 0.5

# LiDAR drawing
MAX_POINTS_TO_DRAW = 1200          # decimate to this many points max
AUTO_DRAW_ANY_SCAN = True          # draw whenever scans arrive, regardless of motor toggle


# ---------------- ROS 2 Node ----------------
class RobotControlNode(Node, QtCore.QObject):
    # Qt Signals for thread-safe GUI updates
    new_image_data = QtCore.Signal(object)     # np.ndarray
    new_imu_data = QtCore.Signal(object)       # Imu
    new_encoder_data = QtCore.Signal(object)   # Int32
    new_lidar_msg = QtCore.Signal(object)      # LaserScan
    new_snapshot_data = QtCore.Signal(object)  # (angles, ranges)
    lidar_scan_complete = QtCore.Signal()
    lidar_scan_progress = QtCore.Signal(float)

    def __init__(self):
        Node.__init__(self, 'robot_control_dashboard_node')
        QtCore.QObject.__init__(self)

        # QoS
        qos_sensor_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_cmd_vel_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subs
        self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, qos_sensor_best_effort)
        self.create_subscription(Imu, IMU_TOPIC, self.imu_callback, qos_sensor_best_effort)
        self.create_subscription(Int32, ENCODER_TOPIC, self.encoder_callback, qos_sensor_best_effort)
        self.create_subscription(LaserScan, LIDAR_SCAN_TOPIC, self.lidar_callback, qos_sensor_best_effort)

        # Pub
        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos_cmd_vel_reliable)

        # Service
        self.lidar_client = self.create_client(SetBool, LIDAR_SERVICE)

        self.bridge = CvBridge()

        # State
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.command_lock = threading.Lock()

        # LiDAR state
        self.lidar_motor_on = False           # describes motor state we requested via service
        self.is_snapshot_scanning = False
        self.lidar_scan_start_time = None
        self.accumulated_snapshot = []
        self.lidar_lock = threading.Lock()

        # Timers
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_current_command)

        # On startup: try to stop lidar motor
        self._try_stop_lidar_on_start()

        self.get_logger().info("Robot Control Dashboard Node is ready.")

    # ---------- Callbacks ----------
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.new_image_data.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def imu_callback(self, msg: Imu):
        self.new_imu_data.emit(msg)

    def encoder_callback(self, msg: Int32):
        self.new_encoder_data.emit(msg)

    def lidar_callback(self, msg: LaserScan):
        # Always forward latest scan to GUI (we now draw in a timer)
        self.new_lidar_msg.emit(msg)

        # Snapshot accumulation (bounded duration)
        if not self.is_snapshot_scanning:
            return

        with self.lidar_lock:
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            ranges = np.asarray(msg.ranges, dtype=np.float32)
            n = ranges.size
            if n == 0 or not np.isfinite(angle_increment) or angle_increment == 0.0:
                return

            angles = angle_min + np.arange(n, dtype=np.float32) * angle_increment
            # Use message-provided valid range
            rmin = max(0.0, float(getattr(msg, 'range_min', 0.0)))
            rmax = float(getattr(msg, 'range_max', np.inf))
            mask = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)
            if np.any(mask):
                self.accumulated_snapshot.append((angles[mask], ranges[mask]))

            if self.lidar_scan_start_time:
                elapsed = time.time() - self.lidar_scan_start_time
                self.lidar_scan_progress.emit(min(elapsed / 5.0 * 100.0, 100.0))
                if elapsed >= 5.0:
                    self._complete_snapshot()

    # ---------- LiDAR control ----------
    def set_live_lidar(self, enable: bool):
        """Start/stop LiDAR motor/stream (does not affect whether we draw)."""
        if not self.lidar_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("LiDAR service not available; cannot change motor state")
            return False

        req = SetBool.Request()
        req.data = bool(enable)
        future = self.lidar_client.call_async(req)

        def _done(_):
            try:
                resp = future.result()
                if resp.success:
                    self.get_logger().info(f"LiDAR motor {'enabled' if enable else 'disabled'}: {resp.message}")
                else:
                    self.get_logger().error(f"LiDAR motor change failed: {resp.message}")
            except Exception as ex:
                self.get_logger().error(f"LiDAR service call failed: {ex}")

        future.add_done_callback(_done)
        self.lidar_motor_on = enable
        return True

    def start_snapshot(self):
        """Begin a one-shot (bounded) scan that will stop the motor when done."""
        if self.is_snapshot_scanning:
            self.get_logger().warn("Snapshot already running")
            return False

        if not self.lidar_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("LiDAR service not available for snapshot")
            return False

        with self.lidar_lock:
            self.accumulated_snapshot.clear()
            self.is_snapshot_scanning = True
            self.lidar_scan_start_time = time.time()

        # Turn on motor/scan stream
        req = SetBool.Request(); req.data = True
        self.lidar_client.call_async(req)
        return True

    def _complete_snapshot(self):
        # Stop the motor first
        req = SetBool.Request(); req.data = False
        self.lidar_client.call_async(req)

        with self.lidar_lock:
            if self.accumulated_snapshot:
                angles_all = np.concatenate([a for a, _ in self.accumulated_snapshot])
                ranges_all = np.concatenate([r for _, r in self.accumulated_snapshot])
                self.new_snapshot_data.emit((angles_all, ranges_all))

            self.is_snapshot_scanning = False
            self.lidar_scan_start_time = None

        self.lidar_scan_complete.emit()

    def _try_stop_lidar_on_start(self):
        if self.lidar_client.wait_for_service(timeout_sec=0.25):
            req = SetBool.Request(); req.data = False
            self.lidar_client.call_async(req)

    # ---------- Motion ----------
    def set_target_command(self, linear_x, angular_z):
        with self.command_lock:
            self.target_linear_x = float(linear_x)
            self.target_angular_z = float(angular_z)

    def publish_current_command(self):
        twist = Twist()
        with self.command_lock:
            twist.linear.x = self.target_linear_x
            twist.angular.z = self.target_angular_z
        self.cmd_vel_publisher.publish(twist)

    def shutdown(self):
        self.get_logger().info("Shutting down dashboard node")
        self.set_target_command(0.0, 0.0)
        self.publish_current_command()
        if self.lidar_client.wait_for_service(timeout_sec=0.25):
            req = SetBool.Request(); req.data = False
            self.lidar_client.call_async(req)
        time.sleep(0.05)
        self.destroy_node()


# ---------------- GUI ----------------
class RobotControlWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node: RobotControlNode):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Robot Control Dashboard (stable/fast)")
        self.resize(1440, 960)

        # Data storage
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
        self._last_cam_ts = 0.0

        # LiDAR buffers
        self.last_scan_msg = None        # LaserScan
        self.last_snapshot = None        # (angles, ranges)

        # Layout
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # Left column
        left = QtWidgets.QVBoxLayout(); layout.addLayout(left, 1)

        # Camera
        self.camera_label = QtWidgets.QLabel("Waiting for camera...")
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setMinimumSize(360, 270)
        self.camera_label.setStyleSheet("border:1px solid #555;background:#222;color:#bbb;")
        left.addWidget(self.camera_label)

        # LiDAR plot
        self.lidar_plot_widget = pg.PlotWidget()
        self._setup_polar_plot()
        left.addWidget(self.lidar_plot_widget)

        # Controls row
        ctrl_row = QtWidgets.QHBoxLayout()
        self.btn_live = QtWidgets.QPushButton("Start LIDAR Motor")
        self.btn_live.setCheckable(True)
        self.btn_live.clicked.connect(self._toggle_live_lidar)
        ctrl_row.addWidget(self.btn_live)

        self.btn_snapshot = QtWidgets.QPushButton("One-shot 5s (O)")
        self.btn_snapshot.clicked.connect(self._start_snapshot_clicked)
        ctrl_row.addWidget(self.btn_snapshot)

        self.lidar_progress = QtWidgets.QProgressBar()
        self.lidar_progress.setRange(0, 100)
        self.lidar_progress.setValue(0)
        ctrl_row.addWidget(self.lidar_progress)
        left.addLayout(ctrl_row)

        # Status label
        self.status_label = QtWidgets.QLabel("")
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        left.addWidget(self.status_label)

        # Right plots
        right = QtWidgets.QVBoxLayout(); layout.addLayout(right, 2)
        pg.setConfigOptions(antialias=True, background='w', foreground='k')

        self.accel_plot = pg.PlotWidget(); self._setup_plot(self.accel_plot, "Linear Accel", "m/s²")
        self.accel_plot.addLegend()
        self.accel_x_curve = self.accel_plot.plot(pen=pg.mkPen('r', width=2), name="Ax")
        self.accel_y_curve = self.accel_plot.plot(pen=pg.mkPen('g', width=2), name="Ay")
        self.accel_z_curve = self.accel_plot.plot(pen=pg.mkPen('b', width=2), name="Az")
        right.addWidget(self.accel_plot)

        self.gyro_plot = pg.PlotWidget(); self._setup_plot(self.gyro_plot, "Angular Vel", "rad/s")
        self.gyro_plot.addLegend()
        self.gyro_x_curve = self.gyro_plot.plot(pen=pg.mkPen('r', width=2), name="Gx")
        self.gyro_y_curve = self.gyro_plot.plot(pen=pg.mkPen('g', width=2), name="Gy")
        self.gyro_z_curve = self.gyro_plot.plot(pen=pg.mkPen('b', width=2), name="Gz")
        right.addWidget(self.gyro_plot)

        self.enc_plot = pg.PlotWidget(); self._setup_plot(self.enc_plot, "Encoder Count", "ticks")
        self.enc_curve = self.enc_plot.plot(pen=pg.mkPen('m', width=2), name="ticks")
        self.enc_plot.addLegend()
        right.addWidget(self.enc_plot)

        # Wire signals
        self.ros_node.new_image_data.connect(self._on_image)
        self.ros_node.new_imu_data.connect(self._on_imu)
        self.ros_node.new_encoder_data.connect(self._on_encoder)
        self.ros_node.new_lidar_msg.connect(self._on_lidar_msg)
        self.ros_node.new_snapshot_data.connect(self._on_snapshot_ready)
        self.ros_node.lidar_scan_complete.connect(self._on_snapshot_complete)
        self.ros_node.lidar_scan_progress.connect(self.lidar_progress.setValue)

        # Timers
        self.plot_timer = QtCore.QTimer(self); self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(int(1000.0 / UI_PLOT_FPS))

        self.lidar_timer = QtCore.QTimer(self); self.lidar_timer.timeout.connect(self._refresh_lidar)
        self.lidar_timer.start(int(1000.0 / LIDAR_DRAW_FPS))

        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # Reduce OpenCV thread fan-out to avoid UI stalls
        try:
            cv2.setNumThreads(1)
        except Exception:
            pass

    # ----- Plots -----
    def _setup_plot(self, plot_widget, title, ylabel):
        plot_widget.setTitle(title)
        plot_widget.setLabel('left', ylabel)
        plot_widget.setLabel('bottom', "Sample #")
        plot_widget.showGrid(x=True, y=True)

    def _setup_polar_plot(self):
        self.lidar_plot_widget.setTitle("LiDAR (polar XY)")
        self.lidar_plot_widget.setAspectLocked(True)
        self.lidar_plot_widget.showGrid(x=True, y=True)
        self.lidar_plot_widget.setLabel('left', "Y (m)")
        self.lidar_plot_widget.setLabel('bottom', "X (m)")

        for r in [1, 2, 3, 4, 5, 6]:
            circle = pg.QtWidgets.QGraphicsEllipseItem(-r, -r, 2*r, 2*r)
            circle.setPen(pg.mkPen(color=(150, 150, 150), width=0.5, style=QtCore.Qt.DotLine))
            self.lidar_plot_widget.addItem(circle)

        self.lidar_scatter = pg.ScatterPlotItem(size=3, pen=None, brush=pg.mkBrush(255, 0, 0, 220))
        self.lidar_plot_widget.addItem(self.lidar_scatter)
        self.lidar_plot_widget.setXRange(-6, 6)
        self.lidar_plot_widget.setYRange(-6, 6)

    # ----- Slots -----
    @QtCore.Slot(object)
    def _on_image(self, cv_image):
        # throttle to UI_CAMERA_FPS
        now = time.time()
        if now - self._last_cam_ts < (1.0 / UI_CAMERA_FPS):
            return
        self._last_cam_ts = now

        try:
            h, w = cv_image.shape[:2]
            target_w = 800
            if w > target_w:
                scale = target_w / float(w)
                cv_image = cv2.resize(cv_image, (int(w*scale), int(h*scale)), interpolation=cv2.INTER_AREA)

            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(cv_image.data, w, h, bytes_per_line, QtGui.QImage.Format_BGR888)
            pixmap = QtGui.QPixmap.fromImage(qt_image)
            self.camera_label.setPixmap(pixmap)
        except Exception as e:
            self._log().error(f"Error updating camera feed: {e}")

    @QtCore.Slot(object)
    def _on_imu(self, msg: Imu):
        self.sample_count += 1
        self.timestamps.append(self.sample_count)
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)
        self.gyro_x.append(msg.angular_velocity.x)
        self.gyro_y.append(msg.angular_velocity.y)
        self.gyro_z.append(msg.angular_velocity.z)

    @QtCore.Slot(object)
    def _on_encoder(self, msg: Int32):
        self.encoder_ticks.append(msg.data)

    @QtCore.Slot(object)
    def _on_lidar_msg(self, msg: LaserScan):
        # store only; drawing now happens in timer to avoid bursts
        self.last_scan_msg = msg

    @QtCore.Slot(object)
    def _on_snapshot_ready(self, data):
        self.last_snapshot = data  # (angles, ranges)

    @QtCore.Slot()
    def _on_snapshot_complete(self):
        self.btn_snapshot.setEnabled(True)
        self.lidar_progress.setValue(100)

    # ----- Lidar drawing (timer) -----
    def _refresh_lidar(self):
        try:
            # Prefer snapshot if available and not currently scanning
            if self.last_snapshot is not None and not self.ros_node.is_snapshot_scanning:
                angles, ranges = self.last_snapshot
                xs = ranges * np.cos(angles)
                ys = ranges * np.sin(angles)
                xs, ys = self._decimate_xy(xs, ys)
                self.lidar_scatter.setData(xs, ys)
                self.status_label.setText(f"Snapshot points: {len(xs)}")
                return

            # Otherwise draw the latest scan message
            msg = self.last_scan_msg
            if msg is None:
                return

            ranges = np.asarray(msg.ranges, dtype=np.float32)
            n = ranges.size
            if n == 0:
                return

            angle_min = float(msg.angle_min)
            angle_inc = float(msg.angle_increment)
            if not np.isfinite(angle_inc) or angle_inc == 0.0:
                return

            # Validity mask using LaserScan's declared limits
            rmin = max(0.0, float(getattr(msg, 'range_min', 0.0)))
            rmax = float(getattr(msg, 'range_max', np.inf))
            mask = np.isfinite(ranges) & (ranges >= rmin) & (ranges <= rmax)
            if not np.any(mask):
                self.status_label.setText("Scan: no valid points")
                return

            idx = np.nonzero(mask)[0]
            # Build angles only for valid indices to save work
            angles = angle_min + idx.astype(np.float32) * angle_inc
            r = ranges[mask]
            xs = r * np.cos(angles)
            ys = r * np.sin(angles)

            xs, ys = self._decimate_xy(xs, ys)

            self.lidar_scatter.setData(xs, ys)
            self.status_label.setText(f"Scan points: {len(xs)}  r∈[{rmin:.2f},{rmax:.2f}] m")

        except Exception as e:
            self._log().error(f"Lidar draw error: {e}")

    def _decimate_xy(self, xs, ys):
        n = xs.size
        if n <= MAX_POINTS_TO_DRAW:
            return xs, ys
        step = max(1, n // MAX_POINTS_TO_DRAW)
        return xs[::step], ys[::step]

    # ----- Periodic UI refresh -----
    def _refresh_plots(self):
        t = list(self.timestamps)
        self.accel_x_curve.setData(t, list(self.accel_x))
        self.accel_y_curve.setData(t, list(self.accel_y))
        self.accel_z_curve.setData(t, list(self.accel_z))
        self.gyro_x_curve.setData(t, list(self.gyro_x))
        self.gyro_y_curve.setData(t, list(self.gyro_y))
        self.gyro_z_curve.setData(t, list(self.gyro_z))
        self.enc_curve.setData(t[-len(self.encoder_ticks):], list(self.encoder_ticks))

    # ----- UI actions -----
    def _toggle_live_lidar(self):
        enable = self.btn_live.isChecked()
        self.btn_live.setText("Stop LIDAR Motor" if enable else "Start LIDAR Motor")
        self.ros_node.set_live_lidar(enable)

    def _start_snapshot_clicked(self):
        self.btn_snapshot.setEnabled(False)
        self.lidar_progress.setValue(0)
        self.ros_node.start_snapshot()

    # ----- Keyboard -----
    def keyPressEvent(self, event):
        key = event.key()
        if key in self.pressed_keys or event.isAutoRepeat():
            return

        if key == QtCore.Qt.Key_Escape:
            self.close(); return
        if key == QtCore.Qt.Key_O:
            self._start_snapshot_clicked(); return
        if key == QtCore.Qt.Key_L:
            self.btn_live.toggle(); self._toggle_live_lidar(); return

        mapped = {
            QtCore.Qt.Key_W, QtCore.Qt.Key_S, QtCore.Qt.Key_A, QtCore.Qt.Key_D,
            QtCore.Qt.Key_Q, QtCore.Qt.Key_E, QtCore.Qt.Key_H, QtCore.Qt.Key_J,
            QtCore.Qt.Key_Space
        }
        if key in mapped:
            self.pressed_keys.add(key)
            self._process_keys()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        key = event.key()
        if event.isAutoRepeat():
            return
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            self._process_keys()
        else:
            super().keyReleaseEvent(event)

    def _process_keys(self):
        lin_x = 0.0; ang_z = 0.0
        if QtCore.Qt.Key_Space in self.pressed_keys:
            pass
        elif QtCore.Qt.Key_Q in self.pressed_keys:
            lin_x = LINEAR_SPEED; ang_z = ANGULAR_SPEED
        elif QtCore.Qt.Key_W in self.pressed_keys:
            lin_x = LINEAR_SPEED
        elif QtCore.Qt.Key_E in self.pressed_keys:
            lin_x = LINEAR_SPEED; ang_z = -ANGULAR_SPEED
        elif QtCore.Qt.Key_A in self.pressed_keys:
            lin_x = -LINEAR_SPEED; ang_z = ANGULAR_SPEED
        elif QtCore.Qt.Key_S in self.pressed_keys:
            lin_x = -LINEAR_SPEED
        elif QtCore.Qt.Key_D in self.pressed_keys:
            lin_x = -LINEAR_SPEED; ang_z = -ANGULAR_SPEED
        elif QtCore.Qt.Key_H in self.pressed_keys:
            ang_z = ANGULAR_SPEED
        elif QtCore.Qt.Key_J in self.pressed_keys:
            ang_z = -ANGULAR_SPEED
        self.ros_node.set_target_command(lin_x, ang_z)

    # ----- Misc -----
    def _log(self):
        return self.ros_node.get_logger()

    def closeEvent(self, event):
        event.accept()


# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    node = RobotControlNode()
    window = RobotControlWindow(node)

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
