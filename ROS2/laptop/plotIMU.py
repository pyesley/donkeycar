#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Imu

from PySide6 import QtWidgets, QtCore
import pyqtgraph as pg
import numpy as np
from collections import deque

# Configuration
MAX_DATA_POINTS = 200  # Number of points to display on the rolling plot
IMU_TOPIC = 'imu/data_raw' # Topic published by your Arduino bridge

# --- ROS 2 Node ---
class IMUListenerNode(Node, QtCore.QObject):
    # Qt Signal to communicate Imu data safely to the GUI thread
    new_imu_data = QtCore.Signal(Imu)

    def __init__(self):
        Node.__init__(self, 'imu_plotter_node')
        QtCore.QObject.__init__(self) # Initialize QObject base class for signals

        self.subscription = self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.imu_callback,
            10) # QoS profile depth
        self.get_logger().info(f"Listening for IMU data on '{IMU_TOPIC}'...")

    def imu_callback(self, msg):
        # Emit the signal with the received message
        # This will be received by the GUI thread
        self.new_imu_data.emit(msg)

    def shutdown(self):
        self.get_logger().info("Shutting down ROS node.")
        self.destroy_node()


# --- PyQtGraph Plotting Window ---
class IMUPlotWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("IMU Data Plotter")
        self.setGeometry(100, 100, 800, 600) # x, y, width, height

        # Data storage using deques for efficient rolling window
        self.timestamps = deque(maxlen=MAX_DATA_POINTS)
        self.accel_x = deque(maxlen=MAX_DATA_POINTS)
        self.accel_y = deque(maxlen=MAX_DATA_POINTS)
        self.accel_z = deque(maxlen=MAX_DATA_POINTS)
        self.gyro_x = deque(maxlen=MAX_DATA_POINTS)
        self.gyro_y = deque(maxlen=MAX_DATA_POINTS)
        self.gyro_z = deque(maxlen=MAX_DATA_POINTS)
        self.sample_count = 0

        self.display_mode = 'accel' # 'accel' or 'gyro'

        # --- GUI Layout ---
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)

        # Plot Widget
        pg.setConfigOptions(antialias=True) # Nicer plots
        self.plot_widget = pg.PlotWidget()
        layout.addWidget(self.plot_widget)

        # Add legend
        self.plot_widget.addLegend(offset=(-10, 10)) # Position slightly offset from top-right

        # Plot curves (PlotDataItem)
        self.curve_x = self.plot_widget.plot(pen=pg.mkPen('r', width=2), name="X-Axis") # Red
        self.curve_y = self.plot_widget.plot(pen=pg.mkPen('g', width=2), name="Y-Axis") # Green
        self.curve_z = self.plot_widget.plot(pen=pg.mkPen('b', width=2), name="Z-Axis") # Blue

        # Button to switch modes
        self.toggle_button = QtWidgets.QPushButton("Switch to Angular Velocity")
        self.toggle_button.clicked.connect(self.toggle_display_mode)
        layout.addWidget(self.toggle_button)

        # --- Connect ROS Signal ---
        # Connect the signal from the ROS node to the GUI update slot
        self.ros_node.new_imu_data.connect(self.update_data)

        # Initial Plot Setup
        self.setup_plot()

    def setup_plot(self):
        """Configures plot titles and labels based on the current mode."""
        if self.display_mode == 'accel':
            self.plot_widget.setTitle("Linear Acceleration vs. Time")
            self.plot_widget.setLabel('left', "Acceleration", units="m/s^2")
            self.toggle_button.setText("Switch to Angular Velocity")
        else: # 'gyro'
            self.plot_widget.setTitle("Angular Velocity vs. Time")
            self.plot_widget.setLabel('left', "Angular Velocity", units="rad/s")
            self.toggle_button.setText("Switch to Linear Acceleration")

        self.plot_widget.setLabel('bottom', "Sample Number")
        self.plot_widget.showGrid(x=True, y=True)
        # self.plot_widget.setYRange(-10, 10) # Optional: Set fixed Y range if needed

    def toggle_display_mode(self):
        """Switches between displaying acceleration and gyroscope data."""
        if self.display_mode == 'accel':
            self.display_mode = 'gyro'
        else:
            self.display_mode = 'accel'

        self.get_ros_logger().info(f"Switched display mode to: {self.display_mode}")
        self.setup_plot() # Update titles/labels
        self.update_plot() # Redraw with the other dataset

    @QtCore.Slot(Imu) # Decorator to mark this as a Qt slot receiving Imu messages
    def update_data(self, msg):
        """Receives new IMU data from the ROS thread via Qt signal."""
        self.sample_count += 1
        # Use sample count as X-axis for simplicity
        self.timestamps.append(self.sample_count)

        # Append data to all deques
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)
        self.gyro_x.append(msg.angular_velocity.x)
        self.gyro_y.append(msg.angular_velocity.y)
        self.gyro_z.append(msg.angular_velocity.z)

        # Update the plot with the relevant data based on current mode
        self.update_plot()

    def update_plot(self):
        """Updates the plot curves with data from the appropriate deques."""
        time_axis = list(self.timestamps) # Use current snapshot of timestamps

        if self.display_mode == 'accel':
            self.curve_x.setData(time_axis, list(self.accel_x))
            self.curve_y.setData(time_axis, list(self.accel_y))
            self.curve_z.setData(time_axis, list(self.accel_z))
        else: # 'gyro'
            self.curve_x.setData(time_axis, list(self.gyro_x))
            self.curve_y.setData(time_axis, list(self.gyro_y))
            self.curve_z.setData(time_axis, list(self.gyro_z))

    def get_ros_logger(self):
        """Helper to access the ROS logger from the GUI class."""
        return self.ros_node.get_logger()

    def closeEvent(self, event):
        """Ensures ROS node is shut down cleanly when the window is closed."""
        self.get_ros_logger().info("GUI window closing.")
        # No explicit shutdown needed here if using ExternalShutdownException handling
        event.accept()


# --- Main Execution ---
def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv) # Qt Application

    # Create ROS node and GUI window
    ros_node = IMUListenerNode()
    window = IMUPlotWindow(ros_node)

    # Run ROS node in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Show the GUI window and start the Qt event loop
    window.show()

    # Start the Qt event loop
    # It will exit when the window is closed
    exit_code = app.exec()

    # Cleanup
    print("Qt loop finished. Shutting down ROS...")
    # rclpy should ideally be shutdown after spin exits.
    # If spin is still running (e.g., due to thread issues), need to trigger shutdown.
    if rclpy.ok():
         # Request shutdown if spin is still somehow active or wasn't interrupted properly
         # Normally, closing the window leads to app.exec() returning, and the main thread ends.
         # The daemon=True on the thread means it won't block exit, but clean shutdown is better.
         # Consider adding manual shutdown trigger if issues arise.
         ros_node.shutdown() # Cleanly destroy the node
         # Give a moment for shutdown
         ros_thread.join(timeout=0.5) # Wait briefly for thread
         # It's generally safer to let rclpy shutdown outside the GUI close event
         if rclpy.ok():
             rclpy.shutdown()


    print("Exiting.")
    sys.exit(exit_code)


if __name__ == '__main__':
    main()