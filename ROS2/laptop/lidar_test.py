#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import numpy as np
import matplotlib.pyplot as plt
import time
import sys


class RplidarTestNode(Node):
    def __init__(self):
        super().__init__('rplidar_test_node')

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Service client to control lidar
        self.lidar_control_client = self.create_client(SetBool, 'start_stop_lidar')

        self.get_logger().info('RPLiDAR Test Node started.')

        # Data storage
        self.latest_scan = None
        self.scan_count = 0
        self.start_time = time.time()
        self.last_scan_time = None

        # Check if service is available
        self.check_service_timer = self.create_timer(1.0, self.check_service_availability)
        self.service_available = False

        # Statistics timer
        self.stats_timer = self.create_timer(5.0, self.print_statistics)

    def check_service_availability(self):
        if self.lidar_control_client.service_is_ready():
            if not self.service_available:
                self.get_logger().info('RPLiDAR control service is available!')
                self.service_available = True
                self.check_service_timer.cancel()  # Stop checking
                self.print_menu()
        else:
            if not self.service_available:
                self.get_logger().info('Waiting for RPLiDAR control service...')

    def print_menu(self):
        print("\n" + "=" * 60)
        print("RPLiDAR Test Node - Control Menu")
        print("=" * 60)
        print("Commands:")
        print("  's' or 'start'  - Start LiDAR scanning")
        print("  'x' or 'stop'   - Stop LiDAR scanning")
        print("  'i' or 'info'   - Show scan statistics")
        print("  'p' or 'plot'   - Show real-time plot (requires scanning)")
        print("  'q' or 'quit'   - Quit the test program")
        print("=" * 60)
        print("Note: LiDAR is stopped by default to prevent vibration.")
        print("Type a command and press Enter:")

    def laser_callback(self, msg):
        self.latest_scan = msg
        self.scan_count += 1
        self.last_scan_time = time.time()

        # Print basic info for first few scans
        if self.scan_count <= 3:
            valid_ranges = [r for r in msg.ranges if not np.isinf(r) and not np.isnan(r)]
            if valid_ranges:
                self.get_logger().info(
                    f'Scan {self.scan_count}: {len(valid_ranges)} valid points, '
                    f'range: {min(valid_ranges):.2f}m - {max(valid_ranges):.2f}m'
                )
            else:
                self.get_logger().info(f'Scan {self.scan_count}: No valid range data')

    def send_lidar_command(self, start=True):
        if not self.service_available:
            self.get_logger().error('LiDAR control service not available')
            return False

        request = SetBool.Request()
        request.data = start

        try:
            future = self.lidar_control_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    action = "started" if start else "stopped"
                    self.get_logger().info(f'LiDAR {action}: {response.message}')
                    return True
                else:
                    self.get_logger().error(f'Failed to {"start" if start else "stop"} LiDAR: {response.message}')
                    return False
            else:
                self.get_logger().error('Service call timed out')
                return False

        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return False

    def print_statistics(self):
        if self.scan_count == 0:
            return  # Don't spam if no scans

        elapsed_time = time.time() - self.start_time
        scan_rate = self.scan_count / elapsed_time if elapsed_time > 0 else 0

        # Check if we're receiving recent data
        if self.last_scan_time:
            time_since_last = time.time() - self.last_scan_time
            if time_since_last > 2.0:
                self.get_logger().warn(f'No scan data received for {time_since_last:.1f} seconds')
                return

        if self.latest_scan is not None:
            valid_ranges = [r for r in self.latest_scan.ranges
                            if not np.isinf(r) and not np.isnan(r) and r > 0]

            if valid_ranges:
                avg_range = np.mean(valid_ranges)
                min_range = min(valid_ranges)
                max_range = max(valid_ranges)

                self.get_logger().info(
                    f'Stats: {self.scan_count} scans, {scan_rate:.1f} Hz, '
                    f'{len(valid_ranges)} points/scan, '
                    f'range: {min_range:.2f}-{max_range:.2f}m (avg: {avg_range:.2f}m)'
                )

    def show_plot(self):
        if self.latest_scan is None:
            print("No scan data available. Start scanning first with 's' command.")
            return

        try:
            # Create polar plot
            fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))

            msg = self.latest_scan
            angles = []
            ranges = []

            for i, range_val in enumerate(msg.ranges):
                if not np.isinf(range_val) and not np.isnan(range_val) and range_val > 0:
                    angle = msg.angle_min + i * msg.angle_increment
                    angles.append(angle)
                    ranges.append(range_val)

            if angles:
                ax.scatter(angles, ranges, c='blue', s=1, alpha=0.6)
                ax.set_title(f'RPLiDAR Scan - {len(angles)} points')
                ax.set_ylim(0, max(ranges) * 1.1 if ranges else 12)
                ax.grid(True)
                plt.show(block=False)
                print("Plot displayed. Close the plot window to continue.")
                plt.show()
            else:
                print("No valid scan points to plot.")

        except Exception as e:
            print(f"Error creating plot: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RplidarTestNode()

        # Give the node time to initialize
        time.sleep(2)

        if not node.service_available:
            print("Waiting for RPLiDAR node to start...")
            # Spin until service is available or timeout
            start_wait = time.time()
            while not node.service_available and (time.time() - start_wait) < 10:
                rclpy.spin_once(node, timeout_sec=0.5)

        if not node.service_available:
            print("ERROR: RPLiDAR control service not available. Is the main node running?")
            print("Start it with: ros2 run rpi_ros2_cpp_nodes rpi_main_nodes")
            return

        node.print_menu()

        # Interactive command loop
        while rclpy.ok():
            try:
                # Non-blocking input check
                rclpy.spin_once(node, timeout_sec=0.1)

                # Check for user input
                import select
                if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                    command = input().strip().lower()

                    if command in ['q', 'quit', 'exit']:
                        print("Stopping LiDAR and exiting...")
                        node.send_lidar_command(False)  # Stop scanning
                        break
                    elif command in ['s', 'start']:
                        print("Starting LiDAR scanning...")
                        node.send_lidar_command(True)
                    elif command in ['x', 'stop']:
                        print("Stopping LiDAR scanning...")
                        node.send_lidar_command(False)
                    elif command in ['i', 'info']:
                        node.print_statistics()
                    elif command in ['p', 'plot']:
                        node.show_plot()
                    elif command == '':
                        continue  # Ignore empty input
                    else:
                        print(f"Unknown command: '{command}'. Try 's', 'x', 'i', 'p', or 'q'")

            except KeyboardInterrupt:
                print("\nReceived Ctrl+C, stopping LiDAR and exiting...")
                node.send_lidar_command(False)  # Stop scanning
                break
            except EOFError:
                break

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()