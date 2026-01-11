# lidar_start_test.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import LaserScan
import time

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        self.cli = self.create_client(SetBool, '/start_stop_lidar')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.n = 0
        self.first_stamp = None

    def cb(self, msg):
        self.n += 1
        if self.first_stamp is None:
            self.first_stamp = self.get_clock().now()

    def start(self):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('start_stop_lidar service not available')
        req = SetBool.Request(); req.data = True
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if not (resp and resp.success):
            raise RuntimeError(f'Failed to start: {resp.message if resp else "no response"}')
        self.get_logger().info(f"Service OK: {resp.message}")

    def stop(self):
        req = SetBool.Request(); req.data = False
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        self.get_logger().info(f"Stop service: {resp.message if resp else 'no response'}")

def main():
    rclpy.init()
    node = LidarTester()
    try:
        node.start()
        node.get_logger().info("Listening on /scan for 5 seconds…")
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        dt = (node.get_clock().now() - node.first_stamp).nanoseconds * 1e-9 if node.first_stamp else 0.0
        hz = node.n/dt if dt > 0 else 0.0
        node.get_logger().info(f"Received {node.n} LaserScan msgs in {dt:.2f}s (~{hz:.2f} Hz)")
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
