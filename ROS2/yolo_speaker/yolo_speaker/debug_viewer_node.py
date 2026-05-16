import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class DebugViewerNode(Node):
    def __init__(self):
        super().__init__('yolo_debug_viewer')

        self.declare_parameter('image_topic', '/yolo/dbg_image')
        self.declare_parameter('window_name', 'YOLO debug')
        self.declare_parameter('reliable', True)

        topic = self.get_parameter('image_topic').value
        self.window_name = self.get_parameter('window_name').value

        qos = QoSProfile(depth=1)
        qos.reliability = (
            ReliabilityPolicy.RELIABLE
            if self.get_parameter('reliable').value
            else ReliabilityPolicy.BEST_EFFORT
        )

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, topic, self.on_image, qos)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"viewing {topic}")

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = DebugViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
