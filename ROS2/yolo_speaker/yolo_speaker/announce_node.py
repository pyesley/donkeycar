import subprocess
import time

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


class AnnounceNode(Node):
    def __init__(self):
        super().__init__('yolo_announce_node')

        self.declare_parameter('target_classes', ['apple', 'orange', 'peach'])
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('cooldown_sec', 4.0)
        self.declare_parameter('detections_topic', '/yolo/detections')

        self.target_classes = set(
            self.get_parameter('target_classes').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)
        topic = self.get_parameter('detections_topic').value

        self.last_announce = 0.0

        self.sub = self.create_subscription(
            DetectionArray, topic, self.on_detections, 10)

        self.get_logger().info(
            f"listening on {topic}, will speak when any of {sorted(self.target_classes)} "
            f"seen with score>={self.min_score} (cooldown {self.cooldown_sec}s)")

    def on_detections(self, msg: DetectionArray):
        now = time.monotonic()
        if now - self.last_announce < self.cooldown_sec:
            return

        # Preserve insertion order, dedupe (so two apples -> say "apple" once).
        seen = []
        for det in msg.detections:
            if (det.class_name in self.target_classes
                    and det.score >= self.min_score
                    and det.class_name not in seen):
                seen.append(det.class_name)

        if not seen:
            return

        self.last_announce = now
        phrase = ', '.join(seen)
        self.get_logger().info(f"saw {seen} -> speaking '{phrase}'")
        subprocess.Popen(
            ['espeak-ng', phrase],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def main():
    rclpy.init()
    node = AnnounceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
