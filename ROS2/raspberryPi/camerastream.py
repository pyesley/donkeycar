import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

# --- Configuration ---
IMG_WIDTH = 640
IMG_HEIGHT = 480
# Let's try a slightly lower framerate to reduce load
FRAMERATE = 20  # Reduced from 25
# --- --- --- --- --- ---

class GstCameraPublisher(Node):
    def __init__(self):
        super().__init__('gst_camera_publisher')

        # Define the GStreamer pipeline
        self.gst_pipeline_string = (
            "libcamerasrc ! "
            f"video/x-raw,format=BGRx,width={IMG_WIDTH},height={IMG_HEIGHT},framerate={FRAMERATE}/1 ! "
            "videoconvert ! "
            "appsink drop=true max-buffers=1 emit-signals=true" # Added emit-signals, max-buffers=1 is good
        )
        # Note: Added framerate to the caps. emit-signals=true can be useful for appsink.

        # Define a QoS profile suitable for sensor data (like images)
        # Prioritize receiving the latest messages, even if some are dropped.
        qos_profile_sensor_data = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1, # Keep only the latest frame, crucial for BEST_EFFORT
            durability=QoSDurabilityPolicy.VOLATILE # For sensor data, usually not SYSTEM_DEFAULT
        )

        self.publisher_ = self.create_publisher(
            Image,
            'camera/image_raw',
            qos_profile_sensor_data # Apply the custom QoS profile
        )
        self.timer_period = 1.0 / FRAMERATE
        self.bridge = CvBridge()

        self.get_logger().info(f"Attempting to open GStreamer pipeline: {self.gst_pipeline_string}")
        self.cap = cv2.VideoCapture(self.gst_pipeline_string, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("!!! Failed to open GStreamer pipeline.")
            # Propagate the error to allow the main script to handle it if necessary
            raise RuntimeError("Failed to open GStreamer pipeline in GstCameraPublisher")

        self.get_logger().info("GStreamer pipeline opened successfully. Starting publishing.")

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.frame_count = 0
        self.log_interval_sec = 5.0 # Log FPS every 5 seconds
        self.last_log_time = time.time()


    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().warn('GStreamer pipeline is not open. Skipping frame capture.')
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from GStreamer pipeline. cap.read() returned False.')
            # You might want to check self.cap.isOpened() again here or attempt a reopen,
            # but for now, just warning is fine.
            return

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_frame" # Consistent frame_id

            self.publisher_.publish(img_msg)

            self.frame_count += 1
            current_time = time.time()
            elapsed_time_for_log = current_time - self.last_log_time
            if elapsed_time_for_log >= self.log_interval_sec:
                fps = self.frame_count / elapsed_time_for_log
                self.get_logger().info(f'Publishing image... (~{fps:.1f} FPS)')
                self.frame_count = 0
                self.last_log_time = current_time
        except Exception as e:
            self.get_logger().error(f"Error during frame processing or publishing: {e}", exc_info=True)


    def destroy_node(self):
        self.get_logger().info("Releasing camera capture in GstCameraPublisher...")
        if self.timer:
            self.timer.cancel()
        if hasattr(self, 'cap') and self.cap and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("cv2.VideoCapture released.")
        super().destroy_node()
        self.get_logger().info("GstCameraPublisher node destroyed.")


def main(args=None): # Standalone main for testing camerastream.py
    rclpy.init(args=args)
    gst_cam_pub_node = None
    try:
        gst_cam_pub_node = GstCameraPublisher()
        rclpy.spin(gst_cam_pub_node)
    except KeyboardInterrupt:
        print("GstCameraPublisher standalone: Keyboard interrupt.")
    except RuntimeError as e: # Catch the RuntimeError from __init__
        if gst_cam_pub_node:
             gst_cam_pub_node.get_logger().error(f"RuntimeError in GstCameraPublisher: {e}")
        else:
            print(f"RuntimeError before GstCameraPublisher initialized: {e}")
    except Exception as e:
        if gst_cam_pub_node:
            gst_cam_pub_node.get_logger().error(f"Unhandled exception in GstCameraPublisher: {e}", exc_info=True)
        else:
            print(f"Unhandled exception in GstCameraPublisher main: {e}")
    finally:
        if gst_cam_pub_node:
            gst_cam_pub_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("GstCameraPublisher standalone shutdown complete.")

if __name__ == '__main__':
    main()