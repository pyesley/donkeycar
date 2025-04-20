import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

# --- Configuration ---
# Note: We are *not* currently using these in the pipeline string below
# to let GStreamer auto-negotiate resolution/framerate initially.
# You *could* try adding them back into the 'video/x-raw' caps later if needed:
# e.g., video/x-raw,format=BGRx,width=1280,height=720,framerate=30/1
IMG_WIDTH = 640
IMG_HEIGHT = 480
FRAMERATE = 25
# --- --- --- --- --- ---

class GstCameraPublisher(Node):
    def __init__(self):
        super().__init__('gst_camera_publisher')

        # Define the GStreamer pipeline - using the working BGRx format
        # We let libcamerasrc auto-negotiate resolution/framerate for now
        self.gst_pipeline_string = (
            "libcamerasrc ! "
            f"video/x-raw,format=BGRx,width={IMG_WIDTH},height={IMG_HEIGHT} ! " # Force BGRx format AND resolution
            "videoconvert ! "            # Convert if needed (maybe redundant now but safe)
            "appsink drop=true max-buffers=1" # Sink for OpenCV
        )

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10) # Publish to camera/image_raw
        # Adjust timer dynamically based on capture rate? Or keep nominal? Let's keep nominal for now.
        self.timer_period = 1.0 / FRAMERATE # Target period (actual rate may differ)
        self.bridge = CvBridge()

        # Initialize camera capture using GStreamer
        self.get_logger().info(f"Attempting to open GStreamer pipeline: {self.gst_pipeline_string}")
        self.cap = cv2.VideoCapture(self.gst_pipeline_string, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error("!!! Failed to open GStreamer pipeline.")
            raise RuntimeError("Failed to open GStreamer pipeline")

        # Attempt to get actual capture properties (might not work reliably with GStreamer)
        # cap_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        # cap_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        # cap_fps = self.cap.get(cv2.CAP_PROP_FPS)
        # self.get_logger().info(f"Pipeline opened. Effective Resolution: {cap_width}x{cap_height}, FPS: {cap_fps}")
        self.get_logger().info("GStreamer pipeline opened successfully. Starting publishing.")

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.frame_count = 0
        self.start_time = time.time()


    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from GStreamer pipeline')
            return

        # Convert the OpenCV image to a ROS Image message using cv_bridge
        # Assuming the frame is BGR8 after videoconvert (or directly from BGRx)
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # Set the timestamp
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_frame" # Optional: Set a frame ID

        self.publisher_.publish(img_msg)

        # Log status periodically
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= 5.0: # Log every 5 seconds
             fps = self.frame_count / elapsed_time
             self.get_logger().info(f'Publishing image... (~{fps:.1f} FPS)')
             self.frame_count = 0
             self.start_time = time.time()


    def destroy_node(self):
        self.get_logger().info("Releasing camera capture...")
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    gst_cam_pub = None # Define outside try block for access in finally
    try:
        gst_cam_pub = GstCameraPublisher()
        rclpy.spin(gst_cam_pub)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"Error during node initialization or runtime: {e}")
    finally:
        # Ensure node is destroyed and shutdown called even on error
        if gst_cam_pub is not None and gst_cam_pub.cap is not None and gst_cam_pub.cap.isOpened():
            gst_cam_pub.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
