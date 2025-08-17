import tkinter as tk
from tkinter import filedialog, Text
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbags.rosbag2 import Reader
from sensor_msgs.msg import Image, Imu
import cv2
import numpy as np
from PIL import Image as PILImage, ImageTk
import threading


class RosBagViewer(tk.Tk):
    """
    A Python application to view camera frames and IMU data from a ROS2 bag file.
    """

    def __init__(self):
        """
        Initializes the main application window and its components.
        """
        super().__init__()
        self.title("ROS2 Bag Viewer")
        self.geometry("1200x800")

        # --- Member Variables ---
        self.bag_path = None
        self.image_messages = []
        self.imu_messages = []
        self.current_frame_index = -1
        self.image_topic = None
        self.imu_topic = None

        # --- UI Components ---
        # Main frame
        main_frame = tk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Control frame for buttons
        control_frame = tk.Frame(main_frame)
        control_frame.pack(fill=tk.X)

        self.open_button = tk.Button(control_frame, text="Open ROS2 Bag", command=self.open_bag_file)
        self.open_button.pack(side=tk.LEFT, padx=5, pady=5)

        self.prev_button = tk.Button(control_frame, text="<< Previous Frame", command=self.prev_frame,
                                     state=tk.DISABLED)
        self.prev_button.pack(side=tk.LEFT, padx=5, pady=5)

        self.next_button = tk.Button(control_frame, text="Next Frame >>", command=self.next_frame, state=tk.DISABLED)
        self.next_button.pack(side=tk.LEFT, padx=5, pady=5)

        self.status_label = tk.Label(control_frame, text="Please open a ROS2 bag file.")
        self.status_label.pack(side=tk.LEFT, padx=5, pady=5)

        # Topic display frame
        topic_frame = tk.Frame(main_frame)
        topic_frame.pack(fill=tk.X, pady=(0, 5))

        self.camera_topic_label = tk.Label(topic_frame, text="Camera Topic: N/A", fg="blue")
        self.camera_topic_label.pack(side=tk.LEFT, padx=10)

        self.imu_topic_label = tk.Label(topic_frame, text="IMU Topic: N/A", fg="blue")
        self.imu_topic_label.pack(side=tk.LEFT, padx=10)

        # Data display frame
        data_frame = tk.Frame(main_frame)
        data_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        data_frame.grid_columnconfigure(1, weight=1)
        data_frame.grid_rowconfigure(0, weight=1)

        # Camera frame display
        camera_frame_container = tk.LabelFrame(data_frame, text="Camera Feed")
        camera_frame_container.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")

        self.camera_label = tk.Label(camera_frame_container)
        self.camera_label.pack(fill=tk.BOTH, expand=True)
        self.timestamp_label = tk.Label(camera_frame_container, text="Timestamp: N/A")
        self.timestamp_label.pack(fill=tk.X)

        # IMU data display
        imu_frame_container = tk.LabelFrame(data_frame, text="IMU Data")
        imu_frame_container.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        self.imu_text = Text(imu_frame_container, wrap=tk.WORD, state=tk.DISABLED)
        self.imu_text.pack(fill=tk.BOTH, expand=True)

    def open_bag_file(self):
        """
        Opens a file dialog to select a ROS2 bag directory and loads the data.
        """
        path = filedialog.askdirectory(title="Select ROS2 Bag Directory")
        if not path:
            return

        # Reset UI for new bag file
        self.bag_path = path
        self.status_label.config(text=f"Loading bag: {self.bag_path}...")
        self.camera_topic_label.config(text="Camera Topic: Searching...")
        self.imu_topic_label.config(text="IMU Topic: Searching...")
        self.prev_button.config(state=tk.DISABLED)
        self.next_button.config(state=tk.DISABLED)
        self.camera_label.config(image=None, text="")
        self.timestamp_label.config(text="Timestamp: N/A")
        self.update()

        # Load data in a separate thread to keep the GUI responsive
        threading.Thread(target=self.load_bag_data).start()

    def load_bag_data(self):
        """
        Reads the ROS2 bag file to extract image and IMU messages.
        """
        try:
            with Reader(self.bag_path) as reader:
                topics = reader.topics
                # Find the first available image and IMU topics
                self.image_topic = next((name for name, t in topics.items() if t.msgtype == 'sensor_msgs/msg/Image'),
                                        None)
                self.imu_topic = next((name for name, t in topics.items() if t.msgtype == 'sensor_msgs/msg/Imu'), None)

                # Update topic labels in the GUI
                if self.image_topic:
                    self.camera_topic_label.config(text=f"Camera Topic: {self.image_topic}")
                else:
                    self.camera_topic_label.config(text="Camera Topic: Not Found")
                    self.status_label.config(text="Error: No Image topic found in the bag.")
                    return

                if self.imu_topic:
                    self.imu_topic_label.config(text=f"IMU Topic: {self.imu_topic}")
                else:
                    self.imu_topic_label.config(text="IMU Topic: Not Found")

                # Get message type definitions
                image_msg_type = get_message(topics[self.image_topic].msgtype)
                imu_msg_type = get_message(topics[self.imu_topic].msgtype) if self.imu_topic else None

                # Reset message lists
                self.image_messages = []
                self.imu_messages = []

                # Read messages from the bag
                for connection, timestamp, rawdata in reader.messages():
                    if connection.topic == self.image_topic:
                        msg = deserialize_message(rawdata, image_msg_type)
                        self.image_messages.append((timestamp, msg))
                    elif self.imu_topic and connection.topic == self.imu_topic:
                        msg = deserialize_message(rawdata, imu_msg_type)
                        self.imu_messages.append((timestamp, msg))

            # Sort messages by timestamp to ensure correct order
            self.image_messages.sort(key=lambda x: x[0])
            self.imu_messages.sort(key=lambda x: x[0])

            if self.image_messages:
                self.current_frame_index = 0
                self.display_current_frame()
                self.next_button.config(state=tk.NORMAL if len(self.image_messages) > 1 else tk.DISABLED)
                self.prev_button.config(state=tk.DISABLED)
                self.status_label.config(text=f"Loaded {len(self.image_messages)} image frames.")
            else:
                self.status_label.config(text="No image messages found on the selected topic.")

        except Exception as e:
            self.status_label.config(text=f"Error loading bag: {e}")
            print(f"Error loading bag: {e}")

    def display_current_frame(self):
        """
        Displays the camera frame and IMU data for the current index.
        """
        if not self.image_messages or not (0 <= self.current_frame_index < len(self.image_messages)):
            return

        # --- Display Camera Image ---
        timestamp, msg = self.image_messages[self.current_frame_index]

        # Convert ROS Image message to an OpenCV image
        if msg.encoding in ['rgb8', 'bgr8', 'mono8']:
            if msg.encoding == 'rgb8':
                cv_image = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'bgr8':
                cv_image = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            elif msg.encoding == 'mono8':
                cv_image = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        else:
            self.camera_label.config(image=None, text=f"Unsupported encoding: {msg.encoding}")
            return

        # Resize image to fit a fixed 640x480 box while maintaining aspect ratio
        h, w = cv_image.shape[:2]
        max_display_w, max_display_h = 640, 480
        scale = min(max_display_w / w, max_display_h / h) if w > 0 and h > 0 else 1
        new_w, new_h = int(w * scale), int(h * scale)

        if new_w > 0 and new_h > 0:
            resized_image = cv2.resize(cv_image, (new_w, new_h))
            img = PILImage.fromarray(resized_image)
            imgtk = ImageTk.PhotoImage(image=img)
            self.camera_label.imgtk = imgtk
            self.camera_label.config(image=imgtk)

        self.timestamp_label.config(text=f"Timestamp: {timestamp} ns")

        # --- Display IMU Data ---
        self.imu_text.config(state=tk.NORMAL)
        self.imu_text.delete(1.0, tk.END)

        if self.current_frame_index > 0:
            prev_timestamp, _ = self.image_messages[self.current_frame_index - 1]
            current_timestamp = timestamp
            imu_in_interval = [
                (ts, imu_msg) for ts, imu_msg in self.imu_messages
                if prev_timestamp < ts <= current_timestamp
            ]

            if imu_in_interval:
                imu_report = f"IMU data between frame {self.current_frame_index - 1} and {self.current_frame_index}:\n"
                imu_report += f"Time interval: {prev_timestamp} ns to {current_timestamp} ns\n\n"
                for ts, imu_msg in imu_in_interval:
                    imu_report += f"Timestamp: {ts} ns\n"
                    imu_report += f"  Linear Accel (x,y,z): ({imu_msg.linear_acceleration.x:.4f}, {imu_msg.linear_acceleration.y:.4f}, {imu_msg.linear_acceleration.z:.4f})\n"
                    imu_report += f"  Angular Vel (x,y,z): ({imu_msg.angular_velocity.x:.4f}, {imu_msg.angular_velocity.y:.4f}, {imu_msg.angular_velocity.z:.4f})\n\n"
                self.imu_text.insert(tk.END, imu_report)
            else:
                self.imu_text.insert(tk.END, "No IMU data in this interval.")
        else:
            self.imu_text.insert(tk.END, "This is the first frame. No previous frame to compare for IMU data.")

        self.imu_text.config(state=tk.DISABLED)

    def next_frame(self):
        """Moves to the next frame in the sequence."""
        if self.current_frame_index < len(self.image_messages) - 1:
            self.current_frame_index += 1
            self.display_current_frame()
        self.update_button_states()

    def prev_frame(self):
        """Moves to the previous frame in the sequence."""
        if self.current_frame_index > 0:
            self.current_frame_index -= 1
            self.display_current_frame()
        self.update_button_states()

    def update_button_states(self):
        """Enables or disables navigation buttons based on the current frame index."""
        self.next_button.config(
            state=tk.NORMAL if self.current_frame_index < len(self.image_messages) - 1 else tk.DISABLED)
        self.prev_button.config(state=tk.NORMAL if self.current_frame_index > 0 else tk.DISABLED)


if __name__ == '__main__':
    try:
        rclpy.init()
    except Exception as e:
        print(f"rclpy initialization skipped: {e}")

    app = RosBagViewer()
    app.mainloop()

    if rclpy.ok():
        rclpy.shutdown()
