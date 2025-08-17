import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.stats import chi2

# --- Configuration ---
BAG_FILE_PATH = '/home/pyesley/ros2Data'  # IMPORTANT: Set this to your bag file path
IMAGE_TOPIC = '/camera/image_raw'
IMU_TOPIC = '/imu/data_raw_mpu6050'

# --- Tuning and Control Parameters ---
CALIBRATION_FRAMES = 2000
START_FRAME = 2000
VISUAL_ODOMETRY_SCALE_FACTOR = 0.1
STATIONARY_THRESHOLD = 0.5
MAX_FEATURES = 200
MIN_FEATURES_TO_TRACK = 100

# --- Outlier Rejection Parameter ---
MAHALANOBIS_THRESHOLD = chi2.ppf(0.99, df=1)  # 99% confidence for outlier rejection

# Provided Camera Intrinsics and Distortion Coefficients
cameraMatrix = np.array([[700.32833455, 0., 633.7861054],
                         [0., 703.47654591, 342.84793261],
                         [0., 0., 1.]])
distCoeffs = np.array([[-0.30944433, 0.1258339, -0.00045581, -0.00057164, -0.01383379]])


class OfflineVIO:
    def __init__(self):
        # EKF State and Covariance
        self.state = np.zeros(5)
        self.covariance = np.eye(5) * 0.1

        # Noise Parameters
        self.Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1])
        self.R_vo = np.diag([0.1])
        self.R_imu = np.diag([0.01])

        # Data for processing
        self.prev_image = None
        self.prev_features = None
        self.last_image_timestamp = None
        self.trajectory = []
        self.all_messages = []
        self.processed_messages = []

        # Plotting
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 7))
        self.im_display = self.ax1.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
        self.ax1.set_title("Camera Feed")
        self.ax1.axis('off')
        self.traj_plot, = self.ax2.plot([], [], 'r-')
        self.ax2.set_title("Estimated Trajectory (Bird's Eye View)")
        self.ax2.set_xlabel("X (m)")
        self.ax2.set_ylabel("Y (m)")
        self.ax2.set_aspect('equal', adjustable='box')
        self.ax2.grid(True)
        plt.tight_layout()

    def load_bag_data(self, bag_path):
        """Reads all messages from the bag and sorts them by timestamp."""
        print(f"Loading data from {bag_path}...")
        with Reader(bag_path) as reader:
            topic_to_msgtype = {name: topic.msgtype for name, topic in reader.topics.items()}
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic in [IMAGE_TOPIC, IMU_TOPIC]:
                    msg_type = get_message(topic_to_msgtype[connection.topic])
                    msg = deserialize_message(rawdata, msg_type)
                    self.all_messages.append({'topic': connection.topic, 'timestamp': timestamp, 'msg': msg})
        self.all_messages.sort(key=lambda x: x['timestamp'])
        print(f"Loaded and sorted {len(self.all_messages)} total messages.")

    def calibrate_and_prepare_data(self, calibration_frames, start_frame):
        """Calibrates IMU noise and prepares the message list for processing."""
        if not self.all_messages: return
        print(f"Calibrating IMU noise using the first {calibration_frames} messages...")
        omegas = [msg_data['msg'].angular_velocity.z for msg_data in self.all_messages[:calibration_frames] if
                  msg_data['topic'] == IMU_TOPIC]
        if omegas:
            imu_variance = np.var(omegas)
            self.R_imu = np.diag([imu_variance])
            print(f"IMU calibration complete. Angular velocity variance (R_imu) set to: {imu_variance:.6f}")
        else:
            print("Warning: No IMU messages found in calibration range. Using default R_imu.")

        print(f"Preparing to start processing from message #{start_frame}...")
        if start_frame > 0 and start_frame < len(self.all_messages):
            initialization_frame_index = next(
                (i for i in range(start_frame - 1, -1, -1) if self.all_messages[i]['topic'] == IMAGE_TOPIC), -1)
            if initialization_frame_index != -1:
                print(f"Initializing visual odometry with image from message #{initialization_frame_index}")
                msg_data = self.all_messages[initialization_frame_index]
                self.process_image(msg_data['msg'], 0, initialize_only=True)
                self.last_image_timestamp = msg_data['timestamp']
            self.processed_messages = self.all_messages[start_frame:]
        else:
            self.processed_messages = self.all_messages
        print(f"Ready to process {len(self.processed_messages)} messages.")

    def predict_step(self, dt):
        """EKF prediction step."""
        if dt <= 0: return
        x, y, theta, v, omega = self.state
        self.state[0] += v * np.cos(theta) * dt
        self.state[1] += v * np.sin(theta) * dt
        self.state[2] += omega * dt
        F = np.array([[1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt, 0],
                      [0, 1, v * np.cos(theta) * dt, np.sin(theta) * dt, 0], [0, 0, 1, 0, dt], [0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 1]])
        self.covariance = F @ self.covariance @ F.T + self.Q

    def process_imu(self, msg):
        """EKF update step for IMU measurement."""
        omega_measured = msg.angular_velocity.z
        H = np.array([[0, 0, 0, 0, 1]])
        y = np.array([omega_measured - self.state[4]])
        S = H @ self.covariance @ H.T + self.R_imu
        K = self.covariance @ H.T @ np.linalg.inv(S)
        self.state += K @ y
        self.covariance = (np.eye(5) - K @ H) @ self.covariance

    def process_image(self, msg, dt, initialize_only=False):
        """EKF update step for Image measurement."""
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8': frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        display_frame, gray_frame = frame.copy(), cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        undistorted_frame = cv2.undistort(gray_frame, cameraMatrix, distCoeffs)

        if self.prev_image is None or self.prev_features is None:
            self.prev_image, self.prev_features = undistorted_frame, cv2.goodFeaturesToTrack(undistorted_frame,
                                                                                             maxCorners=MAX_FEATURES,
                                                                                             qualityLevel=0.01,
                                                                                             minDistance=30)

        if self.prev_features is not None:
            curr_features, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_image, undistorted_frame, self.prev_features,
                                                                None)
            good_new = curr_features[
                status.ravel() == 1] if curr_features is not None and status is not None else np.array([])
            good_old = self.prev_features[status.ravel() == 1] if status is not None else np.array([])

            for point in good_new: cv2.circle(display_frame, tuple(point.ravel().astype(int)), 5, (0, 255, 0), -1)

            if not initialize_only and len(good_new) > 5:
                displacement = np.mean(np.linalg.norm(good_new - good_old, axis=1))
                v_measured = 0.0
                if displacement > STATIONARY_THRESHOLD:
                    E, mask = cv2.findEssentialMat(good_new, good_old, cameraMatrix, method=cv2.RANSAC)
                    if E is not None:
                        _, R, t, _ = cv2.recoverPose(E, good_new, good_old, cameraMatrix, mask=mask)
                        if dt > 1e-6: v_measured = (np.linalg.norm(t[::2]) / dt) * VISUAL_ODOMETRY_SCALE_FACTOR

                H = np.array([[0, 0, 0, 1, 0]])
                y = np.array([v_measured - self.state[3]])
                S = H @ self.covariance @ H.T + self.R_vo
                mahalanobis_dist = y.T @ np.linalg.inv(S) @ y
                if mahalanobis_dist < MAHALANOBIS_THRESHOLD:
                    K = self.covariance @ H.T @ np.linalg.inv(S)
                    self.state += K @ y
                    self.covariance = (np.eye(5) - K @ H) @ self.covariance
                else:
                    print(f"--- Outlier Rejected! (v_measured: {v_measured:.2f}) ---")

            final_features = good_new
            if len(final_features) < MIN_FEATURES_TO_TRACK:
                mask = np.ones(gray_frame.shape[:2], dtype=np.uint8) * 255
                for point in final_features: cv2.circle(mask, tuple(point.ravel().astype(int)), 10, 0, -1)
                new_features = cv2.goodFeaturesToTrack(undistorted_frame, maxCorners=MAX_FEATURES - len(final_features),
                                                       qualityLevel=0.01, minDistance=15, mask=mask)
                if new_features is not None: final_features = np.vstack((final_features, new_features))
            self.prev_features = final_features if final_features is not None and final_features.size > 0 else None

        self.prev_image = undistorted_frame
        self.im_display.set_data(cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB))

    def run_simulation(self):
        if not self.processed_messages: return
        last_timestamp = self.processed_messages[0]['timestamp']

        def update_frame(i):
            nonlocal last_timestamp
            msg_data = self.processed_messages[i]
            timestamp, topic, msg = msg_data['timestamp'], msg_data['topic'], msg_data['msg']
            dt = (timestamp - last_timestamp) / 1e9
            last_timestamp = timestamp

            self.predict_step(dt)
            if topic == IMU_TOPIC:
                self.process_imu(msg)
            elif topic == IMAGE_TOPIC:
                image_dt = (timestamp - self.last_image_timestamp) / 1e9 if self.last_image_timestamp else dt
                self.process_image(msg, image_dt)
                self.last_image_timestamp = timestamp

            self.trajectory.append((self.state[0], self.state[1]))
            traj_x, traj_y = zip(*self.trajectory)
            self.traj_plot.set_data(np.array(traj_x), np.array(traj_y))
            if len(traj_x) > 1:
                min_x, max_x, min_y, max_y = np.min(traj_x), np.max(traj_x), np.min(traj_y), np.max(traj_y)
                center_x, center_y = np.mean(traj_x), np.mean(traj_y)
                range_x, range_y = max_x - min_x, max_y - min_y
                plot_range = max(range_x, range_y, 1) * 1.2
                self.ax2.set_xlim(center_x - plot_range / 2, center_x + plot_range / 2)
                self.ax2.set_ylim(center_y - plot_range / 2, center_y + plot_range / 2)

            self.fig.canvas.setWindowTitle(f"Processing Frame {START_FRAME + i + 1}/{len(self.all_messages)}")
            return self.im_display, self.traj_plot

        # Set blit=False to ensure the axes are redrawn correctly when their limits change.
        ani = FuncAnimation(self.fig, update_frame, frames=len(self.processed_messages), blit=False, interval=20,
                            repeat=False)
        plt.show()


if __name__ == '__main__':
    vio = OfflineVIO()
    vio.load_bag_data(BAG_FILE_PATH)
    vio.calibrate_and_prepare_data(calibration_frames=CALIBRATION_FRAMES, start_frame=START_FRAME)
    vio.run_simulation()
    print("Processing complete.")
