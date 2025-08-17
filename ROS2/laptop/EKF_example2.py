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
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve

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
MAHALANOBIS_THRESHOLD = chi2.ppf(0.99, df=1)

# --- New Pose-Graph and Loop Closure Parameters ---
KEYFRAME_DISTANCE_THRESHOLD = 0.5  # meters
KEYFRAME_ANGLE_THRESHOLD = np.deg2rad(15)  # degrees
MIN_LOOP_CLOSURE_INLIERS = 25  # Minimum feature matches to consider a loop closure
MIN_GEOMETRIC_INLIERS = 15  # Minimum RANSAC inliers for geometric verification
OPTIMIZATION_ITERATIONS = 20  # Number of iterations for the pose-graph optimization

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

        # --- New Pose-Graph and Loop Closure Members ---
        self.orb = cv2.ORB_create(nfeatures=500)
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.keyframes = []
        self.odometry_edges = []
        self.loop_edges = []

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
        self.state[2] = self.normalize_angle(self.state[2] + omega * dt)
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
        self.state[2] = self.normalize_angle(self.state[2])

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
            self.add_keyframe(undistorted_frame, self.state.copy())

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
                    self.state[2] = self.normalize_angle(self.state[2])
                    # --- Online Covariance Adaptation ---
                    alpha = 0.1
                    self.R_vo = alpha * self.R_vo + (1 - alpha) * (y @ y.T)
                else:
                    print(f"--- Outlier Rejected! (v_measured: {v_measured:.2f}) ---")

            if self.is_new_keyframe(self.state):
                self.add_keyframe(undistorted_frame, self.state.copy())
                self.detect_loop_closure()

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

    def is_new_keyframe(self, current_pose):
        """Checks if the robot has moved enough to warrant a new keyframe."""
        if not self.keyframes: return True
        last_kf_pose = self.keyframes[-1]['pose']
        dist_moved = np.linalg.norm(current_pose[:2] - last_kf_pose[:2])
        angle_moved = abs(self.normalize_angle(current_pose[2] - last_kf_pose[2]))
        return dist_moved > KEYFRAME_DISTANCE_THRESHOLD or angle_moved > KEYFRAME_ANGLE_THRESHOLD

    def add_keyframe(self, frame, pose):
        """Adds a new keyframe to the database and an odometry edge."""
        kp, des = self.orb.detectAndCompute(frame, None)
        if des is None: return

        kf_id = len(self.keyframes)
        self.keyframes.append({'id': kf_id, 'pose': pose[:3], 'des': des, 'kp': kp})
        print(f"--- Added Keyframe #{kf_id} ---")

        if kf_id > 0:
            prev_kf = self.keyframes[kf_id - 1]
            T_prev = self.get_transform(prev_kf['pose'])
            T_curr = self.get_transform(pose[:3])
            T_rel = np.linalg.inv(T_prev) @ T_curr
            info_matrix = np.diag([30, 30, 100])  # Low confidence in odometry
            self.odometry_edges.append({'from': kf_id - 1, 'to': kf_id, 'transform': T_rel, 'info': info_matrix})

    def detect_loop_closure(self):
        """Detects loop closures and adds a constraint to the graph."""
        current_kf = self.keyframes[-1]
        if len(self.keyframes) < 20: return

        des_curr = current_kf['des']
        if des_curr is None: return

        best_match_id, max_good_matches, best_matches = -1, 0, None
        for i in range(len(self.keyframes) - 20):
            kf = self.keyframes[i]
            if kf['des'] is None: continue

            matches = self.bf_matcher.knnMatch(des_curr, kf['des'], k=2)
            good_matches = [m for m, n in matches if m.distance < 0.75 * n.distance]
            if len(good_matches) > max_good_matches:
                max_good_matches, best_match_id, best_matches = len(good_matches), kf['id'], good_matches

        if max_good_matches > MIN_LOOP_CLOSURE_INLIERS:
            # --- Geometric Verification of Loop Closure ---
            matched_kf = self.keyframes[best_match_id]
            src_pts = np.float32([current_kf['kp'][m.queryIdx].pt for m in best_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([matched_kf['kp'][m.trainIdx].pt for m in best_matches]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            inlier_count = np.sum(mask)

            if inlier_count > MIN_GEOMETRIC_INLIERS:
                print(
                    f"*** GEOMETRICALLY VERIFIED LOOP: KF {current_kf['id']} -> KF {best_match_id} ({inlier_count} inliers) ***")
                T_rel = np.eye(3)
                info_matrix = np.diag([1000, 1000, 5000])  # High confidence in loop closure
                self.loop_edges.append(
                    {'from': best_match_id, 'to': current_kf['id'], 'transform': T_rel, 'info': info_matrix})
                self.optimize_pose_graph()
            else:
                print(f"--- Loop candidate rejected by geometric check (Inliers: {inlier_count}) ---")

    def huber_weight(self, error_vec, delta=1.0):
        """Calculates the Huber weight for a given residual."""
        norm = np.linalg.norm(error_vec)
        if norm <= delta:
            return 1.0
        return delta / norm

    def optimize_pose_graph(self):
        """Performs pose-graph optimization to correct drift."""
        print("--- Starting Pose-Graph Optimization ---")
        poses = [kf['pose'] for kf in self.keyframes]

        for iteration in range(OPTIMIZATION_ITERATIONS):
            H = lil_matrix((len(poses) * 3, len(poses) * 3))
            b = np.zeros(len(poses) * 3)
            total_error = 0

            for edge in self.odometry_edges + self.loop_edges:
                p_from_idx, p_to_idx = edge['from'], edge['to']
                p_from, p_to = poses[p_from_idx], poses[p_to_idx]
                T_z = edge['transform']
                info = edge['info']

                T_from_inv = self.get_transform_inverse(p_from)
                T_error = np.linalg.inv(T_z) @ T_from_inv @ self.get_transform(p_to)

                err_x, err_y, err_theta = self.get_pose_from_transform(T_error)
                error_vec = np.array([err_x, err_y, self.normalize_angle(err_theta)])

                # --- Use Huber weight for robust estimation ---
                weight = self.huber_weight(error_vec)

                total_error += weight * (error_vec.T @ info @ error_vec)

                th = p_from[2]
                A = np.array([[-np.cos(th), -np.sin(th),
                               -np.sin(th) * (p_to[0] - p_from[0]) + np.cos(th) * (p_to[1] - p_from[1])],
                              [np.sin(th), np.cos(th),
                               -np.cos(th) * (p_to[0] - p_from[0]) - np.sin(th) * (p_to[1] - p_from[1])],
                              [0, 0, -1]])
                B = np.array([[np.cos(th), np.sin(th), 0], [-np.sin(th), np.cos(th), 0], [0, 0, 1]])

                i, j = p_from_idx * 3, p_to_idx * 3
                H[i:i + 3, i:i + 3] += weight * A.T @ info @ A
                H[j:j + 3, j:j + 3] += weight * B.T @ info @ B
                H[i:i + 3, j:j + 3] += weight * A.T @ info @ B
                H[j:j + 3, i:i + 3] += weight * B.T @ info @ A

                b[i:i + 3] += weight * (A.T @ info @ error_vec).flatten()
                b[j:j + 3] += weight * (B.T @ info @ error_vec).flatten()

            H[0:3, 0:3] += np.eye(3)  # Anchor the first pose
            damping_factor = 1e-6
            H += np.eye(H.shape[0]) * damping_factor

            dx = spsolve(H.tocsr(), -b)

            for i in range(len(poses)):
                poses[i] += dx[i * 3: i * 3 + 3]
                poses[i][2] = self.normalize_angle(poses[i][2])

        print(f"--- Optimization Complete (Error: {total_error:.4f}) ---")
        for i, kf in enumerate(self.keyframes):
            kf['pose'] = poses[i]
        self.state[:3] = poses[-1]

    # Helper functions for 2D transformations
    def get_transform(self, pose):
        T = np.eye(3)
        T[0, 0] = np.cos(pose[2]);
        T[0, 1] = -np.sin(pose[2]);
        T[0, 2] = pose[0]
        T[1, 0] = np.sin(pose[2]);
        T[1, 1] = np.cos(pose[2]);
        T[1, 2] = pose[1]
        return T

    def get_transform_inverse(self, pose):
        c, s = np.cos(pose[2]), np.sin(pose[2])
        T_inv = np.eye(3)
        T_inv[0, 0] = c;
        T_inv[0, 1] = s;
        T_inv[0, 2] = -pose[0] * c - pose[1] * s
        T_inv[1, 0] = -s;
        T_inv[1, 1] = c;
        T_inv[1, 2] = pose[0] * s - pose[1] * c
        return T_inv

    def get_pose_from_transform(self, T):
        return T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

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

            if self.keyframes:
                self.trajectory = [kf['pose'][:2] for kf in self.keyframes]
                self.trajectory.append(self.state[:2])

            if self.trajectory:
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

        ani = FuncAnimation(self.fig, update_frame, frames=len(self.processed_messages), blit=False, interval=20,
                            repeat=False)
        plt.show()


if __name__ == '__main__':
    vio = OfflineVIO()
    vio.load_bag_data(BAG_FILE_PATH)
    vio.calibrate_and_prepare_data(calibration_frames=CALIBRATION_FRAMES, start_frame=START_FRAME)
    vio.run_simulation()
    print("Processing complete.")
