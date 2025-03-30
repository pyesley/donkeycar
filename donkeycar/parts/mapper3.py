import cv2
import numpy as np
import math
import logging
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from arduinoSerial import ARDUINONanoSerial

class Mapper:
    def __init__(self):
        # Current position and history
        self.xCurrent, self.yCurrent = 0.0, 0.0
        self.yaw = 0.0
        self.xHistory = [0]
        self.yHistory = [0]

        # EKF state vector: [x, y, theta, v_bias, theta_bias]
        self.state = np.zeros(5)
        self.covariance = np.diag([0.01, 0.01, 0.01, 0.005, 0.005])  # Initial uncertainty

        # Sensor data
        self.velocity = 0
        self.ticks = 0
        self.last_ticks = 0
        self.direction = 1
        self.imgLast = None
        self.lastGray = None
        self.lastKeypoints = None
        self.lastDescriptors = None
        self.last_ticks = 0

        # Arduino communication
        try:
            self.arduino = ARDUINONanoSerial()
        except ImportError:
            self.arduino = None
            print("Warning: ArduinoSerial module not found")

        # Process and measurement noise covariances
        self.Q = np.diag([0.01, 0.01, 0.01, 0.001, 0.001])  # Process noise

        # Confidence metrics for adaptive noise
        self.camera_confidence = 0.5
        self.encoder_confidence = 0.7
        self.imu_confidence = 0.6

        # Feature tracking parameters
        self.orb = cv2.ORB_create(nfeatures=500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.match_threshold = 50
        self.min_good_matches = 15

        # For storing motion estimates from camera
        self.camera_dx = 0
        self.camera_dy = 0
        self.camera_dtheta = 0

    def updateCameraPosition(self, img):
        """
        Extract visual odometry information from camera images
        Returns distance moved and visualization image
        """
        # Convert to grayscale for feature detection
        if len(img.shape) == 3:
            curr_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            curr_gray = img.copy()

        # Initialize on first call
        if self.imgLast is None:
            self.imgLast = img.copy()
            self.lastGray = curr_gray
            self.camera_confidence = 0.5
            return 0, img

        # Detect keypoints and compute descriptors
        curr_keypoints, curr_descriptors = self.orb.detectAndCompute(curr_gray, None)

        if self.lastKeypoints is None:
            self.lastKeypoints, self.lastDescriptors = self.orb.detectAndCompute(self.lastGray, None)

        # Exit early if not enough features
        if (curr_descriptors is None or self.lastDescriptors is None or
            len(curr_keypoints) < self.min_good_matches or
            len(self.lastKeypoints) < self.min_good_matches):
            self.lastGray = curr_gray
            self.camera_confidence *= 0.8  # Reduce confidence when features are sparse
            return 0, img

        # Match features
        matches = self.matcher.match(self.lastDescriptors, curr_descriptors)
        matches = sorted(matches, key=lambda x: x.distance)

        # Filter good matches
        good_matches = [m for m in matches if m.distance < self.match_threshold]

        # Create visualization image
        visual_img = img.copy()

        # Camera movement estimation
        if len(good_matches) >= self.min_good_matches:
            # Extract matched point coordinates
            src_pts = np.float32([self.lastKeypoints[m.queryIdx].pt for m in good_matches])
            dst_pts = np.float32([curr_keypoints[m.trainIdx].pt for m in good_matches])

            # Compute essential matrix with RANSAC to filter outliers
            E, mask = cv2.findEssentialMat(src_pts, dst_pts, focal=700.0, pp=(320, 240),
                                          method=cv2.RANSAC, prob=0.999, threshold=1.0)

            if E is not None:
                # Recover relative pose from essential matrix
                _, R, t, mask = cv2.recoverPose(E, src_pts, dst_pts)

                # Extract rotation angle (yaw) from rotation matrix
                self.camera_dtheta = math.atan2(R[1, 0], R[0, 0])

                # Scale translation vector (needs calibration)
                scale_factor = 0.01 * self.encoder_confidence
                scale_factor = 1.0
                self.camera_dx = t[0, 0] * scale_factor
                self.camera_dy = t[1, 0] * scale_factor

                # Calculate confidence based on number and quality of matches
                inlier_ratio = np.sum(mask) / len(mask) if len(mask) > 0 else 0
                avg_distance = np.mean([m.distance for m in good_matches[:20]])
                self.camera_confidence = min(0.9, inlier_ratio * (1.0 - min(1.0, avg_distance/100)))

                # Draw matches on visualization image
                visual_img = cv2.drawMatches(self.lastGray, self.lastKeypoints,
                                           curr_gray, curr_keypoints,
                                           good_matches[:15], None,
                                           flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                # Update stored features
                self.lastKeypoints = curr_keypoints
                self.lastDescriptors = curr_descriptors
                self.lastGray = curr_gray

                return np.sqrt(self.camera_dx**2 + self.camera_dy**2), visual_img

        # Update for next iteration with reduced confidence
        self.camera_confidence *= 0.9
        self.lastGray = curr_gray
        self.lastKeypoints = curr_keypoints
        self.lastDescriptors = curr_descriptors
        return 0, visual_img

    def updateEncoderPosition(self):
        """
        Process encoder data and update state
        """
        if self.arduino is None:
            return

        try :
            ticks, velocity = self.arduino.request_ENC()
            self.velocity = velocity
            self.ticks = ticks
        except:
            logging.info("Error reading encoder data")

    def updateIMU(self):
        """
        Process IMU data for orientation
        """
        if self.arduino is None:
            return

        roll, pitch, yaw = self.arduino.request_im2()

        if yaw != -999:
            self.yaw = yaw

    def ekf_predict(self):
        """
        EKF prediction step using motion model
        """
        # Extract state variables
        x, y, theta, v_bias, theta_bias = self.state

        # Calculate encoder-based movement
        if hasattr(self, 'last_ticks'):
            delta_ticks = self.ticks - self.last_ticks
            distance = delta_ticks * 0.01 * self.direction  # Scale factor

            # Update state prediction
            theta_rad = theta
            self.state[0] = x + distance * math.cos(theta_rad)  # x position
            self.state[1] = y + distance * math.sin(theta_rad)  # y position
            # theta stays the same in prediction unless we have control input

            # Jacobian of state transition
            F = np.eye(5)
            F[0, 2] = -distance * math.sin(theta_rad)  # dx/dtheta
            F[1, 2] = distance * math.cos(theta_rad)   # dy/dtheta

            # Update covariance
            self.covariance = F @ self.covariance @ F.T + self.Q

    def ekf_update_camera(self):
        """
        EKF update using camera measurements
        """
        if self.camera_confidence < 0.2:
            return

        # Camera measurement in robot frame
        camera_z = np.array([self.camera_dx, self.camera_dy, self.camera_dtheta])

        # Measurement noise (adjusted by confidence)
        R_camera = np.diag([0.2/max(0.1, self.camera_confidence),
                           0.2/max(0.1, self.camera_confidence),
                           0.1/max(0.1, self.camera_confidence)])

        # Measurement function is relative motion in robot frame
        # We need to transform it to world frame for updating state
        theta = self.state[2]
        c, s = math.cos(theta), math.sin(theta)
        R = np.array([[c, -s], [s, c]])  # Rotation matrix

        # Transform camera motion to world frame
        world_dx, world_dy = R @ np.array([self.camera_dx, self.camera_dy])

        # Expected measurement (zero since we've already applied the motion)
        h = np.zeros(3)

        # Measurement Jacobian (relates state to measurement)
        H = np.zeros((3, 5))
        H[0, 0] = 1.0  # dx affects x measurement
        H[1, 1] = 1.0  # dy affects y measurement
        H[2, 2] = 1.0  # dtheta affects theta measurement

        # Innovation
        y = camera_z - h

        # Innovation covariance
        S = H @ self.covariance @ H.T + R_camera

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        I = np.eye(5)
        self.covariance = (I - K @ H) @ self.covariance

    def ekf_update_imu(self):
        """
        EKF update using IMU measurements
        """
        if not hasattr(self, 'yaw'):
            return

        # IMU measurement (in radians)
        imu_theta = math.radians(self.yaw)

        # Measurement noise (adjusted by confidence)
        R_imu = np.array([[0.1/max(0.1, self.imu_confidence)]])

        # Measurement model: h(x) = theta + theta_bias
        h = self.state[2] + self.state[4]  # Current theta + bias

        # Measurement Jacobian
        H = np.zeros((1, 5))
        H[0, 2] = 1.0  # dh/dtheta = 1
        H[0, 4] = 1.0  # dh/dtheta_bias = 1

        # Innovation
        y = np.array([imu_theta - h])

        # Innovation covariance
        S = H @ self.covariance @ H.T + R_imu

        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Update covariance
        I = np.eye(5)
        self.covariance = (I - K @ H) @ self.covariance

    def sensorFusion(self):
        """
        Fuse sensor data using Extended Kalman Filter
        """
        # Prediction step
        self.ekf_predict()

        # Update steps with each sensor
        self.ekf_update_camera()
        self.ekf_update_imu()

        # Extract updated position
        self.xCurrent = self.state[0]
        self.yCurrent = self.state[1]
        self.yaw = math.degrees(self.state[2])

        # Update trajectory history
        self.xHistory.append(self.xCurrent)
        self.yHistory.append(self.yCurrent)

        # Update sensor confidences based on consistency
        self.updateSensorConfidence()

        # Save ticks for next update
        if hasattr(self, 'ticks'):
            self.last_ticks = self.ticks

        # Log position
        logging.info(f"Position: x={self.xCurrent:.2f}, y={self.yCurrent:.2f}, heading={self.yaw:.2f}°")
        logging.info(f"Encoder: {self.ticks:.2f}")
        logging.info(f"Confidences - Encoder: {self.encoder_confidence:.2f}, "
                    f"Camera: {self.camera_confidence:.2f}, IMU: {self.imu_confidence:.2f}")

    def updateSensorConfidence(self):
        """
        Update confidence in each sensor based on consistency
        """
        if len(self.xHistory) < 2:
            return

        # Calculate observed movement
        dx = self.xHistory[-1] - self.xHistory[-2]
        dy = self.yHistory[-1] - self.yHistory[-2]
        actual_distance = math.sqrt(dx*dx + dy*dy)

        # Compare with encoder estimate
        if hasattr(self, 'last_ticks'):
            delta_ticks = self.ticks - self.last_ticks
            encoder_distance = delta_ticks * 0.01 * self.direction
            encoder_error = abs(encoder_distance - actual_distance)

            # Update encoder confidence
            if encoder_distance > 0.001:
                self.encoder_confidence = max(0.2, min(0.9,
                    self.encoder_confidence * (1.0 - min(1.0, encoder_error))))

        # IMU confidence slowly decreases to model drift
        self.imu_confidence = max(0.2, min(0.9, self.imu_confidence * 0.999))

        # Camera confidence is already updated in updateCameraPosition

    def getPlotImage(self):
        """
        Generate a visualization of the robot's path
        """
        # Create figure and axis
        fig = Figure(figsize=(8, 6), dpi=100)
        canvas = FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)

        # Plot trajectory
        ax.plot(self.xHistory, self.yHistory, 'b-', linewidth=2)

        # Mark the current position
        ax.plot(self.xCurrent, self.yCurrent, 'ro', markersize=10)

        # Add start point
        if len(self.xHistory) > 0:
            ax.plot(self.xHistory[0], self.yHistory[0], 'go', markersize=8)

        # Plot uncertainty ellipse
        pos_cov = self.covariance[0:2, 0:2]
        if np.linalg.det(pos_cov) > 0:
            # Get eigenvalues and eigenvectors
            vals, vecs = np.linalg.eig(pos_cov)

            # Width and height of ellipse (95% confidence)
            width, height = 2 * np.sqrt(5.991 * vals)

            # Angle of ellipse
            angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))

            # Create an ellipse - FIX: Use xy parameter instead of separate x,y
            from matplotlib.patches import Ellipse
            ellipse = Ellipse(xy=(self.xCurrent, self.yCurrent),
                              width=width,
                              height=height,
                              angle=angle,
                              fill=False, color='red', linewidth=2)
            ax.add_patch(ellipse)

        # Rest of the function remains the same...
        # Set labels and title
        ax.set_xlabel('X position (m)')
        ax.set_ylabel('Y position (m)')
        ax.set_title('Robot Trajectory (EKF SLAM)')
        ax.grid(True)

        # Equal aspect ratio
        ax.axis('equal')

        # Add margins around the path
        if len(self.xHistory) > 1:
            xmin, xmax = min(self.xHistory), max(self.xHistory)
            ymin, ymax = min(self.yHistory), max(self.yHistory)
            margin = max(0.5, (xmax - xmin) * 0.1, (ymax - ymin) * 0.1)
            ax.set_xlim(xmin - margin, xmax + margin)
            ax.set_ylim(ymin - margin, ymax + margin)
        else:
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)

        # Draw the plot to canvas
        canvas.draw()

        # Convert to numpy array
        s, (width, height) = canvas.print_to_buffer()
        plot_image = np.frombuffer(s, np.uint8).reshape((height, width, 4))

        # Convert RGBA to RGB
        plot_image = cv2.cvtColor(plot_image, cv2.COLOR_RGBA2RGB)

        return plot_image

    def run(self, imageNew, direction=1):
        """
        Main run loop - process new data and update map
        """
        self.direction = direction  # Update direction (1=forward, -1=reverse)

        # Update with sensor data
        visual_distance, visual_img = self.updateCameraPosition(imageNew)
        self.updateEncoderPosition()
        self.updateIMU()

        # Fuse sensor data using EKF
        self.sensorFusion()

        # Generate plot image
        plot_image = self.getPlotImage()

        # Create combined visualization
        h1, w1 = visual_img.shape[:2]
        h2, w2 = plot_image.shape[:2]

        # Resize for consistent display if needed
        if h1 != h2:
            scale = h2 / h1
            visual_img = cv2.resize(visual_img, (int(w1 * scale), h2))

        combined_output = np.hstack((visual_img, plot_image))

        # Update imgLast for next time
        self.imgLast = imageNew.copy()

        return combined_output, visual_img, plot_image