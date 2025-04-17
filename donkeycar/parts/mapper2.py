import cv2
import numpy as np
import math
import logging
import gtsam
import gtsam.utils.plot as gtsam_plot
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from gtsam.symbol_shorthand import L, X

class Mapper:
    def __init__(self):
        # Initialize GTSAM components
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()

        # Optimization parameters
        self.optimizer = gtsam.LevenbergMarquardtOptimizer
        self.isam = gtsam.ISAM2()

        # Current position and pose index
        self.xCurrent, self.yCurrent = 0.0, 0.0
        self.yaw = 0.0
        self.pose_index = 0

        # Sensor data
        self.velocity = 0
        self.ticks = 0
        self.last_ticks = 0
        self.direction = 1
        self.imgLast = None
        self.lastGray = None
        self.lastKeypoints = None
        self.lastDescriptors = None

        # Arduino communication
        self.arduino = ArduinoSerial()

        # Trajectory history for visualization
        self.xHistory = [0]
        self.yHistory = [0]

        # Noise models for sensors
        self.encoder_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
        self.camera_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
        self.imu_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.1]))

        # Add prior on first pose (origin with high certainty)
        first_pose = gtsam.Pose2(0, 0, 0)
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
        self.graph.add(gtsam.PriorFactorPose2(X(0), first_pose, prior_noise))
        self.initial_estimate.insert(X(0), first_pose)

        # Confidence metrics for adaptive noise models
        self.camera_confidence = 0.5
        self.encoder_confidence = 0.7
        self.imu_confidence = 0.6

        # Feature tracking parameters
        self.orb = cv2.ORB_create(nfeatures=500)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.match_threshold = 50
        self.min_good_matches = 15

        # Initialize optimizer
        self.result = self.initial_estimate

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
            len(curr_keypoints) < self.min_good_matches or len(self.lastKeypoints) < self.min_good_matches):
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
                delta_yaw = math.atan2(R[1, 0], R[0, 0])

                # Scale translation vector
                # This scaling factor needs calibration for your specific camera setup
                scale_factor = 0.01 * self.encoder_confidence
                tx = t[0, 0] * scale_factor
                ty = t[1, 0] * scale_factor

                # Calculate confidence based on number and quality of matches
                inlier_ratio = np.sum(mask) / len(mask) if len(mask) > 0 else 0
                avg_distance = np.mean([m.distance for m in good_matches[:20]])
                self.camera_confidence = min(0.9, inlier_ratio * (1.0 - min(1.0, avg_distance/100)))

                # Add visual odometry factor to graph if confidence is sufficient
                if self.camera_confidence > 0.3 and self.pose_index > 0:
                    prev_idx = self.pose_index - 1
                    curr_idx = self.pose_index

                    # Adjust noise model based on confidence
                    adapted_noise = gtsam.noiseModel.Diagonal.Sigmas(
                        np.array([0.2 / self.camera_confidence, 0.2 / self.camera_confidence, 0.1 / self.camera_confidence]))

                    # Create relative pose transform from visual odometry
                    camera_pose = gtsam.Pose2(tx, ty, delta_yaw)
                    self.graph.add(gtsam.BetweenFactorPose2(X(prev_idx), X(curr_idx), camera_pose, adapted_noise))

                # Draw matches on visualization image
                cv2.drawMatches(self.lastGray, self.lastKeypoints, curr_gray, curr_keypoints,
                               good_matches[:15], visual_img, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                # Update stored features
                self.lastKeypoints = curr_keypoints
                self.lastDescriptors = curr_descriptors
                self.lastGray = curr_gray

                # Return estimated movement and visualization
                return np.sqrt(tx**2 + ty**2), visual_img

        # Update for next iteration with reduced confidence
        self.camera_confidence *= 0.9
        self.lastGray = curr_gray
        self.lastKeypoints = curr_keypoints
        self.lastDescriptors = curr_descriptors
        return 0, visual_img

    def updateEncoderPosition(self):
        """
        Process encoder data and add odometry factor to graph
        """
        ticks, velocity = self.arduino.request_ENC()
        self.velocity = velocity

        if hasattr(self, 'last_ticks'):
            delta_ticks = ticks - self.last_ticks
            distance = delta_ticks * 0.01 * self.direction  # Adjust scale factor as needed

            # Only add factors when we have meaningful movement
            if abs(distance) > 0.001 and self.pose_index > 0:
                prev_idx = self.pose_index - 1
                curr_idx = self.pose_index

                # Get previous pose estimate
                prev_pose = self.result.atPose2(X(prev_idx))

                # Calculate odometry-based pose
                heading_rad = prev_pose.theta()
                dx = distance * math.cos(heading_rad)
                dy = distance * math.sin(heading_rad)

                # Create odometry factor
                encoder_pose = gtsam.Pose2(dx, dy, 0.0)  # No rotation from encoder

                # Adjust noise model based on confidence
                adapted_noise = gtsam.noiseModel.Diagonal.Sigmas(
                    np.array([0.1/self.encoder_confidence, 0.1/self.encoder_confidence, 0.2]))

                self.graph.add(gtsam.BetweenFactorPose2(X(prev_idx), X(curr_idx), encoder_pose, adapted_noise))

                # Update encoder confidence based on consistency with other sensors
                if self.camera_confidence > 0.5:
                    # Compare with camera estimate (if we have one)
                    # This would be implemented in sensorFusion
                    pass

        self.last_ticks = ticks
        self.ticks = ticks

    def updateIMU(self):
        """
        Process IMU data and add orientation factors to the graph
        """
        roll, pitch, yaw = self.arduino.request_im2()

        if yaw != -999:
            old_yaw = self.yaw
            self.yaw = yaw

            # Add IMU factor only if we have a significant change and a previous pose
            if self.pose_index > 0 and abs(yaw - old_yaw) > 0.01:
                prev_idx = self.pose_index - 1
                curr_idx = self.pose_index

                # Extract previous pose estimate
                prev_pose = self.result.atPose2(X(prev_idx))

                # Create relative orientation factor
                delta_yaw = yaw - old_yaw
                imu_pose = gtsam.Pose2(0.0, 0.0, delta_yaw)  # Only rotation from IMU

                # Adjust noise model based on confidence
                adapted_noise = gtsam.noiseModel.Diagonal.Sigmas(
                    np.array([0.5, 0.5, 0.05/self.imu_confidence]))

                self.graph.add(gtsam.BetweenFactorPose2(X(prev_idx), X(curr_idx), imu_pose, adapted_noise))

    def sensorFusion(self):
        """
        Fuse sensor data using factor graph optimization
        """
        # Increment pose index for the new position
        self.pose_index += 1

        # Add initial estimate for new pose based on previous optimized result
        if self.pose_index > 0:
            prev_pose = self.result.atPose2(X(self.pose_index - 1))
            heading_rad = prev_pose.theta()

            # Simple motion model prediction
            distance = 0.01 * self.direction  # Small default movement
            dx = distance * math.cos(heading_rad)
            dy = distance * math.sin(heading_rad)

            new_pose = gtsam.Pose2(
                prev_pose.x() + dx,
                prev_pose.y() + dy,
                heading_rad
            )

            self.initial_estimate.insert(X(self.pose_index), new_pose)

        # Optimize factor graph using iSAM2 for efficient incremental updates
        self.isam.update(self.graph, self.initial_estimate)
        self.result = self.isam.calculateEstimate()

        # Clear initial values for next iteration
        self.initial_estimate.clear()

        # Extract current position from optimized graph
        current_pose = self.result.atPose2(X(self.pose_index))
        self.xCurrent = current_pose.x()
        self.yCurrent = current_pose.y()
        self.yaw = math.degrees(current_pose.theta())

        # Update history for visualization
        self.xHistory.append(self.xCurrent)
        self.yHistory.append(self.yCurrent)

        # Update sensor confidences based on consistency
        self.updateSensorConfidence()

        # Log the result
        logging.info(f"Position: x={self.xCurrent:.2f}, y={self.yCurrent:.2f}, heading={self.yaw:.2f}°")
        logging.info(f"Confidences - Encoder: {self.encoder_confidence:.2f}, " +
                    f"Camera: {self.camera_confidence:.2f}, IMU: {self.imu_confidence:.2f}")

    def updateSensorConfidence(self):
        """
        Update confidence in each sensor based on consistency with others
        """
        # Only update if we have at least 2 poses
        if self.pose_index < 2:
            return

        # Get the last two optimized poses
        curr_pose = self.result.atPose2(X(self.pose_index))
        prev_pose = self.result.atPose2(X(self.pose_index - 1))

        # Calculate observed motion
        dx = curr_pose.x() - prev_pose.x()
        dy = curr_pose.y() - prev_pose.y()
        delta_yaw = curr_pose.theta() - prev_pose.theta()

        # Normalize delta_yaw to [-π, π]
        delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi

        # Compare with encoder estimate
        encoder_distance = (self.ticks - self.last_ticks) * 0.01 * self.direction
        actual_distance = math.sqrt(dx*dx + dy*dy)
        encoder_error = abs(encoder_distance - actual_distance)

        # Compare with IMU estimate
        imu_yaw_error = abs(delta_yaw - math.radians(self.yaw - self.yaw))

        # Adjust confidences based on errors
        self.encoder_confidence = max(0.2, min(0.9, self.encoder_confidence * (1.0 - min(1.0, encoder_error))))
        self.imu_confidence = max(0.2, min(0.9, self.imu_confidence * (1.0 - min(1.0, imu_yaw_error))))

        # Camera confidence is already updated in updateCameraPosition

    def getPlotImage(self):
        """
        Generate a visualization of the robot's path
        """
        # Create figure and axis
        fig = Figure(figsize=(8, 6), dpi=100)
        canvas = FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)

        # Plot optimized trajectory
        ax.plot(self.xHistory, self.yHistory, 'b-', linewidth=2)

        # Mark the current position
        ax.plot(self.xCurrent, self.yCurrent, 'ro', markersize=10)

        # Add start point
        if len(self.xHistory) > 0:
            ax.plot(self.xHistory[0], self.yHistory[0], 'go', markersize=8)

        # Visualize uncertainty if we have enough poses
        if self.pose_index >= 2:
            try:
                marginals = gtsam.Marginals(self.graph, self.result)
                for i in range(0, self.pose_index+1, max(1, self.pose_index//10)):  # Plot some of the poses
                    gtsam_plot.plot_pose2(ax, self.result.atPose2(X(i)), 0.2)
                    gtsam_plot.plot_covariance_ellipse(ax, self.result.atPose2(X(i)), marginals.marginalCovariance(X(i)))
            except:
                # If we can't compute marginals, just continue without them
                pass

        # Set labels and title
        ax.set_xlabel('X position (m)')
        ax.set_ylabel('Y position (m)')
        ax.set_title('Robot Trajectory (Factor Graph SLAM)')
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

        # Draw the plot to the canvas
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

        # Fuse sensor data using factor graph
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