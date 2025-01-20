import cv2
import numpy as np
from simple_pid import PID
import logging
import time
import csv
import os

logger = logging.getLogger(__name__)

class LineFollower:
    '''
    OpenCV based controller
    This controller takes a slice of the image, does color detection (green/yellow),
    and uses that to guide a PID controller. Additionally, it uses feature matching
    (ORB + essential matrix) to estimate a rough trajectory (visual odometry). That
    trajectory is stored internally and can also be saved to a CSV file and returned
    from run().
    '''

    def __init__(self, pid, cfg):
        # -- DonkeyCar related config --
        self.overlay_image = cfg.OVERLAY_IMAGE
        self.scan_y = cfg.SCAN_Y   # num pixels from the top to start horiz scan
        self.scan_height = cfg.SCAN_HEIGHT  # num pixels high to grab from horiz scan
        self.color_thr_low = np.asarray(cfg.COLOR_THRESHOLD_LOW)  # hsv dark color
        self.color_thr_hi = np.asarray(cfg.COLOR_THRESHOLD_HIGH)  # hsv light color
        self.target_pixel = cfg.TARGET_PIXEL  # ideal horizontal position of the line
        self.target_threshold = cfg.TARGET_THRESHOLD
        self.confidence_threshold = cfg.CONFIDENCE_THRESHOLD
        self.steering = 0.0   # from -1 to 1
        self.throttle = cfg.THROTTLE_INITIAL   # from -1 to 1
        self.delta_th = cfg.THROTTLE_STEP
        self.throttle_max = cfg.THROTTLE_MAX
        self.throttle_min = cfg.THROTTLE_MIN
        self.imageNumber = 0
        self.imageCount = 1

        self.pid_st = pid

        # ---------------------
        # Motion/trajectory-related
        # Camera Intrinsic Parameters (example placeholders)
        self.fx = 700.32833455
        self.fy = 703.47654591
        self.cx = 633.7861054
        self.cy = 342.84793261
        self.K = np.array([[self.fx, 0,       self.cx],
                           [0,       self.fy, self.cy],
                           [0,       0,       1      ]], dtype=np.float64)
        self.dist_coeffs = np.array([-0.30944433, 0.1258339, -0.00045581, -0.00057164, -0.01383379])

        # ORB feature detector and BFMatcher
        self.orb = cv2.ORB_create(2000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Keep track of the previous frame in grayscale
        self.prev_frame_gray = None

        # Keep track of the robot's position & orientation
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_rotation = np.eye(3)

        # Store the trajectory as a list of (x, y) positions
        self.trajectory = []

        # Optional: specify a CSV file for saving the trajectory
        self.trajectory_csv = getattr(cfg, 'TRAJECTORY_CSV', 'trajectory_data.csv')
        # If you'd like to re-create or append to the file each run, handle that here:
        if not os.path.isfile(self.trajectory_csv):
            with open(self.trajectory_csv, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time', 'x', 'y'])  # header

    def get_i2_color(self, cam_img):
        '''
        Example: find green pixels in the image,
        and return (xmean, confidence, mask) for the PID logic.
        '''
        try:
            if self.imageCount % 500 == 0:
                cv2.imwrite(f'data/images/cropped{self.imageNumber}.png', cam_img)
                self.imageNumber += 1
                self.imageCount = 1
            self.imageCount += 1

            # Define the HSV range for green color
            lower_green = np.array([45, 193, 143])  # Adjust these as needed
            upper_green = np.array([93, 255, 255])

            height, width, _ = cam_img.shape
            # Crop out top/bottom regions for clarity
            start_y = int(height * 0.25)
            end_y = int(height * 0.65)
            cam_img[:start_y, :] = 0
            cam_img[end_y:, :] = 0

            # Convert the image from BGR to HSV (with possible BGR->RGB->HSV fix)
            cam_img_rgb = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
            hsv_image = cv2.cvtColor(cam_img_rgb, cv2.COLOR_BGR2HSV)

            # Create a mask that identifies the green pixels
            mask = cv2.inRange(hsv_image, lower_green, upper_green)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour (assumed to be the line)
                largest_contour = max(contours, key=cv2.contourArea)

                # We'll compute xmean for the horizontal line
                contour_points = largest_contour[:, 0, :]
                x_vals = contour_points[:, 0]
                y_vals = contour_points[:, 1]
                ymean = np.mean(y_vals)

                # Filter points near ymean to reduce noise
                mask_pts = (y_vals >= ymean - 4) & (y_vals <= ymean + 4)
                x_filtered = x_vals[mask_pts]

                if x_filtered.size > 0:
                    xmean = np.mean(x_filtered)
                    # simple confidence
                    confidence = 1.0 if largest_contour.size > 50 else 0.5
                    return xmean, confidence, mask
                else:
                    return 0, 0, mask
            else:
                return 0, 0, mask
        except:
            print('failed on get_i2_color')
            # fallback
            fallback_mask = np.zeros_like(cam_img, dtype=np.uint8)
            return 0, 0, fallback_mask

    def match_features(self, img1_gray, img2_gray):
        """
        Match ORB features between two grayscale images.
        Returns pts1, pts2, keypoints1, keypoints2.
        """
        kp1, des1 = self.orb.detectAndCompute(img1_gray, None)
        kp2, des2 = self.orb.detectAndCompute(img2_gray, None)

        if des1 is None or des2 is None:
            return None, None, None, None

        matches = self.bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        if len(matches) < 5:
            return None, None, None, None

        pts1 = np.array([kp1[m.queryIdx].pt for m in matches], dtype=np.float64)
        pts2 = np.array([kp2[m.trainIdx].pt for m in matches], dtype=np.float64)

        return pts1, pts2, kp1, kp2

    def update_trajectory(self, cam_img):
        """
        Takes the current camera frame (BGR) and does feature matching
        with the previous frame. Recovers the pose, updates self.current_position,
        self.current_rotation, and self.trajectory. Returns a newMask with the
        path drawn in red.
        """
        current_frame_gray = cv2.cvtColor(cam_img, cv2.COLOR_BGR2GRAY)

        if self.prev_frame_gray is None:
            # No previous frame to compare to
            self.prev_frame_gray = current_frame_gray
            return cam_img.copy()  # Just return a copy for now

        # Match features
        pts_prev, pts_curr, kp1, kp2 = self.match_features(self.prev_frame_gray, current_frame_gray)
        if pts_prev is None:
            # Not enough features to do anything
            newMask = cam_img.copy()
            self.prev_frame_gray = current_frame_gray
            return newMask

        # Compute Essential matrix
        E, inliers = cv2.findEssentialMat(pts_curr, pts_prev, self.K,
                                          method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            # Could not find a valid E
            newMask = cam_img.copy()
            self.prev_frame_gray = current_frame_gray
            return newMask

        # Recover pose
        _, R, t, mask_pose = cv2.recoverPose(E, pts_curr, pts_prev, self.K)

        # Update the global rotation and translation
        scale = 1.0  # unknown scale
        self.current_position = self.current_position + self.current_rotation.dot(t.flatten()) * scale
        self.current_rotation = R.dot(self.current_rotation)

        # Add (x, y) to trajectory
        self.trajectory.append((self.current_position[0], self.current_position[1]))

        # Prepare the newMask with trajectory
        newMask = cam_img.copy()

        # Simple scaling/offset for drawing
        scale_factor = 10
        offset_x = 200
        offset_y = 200

        traj_points = []
        for (x, y) in self.trajectory:
            draw_x = int(x * scale_factor + offset_x)
            draw_y = int(y * scale_factor + offset_y)
            traj_points.append((draw_x, draw_y))

        # Draw lines
        for i in range(1, len(traj_points)):
            cv2.line(newMask, traj_points[i - 1], traj_points[i], (0, 0, 255), 2)

        # Update prev frame
        self.prev_frame_gray = current_frame_gray

        return newMask

    def save_trajectory_to_csv(self):
        """
        Append the latest trajectory point (if any) to a CSV file.
        This helps keep a persistent record that can be plotted or analyzed later.
        """
        if len(self.trajectory) < 1:
            return
        # Get the most recent point
        x, y = self.trajectory[-1]
        now = time.time()

        # Append to CSV
        with open(self.trajectory_csv, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([now, x, y])

    def run(self, cam_img):
        '''
        main runloop of the CV controller
        input: cam_img (BGR numpy array)
        output: (steering, throttle, newMask, self.trajectory)
                Note: The extra return (self.trajectory) is optional
                      and might need custom handling in your pipeline.
        '''

        if cam_img is None:
            return 0, 0, None, None

        # 1) Get color info (or green line) and a mask for PID
        try:
            max_yellow, confidence, pid_mask = self.get_i2_color(cam_img)
        except:
            print('strange return from get_i2_color')
            max_yellow = 0
            confidence = 0
            pid_mask = np.zeros_like(cam_img, dtype=np.uint8)

        # 2) Normal PID logic using pid_mask (unmodified)
        if self.target_pixel is None:
            self.target_pixel = max_yellow
            logger.info(f"Automatically chosen line position = {self.target_pixel}")

        # Update setpoint
        if self.pid_st.setpoint != self.target_pixel:
            self.pid_st.setpoint = self.target_pixel

        # Confidence check
        if confidence >= self.confidence_threshold:
            self.steering = self.pid_st(max_yellow)

            if abs(max_yellow - self.target_pixel) > self.target_threshold:
                # turning => slow down
                if self.throttle > self.throttle_min:
                    self.throttle -= self.delta_th
                if self.throttle < self.throttle_min:
                    self.throttle = self.throttle_min
            else:
                # straight => speed up
                if self.throttle < self.throttle_max:
                    self.throttle += self.delta_th
                if self.throttle > self.throttle_max:
                    self.throttle = self.throttle_max
        else:
            logger.info(f"No line detected: confidence {confidence} < {self.confidence_threshold}")

        # 3) Update trajectory & get newMask
        newMask = self.update_trajectory(cam_img)

        # 3b) Save the new point from trajectory to CSV
        self.save_trajectory_to_csv()

        # 4) If we want to overlay debug text, do it on newMask
        if self.overlay_image:
            newMask = self.overlay_display(newMask, pid_mask, max_yellow, confidence)

        # 5) Return steering, throttle, newMask, and entire trajectory
        return self.steering, self.throttle, newMask, list(self.trajectory)

    def overlay_display(self, cam_img, pid_mask, max_yellow, confidence):
        '''
        Overlay text or other debug info on top of cam_img.
        We do NOT modify pid_mask here (we only read from it if desired).
        '''
        display_str = []
        display_str.append(f"STEERING: {self.steering:.1f}")
        display_str.append(f"THROTTLE: {self.throttle:.2f}")
        display_str.append(f"I YELLOW: {max_yellow:.2f}")
        display_str.append(f"CONF: {confidence:.2f}")

        x, y = 10, 20
        for s in display_str:
            cv2.putText(cam_img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0), 2)
            y += 25

        return cam_img
