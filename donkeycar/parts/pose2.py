import cv2
import numpy as np
import time


class Pose:
    """
    A class that uses camera images (optical odometry),
    IMU data (accelerometer, gyro, magnetometer),
    and encoder data to estimate the robot pose (x, y, theta).
    """

    def __init__(self,
                 camera_intrinsics=np.array([
                     [700.32833455, 0.0, 633.7861054],
                     [0.0, 703.47654591, 342.84793261],
                     [0.0, 0.0, 1.0]
                 ], dtype=np.float64),
                 camera_distortion = np.array([-0.30944433,  0.1258339, -0.00045581, -0.00057164, -0.01383379], dtype=np.float64),
                 optical_flow_params={
                     'winSize': (21, 21),  # Window size for flow computation
                     'maxLevel': 3,  # Number of pyramid levels
                     'criteria': (
                             cv2.TERM_CRITERIA_EPS |
                             cv2.TERM_CRITERIA_COUNT,
                             30,
                             0.01
                     )  # Termination criteria: either epsilon or iteration count
                 }
                 ):
        """
        Initialize the Pose object with optional camera calibration data,
        optical flow parameters, and placeholders for storing state.

        :param camera_intrinsics: np.array or similar, camera intrinsic matrix.
        :param camera_distortion: np.array or similar, camera distortion coeffs.
        :param optical_flow_params: dict of params for cv2.calcOpticalFlowPyrLK or similar.
        """

        # Store camera calibration parameters, if needed
        self.camera_intrinsics = camera_intrinsics
        self.camera_distortion = camera_distortion
        self.lastImage = None

        # Set default optical flow parameters if not provided
        # (Below is just an example set of params)
        if optical_flow_params is None:
            self.optical_flow_params = dict(winSize=(21, 21),
                                            maxLevel=3,
                                            criteria=(
                                                cv2.TERM_CRITERIA_EPS |
                                                cv2.TERM_CRITERIA_COUNT,
                                                30,
                                                0.01))
        else:
            self.optical_flow_params = optical_flow_params

        # Robot pose state
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.theta = 0.0  # radians

        # For storing the previous camera frame (grayscale)
        self.prev_gray = None

        # For IMU bias and calibration (example placeholders)
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.mag_bias = np.array([0.0, 0.0, 0.0])

        # Timestamp of the last update (for integration)
        self.last_time = None

    def calibrate_imu(self, imu_readings):
        """
        Example calibration function for IMU. In practice, you'd gather a series of
        readings from the accelerometer, gyroscope, and magnetometer while the robot
        is stationary to estimate biases.

        :param imu_readings: list of readings, each reading containing
                             {accel: [x,y,z], gyro: [x,y,z], mag: [x,y,z]}
        """
        # Here we just average the readings as a naive approach
        accel_data = []
        gyro_data = []
        mag_data = []

        for reading in imu_readings:
            accel_data.append(reading['accel'])
            gyro_data.append(reading['gyro'])
            mag_data.append(reading['mag'])

        self.accel_bias = np.mean(accel_data, axis=0)
        self.gyro_bias = np.mean(gyro_data, axis=0)
        self.mag_bias = np.mean(mag_data, axis=0)

        print(f"Calibrated IMU biases:\n"
              f"    accel_bias = {self.accel_bias}\n"
              f"    gyro_bias  = {self.gyro_bias}\n"
              f"    mag_bias   = {self.mag_bias}")

    def update(self,
               image,
               imu_data,
               encoder_distance):
        """
        Update the robot's pose estimate from the latest sensor readings:
            1. Optical odometry from camera
            2. Orientation from IMU (gyro, magnetometer)
            3. Distance traveled from shaft encoder

        :param image: current camera frame (e.g., BGR or grayscale)
        :param imu_data: dictionary with keys: 'accel', 'gyro', 'mag',
                         each value is a 3-element list or np.array
        :param encoder_distance: float distance traveled since last update (meters)
        :return: (x, y, theta) - updated pose estimate
        """
        # Convert image to grayscale for optical flow
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) \
            if len(image.shape) == 3 else image

        # If we don't have a previous frame, just store this one
        if self.prev_gray is None:
            self.prev_gray = gray
            self.last_time = time.time()
            return (self.x, self.y, self.theta)

        # 1. Optical Flow / Visual Odometry
        #    - detect features in prev_gray
        #    - track them in current gray image
        #    - estimate translation + rotation in image space
        #    - convert image-space movement to real-world displacement
        #
        #    This is a *very* simplified demonstration using cv2.goodFeaturesToTrack
        #    and cv2.calcOpticalFlowPyrLK. In practice, you might use more robust
        #    approaches (ORB, SIFT, or specialized VO pipelines).

        # Detect corners in the previous frame
        prev_pts = cv2.goodFeaturesToTrack(self.prev_gray,
                                           maxCorners=100,
                                           qualityLevel=0.3,
                                           minDistance=7,
                                           blockSize=7)

        # If no corners found, skip optical flow step
        if prev_pts is not None:
            # Compute optical flow
            curr_pts, status, err = cv2.calcOpticalFlowPyrLK(
                self.prev_gray,
                gray,
                prev_pts,
                None,
                **self.optical_flow_params
            )

            # Filter valid points
            valid_prev_pts = prev_pts[status == 1]
            valid_curr_pts = curr_pts[status == 1]

            # Estimate an average translation in the image
            # This is a naive approach that just averages the pixel displacements
            if len(valid_prev_pts) > 0:
                flow_vectors = valid_curr_pts - valid_prev_pts
                avg_flow = np.mean(flow_vectors, axis=0)

                # Convert from pixels to meters (naive approach; real calibration required)
                # Suppose we have a "pixels_per_meter" ratio or use known camera parameters.
                # This is just a placeholder scale factor for demonstration.
                PIXELS_PER_METER = 500.0
                delta_x_cam = avg_flow[0] / PIXELS_PER_METER
                delta_y_cam = avg_flow[1] / PIXELS_PER_METER

                # We'll treat these as forward/backward and lateral motions in the robot frame
                # but in practice, you'd map from camera coordinates to the robot coordinate frame.
                # For instance, if the camera is forward-facing, the x-flow might correspond to
                # lateral motion, etc.

                # Just a very simplistic approach that "fuses" with the encoder distance:
                # We'll trust the direction from optical flow to refine heading,
                # but trust the magnitude from the encoder for total distance.
                # You can do more advanced weighting based on sensor covariances.

        # 2. IMU orientation update
        gyro = np.array(imu_data['gyro']) - self.gyro_bias
        accel = np.array(imu_data['accel']) - self.accel_bias
        mag = np.array(imu_data['mag']) - self.mag_bias

        # Integration over time for the gyro to get delta-theta
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time is not None else 0.0
        self.last_time = current_time

        # For a 2D plane, we might just use the z-axis of the gyro for yaw
        # (assuming x-forward, y-left, z-up coordinate system).
        # This is simplistic and doesn't handle pitch/roll properly.
        gyro_z = gyro[2]
        dtheta_gyro = gyro_z * dt
        self.theta += dtheta_gyro

        # For heading correction, one might rely on the magnetometer (if stable).
        # Let's say we try to compute heading from magnetometer:
        # heading_mag = np.arctan2(mag[1], mag[0])
        # Then we can fuse the gyro-based heading and magnetometer-based heading
        # with a complementary filter or a Kalman filter. Here's a naive complementary filter:

        # alpha is the filter gain (0 < alpha < 1).
        alpha = 0.98
        heading_mag = np.arctan2(mag[1], mag[0])
        # Unwrap the difference to avoid jumps from -pi to pi, etc.
        heading_diff = heading_mag - self.theta
        # Bring heading_diff into [-pi, pi]
        heading_diff = np.arctan2(np.sin(heading_diff), np.cos(heading_diff))
        self.theta = self.theta + (1 - alpha) * heading_diff

        # Normalize theta to [-pi, pi] if desired
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # 3. Use the encoder to get the distance traveled in the robot's forward direction
        #    We'll treat the encoder_distance as the "forward" distance in the direction of self.theta.
        dx_enc = encoder_distance * np.cos(self.theta)
        dy_enc = encoder_distance * np.sin(self.theta)

        # 4. Combine camera's horizontal displacement with encoder distance
        #    This can be done many ways; here's a naive approach:
        #
        #    - We trust the encoder for magnitude
        #    - We optionally add lateral corrections from camera flow
        #
        #    In reality, you want a more formal sensor-fusion approach. For demonstration:
        if prev_pts is not None and len(valid_prev_pts) > 0:
            # Suppose we interpret delta_x_cam, delta_y_cam as lateral and forward offsets
            # in the robot frame. We then rotate them into the global frame.
            # For demonstration, let's rotate them by self.theta:

            # Example: if delta_x_cam is lateral, delta_y_cam is forward
            # (This depends on the orientation of your camera)
            # We'll convert the camera displacement to global frame:
            #   X_global = cos(theta)*x_local - sin(theta)*y_local
            #   Y_global = sin(theta)*x_local + cos(theta)*y_local

            # These placeholders show how you might combine them:
            delta_global_x_cam = (np.cos(self.theta) * delta_x_cam -
                                  np.sin(self.theta) * delta_y_cam)
            delta_global_y_cam = (np.sin(self.theta) * delta_x_cam +
                                  np.cos(self.theta) * delta_y_cam)

            # Weighted fusion (naive):
            w_enc = 0.9
            w_cam = 0.1
            dx_combined = w_enc * dx_enc + w_cam * delta_global_x_cam
            dy_combined = w_enc * dy_enc + w_cam * delta_global_y_cam

        else:
            # Fallback: no camera data => rely purely on encoder
            dx_combined = dx_enc
            dy_combined = dy_enc

        # Update global (x, y)
        self.x += dx_combined
        self.y += dy_combined

        # Prepare for next iteration
        self.prev_gray = gray

        return (self.x, self.y, self.theta)
