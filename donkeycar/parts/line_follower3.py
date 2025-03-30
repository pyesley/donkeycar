import cv2
import numpy as np
from simple_pid import PID
import logging

import matplotlib.pyplot as plt
import math
import time
from donkeycar.parts.arduinoSerial import ARDUINONanoSerial,ArduinoEncoder, ArduinoIMU
from donkeycar.parts.pose2 import Pose

from donkeycar.parts.plot import PosePlotter

logger = logging.getLogger(__name__)


class LineFollower:
    '''
    OpenCV based controller (small change)
    This controller takes a horizontal slice of the image at a set Y coordinate.
    Then it converts to HSV and does a color thresh hold to find the yellow pixels.
    It does a histogram to find the pixel of maximum yellow. Then is uses that iPxel
    to guid a PID controller which seeks to maintain the max yellow at the same point
    in the image.
    '''
    def __init__(self, pid, cfg):
        self.overlay_image = cfg.OVERLAY_IMAGE
        self.scan_y = cfg.SCAN_Y   # num pixels from the top to start horiz scan
        self.scan_height = cfg.SCAN_HEIGHT  # num pixels high to grab from horiz scan
        self.color_thr_low = np.asarray(cfg.COLOR_THRESHOLD_LOW)  # hsv dark yellow
        self.color_thr_hi = np.asarray(cfg.COLOR_THRESHOLD_HIGH)  # hsv light yellow
        self.target_pixel = cfg.TARGET_PIXEL  # of the N slots above, which is the ideal relationship target
        self.target_threshold = cfg.TARGET_THRESHOLD # minimum distance from target_pixel before a steering change is made.
        self.confidence_threshold = cfg.CONFIDENCE_THRESHOLD  # percentage of yellow pixels that must be in target_pixel slice
        self.steering = 0.0 # from -1 to 1
        self.throttle = cfg.THROTTLE_INITIAL # from -1 to 1
        self.delta_th = cfg.THROTTLE_STEP  # how much to change throttle when off
        self.throttle_max = cfg.THROTTLE_MAX
        self.throttle_min = cfg.THROTTLE_MIN
        self.imageNumber = 0
        self.imageCount = 1

        self.lastImage = 0.0
        self.pid_st = pid
        self.lastImage = None

        # IMU integration
        self.imu = ArduinoIMU()  # Instantiate the ArduinoIMU class
        self.encoder = ArduinoEncoder()  # Instantiate the ArduinoIMU class
        self.last_imu_time = time.time()  # Track the last time IMU data was printed
        self.camera_intrinsics = np.array([
            [700.32833455, 0.0, 633.7861054],
            [0.0, 703.47654591, 342.84793261],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.camera_distortion = np.array([-0.30944433, 0.1258339, -0.00045581, -0.00057164, -0.01383379], dtype=np.float64)
        self.optical_flow_params = {
            'winSize': (21, 21),  # Window size for flow computation
            'maxLevel': 3,  # Number of pyramid levels
            'criteria': (
                cv2.TERM_CRITERIA_EPS |
                cv2.TERM_CRITERIA_COUNT,
                30,
                0.01
            )  # Termination criteria: either epsilon or iteration count
        }



        # Pose integration
        #self.Pose = Pose(camera_intrinsics=self.camera_intrinsics,
        #           camera_distortion=self.camera_distortion,
        #           optical_flow_params=self.optical_flow_params)

        #self.plotter = PosePlotter(width=3, height=3, dpi=100)


    def get_i2_color(self, cam_img):

        try:
            if self.imageCount%500 == 0:
                cv2.imwrite(f'data/images/cropped{self.imageNumber}.png', cam_img)
                self.imageNumber += 1
                self.imageCount = 1
            self.imageCount += 1

            # Define the HSV range for green color
            lower_green = np.array([45, 193, 143])  # Adjust these values based on the image
            upper_green = np.array([93, 255, 255])

            height, width, _ = cam_img.shape
            # Calculate crop coordinates
            start_y = int(height * 0.25)  # Starting y-coordinate (top 25% removed)
            end_y = int(height * 0.65)  # Ending y-coordinate (bottom 40% removed)
            # Set the top part of the image to black
            cam_img[:start_y, :] = 0
            # Set the bottom part of the image to black
            cam_img[end_y:, :] = 0
            output_image = cam_img.copy()
            # Crop the image

            #cropped = cam_img[start_y:end_y, :]

            # Convert the image from BGR to HSV
            cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB) #weird linux thing
            hsv_image = cv2.cvtColor(cam_img, cv2.COLOR_BGR2HSV)
            # Create a mask that identifies green pixels
            mask = cv2.inRange(hsv_image, lower_green, upper_green)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


            if contours:
                # Find the largest contour (assumed to be the green line)
                largest_contour = max(contours, key=cv2.contourArea)

                # Draw the contour on the output image
                cv2.drawContours(output_image, [largest_contour], -1, (0, 255, 0), 2)


                # Extract x and y coordinates from the contour
                contour_points = largest_contour[:, 0, :]
                x = contour_points[:, 0]
                y = contour_points[:, 1]
                ymean = np.mean(y)

                # Filter points with y-values within ±4 of ymean
                mask = (y >= ymean - 4) & (y <= ymean + 4)
                x_filtered = x[mask]

                if x_filtered.size > 0:
                    xmean = np.mean(x_filtered)
                    cv2.circle(output_image, (int(xmean + .5), int(ymean + .5) ), radius=10, color=(255, 255, 255),
                               thickness=-1)  # Green filled circle
                    print('drew circle at ' + str(xmean) + ',' + str(ymean))

                    confidence = 0
                    if largest_contour.size > 50:
                        confidence = 1
                    return xmean, confidence, output_image
            else :
                return 0,0,cam_img
        except :
            print('failed on ')
            return 0,0,cam_img

    # ---------------------
    # Camera Intrinsic Parameters
    # Replace these with your camera's calibrated parameters.
    # fx, fy: focal lengths; cx, cy: principal point.
    fx = 700.32833455
    fy = 703.47654591
    cx = 633.7861054
    cy = 342.84793261

    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1]], dtype=np.float64)

    # Distortion coefficients (assume zero or insert your actual distortion from calibration)
    dist_coeffs = [-0.30944433,  0.1258339, -0.00045581, -0.00057164, -0.01383379]



    def get_i_color(self, cam_img):
        '''
        get the horizontal index of the color at the given slice of the image
        input: cam_image, an RGB numpy array
        output: index of max color, value of cumulative color at that index, and mask of pixels in range
        '''
        # take a horizontal slice of the image
        iSlice = self.scan_y
        scan_line = cam_img[iSlice : iSlice + self.scan_height, :, :]

        # convert to HSV color space
        img_hsv = cv2.cvtColor(scan_line, cv2.COLOR_RGB2HSV)

        # make a mask of the colors in our range we are looking for
        mask = cv2.inRange(img_hsv, self.color_thr_low, self.color_thr_hi)

        # which index of the range has the highest amount of yellow?
        hist = np.sum(mask, axis=0)
        max_yellow = np.argmax(hist)

        return max_yellow, hist[max_yellow], mask

    def overlay_map_in_corner(self, frame, map_img, map_scale=0.3):
        """
        Overlays map_img onto the bottom-right corner of 'frame'.
        :param frame: 640x480 main camera frame (BGR).
        :param map_img: the small map image to overlay (BGR).
        :param map_scale: fraction of frame size (0 < map_scale < 1).
        """
        h_frame, w_frame = frame.shape[:2]

        # Resize map
        new_w = int(w_frame * map_scale)
        new_h = int(h_frame * map_scale)
        map_small = cv2.resize(map_img, (new_w, new_h))

        # Bottom-right corner
        x_start = w_frame - new_w
        y_start = h_frame - new_h

        # Region of interest
        roi = frame[y_start:y_start + new_h, x_start:x_start + new_w]

        # Create mask from the map
        map_small_gray = cv2.cvtColor(map_small, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(map_small_gray, 1, 255, cv2.THRESH_BINARY)

        map_bg = cv2.bitwise_and(roi, roi, mask=cv2.bitwise_not(mask))
        map_fg = cv2.bitwise_and(map_small, map_small, mask=mask)
        combined = cv2.add(map_bg, map_fg)
        frame[y_start:y_start + new_h, x_start:x_start + new_w] = combined

        return frame


    def run(self, cam_img):
        '''
        main runloop of the CV controller
        input: cam_image, an RGB numpy array
        output: steering, throttle, and the image.
        If overlay_image is True, then the output image
        includes and overlay that shows how the 
        algorithm is working; otherwise the image
        is just passed-through untouched. 
        '''
        if cam_img is None:
            return 0, 0, False, None




        MarkedImage = self.match_and_visualize(curr_img=cam_img, prev_img=self.lastImage)
        try :
            max_yellow, confidence, mask = self.get_i2_color(cam_img)
        except :
            print('strange return from get_i2_color')

            max_yellow = 0
            confidence = 0
            mask = cam_img

        conf_thresh = 0.001

        '''
        imu = ArduinoIMU()
        data = imu.run()  # or imu.run_threaded() if you're doing multi-threaded ops
        encoder = ArduinoEncoder()
        distance, velocity = encoder.run(self.throttle)  # or imu.run_threaded() if you're doing multi-threaded ops

        x, y, theta = self.Pose.update(image=cam_img, imu_data=data, encoder_distance=distance)

        self.plotter.update(x, y, theta)

        map_img = self.plotter.get_plot_image()

        #output_img = self.overlay_map_in_corner(cam_img, map_img)
        '''


        if self.target_pixel is None:
            # Use the first run of get_i_color to set our relationship with the yellow line.
            # You could optionally init the target_pixel with the desired value.
            self.target_pixel = max_yellow
            logger.info(f"Automatically chosen line position = {self.target_pixel}")

        if self.pid_st.setpoint != self.target_pixel:
            # this is the target of our steering PID controller
            self.pid_st.setpoint = self.target_pixel

        if confidence >= self.confidence_threshold:
            # invoke the controller with the current yellow line position
            # get the new steering value as it chases the ideal
            self.steering = self.pid_st(max_yellow)

            # slow down linearly when away from ideal, and speed up when close
            if abs(max_yellow - self.target_pixel) > self.target_threshold:
                # we will be turning, so slow down
                if self.throttle > self.throttle_min:
                    self.throttle -= self.delta_th
                if self.throttle < self.throttle_min:
                    self.throttle = self.throttle_min
            else:
                # we are going straight, so speed up
                if self.throttle < self.throttle_max:
                    self.throttle += self.delta_th
                if self.throttle > self.throttle_max:
                    self.throttle = self.throttle_max
        else:
            logger.info(f"No line detected: confidence {confidence} < {self.confidence_threshold}")

        # show some diagnostics
        if self.overlay_image:
            cam_img = self.overlay_display(cam_img, mask, max_yellow, confidence)

        return self.steering, self.throttle, combined_output #mask

    def overlay_display(self, cam_img, mask, max_yellow, confidense):
        '''
        composite mask on top the original image.
        show some values we are using for control
        '''

        #mask_exp = np.stack((mask, ) * 3, axis=-1)
        #iSlice = self.scan_y
        #img = np.copy(cam_img)
        #img[iSlice : iSlice + self.scan_height, :, :] = mask_exp
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        display_str = []
        display_str.append("STEERING:{:.1f}".format(self.steering))
        display_str.append("THROTTLE:{:.2f}".format(self.throttle))
        display_str.append("I YELLOW:{:.2f}".format(max_yellow))
        display_str.append("CONF:{:.2f}".format(confidense))

        y = 10
        x = 10

        for s in display_str:
            cv2.putText(cam_img, s, color=(0, 0, 0), org=(x ,y), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.4)
            y += 10

        return cam_img

if __name__ == '__main__':
    # Load the input image
    cam_img = cv2.imread("C:\\Users\\pyesl\\Documents\\donkeyCar\\images\\cropped145.png")
    cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)

    if cam_img is None:
        print("Error: Could not load image.")
        exit(1)

    # Create an instance of the LineFollower class
    pid = PID(1.0, 0.1, 0.05)  # Example PID values, adjust as needed
    cfg = type('Config', (object,), {})()  # Create a simple config object
    cfg.OVERLAY_IMAGE = False
    cfg.SCAN_Y = 100
    cfg.SCAN_HEIGHT = 20
    cfg.COLOR_THRESHOLD_LOW = [20, 100, 100]
    cfg.COLOR_THRESHOLD_HIGH = [30, 255, 255]
    cfg.TARGET_PIXEL = None
    cfg.TARGET_THRESHOLD = 10
    cfg.CONFIDENCE_THRESHOLD = 0.5
    cfg.THROTTLE_INITIAL = 0.2
    cfg.THROTTLE_STEP = 0.05
    cfg.THROTTLE_MAX = 0.5
    cfg.THROTTLE_MIN = 0.1

    line_follower = LineFollower(pid, cfg)

    # Call the get_i2_color method
    xmean, confidence, output_image = line_follower.get_x_location(cam_img)

    # Display the input and output images
    #cv2.imshow('Input Image', cam_img)
    cv2.imshow('Output Image', output_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

