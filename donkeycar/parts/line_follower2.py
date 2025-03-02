import cv2
import numpy as np
import os
from simple_pid import PID
import logging
import matplotlib.pyplot as plt
import math

logger = logging.getLogger(__name__)

def compute_average_hsl_rgb(image, contour):
    # Create a mask for the contour
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)

    # Extract the pixels inside the contour
    masked_image = cv2.bitwise_and(image, image, mask=mask)

    # Convert the image to HLS color space
    hls_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HLS)

    # Compute the average H, S, L values
    h, l, s = cv2.split(hls_image)
    h_mean = cv2.mean(h, mask=mask)[0]
    s_mean = cv2.mean(s, mask=mask)[0]
    l_mean = cv2.mean(l, mask=mask)[0]

    # Compute the average R, G, B values
    b, g, r = cv2.split(masked_image)
    r_mean = cv2.mean(r, mask=mask)[0]
    g_mean = cv2.mean(g, mask=mask)[0]
    b_mean = cv2.mean(b, mask=mask)[0]

    print( f"Average HSL: ({h_mean:.2f}, {s_mean:.2f}, {l_mean:.2f})")
    print( f"Average RGB: ({r_mean:.2f}, {g_mean:.2f}, {b_mean:.2f})")
    return (h_mean, s_mean, l_mean), (r_mean, g_mean, b_mean)

class LineFollower:
    '''
    OpenCV based controller
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
        logger.info("init LineFollower 2 for real")
        self.output_dir = '/home/mdyesley/myPIDcar/data/images/'
        os.makedirs(self.output_dir, exist_ok=True)

    def get_i2_color(self, cam_img):
        success = cv2.imwrite(f'{self.output_dir}cropped{self.imageNumber}.png', cam_img)
        if not success:
            logger.error(f"Failed to write image {self.output_dir}cropped{self.imageNumber}.png")
        else:
            logger.info(f"Saved image {self.output_dir}cropped{self.imageNumber}.png")
        logger.info('get_i2_color')
        try:

            self.imageNumber = self.imageNumber + 1
            '''
            if self.imageCount%5 == 0:

                self.imageNumber += 1
                self.imageCount = 1
            self.imageCount += 1
            '''
            # Define the HSV range for green color
            lower_green = np.array([52,63,54])  # Adjust these values based on the image
            upper_green = np.array([87, 160, 98])

            height, width, _ = cam_img.shape
            # Calculate crop coordinates
            start_y = int(height * 0.40)  # Starting y-coordinate (top 25% removed)
            # Set the top part of the image to black
            cam_img[:start_y, :] = 0
            output_image = cam_img.copy()
            # Crop the image

            # Convert the image from BGR to HSV
            cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB) #weird linux thing
            hsv_image = cv2.cvtColor(cam_img, cv2.COLOR_BGR2HSV)
            # Create a mask that identifies green pixels
            mask = cv2.inRange(hsv_image, lower_green, upper_green)
            return 0, 0, mask

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            print(f'found {len(contours)} contours')


            if contours:
                # Find the largest contour (assumed to be the green line)
                largest_contour = max(contours, key=cv2.contourArea)
                # Compute the average color in the largest contour
                compute_average_hsl_rgb(cam_img, largest_contour)


                # Draw the contour on the output image
                cv2.drawContours(output_image, [largest_contour], -1, (0, 255, 0), 2)
                #if self.imageCount % 5 == 0:
                #    cv2.imwrite(f'data/images/contour{self.imageNumber}.png', output_image)

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



    # ---------------------
    # Function that gets da features of a image and put it on image
    def getFeatures(self, cam_img):
        # Convert to grayscale
        gray_image = cv2.cvtColor(cam_img, cv2.COLOR_BGR2GRAY)
        # Create ORB detector
        orb = cv2.ORB_create()

        # Detect keypoints
        keypoints = orb.detect(gray_image, None)

        # Draw keypoints on the image
        output_image = cv2.drawKeypoints(cam_img, keypoints, None, color=(0, 255, 0), flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)

        return output_image

    def match_and_visualize(self, prev_img, curr_img, distance_threshold=50):
        if prev_img is not None:
            # Convert images to grayscale
            prev_gray = cv2.cvtColor(prev_img, cv2.COLOR_BGR2GRAY)
            curr_gray = cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY)

            # Create ORB detector
            orb = cv2.ORB_create()

            # Detect and compute ORB features
            kp1, des1 = orb.detectAndCompute(prev_gray, None)
            kp2, des2 = orb.detectAndCompute(curr_gray, None)

            # Check if descriptors are valid
            if des1 is None or des2 is None:
                print("Warning: Descriptors could not be computed for one of the images.")
                return curr_img  # Return the current image unmodified

            # Use BFMatcher to match descriptors
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1, des2)

            # Sort matches by distance (lower distance is better)
            matches = sorted(matches, key=lambda x: x.distance)

            # Create a copy of the current image to draw on
            overlay_img = curr_img.copy()

            # Draw green dots for keypoints from the previous image
            for match in matches[:50]:  # Use top 50 matches
                pt1 = tuple(map(int, kp1[match.queryIdx].pt))  # Keypoint from prev_img
                cv2.circle(overlay_img, pt1, 5, (0, 255, 0), -1)  # Green dot (filled circle)

            # Draw red lines for close keypoints
            for match in matches[:50]:  # Use top 50 matches
                pt1 = tuple(map(int, kp1[match.queryIdx].pt))  # Keypoint from prev_img
                pt2 = tuple(map(int, kp2[match.trainIdx].pt))  # Keypoint from curr_img

                # Calculate the Euclidean distance between points
                distance = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)

                # If the distance is below the threshold, draw a red line
                if distance <= distance_threshold:
                    cv2.line(overlay_img, pt1, pt2, (0, 0, 255), 2)  # Red line

            return overlay_img
        else:
            print("No previous image to match with.")
            return curr_img

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
        logger.info(f'cam_img size = {cam_img.shape}')
        #MarkedImage = self.getFeatures(cam_img=cam_img)
        #MarkedImage = self.match_and_visualize(curr_img=cam_img, prev_img=self.lastImage)
        try :
            max_yellow, confidence, mask = self.get_i2_color(cam_img)
        except :
            logger.info('strange return from get_i2_color')
            max_yellow = 0
            confidence = 0
            mask = cam_img

        conf_thresh = 0.001

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
        #if self.overlay_image:
        #    cam_img = self.overlay_display(cam_img, mask, max_yellow, confidence)
        #   #cam_img = NewMask

        self.lastImage = cam_img

        #return self.steering, self.throttle, MarkedImage

        return self.steering, self.throttle, mask

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

