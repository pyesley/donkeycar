import cv2
import numpy as np
from simple_pid import PID
import logging
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from mapper3 import Mapper

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
        self.FramestoDisk = False
        self.mapper = Mapper()

        self.pid_st = pid

    def create_green_mask(self, img):
        """
        Creates a mask where white is when a pixel is more green than red and blue,
        then identifies the contour that contains the most green pixels.

        :param img: The input image in BGR format.
        :return: The contour with the most green pixels.
        """
        # Split the image into its B, G, R components
        B, G, R = cv2.split(img)

        # Create a mask where the green component is greater than the red and blue components
        mask = np.where((G > R) & (G > B), 255, 0).astype(np.uint8)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return np.array([])

        # For each contour, calculate a "greenness score"
        best_contour = None
        best_score = -1

        for contour in contours:
            # Create a mask for just this contour
            contour_mask = np.zeros_like(mask)
            cv2.drawContours(contour_mask, [contour], 0, 255, -1)

            # Get the RGB values within this contour
            green_values = G[contour_mask == 255]
            red_values = R[contour_mask == 255]
            blue_values = B[contour_mask == 255]

            if len(green_values) == 0:
                continue

            # Calculate greenness score: how much greener than other channels
            greenness = np.mean(green_values - (red_values + blue_values) / 2)

            # Weight by area to favor larger contours with similar greenness
            score = greenness * len(green_values)

            if score > best_score:
                best_score = score
                best_contour = contour

        return best_contour if best_contour is not None else np.array([])

    def mask_trapezoid(self, img):
        """
        Masks the area outside of a trapezoid to black. The trapezoid has one edge at the bottom of the image
        and the top edge is a line in the middle of the image with the width of half the image.

        :param img: The input image.
        :return: The masked image.
        """
        height, width = img.shape[:2]

        # Define the trapezoid vertices relative to the image dimensions
        top_width = width // 2
        top_left = (width // 4, height // 2)
        top_right = (3 * width // 4, height // 2)
        bottom_left = (0, height - 10)
        bottom_right = (width, height - 10)

        # Create a mask with the same dimensions as the image
        mask = np.zeros_like(img)

        # Define the polygon (trapezoid) points
        polygon = np.array([top_left, top_right, bottom_right, bottom_left], np.int32)
        polygon = polygon.reshape((-1, 1, 2))

        # Fill the polygon with white color on the mask
        cv2.fillPoly(mask, [polygon], (255, 255, 255))

        # Apply the mask to the image
        masked_img = cv2.bitwise_and(img, mask)

        return masked_img


    def get_x_location(self, cam_img):
        '''
            After doing a complete camara calibration with the intrinsics and extrinsics parameters,
            I decided to do something very simple.  Apply a trapazonidal mask to the image and then
            pick the area that is more green than any other color.  This looks to work very well.
        '''


        if self.imageCount % 500 == 0:
            if self.FramestoDisk :
                cv2.imwrite(f'data/images/cropped{self.imageNumber}.png', cam_img)
                self.imageNumber += 1
            self.imageCount = 1
        self.imageCount += 1

        try :
            output_image = cam_img.copy()
            # Convert the image from BGR to HSV
            cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)  # weird linux thing

            masked = self.mask_trapezoid(cam_img)
            green_contour = self.create_green_mask(masked)

            # Draw the largest contour on the original image
            if len( green_contour )  > 10 :

                cv2.drawContours(cam_img, [green_contour], -1, (0, 255, 0), 2)  # Green color, thickness 2

                # Extract x and y coordinates from the contour
                contour_points = green_contour[:, 0, :]
                x = contour_points[:, 0]
                y = contour_points[:, 1]
                ymean = np.mean(y)

                # Filter points with y-values within ±4 of ymean
                mask = (y >= ymean - 4) & (y <= ymean + 4)
                x_filtered = x[mask]

                if x_filtered.size > 0:
                    xmean = np.mean(x_filtered)
                    cv2.circle(output_image, (int(xmean + .5), int(ymean + .5) ), radius=10, color=(255, 255, 255),
                               thickness=-1)  # White filled circle
                    print('drew circle at ' + str(xmean) + ',' + str(ymean))

                    confidence = 0
                    if green_contour.size > 40:
                        confidence = 1
                    return xmean, confidence, output_image
            else :
                return 0,0,cam_img
        except Exception as e:
            logger.error(f"Error in image processing: {str(e)}")
            # Optional: add traceback for debugging
            return 0, 0, cam_img


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

        try :
            max_yellow, confidence, mask = self.get_x_location(cam_img)
        except :
            print('strange return from get_x_location')
            max_yellow = 0
            confidence = 0
            mask = cam_img

        conf_thresh = 0.001

        combined_output, visual_img, plot_image = self.mapper.run( cam_img )

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