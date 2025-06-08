#!/usr/bin/env python3
import cv2
import numpy as np
import time
import logging
import sys
import os # Add os import if not already present
import cv2.xphoto


# --- Import PID controller ---
try:
    from simple_pid import PID
except ImportError:
    print("Error: 'simple-pid' library not found.")
    print("Please install it: pip install simple-pid")
    sys.exit(1)
# ---

# --- Configuration Constants for Line Following ---
PID_P = -0.01         # proportional mult for PID path follower
PID_I = 0.000         # integral mult for PID path follower
PID_D = -0.0001       # differential mult for PID path follower

PID_KP = -.01
PID_KI = 0.00
PID_KD = 0.01
PID_SAMPLE_TIME = 1.0 / 15.0
DEFAULT_MAX_ANGULAR_SPEED = .2

class LineFollowerController:
    """
    Processes camera images to detect a green line using BGR comparison,
    calculates the error from the center, and uses a PID controller
    to determine the required steering adjustment (angular velocity).
    """
    def __init__(self, node_logger, pid_output_limit=DEFAULT_MAX_ANGULAR_SPEED):
        self.logger = node_logger
        self.pid = PID(PID_KP, PID_KI, PID_KD, setpoint=0)
        self.pid.output_limits = (-abs(pid_output_limit), abs(pid_output_limit))
        #self.pid.sample_time = PID_SAMPLE_TIME

        # Image processing parameters
        self.min_contour_area = 20 # Minimum contour area considered valid
        self.wb = cv2.xphoto.createSimpleWB() #for automatic white balance

        self.last_control_time = time.time()
        self.image_center_x = None

        self.logger.info("LineFollowerController initialized (using BGR comparison).")
        self.logger.info(f"PID Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
        self.logger.info(f"PID Output Limits: +/- {abs(pid_output_limit):.2f} rad/s")

    # --- NEW: Green detection using BGR comparison ---
    def _find_best_green_contour(self, img, debug=False):
        """
        Creates a mask where white is when a pixel is more green than red and blue,
        then identifies the contour that contains the most green pixels based on a score.

        :param img: The input image in BGR format.
        :return: The contour with the best green score, or an empty numpy array if none found.
        """
        if img is None or img.size == 0:
            return np.array([])

        try:
            # Split the image into its B, G, R components
            B, G, R = cv2.split(img)

            # Create a mask where the green component is greater than the red and blue components
            # Ensure comparison handles potential uint8 overflow implicitly with numpy's logic
            mask = np.where((G > R) & (G > B), 255, 0).astype(np.uint8)
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                return np.array([])

            # For each contour, calculate a "greenness score"
            best_contour = None
            best_score = -1

            for contour in contours:
                # Create a mask for just this contour to sample pixels within it
                contour_mask = np.zeros_like(mask)
                cv2.drawContours(contour_mask, [contour], 0, 255, -1) # Fill the contour

                # Get the BGR values ONLY within this contour
                # Using contour_mask > 0 ensures we only select pixels inside
                green_values = G[contour_mask > 0]
                red_values = R[contour_mask > 0]
                blue_values = B[contour_mask > 0]

                num_pixels = len(green_values)
                if num_pixels == 0:
                    continue

                # Calculate greenness score: average difference between Green and avg(Red, Blue)
                # Use float32 for calculations to avoid overflow/rounding issues
                mean_g = np.mean(green_values.astype(np.float32))
                mean_r = np.mean(red_values.astype(np.float32))
                mean_b = np.mean(blue_values.astype(np.float32))

                greenness = mean_g - (mean_r + mean_b) / 2.0

                # Score: Weight greenness by the number of pixels (area)
                # This favors larger, genuinely green areas
                score = greenness * num_pixels

                if score > best_score:
                    best_score = score
                    best_contour = contour

            # Return the best contour found, or empty array if none qualified

            out =  best_contour if best_contour is not None else np.array([])
            if debug:
                return out, mask_bgr
            else  :
                return out

        except Exception as e:
             self.logger.error(f"Error in _find_best_green_contour: {e}", exc_info=True)
             return np.array([])


    def _mask_trapezoid(self, img):
        """
        Masks the area outside a region of interest (trapezoid).
        ADJUST THE VERTICES based on your camera setup.
        """
        height, width = img.shape[:2]
        if self.image_center_x is None:
             self.image_center_x = width // 2
             self.pid.setpoint = self.image_center_x
             self.logger.info(f"LineFollower: Image center set to {self.image_center_x}")

        # --- ADJUST THESE TRAPEZOID COORDINATES ---
        top_y = height // 2 + 40
        bottom_y = height - 20
        top_width_ratio = 0.6
        bottom_width_ratio = 1.0
        # ---

        top_left_x = int(width * (1 - top_width_ratio) / 2); top_right_x = int(width * (1 + top_width_ratio) / 2)
        bottom_left_x = int(width * (1 - bottom_width_ratio) / 2); bottom_right_x = int(width * (1 + bottom_width_ratio) / 2)

        mask = np.zeros_like(img[:, :, 0])
        polygon = np.array([[bottom_left_x, bottom_y],[top_left_x, top_y],[top_right_x, top_y],[bottom_right_x, bottom_y]], dtype=np.int32)
        cv2.fillPoly(mask, [polygon], 255)
        masked_img = cv2.bitwise_and(img, img, mask=mask)
        return masked_img, mask

    # --- MODIFIED: find_line_center now uses the BGR comparison method and returns the image with contour ---
    def find_line_center(self, cv_image):
        """
        Processes an image to find the horizontal center of the green line using BGR comparison.
        Draws the detected contour on the image for visualization.
        Returns: x_position (pixels), confidence (contour area), image_with_contour
                 Returns None, 0, original_image if line not found or error.
        """
        if cv_image is None:
            self.logger.warn("LineFollower: Received None image.")
            return None, 0, None  # Return None for image if input is None

        # Create a copy to draw on, preserving the original
        image_with_contour = cv_image.copy()

        try:
            # Apply white balance (optional, but good practice)
            balanced_image = self.wb.balanceWhite(cv_image)  # Use original cv_image for balancing

            # 1. Apply Trapezoidal Mask (Region of Interest) to the balanced image
            masked_img, roi_mask = self._mask_trapezoid(balanced_image)  # Pass balanced image here

            # 2. Find the best green contour within the masked image using BGR comparison
            best_contour = self._find_best_green_contour(masked_img, debug=False)  # Use the new method

            # 3. Check if a valid contour was returned
            if best_contour is None or best_contour.size == 0:
                # self.logger.debug("LineFollower: No valid green contour found.")
                # Return the original image if no contour is found
                return None, 0, image_with_contour

            # --- Draw the best contour found in green ON THE COPIED IMAGE ---
            cv2.drawContours(image_with_contour, [best_contour], -1, (0, 255, 0), 2)  # Green contour, thickness 2

            # 4. Calculate Area (Confidence)
            confidence = cv2.contourArea(best_contour)

            # --- Confidence Threshold ---
            if confidence < self.min_contour_area:
                # self.logger.debug(f"LineFollower: Contour area {confidence:.0f} < {self.min_contour_area}. Line lost.")
                # Return the image with the (small) contour drawn
                return None, confidence, image_with_contour

            # 5. Calculate Centroid (Moments)
            M = cv2.moments(best_contour)
            if M["m00"] == 0:
                self.logger.warn("LineFollower: Contour found but moment m00 is zero.")
                # Return the image with the contour drawn even if centroid calculation fails
                return None, confidence, image_with_contour

            center_x = int(M["m10"] / M["m00"])

            # --- Draw the centroid as a red dot ON THE COPIED IMAGE ---
            center_y = int(M["m01"] / M["m00"])  # Calculate Y for drawing
            cv2.circle(image_with_contour, (center_x, center_y), 5, (0, 0, 255), -1)  # Red dot at centroid

            # --- Draw the target center line (optional) ON THE COPIED IMAGE ---
            if self.image_center_x is not None:
                cv2.line(image_with_contour, (self.image_center_x, 0),
                         (self.image_center_x, image_with_contour.shape[0]), (255, 0, 0), 1)  # Blue line for target

            # self.logger.debug(f"LineFollower: Line found at x={center_x}, area={confidence:.0f}")
            # Return center_x, confidence, and the image *with the contour and centroid drawn*
            return center_x, confidence, image_with_contour

        except Exception as e:
            self.logger.error(f"LineFollower: Error in find_line_center: {e}", exc_info=True)
            # Return the original image in case of unexpected errors
            return None, 0, image_with_contour


    # --- CORRECTED compute_steering_angle using PID ---
    def compute_steering_angle(self, line_x_position, current_time):
        """Calculates steering angle using PID based on line position error."""
        if line_x_position is None or self.image_center_x is None:
            # No valid measurement or target, reset and return 0
            self.pid.reset()
            return 0.0

        # *** Pass the MEASUREMENT (line_x_position) to the PID controller ***
        # The PID controller will calculate error internally: setpoint - measurement
        # PID setpoint should be self.image_center_x (set in _mask_trapezoid)
        control_output = self.pid(line_x_position)

        # The output needs to be negated because PID calculates (setpoint - measurement)
        # We want steering proportional to (measurement - setpoint)
        # If measurement > setpoint (line on right), error is negative, PID output is positive.
        # We need negative angular_z to turn right (based on manual keys).
        # So, we negate the PID output.
        angular_z = -control_output  # <<< APPLY NEGATION HERE

        # Log the internal PID error and the final output
        # Access the proportional, integral, and derivative components if needed for logging
        # Note: simple_pid stores components based on the last call
        p, i, d = self.pid.components
        internal_error = self.pid.setpoint - line_x_position  # How simple_pid sees the error
        self.logger.info(f"PID Debug: Target={self.pid.setpoint}, Measured={line_x_position}, "
                         f"InternalError={internal_error:.2f}, P={p:.3f}, I={i:.3f}, D={d:.3f}, "
                         f"Final_angular_z={angular_z:.4f}")

        # *** Return the calculated angular_z directly (NO MULTIPLICATION) ***
        return angular_z


# <--- Your existing LineFollowerController code from line_follower_controller.py goes here --->
# Include all the code from the uploaded file line_follower_controller.py here.
# ... (imports, constants, LineFollowerController class definition) ...

# --- NEW Debugging Main Function ---


def main():
    """
    Main function to load images sequentially, process them using
    LineFollowerController methods, and display results for debugging.
    """
    # --- Configuration ---
    # IMPORTANT: Update this path to your image directory
    image_directory = "/home/pyesley/Pictures/images" # <<< CHANGE THIS PATH
    window_name = "Line Follower Debug"

    # --- Basic Logging Setup ---
    # Configure logging similar to how ROS might do it, or use basicConfig
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    node_logger = logging.getLogger("LineFollowerDebug")

    # --- Initialize Controller ---
    # Pass the logger instance to the controller
    controller = LineFollowerController(node_logger=node_logger) #

    # --- Image Navigation ---
    current_image_index = 0
    max_digits = 6 # For ######.jpg format

    while True:
        # --- Load Image ---
        image_filename = f"{current_image_index:0{max_digits}d}.jpg"
        image_path = os.path.join(image_directory, image_filename)

        if not os.path.exists(image_path):
            node_logger.warning(f"Image not found: {image_path}. Reached end or file missing.")
            # Option: Loop back to 0, or stay at the last valid index, or exit.
            # Staying at the last valid index for now. Decrement and try again if possible.
            if current_image_index > 0:
                 current_image_index -= 1 # Go back to the last valid one
                 image_filename = f"{current_image_index:0{max_digits}d}.jpg"
                 image_path = os.path.join(image_directory, image_filename)
                 if not os.path.exists(image_path): # If even that fails, stop
                     node_logger.error("Could not load previous image either. Exiting.")
                     break
            else: # If index was 0 and file not found
                node_logger.error("Image 000000.jpg not found. Exiting.")
                break

        cv_image = cv2.imread(image_path)
        if cv_image is None:
            node_logger.error(f"Failed to load image: {image_path}")
            break # Exit if loading fails
        wb = cv2.xphoto.createSimpleWB()
        cv_image = wb.balanceWhite(cv_image)

        # --- Process Image ---
        # 1. Mask Trapezoid
        masked_img, roi_mask = controller._mask_trapezoid(cv_image) #

        # 2. Find Best Green Contour
        best_contour, mask = controller._find_best_green_contour(masked_img,debug = True ) #

        # --- Visualization ---
        display_image = cv_image.copy()
        height, width = display_image.shape[:2]

        # Draw Trapezoid ROI for reference (recalculate points for visualization)
        top_y = height // 2 + 40
        bottom_y = height - 20
        top_width_ratio = 0.6
        bottom_width_ratio = 1.0
        top_left_x = int(width * (1 - top_width_ratio) / 2); top_right_x = int(width * (1 + top_width_ratio) / 2)
        bottom_left_x = int(width * (1 - bottom_width_ratio) / 2); bottom_right_x = int(width * (1 + bottom_width_ratio) / 2)
        polygon = np.array([[bottom_left_x, bottom_y],[top_left_x, top_y],[top_right_x, top_y],[bottom_right_x, bottom_y]], dtype=np.int32)
        cv2.polylines(display_image, [polygon], isClosed=True, color=(0, 0, 255), thickness=2) # Red trapezoid

        # Create an image to show the contour result
        contour_display = np.zeros_like(masked_img) # Black background
        if best_contour is not None and best_contour.size > 0: #
            # Draw the best contour found in green
            cv2.drawContours(contour_display, [best_contour], -1, (0, 255, 0), 2) # Green contour
            # Optional: Calculate and display centroid
            M = cv2.moments(best_contour) #
            if M["m00"] != 0: #
                center_x = int(M["m10"] / M["m00"]) #
                center_y = int(M["m01"] / M["m00"])
                cv2.circle(contour_display, (center_x, center_y), 5, (0, 0, 255), -1) # Red dot at centroid

        # Combine images for display (Original+ROI | Masked | Contour)
        # Ensure all images have the same number of channels (convert mask if needed)
        #masked_img_bgr = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY) # Convert masked if it's color
        #masked_img_bgr = cv2.cvtColor(masked_img_bgr, cv2.COLOR_GRAY2BGR) # Convert back to BGR

        combined_display = cv2.hconcat([display_image, mask, contour_display])

        # Add text overlay for filename
        cv2.putText(combined_display, f"Image: {image_filename}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Show the combined image
        cv2.imshow(window_name, combined_display)

        # --- Wait for Key Press ---
        key = cv2.waitKey(0) & 0xFF # Wait indefinitely for a key

        if key == ord('q'): # Quit
            node_logger.info("Quit key pressed. Exiting.")
            break
        elif key == 83: # Right Arrow Key (common keycode, may vary)
             current_image_index += 1
             node_logger.info(f"Right Arrow: Loading next image index {current_image_index}")
        elif key == 81: # Left Arrow Key (common keycode, may vary)
            if current_image_index > 0:
                current_image_index -= 1
                node_logger.info(f"Left Arrow: Loading previous image index {current_image_index}")
            else:
                node_logger.info("Left Arrow: Already at the first image.")
        else:
            # Optional: Log other key presses if needed for debugging
            # node_logger.debug(f"Key pressed: {key}")
            pass

    # --- Cleanup ---
    cv2.destroyAllWindows()
    node_logger.info("Application finished.")

if __name__ == "__main__":
    main()