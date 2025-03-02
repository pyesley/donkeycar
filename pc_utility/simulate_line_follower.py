import cv2
import numpy as np
from donkeycar.parts.line_follower2 import LineFollower
from simple_pid import PID


def display_image(cam_img):
    # Display the image
    cv2.imshow('Image', cam_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
def simulator(filename):
    # Load the input image
    cam_img = cv2.imread(filename)
    cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
    display_image(cam_img)
    cv2.imwrite("C:\\Users\\pyesl\\Documents\\donkeyCar\\images\\trueColor.png", cam_img)





    if cam_img is None:
        print("Error: Could not load image.")
        return

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
    xmean, confidence, output_image = line_follower.get_i2_color(cam_img)

    # Display the input and output images
    #cv2.imshow('Input Image', cam_img)
    cv2.imshow('Output Image', output_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example usage
IMAGE_PATH = "C:\\Users\\pyesl\\Documents\\donkeyCar\\images\\"
image = 'cropped145.png'
filename = IMAGE_PATH + image

simulator(filename)