from line_follower2 import LineFollower
from simple_pid import PID
import cv2

# Path to your images folder
IMAGE_PATH = "../images\\"
import os

def loadConfig():
    # Show the current directory path
    current_directory = os.getcwd()
    print("Current Directory:", current_directory)

    config_path = os.path.join(os.getcwd(), 'pid_car', 'config.py')
    print("Config Path:", config_path)

    class Config:
        pass
    cfg = Config()
    with open(config_path) as f:
        code = f.read()
    exec(code, {'__file__': config_path}, cfg.__dict__)
    return cfg

def main():

    cfg = loadConfig()
    pid = PID(cfg.PID_P, cfg.PID_I, cfg.PID_D, setpoint=cfg.TARGET_PIXEL)

    line_follower = LineFollower(pid, cfg)
    image_number = 47
    IMAGE_PATH = os.path.join(os.getcwd(), 'images')
    print("Config Path:", IMAGE_PATH)

    # Load the image
    while True:
        image_path = IMAGE_PATH + '\\cropped'+str(image_number) + ".png"
        print(image_path)
        image = cv2.imread(image_path)
        if image is None:
            print("Image not found!")
            break

        # Run the line follower algorithm
        steering, throttle, overlay_image = line_follower.run(image)

        # Display the image with overlay
        cv2.imshow("Line Follower", overlay_image)
        key = cv2.waitKey(100)  # Wait 100 ms between frames (adjust as needed)

        # Break the loop if 'Esc' is pressed
        key = cv2.waitKey(1)
        if key == 27:
            break

        image_number += 1

    # Clean up
    cv2.destroyAllWindows()

main()