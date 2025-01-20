import cv2
import numpy as np


def nothing(x):
    pass  # Placeholder function for trackbar callbacks


def hsv_tuner(image_path):
    # Load the input image
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Could not load image.")
        return

    # Resize image if it's too large (optional)
    scale_percent = 50  # Scale down by 50% for better visibility
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    image = cv2.resize(image, (width, height))

    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a window for the original and masked images
    cv2.namedWindow('Image Tuner', cv2.WINDOW_NORMAL)

    # Create trackbars for HSV tuning
    cv2.createTrackbar('Lower H', 'Image Tuner', 0, 179, nothing)
    cv2.createTrackbar('Lower S', 'Image Tuner', 0, 255, nothing)
    cv2.createTrackbar('Lower V', 'Image Tuner', 0, 255, nothing)
    cv2.createTrackbar('Upper H', 'Image Tuner', 179, 179, nothing)
    cv2.createTrackbar('Upper S', 'Image Tuner', 255, 255, nothing)
    cv2.createTrackbar('Upper V', 'Image Tuner', 255, 255, nothing)

    while True:
        # Get current positions of all trackbars
        lower_h = cv2.getTrackbarPos('Lower H', 'Image Tuner')
        lower_s = cv2.getTrackbarPos('Lower S', 'Image Tuner')
        lower_v = cv2.getTrackbarPos('Lower V', 'Image Tuner')
        upper_h = cv2.getTrackbarPos('Upper H', 'Image Tuner')
        upper_s = cv2.getTrackbarPos('Upper S', 'Image Tuner')
        upper_v = cv2.getTrackbarPos('Upper V', 'Image Tuner')

        # Define lower and upper HSV bounds
        lower_bound = np.array([lower_h, lower_s, lower_v])
        upper_bound = np.array([upper_h, upper_s, upper_v])

        # Create a mask
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        # Apply the mask to the original image
        masked_image = cv2.bitwise_and(image, image, mask=mask)

        # Concatenate original and masked images for display
        combined_display = np.hstack((image, masked_image))

        # Show the concatenated image
        cv2.imshow('Image Tuner', combined_display)

        # Break the loop if 'Esc' is pressed
        key = cv2.waitKey(1)
        if key == 27:
            break

    # Clean up and close windows
    cv2.destroyAllWindows()


# Example usage
IMAGE_PATH = "D:\\donkeyData\\images\\"
image = '060335.jpg'
filename = IMAGE_PATH + image

hsv_tuner(filename)
