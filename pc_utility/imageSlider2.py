import cv2
import numpy as np
import tkinter as tk
from tkinter import Scale, HORIZONTAL


def apply_white_balance_temperature(image, target_temperature=8000):
    """
    Adjusts the white balance of an image to match a specific color temperature.

    Parameters:
        image (numpy.ndarray): The input image in BGR format.
        target_temperature (int): Target color temperature in Kelvin (default is 6500K, daylight).

    Returns:
        numpy.ndarray: The white-balanced image.
    """
    # Define a mapping of temperatures (in Kelvin) to scaling factors
    # These factors approximate the color temperature to RGB scaling
    temperature_scaling = {
        1000: (1.0, 0.22, 0.0),   # Warm (reddish)
        2000: (1.0, 0.42, 0.1),   # Warmer
        3000: (1.0, 0.54, 0.2),   # Soft white
        4000: (1.0, 0.65, 0.35),  # Neutral white
        5000: (1.0, 0.76, 0.5),   # Cool white
        6500: (1.0, 1.0, 1.0),    # Daylight (neutral white)
        8000: (0.8, 1.2, 1.5),    # Cooler (bluish)
        10000: (0.6, 1.4, 2.0)    # Very cool
    }

    # Find the nearest temperature for scaling
    temperatures = np.array(list(temperature_scaling.keys()))
    nearest_temperature = temperatures[np.abs(temperatures - target_temperature).argmin()]
    scale_r, scale_g, scale_b = temperature_scaling[nearest_temperature]

    # Apply scaling to each channel
    result = image.astype(np.float32)
    result[:, :, 2] *= scale_r  # Red channel
    result[:, :, 1] *= scale_g  # Green channel
    result[:, :, 0] *= scale_b  # Blue channel

    # Clip values to the valid range [0, 255] and convert back to uint8
    result = np.clip(result, 0, 255).astype(np.uint8)
    return result

def update_mask():
    """Update the masked image based on current slider values."""
    # Get current slider values
    lower_h = lower_hue_slider.get()
    lower_s = lower_sat_slider.get()
    lower_v = lower_val_slider.get()
    upper_h = upper_hue_slider.get()
    upper_s = upper_sat_slider.get()
    upper_v = upper_val_slider.get()

    # Define lower and upper HSV bounds
    lower_bound = np.array([lower_h, lower_s, lower_v])
    upper_bound = np.array([upper_h, upper_s, upper_v])

    # Create a mask
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    # Apply the mask to the original image
    masked_image = cv2.bitwise_and(image, image, mask=mask)

    # Concatenate original and masked images for display
    combined_display = np.hstack((image, masked_image))

    # Update the OpenCV window
    cv2.imshow('Image Tuner', combined_display)

def on_closing():
    """Close the Tkinter and OpenCV windows gracefully."""
    root.destroy()
    cv2.destroyAllWindows()

# Load the input image
# Example usage
#IMAGE_PATH = "D:\\donkeyData\\images\\"
#image = '060335.jpg'
#image = '060492.jpg'
image = 'cropped47.png'

#filename = IMAGE_PATH + image
image = cv2.imread('images\\'+image)
#image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
#image = apply_white_balance_temperature(image)
if image is None:
    print("Error: Could not load image.")
    exit()

# Resize image for better visibility (optional)
scale_percent = 50  # Scale down by 50%
width = int(image.shape[1] * scale_percent / 100)
height = int(image.shape[0] * scale_percent / 100)
image = cv2.resize(image, (width, height))

# Convert the image to HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create the Tkinter window
root = tk.Tk()
root.title("HSV Tuner")

# Create sliders with larger labels
lower_hue_slider = Scale(root, from_=0, to=179, orient=HORIZONTAL, label="Lower Hue", length=400, font=("Helvetica", 12))
lower_hue_slider.set(45)
lower_hue_slider.pack()

lower_sat_slider = Scale(root, from_=0, to=255, orient=HORIZONTAL, label="Lower Saturation", length=400, font=("Helvetica", 12))
lower_sat_slider.set(93)
lower_sat_slider.pack()

lower_val_slider = Scale(root, from_=0, to=255, orient=HORIZONTAL, label="Lower Value", length=400, font=("Helvetica", 12))
lower_val_slider.set(143)
lower_val_slider.pack()

upper_hue_slider = Scale(root, from_=0, to=179, orient=HORIZONTAL, label="Upper Hue", length=400, font=("Helvetica", 12))
upper_hue_slider.set(93)
upper_hue_slider.pack()

upper_sat_slider = Scale(root, from_=0, to=255, orient=HORIZONTAL, label="Upper Saturation", length=400, font=("Helvetica", 12))
upper_sat_slider.set(255)
upper_sat_slider.pack()

upper_val_slider = Scale(root, from_=0, to=255, orient=HORIZONTAL, label="Upper Value", length=400, font=("Helvetica", 12))
upper_val_slider.set(255)
upper_val_slider.pack()

# Set up the update loop
def update_loop():
    update_mask()
    root.after(100, update_loop)

# Start the update loop
update_loop()

# Handle window closing gracefully
root.protocol("WM_DELETE_WINDOW", on_closing)

# Run the Tkinter main loop
root.mainloop()
