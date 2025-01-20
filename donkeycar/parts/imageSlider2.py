import cv2
import numpy as np
import tkinter as tk
from tkinter import Scale, HORIZONTAL

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
IMAGE_PATH = "D:\\donkeyData\\images\\"
#image = '060335.jpg'
#image = '060492.jpg'
image = '060753.jpg'

filename = IMAGE_PATH + image
image = cv2.imread(filename)
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
