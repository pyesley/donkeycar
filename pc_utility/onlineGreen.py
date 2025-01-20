import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import shutil
import torchvision.transforms as transforms

IMAGE_PATH = "D:\\donkeyData\\images\\"

def loadImages():

    # Get the list of files in the folder
    all_files = os.listdir(IMAGE_PATH)  # List all files in the directory

    # Filter to include only image files (e.g., .jpg, .png)
    image_files = [f for f in all_files if f.endswith(('.jpg', '.png'))]
    image_files = image_files[0:500]

    # Sort the files (to ensure correct order)
    image_files = sorted(image_files)

    # Load images
    images = [cv2.imread(os.path.join(IMAGE_PATH, file)) for file in image_files]
    return images


def crop_image(image):
    """
    Crops the image by removing the top 25% and bottom 40% of the height.
    This is a function that cuts off parts of the image that the donkey cars
    see.  But are not important.
    """
    # Get the dimensions of the image
    height, width, _ = image.shape

    # Calculate crop coordinates
    start_y = int(height * 0.25)  # Starting y-coordinate (top 25% removed)
    end_y = int(height * 0.65)  # Ending y-coordinate (bottom 40% removed)

    # Crop the image
    cropped_image = image[start_y:end_y, :]

    return cropped_image

def GreenLineDetector( image ,  filename):


    try :
        cropped = crop_image(image)

        # Convert the image from BGR to HSV
        hsv_image = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
        # Define the HSV range for green color
        lower_green = np.array([45, 93, 143])  # Adjust these values based on the image
        upper_green = np.array([93, 255, 255])
        # Create a mask that identifies green pixels
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        # Apply the mask to the original image
        #green_masked_image = cv2.bitwise_and(cropped, cropped, mask=mask)
        #cv2.imwrite("c:\\temp\\green_masked_image.png",green_masked_image)


        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Draw the contours on the original image
        output_image = cropped.copy()
        if contours:
            # Find the largest contour (assumed to be the green line)
            largest_contour = max(contours, key=cv2.contourArea)
            print( largest_contour.size )

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
                cv2.circle(cropped, (int( xmean + .5 ), int( ymean + .5 )), radius=10, color=(255, 255, 255), thickness=-1)  # Green filled circle
                print( 'drew circle at '+str(xmean)+','+str(ymean) )
                return cropped

        else :
            return cropped

    except :
        print('failed on '+filename)
        return image

    return False



def main():
    # Path to your images folder
    IMAGE_PATH = "D:\\donkeyData\\images\\"

    # Get the list of files in the folder
    all_files = os.listdir(IMAGE_PATH)  # List all files in the directory

    # Filter to include only image files (e.g., .jpg, .png)
    image_files = [f for f in all_files if f.endswith(('.jpg', '.png'))]
    image_files = image_files[0:500]

    # Sort the files (to ensure correct order)
    image_files = sorted(image_files)

    # Load images
    images = [cv2.imread(os.path.join(IMAGE_PATH, file)) for file in image_files]

    # Check if images were loaded
    if not images:
        print("No images found!")
        exit()

    # Display images in a loop
    for img, file in zip( images, image_files):
        if img is None:
            continue  # Skip if the image is not loaded properly
        improcess = GreenLineDetector(img, file)
        cv2.imshow("Animation", improcess)  # Display the image
        key = cv2.waitKey(100)  # Wait 100 ms between frames (adjust as needed)
        if key == 27:  # Press 'Esc' to exit early
            break

    # Clean up
    cv2.destroyAllWindows()

main()