import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import shutil
import torchvision.transforms as transforms

IMAGE_PATH = "C:\\Users\\mdyes\\Documents\\DonkeyCar\\data\\images\\"

def RunAnalysis():
    # Load the image
    path = "C:\\Users\\mdyes\\Documents\\DonkeyCar\\data\\images\\"

    #works 1738
    #fails 1490

    for imageNumber in range(60335,83529) : #1490

        try :
            image_path = f'0{imageNumber:05}.jpg'
            image = cv2.imread(path+image_path)


            # Convert the image from BGR to HSV
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define the HSV range for green color
            lower_green = np.array([40, 40, 40])  # Adjust these values based on the image
            upper_green = np.array([80, 255, 255])

            # Mask the upper half of the image (keeping only the lower half)
            height, width = hsv_image.shape[:2]
            mask_half = np.zeros_like(hsv_image[:, :, 0])
            mask_half[int(height // 1.5):, :] = 1  # Adjust to mask from one-third of the height down

            # Create a mask that identifies green pixels
            mask = cv2.inRange(hsv_image, lower_green, upper_green)

            # Apply the mask to the original image
            green_masked_image = cv2.bitwise_and(image, image, mask=mask)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Draw the contours on the original image
            output_image = image.copy()
            if contours:
                # Find the largest contour (assumed to be the green line)
                largest_contour = max(contours, key=cv2.contourArea)
                print( largest_contour.size )

                # Draw the contour on the output image
                cv2.drawContours(output_image, [largest_contour], -1, (0, 255, 0), 2)


                # Extract x and y coordinates from the contour
                contour_points = largest_contour[:, 0, :]
                y = contour_points[:, 0]
                x = contour_points[:, 1]

                # Fit a second-order polynomial (quadratic)
                coefficients = np.polyfit(x, y, 2)
                polynomial = np.poly1d(coefficients)

                # Calculate predicted y values using the fitted polynomial
                y_pred = polynomial(x)
                # Calculate residuals
                residuals = y - y_pred
                # Calculate the standard deviation of the residuals
                std_dev = np.std(residuals)/(np.abs(max(x)-min(x)))


                # Generate x values for evaluation
                x_fit = np.linspace(min(x), max(x), 100)
                y_fit = polynomial(x_fit)

                # Map the fitted polynomial points back onto the image
                #for x_val, y_val in zip(x_fit, y_fit):
                for y_val, x_val in zip(x_fit, y_fit):
                    # Ensure the coordinates are within image bounds
                    if 0 <= int(y_val) < output_image.shape[0] and 0 <= int(x_val) < output_image.shape[1]:
                        # Draw the fitted polynomial on the output image (you can choose color and thickness)
                        cv2.circle(output_image, (int(x_val), int(y_val)), radius=3, color=(0, 0, 255), thickness=-1)


            # Convert the image to RGB for displaying using matplotlib
            output_image_rgb = cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)

            if largest_contour.size < 100: continue

            # Plot the original and processed images
            plt.figure(figsize=(10, 5))

            plt.subplot(1, 2, 1)
            plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            plt.title("Original Image")
            plt.axis('off')

            plt.subplot(1, 2, 2)
            plt.imshow(output_image_rgb)
            plt.title(f'Detected Green Tape Line {std_dev}')
            plt.axis('off')

            plt.tight_layout()
            plt.show()
            cv2.imwrite(f'c:\\temp\\image{imageNumber}.png',output_image_rgb)
            plt.savefig(f'c:\\temp\\processed{imageNumber}.png')
            plt.close()  # Close the current figure to release memory
        except :
            print( f'falied on {imageNumber}')

def ImageMasker( image ):
    '''
        This chops out the parts of the image that are irrelvant
    '''
    # Mask the image with new cuts (Top Cut: 0.4, Bottom Cut: 0.2)
    height, width = image.shape[:2]

    # Create a mask of the same size as the image
    mask = np.zeros((height, width), dtype=np.uint8)
    # Fill the top half of the mask with white (255)
    cv2.rectangle(mask, (0, 0), (width, int(height * .6)), 255, -1)
    # Apply the mask to the image
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    cv2.rectangle(mask, (0, 0), (width, int(height * 0.25)), 0, -1)
    # Apply the mask to the image
    masked_image = cv2.bitwise_and(masked_image, masked_image, mask=mask)
    cv2.imwrite("c:\\temp\\masked_image.png", masked_image)


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

def maskGreen( cropped ):
    '''
        Us this after the crop to dumb down the image for NN training
    '''

    # Convert the image from BGR to HSV
    hsv_image = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
    # Define the HSV range for green color
    lower_green = np.array([30, 30, 30])  # Adjust these values based on the image
    upper_green = np.array([90, 255, 255])
    # Create a mask that identifies green pixels
    mask = cv2.inRange(hsv_image, lower_green, upper_green)
    # Apply the mask to the original image
    green_masked_image = cv2.bitwise_and(cropped, cropped, mask=mask)
    return green_masked_image


def GreenLineDetector( fileName ):
    # Load the image

    imageNumber = str(fileName).split('.')[0]

    try :
        image_path = f'{imageNumber}.jpg'
        image = cv2.imread(IMAGE_PATH+image_path)

        cropped = crop_image(image)
        #cv2.imwrite("c:\\temp\\cropped.png",cropped)


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

            # Draw the contour on the output image
            cv2.drawContours(output_image, [largest_contour], -1, (0, 255, 0), 2)


            # Extract x and y coordinates from the contour
            contour_points = largest_contour[:, 0, :]
            y = contour_points[:, 0]
            x = contour_points[:, 1]

            # Fit a second-order polynomial (quadratic)
            coefficients = np.polyfit(x, y, 2)
            polynomial = np.poly1d(coefficients)

            # Calculate predicted y values using the fitted polynomial
            y_pred = polynomial(x)
            # Calculate residuals
            residuals = y - y_pred
            # Calculate the standard deviation of the residuals
            std_dev = np.std(residuals)/(np.abs(max(x)-min(x)))


            if largest_contour.size < 50: return False
            if std_dev < .09 : return True
    except :
        print('failed on '+fileName)

    return False

def csvEditor():
    data = pd.read_csv('C:\\Users\\mdyes\\Documents\\DonkeyCar\\data\\tub_data.csv')
    data['goodGreen'] = False
    data.reset_index(inplace=True)
    for i in range(0,len(data)):
        fileName = data.iloc[i, 1]  # Use square brackets instead of parentheses
        isGreen = GreenLineDetector(str(fileName))
        data.iloc[i, 4] = isGreen  # Again, use square brackets
    data.to_csv('C:\\Users\\mdyes\\Documents\\DonkeyCar\\TrainingData.csv', index=False)
    csvEditoFiltered()

def csvEditoFiltered():
    # Load the CSV file
    input_csv_path = 'C:\\Users\\mdyes\\Documents\\DonkeyCar\\TrainingData.csv'  # Replace with your file path
    output_csv_path = 'C:\\Users\\mdyes\\Documents\\DonkeyCar\\WideTrainingData2.csv'

    # Read the CSV file into a DataFrame
    df = pd.read_csv(input_csv_path)

    # Drop the first row, then filter to keep only rows where the last column is True
    df_filtered = df.iloc[1:]  # Remove the first row
    df_filtered = df_filtered[df_filtered.iloc[:, -1] == True]  # Keep only rows where the last column is True

    # Drop the first and last columns
    df_final = df_filtered.iloc[:, 1:-1]

    # Save the result to a new CSV file
    df_final.to_csv(output_csv_path, index=False)

    print(f"Filtered data has been saved to {output_csv_path}")



def seeTrainingImages():
    # Load the CSV file
    input_csv_path = 'C:\\Users\\mdyes\\Documents\\DonkeyCar\\TrainingData.csv'  # Replace with your file path
    destination_folder = 'C:\\Users\\mdyes\\Documents\\DonkeyCar\\image_green'  # Replace with your file path

    # Read the CSV file into a DataFrame
    df = pd.read_csv(input_csv_path)

    # Drop the first row, then filter to keep only rows where the last column is True
    df_filtered = df.iloc[1:]  # Remove the first row
    df_filtered = df_filtered[df_filtered.iloc[:, -1] == True]  # Keep only rows where the last column is True

    # Ensure the destination directory exists
    os.makedirs(destination_folder, exist_ok=True)
    file_list = df_filtered.iloc[:, 1].tolist()

    for file_name in file_list:
        file_path = IMAGE_PATH + str(file_name)
        if os.path.isfile(file_path) and file_path.lower().endswith('.jpg'):
            try:
                # Copy file to the destination folder
                shutil.copy(file_path, destination_folder)
                print(f"Copied {file_path} to {destination_folder}")
            except Exception as e:
                print(f"Failed to copy {file_path}: {e}")
        else:
            print(f"Skipped {file_path}, not a .jpg file or does not exist.")

# Function to apply MaskHalf on different ratios and display the results
def apply_mask_half(image_path, ratios):
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Failed to load image: {image_path}")
        return

    height, width = image.shape[:2]

    # Set up the plot
    fig, axes = plt.subplots(2, 5, figsize=(15, 6))
    fig.suptitle("MaskHalf Applied at Different Ratios", fontsize=16)

    for idx, ratio in enumerate(ratios):
        # Create a mask for the lower portion of the image based on the ratio
        mask_half = np.zeros((height, width), dtype=np.uint8)
        mask_half[int(height * ratio):, :] = 255  # Adjust based on ratio

        # Apply the mask to the original image
        masked_image = cv2.bitwise_and(image, image, mask=mask_half)

        # Plot the masked image
        ax = axes[idx // 5, idx % 5]
        ax.imshow(cv2.cvtColor(masked_image, cv2.COLOR_BGR2RGB))
        ax.set_title(f"Cut {ratio:.2f}")
        ax.axis("off")

    plt.tight_layout()
    plt.subplots_adjust(top=0.85)
    plt.show()

def cut_bottom(image_path, ratios):
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Failed to load image: {image_path}")
        return

    height, width = image.shape[:2]

    # Set up the plot
    fig, axes = plt.subplots(2, 5, figsize=(15, 6))
    fig.suptitle("Bottom Cut at Different Ratios", fontsize=16)

    for idx, ratio in enumerate(ratios):
        # Calculate the new height after cutting
        cut_height = int(height * (1 - ratio))  # Retain the top (1 - ratio) portion
        cut_image = image[:cut_height, :, :]  # Cut off the bottom portion

        # Pad the image back to original size for consistent display
        padded_image = cv2.copyMakeBorder(cut_image, 0, height - cut_height, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))

        # Plot the cut image
        ax = axes[idx // 5, idx % 5]
        ax.imshow(cv2.cvtColor(padded_image, cv2.COLOR_BGR2RGB))
        ax.set_title(f"Cut {ratio:.2f}")
        ax.axis("off")

    plt.tight_layout()
    plt.subplots_adjust(top=0.85)
    plt.show()

# Example usage
#image_path = "C:\\Users\\mdyes\\Documents\\DonkeyCar\\data\\images\\060598.jpg"  # Replace with the image path you want to test
#ratios_to_test = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
#apply_mask_half(image_path, ratios_to_test)

csvEditor()
#seeTrainingImages()

#RunAnalysis()
