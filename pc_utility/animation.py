import cv2
import os

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
for img in images:
    if img is None:
        continue  # Skip if the image is not loaded properly
    cv2.imshow("Animation", img)  # Display the image
    key = cv2.waitKey(100)  # Wait 100 ms between frames (adjust as needed)
    if key == 27:  # Press 'Esc' to exit early
        break

# Clean up
cv2.destroyAllWindows()
