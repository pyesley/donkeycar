import pandas as pd
import numpy as np
import cv2
import time
from skimage.filters import gaussian
from skimage.segmentation import active_contour
import matplotlib.pyplot as plt
from skimage.segmentation import slic, mark_boundaries
from skimage.color import rgb2hsv
from skimage.util import img_as_float

from skimage.segmentation import slic, mark_boundaries, find_boundaries


'''
    For this code I have already use the checker board program
    to get the camera matrix and distortion coefficients.
    
    For this code I put lots of stickers dots on the floor,
    I measured all of the locations both in Affinity Photo 
    and with a ruler.  Then I run this code on that image to 
    compute the rotation matrix and translation vector.
'''


#Camera matrix (K):
cameraMatrix = np.array(
    [[700.32833455,   0.,         633.7861054 ],
     [  0.,         703.47654591, 342.84793261],
     [  0.,           0.,           1.        ]] )
#Distortion coefficients:
distCoeffs = np.array( [-0.30944433,  0.1258339,  -0.00045581, -0.00057164, -0.01383379] )
#RMS re-projection error: 0.2227

cache = {}

def undistort_image(img, cameraMatrix, distCoeffs):
    h, w = img.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (w, h), 1, (w, h))
    undistorted_img = cv2.undistort(img, cameraMatrix, distCoeffs, None, newCameraMatrix)
    return undistorted_img, newCameraMatrix

def undistort_points(points, cameraMatrix, distCoeffs, newCameraMatrix):
    points = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    undistorted_points = cv2.undistortPoints(points, cameraMatrix, distCoeffs, P=newCameraMatrix)
    return undistorted_points.reshape(-1, 2)

def draw_yellow_dots(img, points):
    for point in points:
        cv2.circle(img, tuple(point.astype(int)), 5, (0, 255, 255), -1)  # Yellow color in BGR

def compute_circle_points(radius, center, num_points=400):
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = radius * np.cos(theta) + center[0]
    y = radius * np.sin(theta) + center[1]
    z = np.zeros_like(x)
    return np.vstack((x, y, z)).T

def transform_points(points, rotation_matrix, tvec, cameraMatrix, distCoeffs):
    transformed_points = []
    for point in points:
        world_point = np.dot(rotation_matrix, point) + tvec.ravel()
        image_points, _ = cv2.projectPoints(world_point.reshape(1, 3), np.zeros((3, 1)), np.zeros((3, 1)), cameraMatrix, distCoeffs)
        transformed_points.append(image_points[0].ravel())
    return np.array(transformed_points)

def calculate_rotation_translation_matrix(cameraMatrix, distCoeffs, points_2d, points_3d):
    _, rvec, tvec = cv2.solvePnP(points_3d, points_2d, cameraMatrix, distCoeffs)
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    return rotation_matrix, tvec

def print_matrices_as_code(rotation_matrix, tvec):
    print("rotation_matrix = np.array([")
    for row in rotation_matrix:
        print("    [{}],".format(", ".join(map(str, row))))
    print("])")

    print("\ntvec = np.array([")
    for row in tvec:
        print("    [{}],".format(", ".join(map(str, row))))
    print("])")
def CalculateRotation_Matrix ():
    # Load the data from the Excel file
    data = pd.read_excel('donkeyCalibration.xlsx')

    # Load the image
    img = cv2.imread('C:\\Users\\pyesl\\Documents\\donkeyCar\\lineFollower\\pid_car\\1280res\\GreenDots0.png')



    # Check if the image was loaded successfully
    if img is None:
        print("Error: Could not load image.")
        return

    # Undistort the image
    undistorted_img, newCameraMatrix = undistort_image(img, cameraMatrix, distCoeffs)

    # Get the screen points
    screen_points = data[['screenX', 'screenY']].values
    data['worldZ'] = 0
    world_points = data[['worldX', 'worldY', 'worldZ']].values
    world_points = np.array(world_points, dtype=np.float32)

    # Undistort the screen points
    undistorted_points = undistort_points(screen_points, cameraMatrix, distCoeffs, newCameraMatrix)

    # Draw yellow dots on the undistorted image
    draw_yellow_dots(undistorted_img, undistorted_points)

    # Display the undistorted image with yellow dots
    cv2.imshow('Undistorted Image with Yellow Dots', undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Calculate the rotation and translation matrix
    rotation_matrix, tvec = calculate_rotation_translation_matrix(cameraMatrix, distCoeffs, undistorted_points, world_points)

    # Compute the circle points in world space
    circle_points = compute_circle_points(15, [5, 15])

    # Transform the circle points to image space
    transformed_circle_points = transform_points(circle_points, rotation_matrix, tvec, cameraMatrix, distCoeffs)

    # Draw the transformed circle points on the original image
    for i in range(len(transformed_circle_points) - 1):
        start_point = tuple(transformed_circle_points[i].astype(int))
        end_point = tuple(transformed_circle_points[i + 1].astype(int))
        cv2.line(img, start_point, end_point, (0, 0, 255), 2)  # Red color in BGR, thickness 2

    # Connect the last point to the first point to complete the circle
    cv2.line(img, tuple(transformed_circle_points[-1].astype(int)), tuple(transformed_circle_points[0].astype(int)),
             (0, 0, 255), 2)

    # Display the original image with the red circle
    cv2.imshow('Original Image with Red Circle', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return rotation_matrix, tvec, newCameraMatrix

def project_image_to_world(undistorted_img, rotation_matrix, tvec, cameraMatrix, distCoeffs, world_range, pixel_dimensions):
    global cache
    h, w = undistorted_img.shape[:2]
    world_img = np.zeros((pixel_dimensions[1], pixel_dimensions[0], 3), dtype=np.uint8)

    for y in range(pixel_dimensions[1]):
        for x in range(pixel_dimensions[0]):
            world_x = world_range[0] + (world_range[2] - world_range[0]) * x / pixel_dimensions[0]
            world_y = world_range[1] + (world_range[3] - world_range[1]) * y / pixel_dimensions[1]
            world_point = (world_x, world_y, 0)

            if world_point not in cache:
                image_points, _ = cv2.projectPoints(np.array([world_point], dtype=np.float32), rotation_matrix, tvec, cameraMatrix, distCoeffs)
                cache[world_point] = image_points[0].ravel().astype(int)

            image_x, image_y = cache[world_point]
            if 0 <= image_x < w and 0 <= image_y < h:
                world_img[y, x] = undistorted_img[image_y, image_x]

    return world_img


def main():
    rotation_matrix, tvec, newCameraMatrix = CalculateRotation_Matrix()
    print_matrices_as_code(rotation_matrix, tvec)

    '''
    rotation_matrix = np.array([
        [0.9995098479583695, -0.012307813984683304, 0.02878509248128326],
        [0.0257169486668329, -0.2015318024604856, -0.9791443055792621],
        [0.017852237548976253, 0.9794046407445425, -0.20111650181561103],
    ])

    tvec = np.array([
        [-0.17446499121433176],
        [3.7192981312509272],
        [11.120700961796098],
    ])
    '''

    IMAGE_PATH = "C:\\Users\\pyesl\\Documents\\donkeyCar\\images\\"
    image = 'cropped94.png' #'cropped145.png'
    filename = IMAGE_PATH + image

    cam_img = cv2.imread(filename)
    cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
    cam_img = cv2.resize(cam_img, (1280, 720)) #(1280, 720)


    # Display the resulting image
    cv2.imshow('World Coordinates Image', cam_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    # Undistort the image
    undistorted_img, newCameraMatrix = undistort_image(cam_img, cameraMatrix, distCoeffs)

    # Define world coordinates range and pixel dimensions
    world_range = [-10, -2, 10, 20]
    pixel_dimensions = (undistorted_img.shape[1], undistorted_img.shape[0])
    scale = 10.0
    pixel_dimensions = (int( scale*(world_range[2]-world_range[0])), int( scale*(world_range[3]-world_range[1])))

    # Project the undistorted image into world coordinates
    for i in [0,1]:
        start_time = time.time()
        world_img = project_image_to_world(undistorted_img, rotation_matrix, tvec, newCameraMatrix, distCoeffs, world_range,
                                           pixel_dimensions)
        end_time = time.time()
        print(f"{i} call duration: {end_time - start_time} seconds")

    world_img = cv2.flip(world_img, 0)

    # Display the resulting image
    cv2.imshow('World Coordinates Image', world_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    create_green_mask(world_img)

    slic_with_canny_refinement(world_img)

def create_green_mask(img):
    """
    Creates a mask where white is when a pixel is more green than red, else the pixel is black.

    :param img: The input image in BGR format.
    :return: The resulting mask image.
    """

    start_time = time.time()
    # Split the image into its B, G, R components
    B, G, R = cv2.split(img)

    # Create a mask where the green component is greater than the red component
    mask = np.where((G > R) & (G > B), 255, 0).astype(np.uint8)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)

    # Draw the largest contour on the original image
    cv2.drawContours(img, [largest_contour], -1, (0, 255, 0), 2)  # Green color, thickness 2

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time} seconds")

    # Display the resulting mask image
    cv2.imshow('Green Mask', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def mask_trapezoid(img):
    """
    Masks the area outside of a trapezoid to black. The trapezoid has one edge at the bottom of the image
    and the top edge is a line in the middle of the image with the width of half the image.

    :param img: The input image.
    :return: The masked image.
    """
    height, width = img.shape[:2]

    # Define the trapezoid vertices
    top_width = width // 2
    top_left = (width // 4, height // 2)
    top_right = (3 * width // 4, height // 2)
    bottom_left = (0, height-10)
    bottom_right = (width, height-10)

    # Create a mask with the same dimensions as the image
    mask = np.zeros_like(img)

    # Define the polygon (trapezoid) points
    polygon = np.array([top_left, top_right, bottom_right, bottom_left], np.int32)
    polygon = polygon.reshape((-1, 1, 2))

    # Fill the polygon with white color on the mask
    cv2.fillPoly(mask, [polygon], (255, 255, 255))

    # Apply the mask to the image
    masked_img = cv2.bitwise_and(img, mask)

    # Display the resulting mask image
    cv2.imshow('TrapozoidMask', masked_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    create_green_mask(masked_img)

    return masked_img


def main3():
    IMAGE_PATH = "C:\\Users\\pyesl\\Documents\\donkeyCar\\images\\"
    image = 'cropped71.png'
    filename = IMAGE_PATH + image

    cam_img = cv2.imread(filename)
    cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)

    mask_trapezoid(cam_img)
    return

def slic_with_canny_refinement(bgr, n_segments=300, compactness=6):
    """
    1) Perform SLIC superpixel segmentation.
    2) Identify "green" superpixels in HSV.
    3) Create an initial mask of these green superpixels.
    4) Detect edges with Canny and dilate them for tolerance.
    5) For each green superpixel, check boundary overlap with edges.
       Keep only superpixels whose boundary is near edges.
    """

    # 1) Load image (BGR) with OpenCV and convert to RGB for skimage

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    # Convert to float [0,1] for SLIC
    image_float = img_as_float(rgb)

    # SLIC segmentation
    segments = slic(
        image_float,
        n_segments=n_segments,  # number of superpixels
        compactness=compactness,  # balances color/space proximity
        start_label=1
    )

    # 2) Convert to HSV for easier green detection
    hsv = rgb2hsv(image_float)

    # 3) Identify green superpixels by average hue & saturation
    green_labels = []
    for label_id in np.unique(segments):
        region_mask = (segments == label_id)
        mean_hue = np.mean(hsv[..., 0][region_mask])
        mean_sat = np.mean(hsv[..., 1][region_mask])

        # Typical "green" hue range is ~ [0.15, 0.45], saturation > 0.2
        if 0.15 < mean_hue < 0.45 and mean_sat > 0.2:
            green_labels.append(label_id)

    # Create an initial "green mask" from these labels
    initial_green_mask = np.zeros_like(segments, dtype=np.uint8)
    for label_id in green_labels:
        initial_green_mask[segments == label_id] = 255

    # 4) Build edges with Canny (on grayscale) + dilation
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray_blur, 50, 150)

    # Dilate edges to allow some tolerance (so we don't require exact pixel match)
    kernel = np.ones((3, 3), np.uint8)
    edges_dil = cv2.dilate(edges, kernel, iterations=1)
    edges_dil_bin = (edges_dil > 0)

    # 5) Refine the green mask by checking boundary-edge alignment
    refined_mask = np.zeros_like(initial_green_mask)
    for label_id in green_labels:
        region_mask = (segments == label_id)
        # Find boundary of this superpixel
        region_boundary = find_boundaries(region_mask, mode='outer')
        # Overlap between boundary and edges
        overlap = np.logical_and(region_boundary, edges_dil_bin)
        # If enough boundary pixels align with edges, keep the superpixel
        if overlap.sum() > 5:  # threshold to avoid noise; tune as needed
            refined_mask[region_mask] = 255

    # 6) Visualize results
    fig, ax = plt.subplots(1, 3, figsize=(15, 5))

    # Left: SLIC superpixels
    ax[0].imshow(mark_boundaries(image_float, segments))
    ax[0].set_title("SLIC Superpixels")
    ax[0].axis("off")

    # Middle: initial green mask (SLIC-based only)
    ax[1].imshow(initial_green_mask, cmap='gray')
    ax[1].set_title("Initial Green Mask (SLIC)")
    ax[1].axis("off")

    # Right: final refined mask (SLIC + Canny)
    ax[2].imshow(refined_mask, cmap='gray')
    ax[2].set_title("Refined Mask (SLIC & Canny)")
    ax[2].axis("off")

    plt.tight_layout()
    plt.show()

    return refined_mask

def main2():

    IMAGE_PATH = "C:\\Users\\pyesl\\Documents\\donkeyCar\\images\\"
    image = 'cropped71.png'
    filename = IMAGE_PATH + image

    cam_img = cv2.imread(filename)
    cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
    cam_img = cv2.resize(cam_img, (1280, 720))

    slic_with_canny_refinement(cam_img)


if __name__ == "__main__":
    main3()