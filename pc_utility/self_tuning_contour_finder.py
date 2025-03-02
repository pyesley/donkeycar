import cv2
import numpy as np

def find_tape_contour(cam_img):
    # Convert to HSV
    cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
    hsv_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2HSV)

    # ----------------------------
    # STEP 1: Get a small ROI at the bottom where we know the tape exists
    # ----------------------------
    height = hsv_img.shape[0]
    roi_height = int(0.10 * height)  # bottom 10% of the image
    roi = hsv_img[height - roi_height:height, :]

    # ----------------------------
    # STEP 2: Compute median or mean color in the ROI
    # ----------------------------
    # Flatten ROI so shape is (N, 3). Then compute median along axis=0
    roi_reshaped = roi.reshape(-1, 3)
    median_hsv = np.median(roi_reshaped, axis=0)
    # median_hsv is now [medianHue, medianSat, medianVal]

    # Alternatively, you could use mean:
    # mean_hsv = np.mean(roi_reshaped, axis=0)

    # For demonstration, let's define some margin around the median:
    hue_range = 15
    sat_range = 40
    val_range = 40

    # ----------------------------
    # STEP 3: Build dynamic thresholds around median_hsv
    # ----------------------------
    medianH = median_hsv[0]
    medianS = median_hsv[1]
    medianV = median_hsv[2]

    lower_color = np.array([
        max(medianH - hue_range, 0),
        max(medianS - sat_range, 0),
        max(medianV - val_range, 0)
    ], dtype=np.uint8)

    upper_color = np.array([
        min(medianH + hue_range, 179),  # Hue range in OpenCV HSV goes up to 179
        min(medianS + sat_range, 255),
        min(medianV + val_range, 255)
    ], dtype=np.uint8)

    # ----------------------------
    # STEP 4: Threshold the entire image with the dynamic range
    # ----------------------------
    mask = cv2.inRange(hsv_img, lower_color, upper_color)

    # ----------------------------
    # STEP 5: Clean up the mask
    # ----------------------------
    # Morphological open to remove small noise, then close to fill small holes
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # ----------------------------
    # STEP 6: Find contours and pick the largest one
    # ----------------------------
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("No contours found")
        return None, mask

    largest_contour = max(contours, key=cv2.contourArea)

    # Compute centroid of the largest contour (optional)
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0

    # ----------------------------
    # STEP 7: Draw results
    # ----------------------------
    output_img = cam_img.copy()
    cv2.drawContours(output_img, [largest_contour], -1, (0, 255, 0), 2)
    cv2.circle(output_img, (cx, cy), 5, (255, 255, 255), -1)

    # Return largest contour (the tape) and the mask if desired
    return output_img, mask


# Example usage:
if __name__ == "__main__":
    IMAGE_PATH = "C:\\Users\\pyesl\\Documents\\donkeyCar\\images\\"
    image = 'cropped145.png'
    filename = IMAGE_PATH + image
    cam_img = cv2.imread(filename)
    output_img, mask = find_tape_contour(cam_img)
    if output_img is not None:
        cv2.imshow("Tape Contour", output_img)
        cv2.imshow("Mask", mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
