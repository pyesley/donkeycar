import cv2
import numpy as np
import math

# ---------------------
# Camera Intrinsic Parameters
# Replace these with your camera's calibrated parameters.
# fx, fy: focal lengths; cx, cy: principal point.
fx = 700.32833455
fy = 703.47654591
cx = 633.7861054
cy = 342.84793261

K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,   0,  1]], dtype=np.float64)
K_inv = np.linalg.inv(K)

# Distortion coefficients (assume zero or insert your actual distortion from calibration)
dist_coeffs = [-0.30944433,  0.1258339,  -0.00045581, -0.00057164, -0.01383379]

def undistort_image( raw_img ):
    """
    raw_img: distorted input (BGR)
    K: 3x3 camera matrix (fx, fy, cx, cy)
    dist_coeffs: e.g., [k1, k2, p1, p2, k3]
    """
    # Option 1: Use cv2.undistort
    undistorted = cv2.undistort(raw_img, K, dist_coeffs)
    return undistorted

def pixel_to_camera_ray(u, v ):
    """
    Convert an undistorted pixel (u, v) to a 3D ray in camera coordinates.
    K_inv: inverse of camera matrix
    Return: direction vector (3,) normalized
    """
    uv1 = np.array([u, v, 1.0], dtype=np.float64)
    ray = K_inv @ uv1  # shape (3,)
    # Normalize
    ray_norm = ray / np.linalg.norm(ray)
    return ray_norm

def line_2d_to_3d_directions(x1, y1, x2, y2 ):
    """
    Convert 2D line segment endpoints -> two 3D direction vectors.
    """
    d1 = pixel_to_camera_ray(x1, y1, K_inv)
    d2 = pixel_to_camera_ray(x2, y2, K_inv)
    return d1, d2


def visualize_HoughLinesP(edges, lines):

    # Convert edges to BGR so we can draw in color
    edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

    # 4) If lines were found, draw them
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Draw the line in RED
            cv2.line(edges_colored, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # 5) Show the result
    cv2.imshow("Edges (colored) with Hough Lines", edges_colored)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def detect_lines_hough(img):
    """
    Returns an array of line segments [ [x1, y1, x2, y2], ... ] in the undistorted image.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(gray, 50, 150)
    lines_p = cv2.HoughLinesP(edges,
                              rho=1,
                              theta=np.pi/180,
                              threshold=80,
                              minLineLength=40,
                              maxLineGap=10)
    if lines_p is None:
        return []
    return [line[0] for line in lines_p]
def find_two_parallel_floor_lines_and_sample_color(raw_img, K, dist_coeffs, plane_normal, plane_d):
    """
    raw_img: original distorted image from camera
    K: camera intrinsic matrix
    dist_coeffs: lens distortion coefficients
    plane_normal, plane_d: define floor plane equation n^T X + d = 0 in camera coords
                          (assuming camera extrinsics are such that camera center is at origin).
    Returns:
      out_img: visualization (lines & region)
      lower_hsv, upper_hsv: color threshold arrays
    """
    # 1) Undistort
    undistorted = cv2.undistort(raw_img, K, dist_coeffs)
    H, W = undistorted.shape[:2]

    # 2) Detect lines in the undistorted image
    lines_2d = detect_lines_hough(undistorted)  # -> list of [x1, y1, x2, y2]

    if not lines_2d:
        print("No lines found.")
        return None, None, None

    K_inv = np.linalg.inv(K)

    # 3) For each line, compute the floor intersection in 3D
    #    We'll store the 3D endpoints to do parallel checks
    floor_lines_3d = []
    for (x1, y1, x2, y2) in lines_2d:
        # endpoints => rays
        r1 = pixel_to_camera_ray(x1, y1, K_inv)  # shape (3,)
        r2 = pixel_to_camera_ray(x2, y2, K_inv)
        # intersect each ray with plane
        P1 = intersect_ray_with_plane(r1, plane_normal, plane_d)
        P2 = intersect_ray_with_plane(r2, plane_normal, plane_d)
        if P1 is None or P2 is None:
            continue
        floor_lines_3d.append((P1, P2))

    # 4) Among all pairs of floor_lines_3d, find two that are parallel
    best_pair = None
    best_score = 0
    for i in range(len(floor_lines_3d)):
        for j in range(i+1, len(floor_lines_3d)):
            P1A, P2A = floor_lines_3d[i]
            P1B, P2B = floor_lines_3d[j]
            dA = P2A - P1A   # direction
            dB = P2B - P1B
            cross = np.cross(dA, dB)
            cross_norm = np.linalg.norm(cross)
            if cross_norm < 1e-3:  # nearly parallel in 3D
                # maybe define "score" = how far apart they are, or something
                separation = compute_average_distance_between_lines(P1A, P2A, P1B, P2B)
                if separation > best_score:
                    best_score = separation
                    best_pair = ((P1A, P2A), (P1B, P2B))

    if not best_pair:
        print("No parallel lines found in 3D.")
        return None, None, None

    (P1A, P2A), (P1B, P2B) = best_pair

    # 5) Project these 3D lines back into the 2D undistorted image to form polygon
    # We'll find the endpoints in 2D by basic pinhole projection (assuming camera at origin).
    #  x' = (fx * X / Z) + cx, etc.
    A1_2d = project_point_to_2d(P1A, K)
    A2_2d = project_point_to_2d(P2A, K)
    B1_2d = project_point_to_2d(P1B, K)
    B2_2d = project_point_to_2d(P2B, K)

    # form polygon (some intersection of these lines with top/bottom if needed)
    # or simply the quadrilateral [A1_2d, A2_2d, B2_2d, B1_2d],
    # but you may have to ensure correct ordering and in-bounds check
    polygon_pts = np.array([A1_2d, A2_2d, B2_2d, B1_2d], dtype=np.int32)

    # 6) Sample HSV within that polygon
    mask = np.zeros((H,W), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon_pts], 255)

    hsv_img = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)
    roi_pixels = hsv_img[mask == 255]
    if len(roi_pixels) == 0:
        print("No pixels in polygon region.")
        return None, None, None

    hsv_min = roi_pixels.min(axis=0)
    hsv_max = roi_pixels.max(axis=0)
    lower_hsv = hsv_min.astype(np.uint8)
    upper_hsv = hsv_max.astype(np.uint8)

    # 7) Visualization
    out_img = undistorted.copy()
    # draw the lines
    cv2.polylines(out_img, [polygon_pts], True, (0,0,255), 3)

    return out_img, lower_hsv, upper_hsv


# Helper function examples:

def intersect_ray_with_plane(ray, plane_normal, plane_d):
    """
    Ray param: camera at origin (0,0,0), direction = ray
    Plane eq: plane_normal^T X + plane_d = 0
    Solve for t so that (t * ray) is on plane.
    """
    denom = plane_normal.dot(ray)
    if abs(denom) < 1e-8:
        return None  # ray is parallel to plane
    t = -plane_d / denom
    if t < 0:
        return None  # intersection is behind camera
    return t * ray  # 3D point

def project_point_to_2d(P, K):
    """
    P: 3D point (x, y, z)
    K: 3x3 camera intrinsics (assuming no rotation/trans for camera at origin).
    Return (u, v) in pixel coords.
    """
    X, Y, Z = P
    if Z <= 0:
        return (-1, -1)  # behind or at camera plane, invalid
    fx = K[0,0]
    fy = K[1,1]
    cx = K[0,2]
    cy = K[1,2]
    u = (fx * X / Z) + cx
    v = (fy * Y / Z) + cy
    return (int(u), int(v))

def compute_average_distance_between_lines(P1A, P2A, P1B, P2B):
    """
    Very rough measure of how far apart two lines in 3D are.
    E.g. average distance between midpoints or something similar.
    """
    midA = 0.5*(P1A + P2A)
    midB = 0.5*(P1B + P2B)
    dist = np.linalg.norm(midA - midB)
    return dist





def find_vertical_tape_by_edges(cam_img,
                                canny_lower=50,
                                canny_upper=150,
                                hough_threshold=50,
                                min_line_length=100,
                                max_line_gap=10):
    """
    Finds a long, near-vertical line extending from bottom to top
    in the image, using edge detection + HoughLinesP.

    Returns:
        output_img: copy of cam_img with the detected line drawn (if found).
        best_line: (x1, y1, x2, y2) coordinates of the best line or None.
    """
    cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
    # Convert to grayscale
    gray = cv2.cvtColor(cam_img, cv2.COLOR_BGR2GRAY)

    # Optional blur to reduce noise
    #gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # 1) Detect edges with Canny
    edges = cv2.Canny(gray, canny_lower, canny_upper)
    cv2.imshow('Edges', edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 2) Use HoughLinesP to find line segments
    # hough_threshold: minimum votes needed;
    # min_line_length, max_line_gap are important for continuity
    lines = cv2.HoughLinesP(edges,
                            rho=1,  # distance resolution
                            theta=np.pi / 180,  # angular resolution (1 degree)
                            threshold=hough_threshold,
                            minLineLength=min_line_length,
                            maxLineGap=max_line_gap)
    visualize_HoughLinesP(edges, lines)
    if lines is None:
        # No lines found
        return cam_img, None

    # 3) Filter out lines that are not near-vertical or not long enough
    H, W = cam_img.shape[:2]
    best_line = None
    best_length = 0

    # Vertical threshold: if line is near vertical, the angle is ~90 deg
    # We'll define "near vertical" as slope magnitude > some large number
    # or angle close to 90 deg.

    def is_vertical(x1, y1, x2, y2, max_angle_deviation_degrees=10):
        # Compute angle in degrees wrt horizontal
        dx = x2 - x1
        dy = y2 - y1
        angle = math.degrees(math.atan2(dy, dx))
        # For a vertical line, angle ~ ±90 deg. We'll check ±10 deg around ±90.
        # But note angle can be near -90 or +90.
        # We can just shift angle by 90, check if near 0
        angle_diff = abs(abs(angle) - 90)
        return angle_diff <= max_angle_deviation_degrees

    edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    # 4) Loop through all detected lines
    for line in lines:
        x1, y1, x2, y2 = line[0]

        # Check orientation
        if not is_vertical(x1, y1, x2, y2, max_angle_deviation_degrees=30):
            continue
        cv2.line(edges_colored, (x1, y1), (x2, y2), (0, 0, 255), 2)
        # Check length
        length = math.hypot(x2 - x1, y2 - y1)
        if length < min_line_length:
            continue
        cv2.line(edges_colored, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Check if line extends from (or near) bottom to top
        # Because it's near-vertical, we can check min_y and max_y
        line_min_y = min(y1, y2)
        line_max_y = max(y1, y2)

        # For instance: let's require that line_max_y be near bottom
        # (within some margin of H) and line_min_y is significantly
        # higher up (some fraction of the image). Adjust to your scenario.
        if abs(line_max_y - H) > 50:
            # e.g. it doesn't extend close enough to bottom
            continue

        # Another possible check: require line_min_y < some fraction
        if line_min_y > 0.2 * H:
            # it doesn't reach far enough up
            continue

        # Keep the best (longest) candidate
        if length > best_length:
            best_length = length
            best_line = (x1, y1, x2, y2)


    # 5) Visualize the best line
    cv2.imshow("Edges (colored) with Hough Lines", edges_colored)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 5) Draw the best line if found
    output_img = cam_img.copy()
    if best_line is not None:
        x1, y1, x2, y2 = best_line
        cv2.line(output_img, (x1, y1), (x2, y2), (0, 255, 0), 3)

    return output_img, best_line


# ---- Example usage ----
if __name__ == "__main__":
    IMAGE_PATH = "C:\\Users\\pyesl\\Documents\\donkeyCar\\images\\"
    image = 'cropped145.png'
    filename = IMAGE_PATH + image
    img = cv2.imread(filename)

    out, line_coords = find_vertical_tape_by_edges(img)

    if line_coords is not None:
        print("Found vertical line:", line_coords)
    else:
        print("No vertical line found.")

    cv2.imshow("Detected line", out)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
