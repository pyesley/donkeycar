import cv2
import numpy as np
import matplotlib.pyplot as plt

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

# Distortion coefficients (assume zero or insert your actual distortion from calibration)
dist_coeffs = [-0.30944433  0.1258339  -0.00045581 -0.00057164 -0.01383379]


# ---------------------
# Feature Detector and Matcher
# Use ORB features for demonstration.
orb = cv2.ORB_create(2000)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# ---------------------
# Helper function to extract and match features between two frames
def match_features(img1, img2):
    # Detect and compute ORB features
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    if des1 is None or des2 is None:
        return None, None, None, None

    # Match features
    matches = bf.match(des1, des2)
    # Sort them by distance
    matches = sorted(matches, key=lambda x: x.distance)

    # Extract matched keypoints
    pts1 = np.array([kp1[m.queryIdx].pt for m in matches], dtype=np.float64)
    pts2 = np.array([kp2[m.trainIdx].pt for m in matches], dtype=np.float64)

    return pts1, pts2, kp1, kp2

# ---------------------
# This is the main loop that reads frames from the camera,
# estimates motion, and accumulates robot trajectory.
def main():
    # Open camera. On Raspberry Pi, adjust the index or use a specific pipeline.
    cap = cv2.VideoCapture(0)

    # Read first frame
    ret, prev_frame = cap.read()
    if not ret:
        print("Could not read from camera.")
        return

    prev_frame_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

    # Initial pose
    trajectory = []
    # We consider the initial camera pose as (x=0, y=0).
    current_position = np.array([0.0, 0.0, 0.0])  # 3D position, Z may be irrelevant if flat motion is assumed
    current_rotation = np.eye(3)  # Identity rotation

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Match features between previous and current frame
        pts_prev, pts_curr, kp1, kp2 = match_features(prev_frame_gray, frame_gray)
        if pts_prev is None or len(pts_prev) < 5:
            # Not enough features, skip this frame
            prev_frame_gray = frame_gray
            continue

        # Compute the Essential matrix
        E, mask = cv2.findEssentialMat(pts_curr, pts_prev, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            # Could not find a valid Essential matrix
            prev_frame_gray = frame_gray
            continue

        # Recover pose from essential matrix
        # Note: recoverPose returns rotation R and translation t that transforms points from prev to current frame
        _, R, t, mask_pose = cv2.recoverPose(E, pts_curr, pts_prev, K)

        # Update the global rotation and translation
        # The new camera position = old_position + old_rotation * t
        # The rotation gets updated as old_rotation * R
        scale = 1.0  # Scale factor is unknown. You may need heuristics or additional sensors to get it right.
        current_position = current_position + current_rotation.dot(t.flatten()) * scale
        current_rotation = R.dot(current_rotation)

        # Store the (x, y) for trajectory
        # We assume the plane of motion is XY plane, and Z is up/down.
        # Depending on your setup, you might consider a different axis alignment.
        trajectory.append((current_position[0], current_position[1]))

        # Display the frame with some info
        cv2.putText(frame, f"Position X: {current_position[0]:.2f}, Y: {current_position[1]:.2f}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Frame', frame)

        key = cv2.waitKey(1)
        if key == 27:  # ESC to quit
            break

        prev_frame_gray = frame_gray

    cap.release()
    cv2.destroyAllWindows()

    # Plot the trajectory
    xs = [p[0] for p in trajectory]
    ys = [p[1] for p in trajectory]

    plt.figure()
    plt.scatter(xs, ys, c='blue', s=5)
    plt.title('Estimated Robot Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
