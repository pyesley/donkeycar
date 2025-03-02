import cv2
import numpy as np
from picamera2 import Picamera2

#Camera matrix (K):
cameraMatrix = [[700.32833455,   0.,         633.7861054 ],
     [  0.,         703.47654591, 342.84793261],
     [  0.,           0.,           1.        ]]
#Distortion coefficients:
distCoeffs = [[-0.30944433,  0.1258339,  -0.00045581, -0.00057164, -0.01383379]]
#RMS re-projection error: 0.2227


def get_checkerboard_plane_points_2d(pattern_size, square_size):
    """
    Returns a (N, 2) array of 2D coordinates for the checkerboard corners
    in a "floor-plane coordinate system", where (0,0) is one corner and
    x increments by square_size along one axis, y increments by square_size along the other.
    """
    # pattern_size = (rows, cols) or (height, width) in corners
    # For a 5x7 pattern, that means 5 corners in one dimension, 7 in the other.
    # Adjust as needed for your pattern orientation.
    rows, cols = pattern_size
    objp_2d = np.zeros((rows*cols, 2), np.float32)
    # We'll map (row, col) -> (x, y) = (col*square_size, row*square_size)
    # so that the "columns" go in the x-direction, "rows" in the y-direction
    # This is just a convention; the important part is consistency.
    for r in range(rows):
        for c in range(cols):
            idx = r * cols + c
            objp_2d[idx, 0] = c * square_size  # x
            objp_2d[idx, 1] = r * square_size  # y
    return objp_2d

def phase2_homography_estimation(cameraMatrix, distCoeffs,
                                 pattern_size=(5,7),
                                 square_size=3.0):
    """
    Phase 2:
      1) Ask the user to place the checkerboard on the floor plane.
      2) Capture multiple images to detect the corners.
      3) Compute a homography that maps floor-plane coordinates -> image coordinates.
      4) Demonstrate a bird's-eye warp using H^-1.

    :param cameraMatrix: 3x3 numpy array from calibration (intrinsic matrix)
    :param distCoeffs: distortion coefficients from calibration
    :param pattern_size: (rows, cols) of checkerboard interior corners
    :param square_size: physical size of each square (mm, cm, or your chosen unit)
    :return: The computed homography H (plane->image),
             plus its inverse H_inv (image->plane).
    """

    # 1. Prepare reference 2D coordinates for the entire checkerboard in floor-plane space.
    board_points_2d = get_checkerboard_plane_points_2d(pattern_size, square_size)

    # We'll accumulate all captures in these lists:
    all_plane_pts = []  # 2D floor-plane coords (x, y)
    all_image_pts = []  # 2D image coords (u, v)

    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (1280, 720)})
    picam2.configure(config)
    picam2.start()

    print("\n--- Phase 2: Homography (Bird's Eye) Estimation ---")
    print("Please place the checkerboard on the floor plane.")
    print("Press 'c' to capture, 'f' to finish, 'q' to quit.")
    print("You need at least one valid capture, but multiple captures can help verify consistency.")

    capture_count = 0
    last_frame = None

    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # weird linux thing
        last_frame = frame.copy()

        # For display, we're using the raw frame:
        display_frame = frame

        # Text overlay for live feed
        cv2.putText(display_frame, f"Captures: {capture_count}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.putText(display_frame, "Press 'c' to capture, 'f' to finish, 'q' to quit", (10,70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow('Homography Estimation - Live Feed', display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            # Attempt to find chessboard corners
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_corners, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if ret_corners:
                # Refine corners
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

                # Flatten the corners into (N, 2) shape
                image_pts_2d = corners_refined.reshape(-1, 2)

                # Append the plane coordinates and the matching image coordinates
                all_plane_pts.extend(board_points_2d)
                all_image_pts.extend(image_pts_2d)

                capture_count += 1
                print(f"[Capture {capture_count}] Chessboard detected. {len(image_pts_2d)} corners added.")

                # Show detection for debugging: draw both the detected corners and
                # overlay the expected physical coordinates from board_points_2d.
                tmp_detect = frame.copy()
                cv2.drawChessboardCorners(tmp_detect, pattern_size, corners_refined, True)
                # Add the coordinate text for each corner
                for corner, board_pt in zip(corners_refined.reshape(-1,2), board_points_2d):
                    pt = (int(corner[0]), int(corner[1]))
                    text = f"({board_pt[0]:.1f},{board_pt[1]:.1f})"
                    # Offset the text so it doesn't cover the corner marker
                    cv2.putText(tmp_detect, text, (pt[0] + 5, pt[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1, cv2.LINE_AA)

                cv2.imshow('Detected Corners', tmp_detect)
                cv2.waitKey(5000)
                cv2.destroyWindow('Detected Corners')

            else:
                print("No corners detected. Try again.")

        elif key == ord('f'):
            if capture_count < 1:
                print("At least one valid capture is needed.")
            else:
                # 2. Compute homography
                plane_pts = np.array(all_plane_pts, dtype=np.float32)
                img_pts   = np.array(all_image_pts,  dtype=np.float32)

                # plane_pts: Nx2 in floor-plane coords
                # img_pts:   Nx2 in image pixel coords
                print("\nComputing homography using RANSAC...")
                H, mask = cv2.findHomography(plane_pts, img_pts, cv2.RANSAC, 5.0)
                if H is None:
                    print("Homography computation failed. Check your captures.")
                    continue
                print("Homography H (plane->image):\n", H)

                # Inverse homography for image->plane
                H_inv = np.linalg.inv(H)
                print("Inverse Homography H_inv (image->plane):\n", H_inv)

                # 3. Demonstrate bird's-eye warp on the last frame captured
                scale = 10
                board_w = (pattern_size[1] - 1) * square_size  # (cols-1)*square_size
                board_h = (pattern_size[0] - 1) * square_size  # (rows-1)*square_size
                out_w = int(board_w * scale)
                out_h = int(board_h * scale)

                warped = cv2.warpPerspective(last_frame, H_inv, (out_w, out_h))

                cv2.imshow('Birds-Eye View', warped)
                print(f"\nBird's-eye view displayed at ~{scale} px per {square_size} mm. Press any key to continue.")
                cv2.waitKey(0)
                cv2.destroyWindow('Birds-Eye View')

                print("Finished homography estimation.\n")
                picam2.stop()
                cv2.destroyAllWindows()
                return H, H_inv

        elif key == ord('q'):
            print("Quitting Phase 2 without computing homography.")
            picam2.stop()
            cv2.destroyAllWindows()
            return None, None

    picam2.stop()
    cv2.destroyAllWindows()
    return None, None



def cameraCalibraion():
    pattern_size = (5, 7)
    square_size = 3.0 #cm?

    objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []
    imgpoints = []

    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": (1280, 720)})
    picam2.configure(config)
    picam2.start()

    print("Camera Calibration Routine!")
    print("Press 'c' to capture, 'f' to finish calibration, 'q' to quit.")
    print("You need at least 10 captures.")

    capture_count = 0
    while True:
        frame = picam2.capture_array()
        frame= cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # weird linux thing
        cv2.putText(frame, f"Captures: {capture_count}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.putText(frame, "Press 'c' to capture, 'f' to finish, 'q' to quit", (10,70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow('Calibration - Live Feed', frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_corners, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if ret_corners:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners_refined)
                capture_count += 1
                print(f"Captured image {capture_count} with valid corners!")
                detected_frame = frame.copy()
                cv2.drawChessboardCorners(detected_frame, pattern_size, corners_refined, True)
                cv2.imshow('Detected Corners', detected_frame)
                cv2.waitKey(500)
                cv2.destroyWindow('Detected Corners')
                if capture_count >= 10:
                    print("Press 'f' to finish calibration or keep capturing more images.")
            else:
                print("No corners detected. Try again.")

        elif key == ord('f'):
            if capture_count < 10:
                print("Not enough captures. Need at least 10.")
            else:
                print("Calculating camera calibration...")
                ret_cal, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
                    objpoints, imgpoints, gray.shape[::-1], None, None
                )
                print("Calibration complete!")
                print("Camera matrix (K):")
                print(cameraMatrix)
                print("Distortion coefficients:")
                print(distCoeffs)
                print(f"RMS re-projection error: {ret_cal:.4f}")
                break

        elif key == ord('q'):
            print("Quitting without calibration.")
            break

    cv2.destroyAllWindows()
    picam2.stop()

def main():
    phase2_homography_estimation(cameraMatrix, distCoeffs)
    #cameraCalibraion()


if __name__ == "__main__":
    main()


'''
Camera matrix (K):
[[700.32833455   0.         633.7861054 ]
 [  0.         703.47654591 342.84793261]
 [  0.           0.           1.        ]]
Distortion coefficients:
[[-0.30944433  0.1258339  -0.00045581 -0.00057164 -0.01383379]]
RMS re-projection error: 0.2227
'''
