import cv2
import numpy as np
from picamera2 import Picamera2

def main():
    pattern_size = (5, 7)
    square_size = 3.0

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
        # Manually flip frame both horizontally and vertically
        frame = cv2.flip(frame, -1)

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
