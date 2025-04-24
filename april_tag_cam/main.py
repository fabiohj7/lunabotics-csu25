import time
import cv2
import numpy as np
import glob
import os
import json
from pupil_apriltags import Detector

# === GLOBALS ===
camera_params = None
camera_index = 0  # Will hold calibrated [fx, fy, cx, cy]
tag_size = 0.165  # meters (6.5 inches)
calibration_file = "camera_params.json"

def save_camera_params(params):
    with open(calibration_file, "w") as f:
        json.dump(params, f)

def load_camera_params():
    global camera_params
    if os.path.exists(calibration_file):
        with open(calibration_file, "r") as f:
            camera_params = json.load(f)
            print("[INFO] Loaded previous camera calibration:", camera_params)
    else:
        print("[WARNING] No saved calibration found.")

def capture_and_calibrate_auto():
    global camera_params
    try:
        num_images = int(input("Enter number of calibration images to capture: ").strip())
    except ValueError:
        print("[ERROR] Invalid number. Using default of 15.")
        num_images = 15

    old_images = glob.glob("calib_img_*.jpg")
    for img_file in old_images:
        try:
            os.remove(img_file)
            print(f"[DELETE] Deleted: {img_file}")
        except Exception as e:
            print(f"[WARNING] Could not delete {img_file}: {e}")

    cap = cv2.VideoCapture(camera_index)
    count = 0
    last_saved_time = time.time()
    pattern_size = (10, 7)  # For an 8x11 square checkerboard (10x7 inner corners)
    square_size = 25.4  # mm (1 inch per square)

    print(f"[INFO] Capturing {num_images} calibration images...")

    while count < num_images:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if found:
            clean_frame = frame.copy()
            cv2.drawChessboardCorners(frame, pattern_size, corners, found)
            cv2.putText(frame, "Checkerboard Detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            current_time = time.time()
            if current_time - last_saved_time > 1.5:
                filename = f"calib_img_{count}.jpg"
                cv2.imwrite(filename, clean_frame)
                print(f"[OK] Saved: {filename}")
                count += 1
                last_saved_time = current_time
        else:
            cv2.putText(frame, "No pattern detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.putText(frame, f"Images Saved: {count}/{num_images}", (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow('Calibration Mode (Auto-Save)', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("[EXIT] Manual exit.")
            cap.release()
            cv2.destroyAllWindows()
            return

    cap.release()
    cv2.destroyAllWindows()
    print("[DONE] Done capturing. Starting calibration...")

    objp = np.zeros((np.prod(pattern_size), 3), np.float32)
    objp[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    objp *= square_size

    objpoints = []
    imgpoints = []

    images = glob.glob('calib_img_*.jpg')
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        print(f"[INFO] {fname}: checkerboard detected? {ret}")
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            objpoints.append(objp)
            imgpoints.append(corners2)

    if not imgpoints:
        print("[ERROR] Calibration failed: No checkerboards detected in saved images.")
        return

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if ret:
        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]
        camera_params = [fx, fy, cx, cy]
        save_camera_params(camera_params)
        print("[OK] Calibration successful.")
        print("Camera Matrix:")
        print(mtx)
    else:
        print("[ERROR] Calibration failed unexpectedly.")

def detect_apriltags_live():
    global camera_params
    cap = cv2.VideoCapture(camera_index)
    detector = Detector(families='tag36h11')

    print("[INFO] AprilTag detection started.")
    print("Press 'c' to re-calibrate, 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if camera_params:
            tags = detector.detect(gray, estimate_tag_pose=True, tag_size=tag_size, camera_params=camera_params)
        else:
            tags = detector.detect(gray)

        for tag in tags:
            corners = tag.corners.astype(int)
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            center = tuple(tag.center.astype(int))
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"ID: {tag.tag_id}", (center[0] - 10, center[1] - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            if camera_params and hasattr(tag, 'pose_t') and tag.pose_t is not None:
                try:
                    distance = np.linalg.norm(tag.pose_t)
                    cv2.putText(frame, f"Dist: {distance:.2f} m", (center[0] - 10, center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

                    # Calculate yaw and pitch from rotation matrix
                    R = tag.pose_R
                    yaw_rad = np.arctan2(R[1, 0], R[0, 0])
                    pitch_rad = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
                    yaw_deg = np.degrees(yaw_rad)
                    pitch_deg = np.degrees(pitch_rad)
                    cv2.putText(frame, f"Yaw: {yaw_deg:.1f} deg", (center[0] - 10, center[1] + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    cv2.putText(frame, f"Pitch: {pitch_deg:.1f} deg", (center[0] - 10, center[1] + 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                except Exception as e:
                    print(f"[WARNING] Pose data error: {e}")
                    cv2.putText(frame, "Dist: unavailable", (center[0] - 10, center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "Dist: unavailable", (center[0] - 10, center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            if camera_params and hasattr(tag, 'pose_t') and tag.pose_t is not None:
                try:
                    distance = np.linalg.norm(tag.pose_t)
                    cv2.putText(frame, f"Dist: {distance:.2f} m", (center[0] - 10, center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                except Exception as e:
                    print(f"[WARNING] Pose data error: {e}")
                    cv2.putText(frame, "Dist: unavailable", (center[0] - 10, center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "Dist: unavailable", (center[0] - 10, center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.putText(frame, "Press 'c' to calibrate | 'q' to quit", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 255), 2)

        cv2.imshow("AprilTag Detection (Live Menu)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            cap.release()
            cv2.destroyAllWindows()
            capture_and_calibrate_auto()
            load_camera_params()
            cap = cv2.VideoCapture(camera_index)
            print("[RELOAD] Returned to AprilTag detection.")

    cap.release()
    cv2.destroyAllWindows()

def select_camera_index():
    global camera_index
    try:
        camera_index = int(input("Enter camera index (default is 0): ") or 0)
    except ValueError:
        print("[WARNING] Invalid input. Using default camera index 0.")
        camera_index = 0

def main_menu_loop():
    print("[MENU] Menu: Press a key")
    print("   [c] Calibrate")
    print("   [a] AprilTag detection")
    print("   [q] Quit")

    load_camera_params()

    while True:
        key = input("\nEnter option [c/a/q]: ").strip().lower()
        if key == 'c':
            capture_and_calibrate_auto()
            load_camera_params()
        elif key == 'a':
            detect_apriltags_live()
        elif key == 'q':
            print("[QUIT] Exiting.")
            break
        else:
            print("Invalid input. Press 'c', 'a', or 'q'.")

if __name__ == "__main__":
    select_camera_index()
    main_menu_loop()
