import cv2
import glob
import os
import json
import time
import numpy as np

camera_params = None


def save_camera_params(params):
    calibration_file = "camera_params.json"
    with open(calibration_file, "w") as f:
        json.dump(params, f)


def auto_calibrate(camera_index):
    global camera_params
    num_images = 15

    old_images = glob.glob("calib_img_*.jpg")
    for img_file in old_images:
        try:
            os.remove(img_file)
            print(f"Deleted: {img_file}")
        except Exception as e:
            print(f"Failed to delete {img_file}: {e}")

    cap = cv2.VideoCapture(camera_index)
    count = 0
    last_saved_time = time.time()

    # For an 8x11 square checkerboard (10x7 inner corners)
    pattern_size = (10, 7)
    square_size = 20  # mm (1 inch per square)

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
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            objpoints.append(objp)
            imgpoints.append(corners2)

    if not imgpoints:
        print("[ERROR] Calibration failed: No checkerboards detected in saved images.")
        return

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)

    if ret:
        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]
        camera_params = [fx, fy, cx, cy]
        save_camera_params(camera_params)
        # Cleanup calibration images after saving calibration data
        for img_file in glob.glob("calib_img_*.jpg"):
            try:
                os.remove(img_file)
                print(f"[DELETE] Removed calibration image: {img_file}")
            except Exception as e:
                print(f"[WARNING] Could not delete {img_file}: {e}")

        print("[OK] Calibration successful.")
        print("Camera Matrix:")
        print(mtx)
    else:
        print("[ERROR] Calibration failed unexpectedly.")


if __name__ == "__main__":
    auto_calibrate(-1)
