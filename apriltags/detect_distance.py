import cv2
import os
import json
import time
import numpy as np
from pupil_apriltags import Detector

camera_params = None
tag_size = .125
cap = None


def load_camera(camera_index):
    global camera_params
    global cap
    calibration_file = "camera_params.json"
    if os.path.exists(calibration_file):
        with open(calibration_file, "r") as f:
            camera_params = json.load(f)
            print("[INFO] Loaded previous camera calibration:")
    else:
        print("[ABORTING] No saved calibration found.")
        exit()
    cap = cv2.VideoCapture(camera_index)


def detect_on_call():
    global camera_params
    global cap

    detector = Detector(families='tag36h11')
    id, distance = (None, None)

    print("Attempting to detect an April Tag")

    # Take an image
    ret, frame = cap.read()
    if not ret:
        print("failed to capture image")
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    tags = detector.detect(
        gray, estimate_tag_pose=True, tag_size=tag_size, camera_params=camera_params)

    for tag in tags:
        if camera_params and hasattr(tag, 'pose_t') and tag.pose_t is not None:
            try:
                distance = np.linalg.norm(tag.pose_t)
                id = tag.tag_id
                print(f'Tag with id: {id} found {distance}m away')
            except Exception as e:
                print(f"[WARNING] Pose data error: {e}")
        else:
            print("Distance unavailable")

    if id and distance:
        return ((id, distance))


def detect_continuous():

    global camera_params
    global cap

    detector = Detector(families='tag36h11')

    print("[INFO] AprilTag detection started.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = detector.detect(
            gray, estimate_tag_pose=True, tag_size=tag_size, camera_params=camera_params)

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
                    print(distance)
                except Exception as e:
                    print(f"[WARNING] Pose data error: {e}")
            else:
                print("Distance unavailable")

        cv2.imshow("AprilTag Detection (Live Menu)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    load_camera(0)
    # detect_continuous(0)
    # detect_apriltags(0)
    while True:
        detect_on_call()
        time.sleep(1)
