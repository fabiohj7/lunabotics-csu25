import cv2
import os
import json
import time
import numpy as np
from pupil_apriltags import Detector

camera_params = None
tag_size = .125
cap = None


class DistanceDetector():

    def __init__(self, camera_index, tag_size, param_file_path):
        self.camera = cv2.VideoCapture(camera_index)
        self.tag_size = tag_size
        if os.path.exists(param_file_path):
            with open(param_file_path, 'r') as f:
                self.camera_params = json.load(f)
        else:
            print(f'''UNABLE TO OPEN FILE AT PATH "{
                  param_file_path}" THIS DETECTOR WILL FAIL IF USED''')

    def detect_tags(self, visualize=False):
        tag_detector = Detector(families='tag36h11')
        id, distance = (None, None)

        print("Attempting to detect an April Tag")

        # Take an image
        ret, frame = self.camera.read()
        if not ret:
            print("failed to capture image, aborting this attempt")
            return

        # convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # find all tags in the image
        tags = tag_detector.detect(
            gray, estimate_tag_pose=True, tag_size=self.tag_size, camera_params=self.camera_params)

        # iterate over all tags found and print their distances
        for tag in tags:
            if self.camera_params and hasattr(tag, 'pose_t') and tag.pose_t is not None:
                try:
                    distance = np.linalg.norm(tag.pose_t)
                    id = tag.tag_id
                    print(f'Tag with id: {id} found {distance}m away')
                except Exception as e:
                    print(f"[WARNING] Pose data error: {e}")
            else:
                print("Distance unavailable")

        if visualize:
            self.visualize_tags(tags, frame)

        if id and distance:
            return ((id, distance))

    def visualize_tags(self, tags, frame):
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

        cv2.imshow("AprilTag Detection (Live Menu)", frame)
        cv2.waitKey(1)

    def end_resources(self):
        cv2.destroyAllWindows()
        self.camera.release()


if __name__ == '__main__':
    dd = DistanceDetector(0, .125, 'camera_params.json')
    while True:
        dd.detect_tags(True)
        time.sleep(1)
