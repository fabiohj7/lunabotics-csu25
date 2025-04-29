import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
#from distance_detector import DistanceDetector
from pupil_apriltags import Detector, Detection
from crunch_tags_msgs.msg import AprilTag, AprilTags
CAMERA_PARAMS = [321.57651609166743, 441.58510187937816, 244.93956987378624, 201.58593543666143]

class CrunchTags(Node):
    def __init__(self):
        super().__init__('crunch_tags')
        self.create_subscription(Image, '/image_raw', self.cam_callback, 10)
        self.get_logger().info('Listening to /image_raw topic for camera stream...')
        self.get_logger().info('Camera calibration DOES NOT happen automatically!')
        self.get_logger().info('If calibration is needed, run apriltags/calibrate_camera.py and copy the data in camera_params.json to the array at the top of this code.')
        self.get_logger().info('AprilTag square sizes are hardcoded to 25mm, change this in code if needed.')
        #self.april_publish = self.create_publisher(Detection, '/apriltags', 10)
        self.cv_bridge = CvBridge()
        self.tag_detector = Detector(families='tag36h11')  
        self.tag_publisher = self.create_publisher(AprilTags, '/apriltags', 10)
        self.visualize = True
        if self.visualize:
            self.visualizer = self.create_publisher(Image, '/apriltag_visualization', 10)
            self.get_logger().info('Visualizing the AprilTags on /apriltag_visualization')

    def cam_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
        tags = self.tag_detector.detect(cv_image, estimate_tag_pose=True, tag_size=0.125, camera_params=CAMERA_PARAMS)
        # for tag in tags:
        #     self.april_publish.publish(tag)
        aprils = AprilTags()
        for tag in tags:
            april = AprilTag()
            april.tag_family = tag.tag_family.decode()
            april.tag_id = tag.tag_id
            april.hamming = tag.hamming
            april.decision_margin = tag.decision_margin
            april.homography = np.reshape(tag.homography, -1).tolist()
            april.center = np.reshape(tag.center, -1).tolist()
            april.corners = np.reshape(tag.corners, -1).tolist()
            april.pose_r = np.reshape(tag.pose_R, -1).tolist()
            april.pose_t = np.reshape(tag.pose_t, -1).tolist()
            april.pose_err = tag.pose_err
            april.distance = np.linalg.norm(tag.pose_t)
            aprils.apriltags.append(april)
        self.tag_publisher.publish(aprils)
        if self.visualize:
            for tag in tags:
                corners = tag.corners.astype(int)
                for i in range(4):
                    pt1 = tuple(corners[i])
                    pt2 = tuple(corners[(i + 1) % 4])
                    cv2.line(cv_image, pt1, pt2, (0, 255, 0), 2)

                center = tuple(tag.center.astype(int))
                cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
                cv2.putText(cv_image, f"ID: {tag.tag_id}", (center[0] - 10, center[1] - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            publish = self.cv_bridge.cv2_to_imgmsg(cv_image)
            self.visualizer.publish(publish)

def main(args=None):
    rclpy.init(args=args)
    node = CrunchTags()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
