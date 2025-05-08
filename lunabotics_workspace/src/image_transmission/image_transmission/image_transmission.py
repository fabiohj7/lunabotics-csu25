import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import socket

class ImageTransmission(Node):
    def __init__(self):
        super().__init__('image_transmission')
        self.create_subscription(Image, '/camera_node/image_raw', self.cam_callback, 10)
        #self.debug = self.create_publisher(Image, '/debug', 10)
        self.get_logger().info('Listening for camera stream...')
        self.network = self.create_publisher(ByteMultiArray, '/networking_outgoing', 10)
        self.create_timer(1/3, self.image_timer)
        self.bridge = CvBridge()
        self.curr_msg = None

    def image_timer(self):
        if self.curr_msg == None:
            return
        cv_img = self.bridge.imgmsg_to_cv2(self.curr_msg, 'mono8')
        downscaled = cv2.resize(cv_img, (100, 100))
        result, compressed = cv2.imencode(".jpg", downscaled, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
        #self.debug.publish(self.bridge.cv2_to_imgmsg(downscaled))
        data = compressed.tobytes()
        publish = ByteMultiArray()
        publish.data = [bytes([x]) for x in data]
        self.network.publish(publish)
        
    def cam_callback(self, msg: Image):
        self.curr_msg = msg
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ImageTransmission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
