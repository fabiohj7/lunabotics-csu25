import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import board
import adafruit_mcp4728
import RPi.GPIO as GPIO
from robo_driver_msgs.msg import RoboCommand
from crunch_tags_msgs.msg import AprilTags

# GPIO numberings, not real pin numberings
motor_l_p = 5
motor_r_p = 6
motor_l_n = 13
motor_r_n = 19

### TODO: Add mode switching to here AND manual_drive, spec is in discord

class Autonomous(Node):
    def __init__(self):
        super().__init__('autonomous')
        self.create_subscription(ByteMultiArray, "/networking_incoming", self.net_callback, 10)
        self.get_logger().info('Listening to network for commands')
        self.driver = self.create_publisher(RoboCommand, "/robo_driver", 10)
        self.get_logger().info('Connected to Robo Driver')
        self.create_subscription(AprilTags, '/apriltags', self.april_callback, 10)
        self.create_timer(0.25, self.auto_loop)
        self.enabled = False
        self.excavate_distance = 10
        self.deposit_distance = 3
        # 0 = drop blade, 1 = move to deposit, 2 = lift blade, 3 = reverse into excavate
        self.auto_state = 0
        self.april_dist = 0

    def april_callback(self, msg: AprilTags):
        if len(msg.apriltags) > 0:
            april = msg.apriltags[0]
    
    def auto_loop(self):
        match self.auto_state:
            case 0:
                # drop the blade
                pass
            case 1:
                if self.april_dist > self.deposit_distance:
                    publish = RoboCommand()
                    publish.left_track_speed, publish.right_track_speed = 0.25
                    publish.left_track_forward, publish.right_track_forward = True
                    self.driver.publish(publish)
                else:
                    self.auto_state = 2
            case 2:
                # lift the blade
                pass
            case 3:
                if self.april_dist < self.excavate_distance:
                    publish = RoboCommand()
                    publish.left_track_speed, publish.right_track_speed = 0.25
                    publish.left_track_forward, publish.right_track_forward = False
                    self.driver.publish(publish)
                else:
                    self.auto_state = 0


    def net_callback(self, msg):
        nums = [int.from_bytes(x, 'little') for x in msg.data]
        if len(nums) == 1:
            match nums[0]:
                case 0x20:
                    self.enabled = False
                case 0x40:
                    self.enabled = True
                case 0x60:
                    self.deposit_distance = self.april_dist
                case 0x80:
                    self.excavate_distance = self.april_dist

def main(args=None):
    rclpy.init(args=args)
    node = Autonomous()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
