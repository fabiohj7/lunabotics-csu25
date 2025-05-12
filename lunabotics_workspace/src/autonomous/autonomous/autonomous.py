import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import board
import adafruit_mcp4728
import RPi.GPIO as GPIO
import time
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
        self.excavate_distance = 2
        self.deposit_distance = 1
        # 0 = drop blade, 1 = move to deposit, 2 = lift blade, 3 = reverse into excavate
        self.auto_state = 0
        self.april_dist = 0

    def april_callback(self, msg: AprilTags):
        if len(msg.apriltags) > 0:
            self.april_dist = msg.apriltags[0].distance
    
    def auto_loop(self):
        if not self.enabled:
            return
        self.get_logger().info("Auto State: " + str(self.auto_state)) 
        match self.auto_state:
            case 0:
                # drop the blade
                publish = RoboCommand()
                publish.blade_speed = 0
                self.driver.publish(publish)
                # Give time for blade
                self.auto_state = 1
                time.sleep(2)
                self.auto_state = 2
                pass
            case 2:
                if self.april_dist > self.deposit_distance:
                    publish = RoboCommand()
                    publish.left_track_speed, publish.right_track_speed = (0.25, 0.25)
                    publish.left_track_forward, publish.right_track_forward = (True, True)
                    publish.blade_speed = 1
                    self.driver.publish(publish)
                else:
                    self.auto_state = 3
            case 3:
                # lift the blade
                publish = RoboCommand()
                publish.blade_speed = 2
                self.driver.publish(publish)
                # Give time for the blade
                self.auto_state = 4
                time.sleep(2)
                self.auto_state = 5
                pass
            case 5:
                if self.april_dist < self.excavate_distance:
                    publish = RoboCommand()
                    publish.left_track_speed, publish.right_track_speed = (0.25, 0.25)
                    publish.left_track_forward, publish.right_track_forward = (False, False)
                    publish.blade_speed = 1
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
