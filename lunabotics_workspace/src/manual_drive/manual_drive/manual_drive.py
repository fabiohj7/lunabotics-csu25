import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import board
import adafruit_mcp4728
import RPi.GPIO as GPIO
from robo_driver_msgs.msg import RoboCommand

# GPIO numberings, not real pin numberings
motor_l_p = 5
motor_r_p = 6
motor_l_n = 13
motor_r_n = 19

class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        self.create_subscription(ByteMultiArray, "/networking_incoming", self.net_callback, 10)
        self.get_logger().info('Listening to network for commands')
        self.driver = self.create_publisher(RoboCommand, "/robo_driver", 10)
        self.get_logger().info('Connected to Robo Driver')
        self.enabled = True

    def net_callback(self, msg):
        nums = [int.from_bytes(x, 'little') for x in msg.data]

        if len(nums) == 1:
            match nums[0]:
                case 0x20:
                    self.enabled = True
                case 0x40:
                    self.enabled = False
        elif self.enabled:
            speed1 = abs(nums[0] - 62) / 63
            speed2 = abs(nums[1] - 62) / 63
            speed1 = speed1 if speed1 > 0.25 else 0
            speed2 = speed2 if speed2 > 0.25 else 0
            nums[2] = int(msg.data[2].decode('ascii'))
            # Jarod said "full speed is too fast wah wah wah"
            speed1 /= 2
            speed2 /= 2

            publish = RoboCommand()
            publish.left_track_speed = speed1
            publish.right_track_speed = speed2
            publish.left_track_forward = nums[0]>62
            publish.right_track_forward = nums[1]>62
            publish.blade_speed = nums[2]

            self.driver.publish(publish)

            self.get_logger().info(str(nums) + ' ' + str([speed1, speed2]) + ' ' + str([publish.left_track_forward, publish.right_track_forward]))


def main(args=None):
    rclpy.init(args=args)
    node = ManualControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
