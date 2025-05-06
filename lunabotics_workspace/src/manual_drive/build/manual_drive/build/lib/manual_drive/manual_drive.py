import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import board
import adafruit_mcp4728
import RPi.GPIO as GPIO

forward_pin_1 = 1
forward_pin_2 = 7
backward_pin_1 = 8
backward_pin_2 = 25


class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        self.create_subscription(ByteMultiArray, "/networking_incoming", self.net_callback, 10)
        self.get_logger().info('Listening to network for commands')
        self.mcp = adafruit_mcp4728.MCP4728(board.I2C())
        self.get_logger().info('Connected to MCP4728')
        self.get_logger().info('here')
        #GPIO.setmode(GPIO.BCM)
        #GPIO.setup(forward_pin_1,GPIO.OUT)
        #GPIO.setup(forward_pin_2,GPIO.OUT)
        #GPIO.setup(backward_pin_1,GPIO.OUT)
        #GPIO.setup(backward_pin_2,GPIO.OUT)
        

    def net_callback(self, msg):
        nums = [int.from_bytes(x, 'little') for x in msg.data]
        #self.get_logger().info(str(nums))
        speed1 = abs(nums[0] - 62) * 1000
        speed2 = abs(nums[1] - 62) * 1000
        #fw1 = (GPIO.LOW if nums[0]<62 else GPIO.HIGH)
        #bw1 = (GPIO.HIGH if nums[0]<62 else GPIO.LOW)
        #fw2 = (GPIO.LOW if nums[1]<62 else GPIO.HIGH)
        #bw2 = (GPIO.HIGH if nums[1]<62 else GPIO.LOW)
        self.get_logger().info('here')
        self.mcp.channel_a.value = speed1
        self.mcp.channel_b.value = speed1
        self.mcp.channel_c.value = speed2
        self.mcp.channel_d.value = speed2
        
        #GPIO.output(forward_pin_1,fw1)
        #GPIO.output(backward_pin_1,bw1)
        #GPIO.output(forward_pin_2,fw2)
        #GPIO.output(backward_pin_2,bw2)

def main(args=None):
    rclpy.init(args=args)
    node = ManualControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
