import rclpy
from rclpy.node import Node
import board
import adafruit_mcp4728
import RPi.GPIO as GPIO
from robo_driver_msgs.msg import RoboCommand

# GPIO numberings, not real pin numberings
motor_l_p = 5
motor_r_p = 6
motor_l_n = 13
motor_r_n = 19

class RoboDriver(Node):
    def __init__(self):
        super().__init__('robo_driver')
        self.create_subscription(RoboCommand, "/robo_driver", self.driver_callback, 10)
        self.mcp = adafruit_mcp4728.MCP4728(board.I2C())
        self.get_logger().info('Connected to MCP4728')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(motor_l_p,GPIO.OUT)
        GPIO.setup(motor_r_p,GPIO.OUT)
        GPIO.setup(motor_l_n,GPIO.OUT)
        GPIO.setup(motor_r_n,GPIO.OUT)
        

    def driver_callback(self, msg):
        self.mcp.channel_a.normalized_value = msg.left_track_speed
        self.mcp.channel_b.normalized_value = msg.left_track_speed
        self.mcp.channel_c.normalized_value = msg.right_track_speed
        self.mcp.channel_d.normalized_value = msg.right_track_speed
        
        GPIO.output(motor_l_p, GPIO.HIGH if msg.left_track_forward else GPIO.LOW)
        GPIO.output(motor_r_p, GPIO.HIGH if msg.right_track_forward else GPIO.LOW)
        GPIO.output(motor_l_n, GPIO.LOW if msg.left_track_forward else GPIO.HIGH)
        GPIO.output(motor_r_n, GPIO.LOW if msg.right_track_forward else GPIO.HIGH)


def main(args=None):
    rclpy.init(args=args)
    node = RoboDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
