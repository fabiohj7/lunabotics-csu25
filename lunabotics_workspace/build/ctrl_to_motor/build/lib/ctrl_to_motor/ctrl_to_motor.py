import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import serial
import time
import smbus

LEFT_TRACK_SLAVE = 0x20
RIGHT_TRACK_SLAVE = 0x40

class CtrlToMotor(Node):
    def __init__(self):
        super().__init__('ctrl_to_motor')
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info('Listening to /joy topic for button presses...')
        self.i2c = smbus.SMBus(1)
        self.get_logger().info('Initialized I2C master')

    def joy_callback(self, msg):
        self.get_logger().info("Tankdrive: (" + str(msg.axes[1]) + ", " + str(msg.axes[3]) + ")")
        self.i2c.write_byte(LEFT_TRACK_SLAVE, round(((msg.axes[1] + 1) / 2) * 255 - 128))
        self.i2c.write_byte(RIGHT_TRACK_SLAVE, round(((msg.axes[3] + 1) / 2) * 255 - 128))

def main(args=None):
    rclpy.init(args=args)
    node = CtrlToMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
