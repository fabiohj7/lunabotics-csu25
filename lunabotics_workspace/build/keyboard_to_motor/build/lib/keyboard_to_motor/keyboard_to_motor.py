import rclpy
from rclpy.node import Node

import serial
import time
from pynput import keyboard

class KeyboardToMotor(Node):
    def __init__(self):
        super().__init__('keyboard_to_motor')
        self.keyboard = keyboard.Listener(on_press=self.key_down, on_release=self.key_up)
        self.keyboard.start()
        self.keyboard.join()
        self.get_logger().info('Initialized keyboard hook')
        self.ser = serial.Serial('/dev/serial0', 115200, timeout=1)
        self.get_logger().info('Initialized connection with Pico')
        
    def key_down(self, key):
        self.get_logger().info('Key pressed: ' + key)
    
    def key_up(self, key):
        self.get_logger().info('Key released: ' + key)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
