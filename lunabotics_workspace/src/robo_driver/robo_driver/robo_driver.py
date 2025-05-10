import rclpy
from rclpy.node import Node
import board
import adafruit_mcp4728
import RPi.GPIO as GPIO
import time
from robo_driver_msgs.msg import RoboCommand

# GPIO numberings, not real pin numberings

# Drive Motor GPIO
motor_l_p = 6
motor_l_n = 5
motor_r_p = 13
motor_r_n = 19

# Linear Actuator GPIO
actuator_pul = 17
actuator_dir = 27

# Other parameters
pulse_duration = 500  # micro seconds (uS)
last_step_time = 0
step_current_state = False

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
        GPIO.setup(actuator_pul,GPIO.OUT)
        GPIO.setup(actuator_dir,GPIO.OUT)

    def step_actuator_motor(self, pulse_pin: int) -> None:
        global last_step_time, step_current_state
        now = time.time()
        elapsed_uS = (now - last_step_time) * 1000000

        if elapsed_uS >= pulse_duration:
            step_current_state = not step_current_state

            GPIO.output(pulse_pin, GPIO.HIGH)

            last_step_time = now
        else:
            GPIO.output(pulse_pin, GPIO.LOW)


    def driver_callback(self, msg):
        self.mcp.channel_a.normalized_value = msg.left_track_speed
        self.mcp.channel_b.normalized_value = msg.left_track_speed
        self.mcp.channel_c.normalized_value = msg.right_track_speed
        self.mcp.channel_d.normalized_value = msg.right_track_speed
        
        # Drive Motor Outputs
        GPIO.output(motor_l_p, GPIO.HIGH if msg.left_track_forward else GPIO.LOW)
        GPIO.output(motor_r_p, GPIO.HIGH if msg.right_track_forward else GPIO.LOW)
        GPIO.output(motor_l_n, GPIO.LOW if msg.left_track_forward else GPIO.HIGH)
        GPIO.output(motor_r_n, GPIO.LOW if msg.right_track_forward else GPIO.HIGH)

        # Actuator Outputs
        GPIO.output(actuator_dir, GPIO.HIGH if msg.blade_speed == 2 or msg.blade_speed == 1 else GPIO.LOW)
        self.step_actuator_motor(actuator_pul)

def main(args=None):
    rclpy.init(args=args)
    node = RoboDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
