import smbus
import time

# I2C address of the Arduino slave
ARDUINO_ADDR = 0x20

# Initialize I2C (bus 1 is the default on Raspberry Pi)
bus = smbus.SMBus(1)

# Let I2C settle
time.sleep(1)

def send_data(data):
    bus.write_byte(ARDUINO_ADDR, ord(data))  # Send a single byte to the Arduino
    print(f"Sent: {data}")

def main():
    while True:
        # Send data to the Arduino
        send_data('A')  # Send 'A'
        time.sleep(1)
        send_data('B')  # Send 'B'
        time.sleep(1)

if __name__ == "__main__":
    main()
