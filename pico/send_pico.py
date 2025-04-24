import serial
import time 

ser = serial.Serial('/dev/serial0', 115200, timeout=1)
time.sleep(2)

while True:
    cmd = input("Enter command: ")
    ser.write((cmd + "\n").encode())
    print(f"Sent: {cmd}")
