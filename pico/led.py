import machine
import time

uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))

led = machine.Pin(2, machine.Pin.OUT)

l_joystick = (0,0)

COMMANDS = ["ON", "OFF"]

def readline_uart(uart):
    ret = ""
    while "\n" not in ret:
        if uart.any():
            msg = uart.readline()
            print(bytes(msg, 'utf-8'))
            ret += msg.decode()
    return ret

while True:
    if uart.any():
        data = readline_uart(uart)
        if data:
            cmd = data.strip()
            print("Command: ", cmd)
	    
            if cmd.startswith("LJV "):
            	cmd = cmd.replace("LJV ", "")
            	l_joystick[1] = float(cmd)

    time.sleep(0.1)

