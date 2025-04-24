import machine
import time

uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))

led = machine.Pin(2, machine.Pin.OUT)

l_joystick = [0,0]

COMMANDS = ["ON", "OFF"]

def readline_uart(uart):
    line = ""
    while True:
        if uart.any():
            ch = uart.read(1)
            if ch:
                ch = ch.decode()
                if ch == '/':
                    break
                line += ch
    return line


while True:
    print("Left Joystick: " + str(l_joystick))
    if uart.any():
        data = readline_uart(uart)
        if data:
            cmd = data.strip()
            #print("Command: ", cmd)
            try:
                if cmd.startswith("LJV "):
                    l_joystick[1] = float(cmd[4:])
                elif cmd.startswith("LJH "):
                    l_joystick[0] = float(cmd[4:])
            except ValueError:
                print("Failed to parse command:", cmd)

