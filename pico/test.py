import machine
import time 

led = machine.Pin(2, machine.Pin.OUT)


while True:
    print("ON")
    led.value(1)
    time.sleep(1)
    led.value(0)
    time.sleep(1)
