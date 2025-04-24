# UART communication

To make UART work there are were some configurations that needed to be changed

1. Allow UART on the raspberry pi

Change/Add this file: "/boot/firmware/config.txt"

"enable_uart=1"
"dtoverlay=uart1"

2. Disable serial console, I don't know if this is necessary but it is here just in case

`sudo systemctl stop serial-getty@ttyS0.service`
`sudo systemctl disable serial-getty@ttyS0.service`

reboot to apply changes


