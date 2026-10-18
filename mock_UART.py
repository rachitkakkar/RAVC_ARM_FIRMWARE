# Code to mock ROS UART packet

import serial
import time

with serial.Serial("/dev/cu.usbserial-XXXX", 115200) as uart:
    for _ in range(40):
        uart.write(b"M,100,0,-100\n")
        time.sleep(0.05)
    # Stop sending: outputs should become neutral after >250 ms.
    time.sleep(0.5)