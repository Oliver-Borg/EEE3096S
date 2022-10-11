import re
import serial

import time

import os

import matplotlib.pyplot as plt

timeout = 10

t1 = time.time()


while(time.time()-t1 < timeout):
    try:
        serialPort = serial.Serial(port = "COM5", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        break
    except:
        pass
    try:
        serialPort = serial.Serial(port = "/dev/ttyUSB0", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        break
    except:
        pass

if(time.time()-t1 >= timeout):
     serialPort = serial.Serial(port = "/dev/ttyUSB0", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

serialString = ""                           # Used to hold data coming over UART

received = 0

full_received = 0

graph = plt.plot([], [])
samples = []
wanted = 512

while(1):
    # Wait until there is data waiting in the serial buffer
    if(serialPort.in_waiting > 0):
        # Read data out of the buffer until a carraige return / new line is found
        serialString = str(serialPort.readline().decode('Ascii')).replace('\x00', '').strip()
        
        if "Mode" in serialString:
            print(serialString)
            samples = []
        elif serialString != "":
            try:
                if int(serialString) >= 1024:
                    continue
            except:
                continue
            if len(samples) < wanted:
                samples.append(int(serialString))
                print(serialString)
            else:
                graph = plt.plot(list(range(0, len(samples))), samples)
                plt.draw()
                plt.show()
                samples = []
                
                
