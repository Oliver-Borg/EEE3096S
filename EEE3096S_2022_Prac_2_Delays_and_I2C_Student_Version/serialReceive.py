import re
import serial

import time

import os


timeout = 10

t1 = time.time()


while(time.time()-t1 < timeout):
    try:
        serialPort = serial.Serial(port = "COM3", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
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

while(1):
    # Wait until there is data waiting in the serial buffer
    if(serialPort.in_waiting > 0):
        # Read data out of the buffer until a carraige return / new line is found
        serialString = str(serialPort.readline().decode('Ascii'))
        print(serialString)
        if serialString.replace('\x00', '').strip() == "Start":
            break

t1 = time.time()
while(1):
    # Wait until there is data waiting in the serial buffer
    if(serialPort.in_waiting > 0):
        # Read data out of the buffer until a carraige return / new line is found
        serialString = serialPort.readline().decode('Ascii').replace('\x00', '').strip()
        if serialString == "Start":
            received = 0
            full_received = 0
            t1 = time.time()
            continue
        # Print the contents of the serial data
        if received % 3 == 0 and received > 0:
            full_received += 1
            os.system('cls')       
            tdelta = time.time()-t1
            if full_received > 0:
                print("Received {} messages in {} seconds. ({:.5f}s/m)".format(full_received, tdelta, tdelta/full_received))
        print(serialString)
        if(serialString != ""):
            received += 1
