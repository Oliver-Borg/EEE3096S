import serial

import time

import os
while(1):
    try:
        serialPort = serial.Serial(port = "COM3", baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        break
    except:
        continue

serialString = ""                           # Used to hold data coming over UART

received = 0

t1 = 0

while(1):
    # Wait until there is data waiting in the serial buffer
    if(serialPort.in_waiting > 0):
        # Read data out of the buffer until a carraige return / new line is found
        serialString = serialPort.readline()
        # Print the contents of the serial data
        if received % 3 == 0:
            os.system('cls')
            if t1 == 0:
                t1 = time.time()
            tdelta = time.time()-t1
            if received > 0:
                print("Received {} messages in {} seconds. ({:.5f}s/m)".format(received//3, tdelta, tdelta/(received//3)))
        print(serialString.decode('Ascii'))
        received += 1
