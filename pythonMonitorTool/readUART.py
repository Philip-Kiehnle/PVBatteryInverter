#!/usr/bin/python3

import serial
import sys
import time

if len(sys.argv) != 2:
    print('Usage', sys.argv[0], '/dev/ttyUSB')
    sys.exit()

#ser = serial.Serial(sys.argv[1], 115200, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) #Tried with and without the last 3 parameters, and also at 1Mbps, same happens.

ser = serial.Serial(sys.argv[1], 115200, timeout=2)

def start_monitor_mode():
    ser.write('m'.encode())  # start binary mode
    magic_bytes = b'Monitoring ENABLE\n'

    while 1:
        line = ser.readline()
        print(line)
        if line == magic_bytes:
            print("started monitoring")
            break

with open('monitor_vars.bin', 'wb') as file:
    ser.flushInput()
    ser.flushOutput()
    start_monitor_mode()
    while True:
        bytesToRead = ser.inWaiting()
        if (bytesToRead >= 31) :
            bytes = ser.read(31)
            file.write(bytes)
            print(bytes)
