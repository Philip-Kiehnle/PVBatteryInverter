#!/usr/bin/python3

import ctypes
import glob
import serial
import signal
import sys
import time


MONITOR_VARS_LEN = 36  # todo autodectet
HEADER_LEN = 4
CRC_LEN = 4
PACKET_LEN = MONITOR_VARS_LEN+HEADER_LEN+CRC_LEN

exit_req = False

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    global exit_req
    exit_req = True

def ser_read_exact(ser, size):
    bytesAvail = ser.inWaiting()
    while bytesAvail < size:
        bytesAvail = ser.inWaiting()
    packet = ser.read(size)
    return packet

def start_monitor_mode(ser):

    # test if monitor mode already running
    packet = ser_read_exact(ser, 2*PACKET_LEN) # search in two packets
    if search_header(packet) >=0:
        print("sync to ongoing stream...")
    else:
        ser.write('m'.encode())  # start binary mode
        magic_bytes = b'Monitoring ENABLE\n'
        while not exit_req:
            line = ser.readline()
            print(line)
            if line == magic_bytes:
                print("started monitoring")
                break

def search_header(packet):
    for i in range(len(packet)-(HEADER_LEN+CRC_LEN)) :
        if packet[i:i+(HEADER_LEN+CRC_LEN)] == b'\xef\xbe\xad\xde\x00\x00\x00\x00':
        #if packet[i:+(HEADER_LEN+CRC_LEN)] == ctypes.c_long(0xDEADBEEF00000000):
            return i
    return -1


def main():
    if len(sys.argv) != 2:
        print('Usage', sys.argv[0], '/dev/ttyUSB')
        sys.exit()

    #ser = serial.Serial(sys.argv[1], 115200, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) #Tried with and without the last 3 parameters, and also at 1Mbps, same happens.

    #signal.signal(signal.SIGINT, signal_handler)

    ser = serial.Serial(sys.argv[1], 115200, timeout=2)
    start_monitor_mode(ser)

    with open('monitor_vars.bin', 'wb') as file:
        ser.flushInput()
        ser.flushOutput()
        bytesToRead = PACKET_LEN
        print_cnt = 0
        while not exit_req:
            packet = ser_read_exact(ser, PACKET_LEN)
            header_start = search_header(packet)
            if header_start >=0:
                print_cnt += 1
                if print_cnt == 50:
                    print_cnt = 0
                    print('header_start', header_start, 'PACKET_LEN', PACKET_LEN)
                payload_start = header_start+(HEADER_LEN+CRC_LEN)
                rest = ser_read_exact(ser, header_start)
                mon_vars = packet[payload_start:] + rest
                file.write(mon_vars)
                #print(bytes)
            time.sleep(0.010)  # 50Hz/20ms -> 10ms sleep is okay

    ser.close()


if __name__=='__main__':
    main()

