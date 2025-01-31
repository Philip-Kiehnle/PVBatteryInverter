#!/usr/bin/python3

import ctypes
import serial
#import signal
import matplotlib.pyplot as plt
import sys
import struct
import numpy as np


d = np.dtype([
('duty', 'u2'),
('v_dc_modulator_100mV', 'u2'),
('v_ac_100mV', 'i2'),
('i_ac_10mA', 'i2'),
('i_ac_ref_amp_10mA', 'i2'),
])

# b int8
# B uint8
# h int16
# H uint16
# i uint32
# I uint32
# f float32
fast_monitor_vars_t = "HHhhh"
struct_size = struct.calcsize(fast_monitor_vars_t)

for name in d.names:
    print("{0:20} {1}".format(name,  d.fields[name]))

if (d.itemsize != struct_size) :
    print('struct missmatch: d.itemsize = ', d.itemsize, '!=', struct_size, 'struct_size')
    sys.exit()

FAST_MON_FRAMES = 400
FAST_MON_BYTES = FAST_MON_FRAMES*struct_size

vars = np.zeros(FAST_MON_FRAMES, dtype=d)
x = np.arange(FAST_MON_FRAMES)

def ser_read_exact(ser, size):
    bytesAvail = ser.inWaiting()
    while bytesAvail < size:
        bytesAvail = ser.inWaiting()
    packet = ser.read(size)
    return packet

def start_fast_monitor_mode(ser):
    if ser.read(1):
        ser.write('d'.encode())  # stop active monitor mode
        ser.readall()
    ser.write('f'.encode())  # start fast monitor mode
    magic_bytes = b'Fast monitor TRIG\n'
    line = ser.readline()
    print(line)
    if line == magic_bytes:
        print("started monitoring")

fig, ax1 = plt.subplots()
ax1.grid(axis='x')
curr_ax = ax1

def plot_ax(name, gain = 1.0):

    y = np.clip(gain*vars[name], -5000, 5000)  # clip UART errors
    curr_ax.plot(x, y, label=name)


if len(sys.argv) != 2:
    print('Usage', sys.argv[0], '/dev/ttyUSB')
    sys.exit()

#signal.signal(signal.SIGINT, signal_handler)

if sys.argv[1].find('/dev/tty') != -1:
    ser = serial.Serial(sys.argv[1], 115200, timeout=1)
    start_fast_monitor_mode(ser)

    packet = ser_read_exact(ser, FAST_MON_BYTES)
    ser.close()

    with open('monitor_vars_fast.bin', 'wb') as file:
        file.write(packet)

    for i in range(FAST_MON_FRAMES):
        vars[i] = struct.unpack_from(fast_monitor_vars_t, packet[i*struct_size:(i+1)*struct_size])

else:
    with open(sys.argv[1], 'rb') as file:
        for i in range(FAST_MON_FRAMES):
            vars[i] = struct.unpack_from(fast_monitor_vars_t, file.read(struct_size))

value = np.clip(0.1*vars['v_ac_100mV'], -5000, 5000)
print(f'v_ac_100mV min={min(value):.3f} avg={sum(value)/FAST_MON_FRAMES:.3f} max={max(value):.3f} ')
value = np.clip(0.01*vars['i_ac_10mA'], -5000, 5000)
print(f'i_ac_10mA  min={min(value):.3f} avg={sum(value)/FAST_MON_FRAMES:.3f} max={max(value):.3f} ')
value = np.clip(0.01*vars['i_ac_ref_amp_10mA'], -5000, 5000)
print(f'i_ac_ref_amp_10mA  min={min(value):.3f} avg={sum(value)/FAST_MON_FRAMES:.3f} max={max(value):.3f} ')

plot_ax('duty', 0.1)
plot_ax('v_dc_modulator_100mV', 0.1)
plot_ax('v_ac_100mV', 0.1)


ax2 = ax1.twinx()
ax2._get_lines.prop_cycler = ax1._get_lines.prop_cycler
curr_ax = ax2

plot_ax('i_ac_10mA', 0.01)
plot_ax('i_ac_ref_amp_10mA', 0.01)

ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

#leg = interactive_legend()

#plt.savefig('timeplot.png', dpi=300)
plt.show()


