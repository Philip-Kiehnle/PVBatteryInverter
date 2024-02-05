#!/usr/bin/python3
#
# script plots samples from file
#  based on c struct
#
# ^
# |                 +                   +                    +
# |               +   +               +   +                +   +
# |              +     +             +     +              +     +
# |             +       +           +       +            +       +
# |+++++++++++++---------+---------+---------+----------+---------+-----> t
# |                       +        +           +       +           +
# |                        +      +             +     +             +
# |                          +   +               +   +
# |                            +                   +
#         
# <------------>        
samples_skip = 0
#
#               <--------------------->        
samples_to_consume = 99e6  # set to -1 to evaluate all samples of file(excluding samples_skip)
#
# <---------------------------------------------------------------->        
#                          samples_in_file
#

import matplotlib.pyplot as plt
import os
import struct
import numpy as np

import sys
#sys.path.append('../')
from InteractiveLegend import *

def align_yaxis(ax1, ax2):
    ax1_ylims = ax1.axes.get_ylim()           # Find y-axis limits set by the plotter
    ax1_yratio = ax1_ylims[0] / ax1_ylims[1]  # Calculate ratio of lowest limit to highest limit

    ax2_ylims = ax2.axes.get_ylim()           # Find y-axis limits set by the plotter
    ax2_yratio = ax2_ylims[0] / ax2_ylims[1]  # Calculate ratio of lowest limit to highest limit


    # If the plot limits ratio of plot 1 is smaller than plot 2, the first data set has
    # a wider range range than the second data set. Calculate a new low limit for the
    # second data set to obtain a similar ratio to the first data set.
    # Else, do it the other way around

    if ax1_yratio < ax2_yratio: 
        ax2.set_ylim(bottom = ax2_ylims[1]*ax1_yratio)
    else:
        ax1.set_ylim(bottom = ax1_ylims[1]*ax2_yratio)

   
if len(sys.argv)!=2 :    
    print('Please specify filename.')
    sys.exit(0)

path = sys.argv[1]
filename = os.path.basename(path)
fileinfo = os.stat(path)

#modes_colorstyle = ['y', 'c', 'r', 'g', 'orange', 'b']  # Tektronix channel colors
modes_linestyle = ['-', '--', '-.', ':']


d = np.dtype([
('id', 'u2'),
('sys_mode', 'u1'),
('sys_errcode', 'i1'),
('stateDC', 'u1'),
('dcdc_mode', 'u1'),
('dutyDC_HS', 'u2'),
('p_dc_filt50Hz', 'f4'),
('v_dc_filt50Hz', 'f4'),
('stateAC', 'u2'),
('f_ac_10mHz', 'i2'),
('v_ac_rms_100mV', 'i2'),
('i_ac_amp_10mA', 'i2'),
('p_ac_filt50Hz', 'i2'),
('p_ac_ref', 'i2'),
('bat_p', 'i2'),
('bat_soc', 'u1'),
('bat_soc_kalman', 'u1'),
('v_dc_FBboost_sincfilt_100mV', 'u2'),
('v_dc_FBgrid_sincfilt_100mV', 'u2')
])

# b int8
# B uint8
# h int16
# H uint16
# i uint32
# I uint32
# f float32
monitor_vars_t = "HBbBBHffHhhhhhhBBHH"
struct_size = struct.calcsize(monitor_vars_t)
print("struct_size", struct_size)

for name in d.names:
    print("{0:20} {1}".format(name,  d.fields[name]))

if (d.itemsize != struct_size) :
    print('struct missmatch: d.itemsize = ', d.itemsize, '!=', struct_size, 'struct_size')
    sys.exit()

samples_in_file = int(fileinfo.st_size/struct_size)
samples_to_consume = min(samples_in_file,samples_to_consume)

filehandle = open(path, "rb")
#filehandle.seek(2*samples_skip)


vars = np.zeros(samples_to_consume, dtype=d)
#var_stateDC = np.zeros(samples_to_consume, dtype=int)

prev_id = 0
id_errors = 0

for i in range(samples_to_consume):

	vars[i] = struct.unpack_from(monitor_vars_t,filehandle.read(struct_size))

	id = vars['id'][i]
	if id != prev_id+1 and i != 0:
		print(id, ' != prev_id+1 at pos', i)
		id_errors += 1
	prev_id = id

#exit()
fig, ax1 = plt.subplots()
ax1.grid(axis='x')

#ax1.set_ylabel(r'v$_\mathrm{pv}$'+' (V), ' + r'p$_\mathrm{pv}$'+' (W)')

x_is_id = False

if id_errors > 5:  # too many id errors distort x-axis
    x_scale = 1
    print('Too many id errors distort x-axis -> using id and scale=', x_scale)
    x = vars['id']-vars['id'][0]
    x_is_id = True
    id_offset = 0
    x_prev = 0
    for i in range(len(x)):
        x[i] += id_offset
        if x[i] < x_prev:
            id_offset += 65536  # 2^16
            x[i] += id_offset
else:
    #x = range(samples_to_consume)
    x = np.arange(samples_to_consume)
    #x_scale = 1
    x_scale = 1/50

if x_scale == 1:
    ax1.set_xlabel(r'$x$' + ' in samples')
else:
    ax1.set_xlabel(r'$x$' + ' in seconds')
x = x * x_scale

curr_ax = ax1

def plot_ax(name, gain = 1.0, vert_lines = False, color='b'):

    y = np.clip(gain*vars[name], -5000, 5000)  # clip UART errors


    if vert_lines:
        curr_ax.plot(x, y, color=color, label=filename + ' ' + name)
        y_prev = y[0]
        varargs = {}
        for idx, y_single in enumerate(y):
            if y_single != y_prev:
#                plt.axvline(x=idx)
                if x_is_id:
                    x_loc = x[idx]
                else:
                    x_loc = x_scale*idx
                    if y_single >=0 and y_single <= 3:
                        varargs = {"linestyle":modes_linestyle[int(y_single)]}

                plt.axvline(x=x_loc, color=color, label='state:' + str(y_single), **varargs)
            y_prev = y_single
    else:
        curr_ax.plot(x, y, label=filename + ' ' + name)


#plot_ax('id')
plot_ax('sys_errcode', vert_lines=True, color='r')
plot_ax('dutyDC_HS')
plot_ax('stateDC', vert_lines=True, color='b')
plot_ax('dcdc_mode', vert_lines=True, color='y')
plot_ax('stateAC', vert_lines=True, color='g')



ax2 = ax1.twinx()
ax2._get_lines.prop_cycler = ax1._get_lines.prop_cycler
curr_ax = ax2

plot_ax('p_dc_filt50Hz')
#plot_ax('v_pv_filt50Hz')
plot_ax('v_dc_filt50Hz')

#plot_ax('f_ac_10mHz', 0.01)
plot_ax('v_ac_rms_100mV', 0.1)
plot_ax('i_ac_amp_10mA', 0.01)

#plot_ax('v_dc_FBboost_sincfilt_100mV', 0.1)
plot_ax('p_ac_ref')
plot_ax('p_ac_filt50Hz')
plot_ax('bat_p')


#ax2.plot(x, var_stateDC, label=name + ' var_stateDC')
#ax2.plot(x, var_pdc_filt50Hz, label=name + ' var_pdc_filt50Hz')
#ax2.plot(x, var_v_pv_filt50Hz, label=name + ' var_v_pv_filt50Hz')
#ax2.plot(x, var_v_dc_filt50Hz, label=name + ' var_v_dc_filt50Hz')


#ax2.set_ylabel(r'i$_\mathrm{pv}$'+' (A)')




ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

leg = interactive_legend()

align_yaxis(ax1, ax2)


# plt.xlabel('Sample number')
# plt.ylabel('U (V)')


#plt.savefig('timeplot.png', dpi=300)
plt.show()


#if __name__ == '__main__':

#    main()

    
    
