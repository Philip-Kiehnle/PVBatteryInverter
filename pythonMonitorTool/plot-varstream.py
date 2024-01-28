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

# b int8
# B uint8
# h int16
# H uint16
# i uint32
# I uint32
# f float32
	
#uint16_t ID;
#int16_t sys_errcode;
#uint16_t duty;
#float pdc_filt50Hz;
#float v_pv_filt50Hz;
#float v_dc_filt50Hz;
#uint16_t stateAC;
#int16_t f_ac_10mHz;
#int16_t v_ac_rms_100mV;
#int16_t v_amp_pred_100mV;
#int16_t i_ac_amp_10mA;
#	uint16_t VdcFBgrid_sincfilt_100mV;

#	uint16_t ID;  // sys variables
#	uint8_t sys_mode;
#	int8_t sys_errcode;
#	uint8_t stateDC;  // DC variables
#	uint8_t dcdc_mode;
#	uint16_t dutyDC_HS;
#	float pdc_filt50Hz;
#	float v_pv_filt50Hz;
#	float v_dc_filt50Hz;
#	uint16_t stateAC;  // AC variables
#	int16_t f_ac_10mHz;
#	int16_t v_ac_rms_100mV;
#	int16_t v_amp_pred_100mV;
#	int16_t i_ac_amp_10mA;
#	int16_t p_ac;
#	uint16_t VdcFBgrid_sincfilt_100mV;  // for debugging
#	uint16_t VdcFBboost_sincfilt_100mV;


monitor_vars_t = "HBbBBHfffHhhhhhHH"
struct_size = struct.calcsize(monitor_vars_t)
print("struct_size", struct_size)

d = np.dtype([
('ID', 'u2'),
('sys_mode', 'u1'),
('sys_errcode', 'i1'),
('dcdc_mode', 'u1'),
('stateDC', 'u1'),
('dutyDC_HS', 'u2'),
('pdc_filt50Hz', 'f4'),
('v_pv_filt50Hz', 'f4'),
('v_dc_filt50Hz', 'f4'),
('stateAC', 'u2'),
('f_ac_10mHz', 'i2'),
('v_ac_rms_100mV', 'i2'),
('v_amp_pred_100mV', 'i2'),
('i_ac_amp_10mA', 'i2'),
('p_ac', 'i2'),
('VdcFBgrid_sincfilt_100mV', 'u2'),
('VdcFBboost_sincfilt_100mV', 'u2')
])

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

prev_ID = 0

for i in range(samples_to_consume):

	vars[i] = struct.unpack_from(monitor_vars_t,filehandle.read(struct_size))

	ID = vars['ID'][i]
	if ID != prev_ID+1:
		print(ID, ' != prev_ID+1 at pos', i)
	prev_ID = ID

#exit()
fig, ax1 = plt.subplots()
ax1.grid(axis='x')

#ax1.set_ylabel(r'v$_\mathrm{pv}$'+' (V), ' + r'p$_\mathrm{pv}$'+' (W)')


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
    curr_ax.plot(x, y, label=filename + ' ' + name)

    if vert_lines:
        y_prev = 0
        for idx, y_single in enumerate(y):
            if y_single != y_prev:
#                plt.axvline(x=idx)
                plt.axvline(x=x_scale*idx, color=color, label='state:' + str(y_single))
            y_prev = y_single


#plot_ax('ID')
plot_ax('sys_errcode')
plot_ax('dutyDC_HS')
plot_ax('stateDC', vert_lines=True)
plot_ax('stateAC', vert_lines=True, color='r')

ax2 = ax1.twinx()
ax2._get_lines.prop_cycler = ax1._get_lines.prop_cycler
curr_ax = ax2

plot_ax('pdc_filt50Hz')
plot_ax('v_pv_filt50Hz')
plot_ax('v_dc_filt50Hz')

plot_ax('f_ac_10mHz', 0.01)
plot_ax('v_ac_rms_100mV', 0.1)
plot_ax('v_amp_pred_100mV', 0.1)
plot_ax('i_ac_amp_10mA', 0.01)

plot_ax('VdcFBgrid_sincfilt_100mV', 0.1)


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

    
    
