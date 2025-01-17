#!/usr/bin/python3
# saturating inductor lookup table generator
import numpy as np
import matplotlib.pyplot as plt

FGRID=50
#R=0.5707  # current leading 510us
#R=0.630  460us
R=0.8428
L_ref = 10.2e-3

#//define IAC_AMP_MAX_10mA 2.0*100  // -1.90A to 1.88A 50Hz  95%
#//define IAC_AMP_MAX_10mA 4.0*100  // 4Aamp*45V/2=90W    -4.33A to 4.30A 50Hz  107%
#//define IAC_AMP_MAX_10mA 5.0*100  // -6.00A to 6.00A 50Hz -> 6Aamp*45V/2=135W   120% inductor saturation?
#//define IAC_AMP_MAX_10mA 5.2*100   // -6.43A to 6.48A   6.46/5.2 = 124%
#define IAC_AMP_MAX_10mA 6.0*100   // -8.14A to 8.20A    8.17/6= 136%


#i_corr_L = np.array([ [1.89, 1/0.95],  # current : inductance correction factor for nominal inductance
#                    [4.31, 1/1.07],
#                    [6.00, 1/1.20],
#                    [6.46, 1/1.24],
#                    [8.17, 1/1.36]])
#i_corr_L[1:,1] *= 0.5  # extreme saturation -> 5.78A instead of 6A

i_corr_L = np.array([ [0.5, 1],  # current : inductance correction factor for nominal inductance
                    [8.0, 0.38],
                    [10.0, 0.35]])

N = 105  # 0.0 to 10.4Ampere
L_LUT = np.zeros(N)

for i in range(N):

    ampere = i/10
    L_LUT[i] = L_ref * np.interp(ampere, i_corr_L[:,0], i_corr_L[:,1])
#    print( "{:d}, ".format(int( AMP * np.sin( i/N * 2*np.pi/4 ) )), end='')

ZL = 2*np.pi*FGRID*L_LUT

lut_str = "uint16_t Ztot_LUT["+str(N)+"] = {\n"
b_field_arr = []
b_field = 0
for i in range(N):

    b_field += L_LUT[i]
    b_field_arr.append(b_field)

    #define Ztot sqrt(R*R + ZL*ZL)
    Ztot = np.sqrt(R**2 + ZL[i]**2)

    lut_str += str(int(Ztot*2**14))
#    lut_str += str(Ztot)

    if i == N-1:
        lut_str += "  // ... " + str((N-1)/10) + "A\n"
    else:
        lut_str += ","

    if i == 9:
        lut_str += "  // 0.0A, 0.1A, ..."

    if (i+1)%10 == 0:
        lut_str += "\n"

lut_str += "};"
print(lut_str)
fig, ax1 = plt.subplots()
ax1.set_xlabel(r'$i$' + ' in A')
x = np.arange(len(L_LUT))/10
ax1.plot(x, L_LUT, label='L_LUT')

# phase LUT gen
#return (int16_t)( (1<<15) * atan(ZL/R) / (2*M_PI));  // calculated at compile time
lut_str = "int16_t ZLphase_LUT["+str(N)+"] = {\n"
phase_deg_arr = []
for i in range(N):

    phase_rad = np.arctan(ZL[i] / R)
    phase_deg_arr.append(180/np.pi * phase_rad)


    lut_str += str(int(2**15 * phase_rad/(2*np.pi)))

    if i == N-1:
        lut_str += "  // ... " + str((N-1)/10) + "A\n"
    else:
        lut_str += ","

    if i == 9:
        lut_str += "  // 0.0A, 0.1A, ..."

    if (i+1)%10 == 0:
        lut_str += "\n"

lut_str += "};"
print(lut_str)

ax1.legend()
ax2 = ax1.twinx()
ax2._get_lines.prop_cycler = ax1._get_lines.prop_cycler
#ax2.plot(phase_deg_arr)
ax2.plot(x, b_field_arr, label='b_field')

ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

plt.show()
