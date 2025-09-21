
# PVBatteryInverter - Highly Efficient Photovoltaic and Battery Hybrid Inverter using GaN Semiconductors
This project was inspired by multiple drivers:
 * AC-connected battery systems have disadvantages in terms of charging efficiency ( PV -> DC/AC -> AC/DC -> Battery ) and cost/environmental impact, because of the extra inverter.  
 A multilevel battery converter is efficient but does not offer a DC port for PV and thus, increases installed converter power in the grid and grid operators have to calculate the worst case for grid utilization.
 * DC-connected battery systems do not guarantee a better efficiency by default. E.g. in case of a large inverter (~10 kW) operating under small loads during nighttime (30–150 Watt).
 * Commercial solutions often require an additional power meter for the self-consumption power control. In my opinion, this is a complete waste of time, space, resources and energy:
   * The installation is complicated, and requires space in the meter cabinet.
   * The extra power meter requires extra power of 0.4–2 Watt.
   * The extra power meter has an offset from the official meter measurement, meaning it cannot be used for exact zero-import control.
   * The digital energy meters in Germany already offer an optical interface, which can be set to send the actual power every second  [More details below](#head_electricityMeter)
 * GaN instead of SiC or Si-IGBT offers improved efficiency.
 * No GaN-based inverter is out there at the moment (9.12.2023)
 * Second-life 6 kWh battery was available
 * GaN full-bridge cells were available from another project
 * New west oriented PV plant has some shaded areas, which are directly connected to microinverter. But 8 PV-modules are unshaded most of the time and require a string inverter. A Hybrid (PV+Battery) Inverter for the unshaded PV-modules and a microinverter for the shaded PV-modules looks like the optimal solution.

Let's build an efficient 2kW hybrid inverter, which solves all the problems shown above!


# Components

## PV Plant
```
8 x 405 W PV-modules in the string
STC:
Vmpp_module(25°C) = 36.48 V
Impp_module(25°C) = 11.10 A
Voc_module(25°C)  = 46.30 V

Temperature Coefficients of Voc = -0.28 %
Voc_module(-15°C)  = 51.49 V
Voc_string(-15°C)  = 411.9 V -> 650 V GaN semiconductors
```

## High-Voltage Battery
```
nunm_cells = 96
Vmin = 96 * 3.3 V = 316.8 V (grid feed-in is only possible when grid rms voltage is below 224 V or by using reactive power)
Vnom = 96 * 3.7 V = 355.2 V
Vmax = 96 * 4.2 V = 403.2 V
energy = 18 Ah * 355.2 V = 6.4 kWh
```

## Inverter Housing
An unused inverter housing was available, including passive components like boost inductors and an LCL-filter, which can be reused.

### PV input
Common-Mode-Choke (CMC) -> Capacitor: MKP 900V  12.5uF +-5% -> Inductor -> Diode for PV string protection during development -> GaN Half-Bridge

### LCL-Filter
Inverter side inductor: 1mH

### GaN Full-Bridge
```
GaN semiconductors:
Gan Systems GS66508B 650 V
Rds_on(25°C)  =  50 mOhm
Rds_on(112°C) = 100 mOhm

estimated loss:
I = 2000 W ÷ 230 V = 8.7 A
50 % conduction of low and high-side switch
Pcond(112°C) = 0.5*0.1 Ohm*(8.7 A)^2 = 3.8 W per switch

Switching loss (according to datasheet at Vds = 400 V, Ids = 15 A):
Eon=47.5 µJ
Eoff=8 µJ
Pswitch = 20 kHz * (47.5+8) µJ = 1.1 W
-> 4.9 W have to be dissipated for each transistor 

Thermal Model:
R_JC = 0.5 K/W (manufacturer)
R_PCB = 3.39 K/W
R_TIM = 0.28 K/W
R_HSA = 4.8 K/W
R= 8.97 K/W
Thermal interface material coefficient seems too optimistic -> use R=10K/W
With ambient 40°C and junction = 112°C each transistor can dissipate P=(112-40)/10=7.2W
This assumption has enough tolerance because:
  -operating junction temperature is 150°C and not 112°C
  -dissipated power will be lower than 7.2W

If the power rating is set to 2300 W, the loss will increase to:
Ploss = Pswitch + 0.5*0.1 Ohm*(10 A)^2 = 1.1 W + 5 W = 6.1 W
But the inverter's inductors had an original design power of 2 kW. On the other hand, the ripple is lower than in the original design, because of an increased switching frequency.
This increases the core loss but the cental inductor of phase L2 can be assumed as a heat sink, because it is not used.

Start heatsink fan at 1200W (5.2A) assume 90mOhm:
Ploss = Pswitch + 0.5*0.09 Ohm*(5.2 A)^2 = 1.1 W + 1.2 W = 2.3 W
Minimum runtime is 30 sec, stop if power after timer runtime is lower than 1 kW. Restart timer if power > 1.2 kW during fan runtime.
```

### Control board [PVBatteryInverter_Mainboard](PVBatteryInverter_Mainboard)
An STM32G474 microcontroller is used to control the system.

#### Code Execution:

HRTIM 20 kHz controls Grid Full-Bridge
1. HRTIM period triggers ADC1
2. ADC1 DMA done calls ADC_ConvComplete with hadc1 instance
3. ADC2 for PV current is started (two ADCs simultaneously cause disturbance!)
4. AC control algorithm is executed and new dutycycle is set
5. 45 µs after ADC2 start, the 128 PV/DC current samples are acquired and ADC DMA calls ADC_ConvComplete with hadc2 instance
6. DC control algorithm is executed and new dutycycle is set. Exact timing not relevant for slow MPP-Tracker
7. wait for new ADC1 samples

## Energy Management System
Todo: opensource the git repo and add link

## <a name="head_electricityMeter"></a>Default Electricity Meter

The Iskra electricity meter offers a power resolution of 1 Watt and a temporal resolution of 1 second.  
Its permanent energy counting registers are written every 100 mWh.  
This equals to a power mismatch of 360 W for 1 second.  
So the controller has to avoid that, which is possible most of the time.  
In case of large load steps, like hobs switching with 2 kW, there will be a mismatch.  
But experiments have shown that for a cooking session of 30 minutes, a mismatch of about 12 Wh occurs.  
If this amount is equally spread over 24 hours, the mismatch in each hour is 0.5 W.
To beat that value, an extra energy meter has to:
  * consume less than 0.5 W
  * offer better temporal resolution than 1 second
  * offers no offset compared to the main electricity meter, or use an offset in feed-in direction and thus always feeds a little bit of power into the grid
  * comes at no cost

This demonstrates the whole extent of stupidity to install an extra energy meter, if a digital electricity meter is already installed!  
A detailed analysis is also carried out in the [PowerSensorEmulator
](https://github.com/Philip-Kiehnle/PowerSensorEmulator
) repository.


# Efficiency

