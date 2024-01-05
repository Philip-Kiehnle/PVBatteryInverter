
# PVBatteryInverter - High efficient photovoltaic and battery inverter using GaN semiconductors
This project was inspired by multiple drivers:
 * AC-connected battery systems have dissadvantages in case of charging efficiency ( PV -> DC/AC -> AC/DC -> Battery ) and cost/environmental impact, because of the extra inverter.
 * DC-connected battery systems do not guarantee a better efficiency by default. E.g. in case of a large inverter (~10 kW) operating under small loads during nighttime (30-150 Watt).
 * Commercial solutions require an additional power meter for the self-consumption power control. In my opinion, this is a complete waste of time, space, resources and energy:
   * installation is complicated, and requires space in the meter cabinet
   * extra power meter requires extra power of 0.5-3 Watt
   * extra power meter has an offset and can not control the official meter measurement to zero
   * The digital energy meters in Germany already offer an optical interface, which can be set to send the actual power every second  [More details below](#head_electricityMeter)
 * GaN instead of SiC or Si-IGBT offers improved efficiency.
 * No GaN-based inverter out there at the moment 9.12.2023
 * Second-life 6 kWh battery was there
 * GaN full-bridge cells were there
 * New west PV plant has some shaded areas, which are directly connected to microinverter. But 8 PV-modules are unshaded and require a string inverter.
 * -> Small Hybrid (PV+Battery) Inverter for some PV-modules and microinverter or PV-sting inverter for the other PV-modules looks like the optimal solution


Let's build an efficient 2kW hybrid inverter, which solves all problems shown above!


# Components

## PV plant
```
8 x 405 W PV-modules in string
STC:
Vmpp_module(25°C) = 36.48 V
Impp_module(25°C) = 11.10 A
Voc_module(25°C)  = 46.30 V

Temperature Coefficients of Voc = -0.28 %
Voc_module(-15°C)  = 51.49 V
Voc_string(-15°C)  = 411.9 V
```

## High-Voltage Battery
```
cells = 96
Vnom = 96*3.7 V = 355.2 V
Vmax = 96*3.7 V = 403.2 V
energy = 18 Ah * 355.2V = 6.4 kWh
```

## GaN Full-Bridge
```
GaN semiconductors:
GS66508B 650 V
Rds_on(25°C)  =  50 mOhm
Rds_on(112°C) = 100 mOhm

estimated loss:
10 A duty 50 %
Pcond(112°C) = 0.5 * 0.5*0.1 Ohm*(10 A)^2 = 2.5 W
Pswitch(112°C) = ~2 W
-> 4.5 W have to be dissipated for each transistor 
```

## Control board [PVBatteryInverter_Mainboard](PVBatteryInverter_Mainboard)
STM32G474 microcontroller

## PV inverter housing
An old inverter case is used, which provides some EMV relevant components, which can be reused.

## Energy management system
Todo: git repo link

## <a name="head_electricityMeter"></a>Default Electricity Meter

The Iskra electricity meter offers a power resoltution of 1 Watt and a temporal resolution of 1 second.  
Its permanent energy counting registers are written every 100 mWh.  
This equals to a power missmatch of 360 W for 1 second.  
So the controller has to avoid that, which is possible most of the time.  
In case of large load steps, like hobs switching with 2 kW, there will be a missmatch.  
But experiments have shown that for a cooking session of 30 minutes, a missmatch of about 12 Wh occurs.  
If this amount is equally spread over 24 hours, the missmatch in each hour is 0.5 W.
To beat that value, an extra energy meter has to:
  * consume less than 0.5 W
  * offer better temporal resolution than 1 second
  * offers no offset compared to the main electricity meter, or an offset in feedin direction and thus always feeds a little bit of power into the grid
  * comes at no cost

This demonstrates the whole extent of stupidity to install an extra energy meter, if a digital electricity meter is already installed!


# Efficiency


GaN PCB 
5V von 230V-Samsung Netzteil:
## Ohne Enable, ohne Gatesignale
1,4W mit Lüfter
0,6W ohne Lüfter


https://www.dsprelated.com/showarticle/1337.php

CIC intergrartor:
each stage is implemented with two’s complement (non-saturating) arithmetic
the range of a stage's number system is greater than or equal to the maximum value expected at the stage's output.





