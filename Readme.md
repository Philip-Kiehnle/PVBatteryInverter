
# PVBatteryInverter - High efficient photovoltaic and battery inverter using GaN semiconductors
This project was inspired by multiple drivers:
 * AC-connected battery systems have dissadvantages in terms of charging efficiency ( PV -> DC/AC -> AC/DC -> Battery ) and cost/environmental impact, because of the extra inverter. Multilevel Battery is efficient but does not offer DC port for PV and thus, increases installed power in grid and grid operator has to calculate the worst case.
 * DC-connected battery systems do not guarantee a better efficiency by default. E.g. in case of a large inverter (~10 kW) operating under small loads during nighttime (30-150 Watt).
 * Commercial solutions require an additional power meter for the self-consumption power control. In my opinion, this is a complete waste of time, space, resources and energy:
   * installation is complicated, and requires space in the meter cabinet
   * extra power meter requires extra power of 0.5-3 Watt
   * extra power meter has an offset and can not control the official meter measurement to zero
   * The digital energy meters in Germany already offer an optical interface, which can be set to send the actual power every second  [More details below](#head_electricityMeter)
 * GaN instead of SiC or Si-IGBT offers improved efficiency.
 * No GaN-based inverter is out there at the moment 9.12.2023
 * Second-life 6 kWh battery was available
 * GaN full-bridge cells were available from another project
 * New west oriented PV plant has some shaded areas, which are directly connected to microinverter. But 8 PV-modules are unshaded most of the time and require a string inverter.
 * -> Small Hybrid (PV+Battery) Inverter for some PV-modules and microinverter or PV-sting inverter for the other PV-modules looks like the optimal solution


Let's build an efficient 2kW hybrid inverter, which solves all the problems shown above!


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
Vmin = 96*3.3 V = 316.8 V (feed into grid is only possible when grid voltage is below 224V)
Vnom = 96*3.7 V = 355.2 V
Vmax = 96*4.2 V = 403.2 V
energy = 18 Ah * 355.2V = 6.4 kWh
```

## GaN Full-Bridge
```
GaN semiconductors:
GS66508B 650 V
Rds_on(25°C)  =  50 mOhm
Rds_on(112°C) = 100 mOhm

estimated loss:
2000W÷230V=8.7A
50 % conduction of low and highside switch
Pcond(112°C) = 0.5*0.1 Ohm*(8.7 A)^2 = 3.8 W

Switching loss (according to datasheet at Vds = 400 V, Ids = 15 A):
Eon=47.5 µJ
Eoff=8 µJ
Pswitch = 20kHz * (47.5+8) µJ = 1.1W
-> 4.9 W have to be dissipated for each transistor 

Thermal Model:
R_JC = 0.5K/W (manufacturer)
R_PCB = 3.39K/W
R_TIM = 0.28K/W
R_HSA = 4.8K/W
R= 8.97K/W
Thermal interface material coef seems too optimistic -> use R=10K/W
With ambient 40°C and junction = 112°C each transistor can dissipate P=(112-40)/10=7.2W
This assumption has enough tolerance because:
  -operating junction temperature is 150°C and not 112°C
  -dissipated power will be lower than 7.2W

If the power rating is set to 2300W, the loss will increase to:
Ploss = Pswitch + 0.5*0.1 Ohm*(10 A)^2 = 1.1W + 5W = 6.1W
But the inverters inductors had an original design power of 2kW. On the other hand, the ripple is lower than in the original design, because of a higher switching frequency.
This increases the core loss but the middle inductor of phase 2 can be assumed as a heat sink, because it is not used.

Start fan at 1200W (5.2A) assume 90mOhm:
Ploss = Pswitch + 0.5*0.09 Ohm*(5.2 A)^2 = 1.1W + 1.2W = 2.3W
Minimum runtime is 30sec, stop if power after timer runtime is lower than 1kW. Restart timer if power > 1.2kW during fan runtime.
```



## Control board [PVBatteryInverter_Mainboard](PVBatteryInverter_Mainboard)
STM32G474 microcontroller

###
Code execution

HRTIM 20kHz controls Grid Fullbridge
1. HRTIM period triggers ADC1
2. ADC1 DMA done calls ADC_ConvComplete with hadc1 instance
3. ADC2 for PV current is started (two ADCs simultaneously cause disturbance!)
4. AC control algorithm is executed and new dutycycle is set
5. 45us after ADC2 start, the 128 PV/DC current samples are acquired and ADC DMA calls ADC_ConvComplete with hadc2 instance
6. DC control algorithm is executed and new dutycycle is set. Exact timing not relevant for slow MPP-Tracker
7. wait for new ADC1 samples

Drawback: 1.5TA for AC control algorithm, which need only 17us(0.34TA) -> Set the HRTIM to call the ADC in half period using Postscaler, which has the following function in up-downcounting PWM
Option1: Up Down mode rollover event set to 0 and period, then use ADC Trigger Config Post Scaler
ADROM[1:0] = 00: event generated both during up and down-counting phases
ADROM[1:0] = 01: event generated during down-counting phases
ADROM[1:0] = 10: event generated during up-counting phases
test: Trigger on reset, ADROM = 0x1 -> 

Option2: Up Down mode rollover event used by ADC trigger set to "generated when counter == 0" or ""generated when counter = period". Does not matter, because
the update event is set to 0 or period, and updates the dutycycle twice per period. So no change from control algorithm is missed.
Caution: Option in ADC Trigger config:"ADC trigger on timer A reset" or "ADC trigger on timer A period" does not reduce the number of interrupts if the Up Down mode rollover event used by ADC trigger set to "generated when counter = 0 or period"

Used Config: See documentation/img folder
Up Down mode rollover event used by ADC trigger set "generated when counter == period"
ADC Trigger config: "ADC trigger on timer A period"
HRTIM1.ADCTrigger1_Source1=HRTIM_ADCTRIGGEREVENT13_TIMERA_PERIOD
HRTIM1.ADCTrigger_Id1=HRTIM_ADCTRIGGER_1
HRTIM1.Adrom_TA=HRTIM_TIM_ADROM_CREST  ( other options: HRTIM_TIM_ADROM_BOTH, HRTIM_TIM_ADROM_VALLEY)

Caution: ADC1 has only 50us-45.18us=4.82us for undisturbed conversion, when ADC2 idles.
With 4x oversampling for ADC1 the two channels need: 4x2×(2,5+12,5)÷42,5 = 2.82us -> use this to avoid overlap of 8x oversampling(0.83us) + software runtime for ADC2 start
With 8x oversampling for ADC1 the two channels need: 8x2×(2,5+12,5)÷42,5 = 5.647us -> this is used and take 6.6us until DMA triggers the HAL_ADC_ConvCpltCallback
TODO: use extra timer to trigger ADC before the period event, so that sampling is centered around the peak of the PWM counter.
Advantages:
1. exact current value
2. more calculation time for the 0.5 x Tc control time version

Check if ADC interrupt and manual copy is more efficient instead of DMA for only two channels.
Use Hardware for offset and gain correction for ADC values. -> "Offset correction is not supported in oversampling mode"

## PV inverter housing
An old inverter case is used, which provides some EMV relevant components, which can be reused.

### PV input
CMC -> Capacitor: MKP 900V  12.5uF +-5%

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

Pac of converter -> without transformer loss
Pac=79W Pbat=86W n=91.86%

Pac_meter before and after switchoff of converter -> including transformer
Pac_meter=235-120=115W Pbat=139W n=82.7%
30Min Test 4,9Aamp -> GaN kaum warm, L2 Netzdrossel am wärmsten von allen Bauteilen, aber okay


https://www.dsprelated.com/showarticle/1337.php

CIC intergrartor:
each stage is implemented with two’s complement (non-saturating) arithmetic
the range of a stage's number system is greater than or equal to the maximum value expected at the stage's output.

# Comparable products
Enphase IW 5P AC battery has 2x overload capability for 3seconds for off grid loads.
Pnom,1ph = 3.84kVA -> 7.68kVA?
VDE4105 allows 4.6kVA in on-grid mode.

PVBatteryInverter:
Pnom,1ph = 2.3kVA (10Amp) -> 3kVA?

# Loss
## Reactive power
LCL Filter grid side inductor has 5mH at low currents and 2.2uF/2 between L and N before and after grid inductor:
Grid side: 230V÷(1÷(2×π×50×1,1×10^−6)) = 79mA  Q=18.3VAr
After grid inductor: 230÷(2×π×50×0,005 + 1÷(2×π×50×1,1×10^−6)) = 79mA  Q=18.3VAr
-> Inductance does not matter at 50Hz
Total reactive power: Q=36,6VAr

#Safety
## Under voltage lockout
At ~4.65V microcnotroller goes into error state, to protecte the gatedriver from low voltage.

The gatdriver Undervoltage Lockout (UVLO) protection has 3V UVLO and this means:
min 2.7V typ 3.5V max 4.0V -> not useful for the zenerdiode circuit with negative gate turnoff.

PESE1-S5-S9-M  5V to 9V DCDC has no output regulation:
Datasheet:
  input voltage range: 4.5V to 5.5V
  min output current: 12mA -> caution, the 6V zener diode and 10k resistor use only 3V/10k=0.3mA. The gatdriver needs 2.5 to 4.0mA quiescent current. Assume the regulation is worse than with the requested minimum DCDC output current.

Measurement (Gatdriver was not active (EN low)):
5V rail at 4.82V: V_HS=9.01V  V_LS=9.01V -> 4.82V is in tolerance. But do not risk a drop in negative gate voltage supply and protect the powerstage if voltage falls below ~4.65V.

## Battery
Li-Dendriten auf der Anode sind das Problem bei Kälte durch die geringer 
Diffunssionsgeschwindigkeit
Heißer Tip: Das Plating betrifft die Anode und LiFePo ist eine Kathode

Laden von Li Akku ist endotherm.
Selbst beobachtet bei Pchg=450W (ca. 0.07C) sank Temperatur von 2°C auf 1°C trotz ca. 3°C Außentemperatur. Wahrschienlich bei 0.2C insgesamt nicht mehr endotherm, da Verluste am Innenwiderstand im Quadrat ansteigen.

Innenwiderstand bei 80-90% SoC minimal.

Optimale Strategie laut
https://batteryuniversity.com/article/bu-808-how-to-prolong-lithium-based-batteries
Bei 60% Energiebedarf zwischen 85 und 25% 2000 cycles
Bei 50% Energiebedarf zwischen 75 und 25% 3000 cycles
Bei 10% Energiebedarf zwischen 75 und 65%, da so die Alterung minimiert wird.
Erklärungsversuch: Wenn SoC noch geringer wäre, würden durch die geringerer Spannungslage höhere Ströme notwendig um die gleiche Energie umzusetzen.

## Batteriekabel
Am Wechselrichter bei 48V:
schwarz mit Aufdruck H07V-K 2,5mm², blau von Ring mit gleichem Typ:
Einzelader flexibel H07V-K 2,5mm²: Spannungen bis 1 000 V Wechselspannung oder bis 750 V Gleichspannung gegen Erde
Ab 25.01.2024:
alle Kabel PV Kabel mit doppelter Isolation

## 5V power consumption
ELV Energy Master:
Main PCB + BatterySupervisor:
Netzteil Alcatel 5V 550mA: 2.7W ELV, bei 80% Wirkungsgrad ca. 2.2W 
+Gan Switching 0.2W?
+Contactor 0.4W
+Fan 0.6W
3.4W/5V = 680mA > 550mA !

Netzteil Noname 5V 1A: 3.1W ELV, bei 70% Wirkungsgrad ca. 2.2W 

Netzteil Samsung2015 Class V 5V 2A: 2.4W ELV, bei 90% Wirkungsgrad ca. 2.2W 

## RCD
Für Ableitstrom als PE-Strom liegt der Grenzwert bei 3,5 mA. Höherfrequente Ströme auch mehr.

Ohne Berücksichtigung von Induktivitäten oder harmonischen Spannungen:
Cpe von PV+ nach PE: 100nF aber halbe Spannung
Cpe von PV- nach PE: 100nF aber halbe Spannung
-> entspricht 1x100nF mit voller Spannung
230V÷(1÷(2×π×50×100×10^−9)) = 7.2mA < 7.9mA
-> Wenn der PV Generator nass ist sind maximal ~100nF-17nF=83nF mit dem 10mA FI möglich, da Y-Kondensator zur Entstörung auch dazu zählt.
Laut Bruno Burger ~50nF/kWp (bei welchem Wirkungsgrad?), d.h. bei 3.2kWp maximal 160nF. Heute wahrscheinlich eher weniger, da ein kWp weniger Fläche einnimmt.


### 10mA Typ: löst laut Norm bei 5-10mA aus (meiner bei 7,9mA RMS)
löste aus bei 120nF Y-Kapazität -> zwei Y-Kondesatoren entfernt (N und L2).
Kapazität von auf 60nF reduziert, aber FI löst immernoch aus. Auch wenn LCL nicht an Vollbrücke und/oder AC Seite nicht getaktet.
230V÷(1÷(2×π×50×60×10^−9)) = 4.3mA
Noch eine Y-Kondesnator bei L3 entfernt.
AC Anschluss hat jetzt:
C_L1_PE = 30,5nF
C_N_PE = 30,0nF

Löst nicht aus wenn N nicht angeschlossen. Amprobe: 3.3mA
Löst aus wenn N auch angeschlossen. Ohne 10mA FI nur mit 30mA Zentral-FI: Amprobe: 124mA LCL Blindleistung. FI brummt wenn überbrückt, aber noch am Netz angeschlossen.

L an Vollbrücke ändert nichts.
Mit Gate enable länger am Netz ca. 500ms da LCL schon vorgeladen und nicht sofort den FI auslöst.

Nur mit 30mA RCD:
Strom stellt sich auf 560mA Blindstrom ein. Theoretisch nur 160mA RMS (siehe oben)

Wenn Amplitude nicht passt:
Annahme 1V Unterschied: 1V÷(2×π×50×0,005 + 0.5Ohm) = 480mA
-> Amplitudenfehler erzeugt sofort einen Blindstrom

Phasenfehler erzeugt Wirkstrom

Letzten Y-Kondesator entfernt.
AC Anschluss hat jetzt:
C_L1_PE = 10,9nF
C_N_PE = 10,9nF
10mA FI löst immernoch aus. Auch 15nF zwischen L und N am FI bringt keine Verbesserung. Vermutlich verursacht eine Gleichtaktstörung das Problem. 
Zusätzliche Gleichtaktdrossel eingebaut. Bringt nichts, FI löst nach 362ms aus. Gleichtakt schwingt nur mit extra CMC mehr als mit dem 15nF Kondesator.

AC Anschluss ohne Mid1 oder Mid2 an Drossel:
C_L1_PE = 10,0nF
C_N_PE = 10,0nF

AC Anschluss ohne Mid2 und ohne Mid1 an Drossel:
C_L1_PE = 0,6nF
C_N_PE = 0,6nF
-> Kapaität kommt durch Vollbrücken oder PV Seite.

AC Anschluss ohne Mid2 und ohne Mid1 an Drossel und PV-Emulator vom Netz abgesteckt:
C_L1_PE = 1,3nF
C_N_PE = 1,3nF
-> PV-Emulator Trafo hat starke kapazitive Kopplung-
-> PV-Emulator Trafo nach dem 10mA FI aber vor der 2A Sicherung anschließen.
10mA FI löst auch bei 5V Netzteil aus und bei 24V Trafo
-> FAIL: FI falsch angeschlossen! L-Klemme am Summenstromwandler vorbei war auch nach außen geführt!

FI richtig verkabelt:
C_L1_PE = 2,2nF
C_N_PE = 2,2nF

-> Falls Entstörung nach PE doch Vorteile bringt, zur Sicherheit
33nF in Serie mit 33nF von L nach PE wieder eingelötet:
C_L1_PE = 17,4nF
C_N_PE = 17,2nF (kommt durch hohes C beim LCL Filter zu Stande.

FI im Betrieb getestet mit Widerstand von DC- (Batterieanschluss) nach PE:
Dieses Potential hat laut Theorie eine 50Hz Schwingung mit der halben Netzamplitude.
33k||220k = 28.69kOhm keine Auslösung (230V/2)/28,69k=4,0mA RMS
10kOhm Auslösung (230V/2)/10k=11,5mA RMS

#ADC measurement noise:
no grid; no PV; PV booster seems to have no influence

single sampling(2.5+12.5 @ 170/4=42.5MHz);
v_ac_100mV -3 +4V
i_ac_10mA -0.32 +0.25A   

single sampling(6.5+12.5 @ 170/4=42.5MHz)
seems even worse

2x oversampling(2.5+12.5 @ 170/4=42.5MHz)
no improvement. Auch ohne STecker zur FB boost oder FB grid.

8x oversampling(2.5+12.5 @ 170/4=42.5MHz) mit und ohne debug pin toggling
v_ac_100mV -0 +2.8V      -6LSB +5LSB from average (1.5V)
i_ac_10mA -0.32 +0.25A  -14LSB +11 LSB from average

8x oversampling(2.5+12.5 @ 170/4=42.5MHz) ohne debug pin toggling; 330pF zwischen Vadc diff lines.
v_ac_100mV -0.2 +1.2V     -2LSB +3LSB from average (0.14V)  -> seems okay now
i_ac_10mA -0.32 +0.25A  -14LSB +11 LSB from average

Improve ac current accuracy for better p_ac calculation and to make optinal current control possible:
extra 10nF (60kHz lowpass) does not help

Signal Kurzschließen -> -60.5A
Vac nicht samplen? -> bringt nichts
Problem: PC3 nFault 3.6V IO driven by 5V -> Pin mit 10kOhm nach GND auf ~2.5V ziehen bringt nichts
Stromsensorausgang min load 10kOhm -> 10k drangehalten -> bringt nichts
Nyquist zumindest im Abtastmoment einhalten -> 20k*8oversampling/2=160kHz -> 60kHz Tiefpass sollte reichen
47uF an 5V -> bringt nichts

ADC2 Oversampling 128x disturbs the ADC1:
without oversamping both are okay;
ADC1=1x ADC2=64x 2,5cyc, ADC1 is disturbed in both channels Vac: 2007−1968=39LSB and iac=2640−2615=25LSB Vac has high freqeuncy, iac has single bursts
ADC1=1x ADC2=128x 2,5cyc, ADC1 is disturbed in both channels Vac: 2007−1966=41LSB and iac=2640−2615=25LSB Vac has high freqeuncy, iac has single bursts
ADC1=1x ADC2=128x 2,5cyc, with 100nF extra ADC2 Ipv_in; ADC1 is disturbed in both channels Vac: 2007−1970=37LSB and iac=2640−2615=25LSB Vac has high freqeuncy, iac has single bursts
ADC1=1x ADC2=128x 6,5cyc, with 100nF extra ADC2 Ipv_in;; ADC1 is disturbed in both channels Vac: 2009−1969=40LSB and iac=2640−2623=17LSB Vac has high freqeuncy, iac has single bursts

8x oversampling(2.5+12.5 @ 170/4=42.5MHz) should take: 8*352ns = 2.8us
two channels are converted -> 5.6us which is ~11% of PWM period

ADC1 two channel generate two interrupts with 25us in between, setting End of conversion to sequence does not help.
Both values seem updated every 25us (DMA continuous request=ON, wird auch benötigt), 
With 1 channel, only one interrupt per control period (50us).

-> Switch to completely new scheme, were ADC1 and ADC2 do not run simultaneously
New scheme:
ADC1=4x ADC2=128x 2,5cyc, with 100nF extra ADC2 Ipv_in:
Vac: 1986−1974=12LSB and iac=2634−2630=4LSB idc=2633-2630=3LSB

ADC1=8x ADC2=128x 2,5cyc, with 100nF extra ADC2 Ipv_in:
Vac: 1983−1977=6LSB and iac=2634−2630=4LSB idc=2633-2631=2LSB

Optinal Improvements:
Set DMA to Word instead of half-word and check if poerformanece is increased.
Check if end of conversion: sequence instead of single makes a difference. -> Not with DMA usage, maybe in ADC Interrupt mode.
Measure STM32 internal temperature and generate error if above e.g. 80°C

#Test am 230V Netz
Ohne EnergyPacket Control mit Fön für weniger Potentialsprünge
S 1
DC 2 Bat 0
AC 1; 3
Ppcc 0 connWarn
Pac -10  v_filt50Hz 228.1  rI 0.00
vdc b 340.8  g 341.2

S 1
DC 2 Bat 0
AC 1; 3
Ppcc 0 connErr
Pac -8  v_filt50Hz 227.8  rI 0.00
vdc b 339.2  g 339.3

S 1
DC 4 Bat 0
AC 6; 4
Ppcc 0 connErr
Pac 55  v_filt50Hz 212.3  rI 0.00
vdc b 294.2  g 292.7
E -6 EC_I_AC_SENSOR : AC current sensor 60A overcurrent fault. 5V reset required.

S 1
DC 1 Bat 0
AC 0; 3
Ppcc 0 connErr
Pac 0  v_filt50Hz 0.0  rI 0.00
vdc b 299.6  g 299.8
E -6 EC_I_AC_SENSOR : AC current sensor 60A overcurrent fault. 5V reset required.

-> Geht auch ohne 5V reset, da nur durch Störung verursacht

Am 30.1.2025 am Netz mit 150W Glühbirnen PV-Emulator:
E -6 EC_I_AC_SENSOR : AC current sensor 60A overcurrent fault. 5V reset required.
-> Tatsächlich braucht es einen 5V Reset, da der Stromsensor den Überstrom detektiert hat.
Theorie 1: Da es während des Zuschaltens passiert ist, wurde der Y-Kondesator geladen.
Warum schwingt dann der Strom auf L1 2ms?

#TODO Battery Supervisor
Check analog watchdog when using oversampling:
"The comparison is performed on the most significant 12-bit of the 16-bit
oversampled results ADC_DR[15:4]"
After shift? -> I think yes.
Examples:
4bit shift in 16x oversampling: effective comparison between ADC_DR[15(11):4] and THRES[15(11):4] -> 8 bits are compared, upper four bits are zero in data. -> Shift THRES down 4bits
4bit shift in 32x oversampling: effective comparison between ADC_DR[15(12):4] and THRES[15(12):4] -> 9 bits are compared, upper three bits are zero in data. -> Shift THRES down 3bits
3bit shift in 8x oversampling: effective comparison between ADC_DR[15(11):4] and THRES[15(11):4] -> 8 bits are compared, upper four bits are zero in data. -> Shift THRES down 4bits
3bit shift 128x oversampling: effective comparison between ADC_DR[15:4] and THRES[15:4] -> 12 bits are compared -> Nothing to do
30.04.2025 : Analog Watchdog hat EC_BATTERY_I_DISCHARGE_PULSE_MAX ausgelöst, scheint also zu funktionieren.


Kontaktchemie Plastik 70 Transparenter Schutzlatz "isoliert, schützt, versiegelt, dichtet" speziell entwickelt für Leiterplatten 20kV/mm. Auch für Metall, Holz, viele Kunststoffe, Leder, Karton, Papier.
 Trockenzeit (ca. 20 min) bei 20°C

#TODO:
git commit -m "

7.6.2025: P_dischg_max schwankt zu schnell

PCC Regler schneller machen, Überschwinger nicht so wichtig.

Fallback, falls ext EMS ausfällt:
backup_p_pv_charge_start = 500  // 500Watt von eigener DC PV -> Laden
backup_p_feedin_min = 100  // 100 Watt werden insgesamt am PCC noch eingespeist

Geringerer Stromgrenze bei Packet Mode wenn Gates aus sind. Da höhere Durchlassverluste im Fall von Netzspannungspeaks. Peaks werden von Drossel begrenzt, außerdem müssen sie höher als Vgs,th + Vdc sein, zuvor leiten alle Diodengleichrichter im Netz.

Relay contact welding Testroutine implementieren


EMS:
randomizer für Schalthandlungen, um zu vermeiden, dass alle EMS zur vollen Stunde umschalten.
3x autoreset pro 24h

Wenn InfluxDB hängt, geht mon-thermal nach Neustart von inlfux wieder, log-PV braucht aber selbst einen Neustart. Influx Code scheint in beiden gleich. 

Log:
05.09.2025 ? err -3: EC_V_BUS_V_BATTERY_DEVIATION
27.07.2025 20:59 err -56: EC_I_AC_PULSE_MAX
03.07.2025 02:06 err -3: EC_V_BUS_V_BATTERY_DEVIATION SoC=57%
02.07.2025 22:23 err -7 EC_V_SUPPLY_5V_LOW. Evtl. weil direkt Lüfter Start. -> Temperaturschaltung bei FB_Boost hatte 1kOhm statt 10kOhm -> Gefixt, spart 4,5mA.
01.07.2025 16:57 err -12 kommt bei DC_LOCK_HARD (erzeugt vermutlich kleine Störung), aber AC entladen geht. Am nächsten Tag mit Abkühlung gehts.
30.06.2025 17:51 err -12 EC_TEMPERATURE_FB_BOOST beim Laden
15.06.2025 10-13Uhr err -56: EC_I_AC_PULSE_MAX
05.06.2025 00:56 err -3: EC_V_BUS_V_BATTERY_DEVIATION  Vbat=347V SoC=34%
03.06.2025 02:38 err -3: EC_V_BUS_V_BATTERY_DEVIATION  Vac_rms=227V Vbat=347V SoC=33% Vgrid_peak von (347+15)/325 von 11,4% über der nominalen Netzspannung kann Fehler erzeugen. -> Wechsel in PV2AC beim leerer Batterie hat vermutlich Fehler erzeugt.
23.05.2025 22:26 err -7 EC_V_SUPPLY_5V_LOW bei 1.7kW, Lüfter war aber schon länger an. EMV?
21.05.2025 23:52 leerer Bat geht nicht aus. Pac_filt=-49 Pac_ref=53, Pbat=41 -> ref wird nicht umgesetzt. OFF schaltet BMS nicht aus. Noch aktuell?

15.05.2025 21:02 err -55 EC_I_AC_RMS_MAX bei 1600Watt -> Temporär auf 1550W limitiert.

10.5.2025 1:05 Selbstständiger reboot Warum? PV-DCDC Wandler aktiv.

08.05.2025 morgens Vcell error -> Todo: Batterie soll ausschalten wenn P_dischgmax unter 100 Watt gefallen ist. Fix: 40W okay but recalibrated DC current

06.05.2025 23:18 sysmode auf 4 gewechselt und Batterie geladen -> Fix: EMS controller prüft P_pcc nun nochmal vor Ladestart
Aber warum hat internal control nicht dagegen gewirkt? -> ToDo: Resonanzstromregeler statt Amplitude control

4.5.2025 11:42:
EC_FREQ_AC_HIGH              = -54
Pac=0 Gate Signale aus, Netzrelais an -> Überfrequenz muss aus dem lokalen Netz gekommen sein.

30.4.2025:
err -3: EC_V_BUS_V_BATTERY_DEVIATION Evtl. weil Strom zu hoch. Battery Supervisor leuchtet rot und hat E -107 EC_BATTERY_I_DISCHARGE_PULSE_MAX  -> ADC Watchdog von 10 auf 15A erhöht

29.4.2025:
CSC Fehler sind scheinbar verschwunden, da nach einem Microcontroller Update, also BMS war kurz aus bei Vcell,min=3,684V und Vcell,max=3,698V ein balancing command auf 3,680V alle Zellen zum Balancen bringt.

22.3.2025
warum pacref 100W also höher als pac 78W? verhindert manchmal Packet Mode

Priority für jeden Controller beim Android Code: global verfügbare Variable P_pv_excess. Thermal höher als AC Charger, aber DC battery höher als Thermal_extra_heat.
MILP Optimierung, SMARD.de, Strompreise, Solardaten.

Nachts z.B. 3W statt 0W vorgeben, da Stromzähler <4W ? nichts zählt. Aber Regelung schwierig wenn Zähler nichts anzeigt.
Einphasiger Batteriewechselrichter kann aber nicht auf allen drei Phasen 3-4W zulassen.
Vlt hat Zähler auch ein Zeitmechanismus und die kleinen Leistungen werden erst zu null gesetzt, wenn für eine bestimmte Zeit keine höherer Leistung messbar war.
Annahme: unter der Rauschgrenze von z.B. 10mA (80dB) -> pro Phase 2.3W wird nichts gezählt. Sonst würde in ungenutzten Immobilien z.B. ein Jahresverbrauch von 2.3W*8760h=20kWh entstehen.

old 48V system:
p_low_power_enter: 200 leads to 400mWh sold in 10 minutes and 600mWh bought

DC_LOCK_INACTIVE does not start system. Soft reset auch nicht. Hard Reset required.
Weil Batterie unter 30%?
Oder 3,673−3,49=183mV imbalance > 180mV Imbalance Info?

Contactor raussägen und Control PCB hinter CMC und Y-Kondensator anschließen. Vorteil: keine AC spikes in Strommessung. Bei 2kW unsicher ob verschweißte Relaiskontakte dauerhaft halten.

PI Controller reset implementieren, da y=0 nicht mehr reicht!

11.03.2025 0:44 err-6 EC_I_AC_SENSOR software reset reicht nicht, d.h. tatsächlich 60A geflossen!
Zum Y-Kondensator? 2A AC Leitungsschutzschalter hat nicht ausgelöst. Vermutung1: Spannungsspitze oder Einbruch im Netz hat Y-Kondensator aufgeladen oder entladen. Vermutung2: Störung beim Abschalten, da zu diesem Zeitpunkt/Ladestand die Batterie am Tag zuvor abgeschaltet hat.

11.03.2025 18:27 err-55 EC_I_AC_RMS_MAX -> Limit von 1.3kW auf 2kW erhöht
22.03.2025 20:50 Smartphone war aus, Akku nach Start nurnoch 7%
25.03.2025 21:51 reset command als noch AC activ -> EC_I_AC_SENSOR
30.03.2025 6:13 system reset because stateDC 1->2, but sys_mode still 3, PV-Emulator was off. Vermutung1: v_ac_amp_filt50Hz war kurz höher als Vbat(361V), also 255Vac +11%. Dann sys_mode für kurze Zeit OFF, aber nicht sichtbar.
-> Batterie12V war locker da verzinnte Kontankte. -> Aderendhülse über verzinnte Kontakte gepresst zum Test, ob das auch kriecht. Besser wäre ohne Zinn. 

System counter ID loggen 16bit -> 32bit
Kein Logging wenn BMS aus und 0 bzw cell voltage init 1-96

Ladeleistung fordert zuerst 460W bis ac controller reagiert?


Ableitstrom nur bei Regen kritisch -> Bei Sonnenschein 3 Level Modulation?
Im Sinus Maximum auch 3-Level Modulation?


8.3.25:
-p -430 grafana Pinv=520W bei 225Vac
-p -300 grafana Pinv=380W bei 225Vac
p=0     grafana Pinv=100W bei 231Vac
-> variable Gain controller eingeführt

Batteriestrom statt Leistung limitieren

Einschaltvorgang mit manuellem DC-Schalter und ausgeschalteten Gates:
14A PV Strom, beide HS GaN 7A
ohne 5V supply: 1.7V*7A=11.9W Verlust
mit  5V supply: 4.7V*7A=32.9W Verlust
Annahme 4*390uF 320V Q=C*U=0.5As E=0.5*C*U^2=80J 320V*0.5A*1s=160J (Quellenenergie -> 50% also 80J landen im Kondensator)
Mit 7A Ladestrom, dauert der Vorgang 0.5As/7A=71ms
Energie am Halbleiter: 71ms*33W=2.3J
Datenblatt: 50us Pulse mit 60A. E=50us*50mOhm*(60A)^2 = 0.009J
GaN Die: 2.5mm*6mm*0.5mm = 7.5mm³ (die height might be lower than 0.5mm but copper is also fine)
silicon density 2.33g/cm³ -> m_die=7.5mm³*rho_silicon = 0.017g
specific heat silicon 0.7J/(g*K)
Temperature rise: dT = 2.3J / (0.017g * ( 0.7J/(g*K))) = 193K
Initial Temperature: 35°C + 193K = 228°C -> Lower than solder temperature. With heat transfer to copper in the GaNpx package and solder/PCB, the temperature rise will be lower.
Ganz sichere Lösung: Gatetreiber EN immer high aber dutycycle bei 100% im error case. Erst bei PV Anschluss einführen, da Risiko einer fehlerhaften Implementierung.

System reboot wenn Ppv zu klein?
16:53 id=14153
17:14 id=36900 + 7.5min reboot was in between Log sagt 14:04 Ppv=0-3W sonst 5W
17:27 id=5824 im log nichts sichtbar. 16bit ID? Ja, Überlauf nach 21.8 Minuten

Shutdown command causes reboot. Because BMS?

DUty auf min auf 50% -> 80% limitieren. Laut LTspice reichen 80% um unter 12A zu bleiben. -> Done

step_pi_Vdc2IacAmp Fix

Neue Stage DC voltage calib testen, Mit Batterie vergleichen.

Ratelimit DC Dutycycle in increase direction (da Diode vorhanden, kann kein Rückstrom durch schnelles erhöhen auftreten)
 nur um 1 pro Takt ändern
4250/20kHz = 200ms bis komplett von 0 auf 100
Dadurch wird Eingangskondensator nicht schlagartig entladen.

Stromgrenzen Batteriestrom (1sec mean from CANbus) zur Reduktion der Batterieleistung nutzen, auch im AC mode.

Modbus:
discharge to 3.5Vcell
P_BAT_MIN_CHARGE=2W? -> Modbus
Modbus discharge power limit or Discharge Ramp wie bei Charge, um über die Nacht zu kommen. -> Oder Booster deaktivieren und Morgens aktivieren.
Batterie Total Charge and Discharge Modbus Register sowie PV total und Energy,ac total
Modbus firmware version git hash

21.2.25 18:30 err -3 EC_V_BUS_V_BATTERY_DEVIATION
Warum, wie wird Vbat berechnet? Evtl weil bis 22.2. Bug in Lesen von Zellspannugn aber dann müsste auch Vcell error kommen. Es sein denn Bug ist im Supervisor aufgetreten und hat Schütz abgeschaltet.

ca. 1Wh pro Stunde missmatch
mehr P Anteil im PCC Regler?
sinusformigerer Strom?
Pac zu gering? daher gain einführen -> Gain 1.15 hat Problem quasi nicht verändert (0.8-1.2Wh pro Stunde missmatch)

Battery has 3°C after inactive night with -1°C -> Tempsensors seems to be wrong at negative temperatures or have ~3°C offset. 
temperature[0] = 17
temperature[14] = 23 
temperature[15] = 10
temperature[28] = 16
temperature[29] = 8
temperature[42] = 17
sind PCB Sensoren, da 4°C höher als Rest beim Balancing

Geringste Temperatur am 17.2.2025 um 12:39 1°C auf den PCBs
temperature_PCB[0] = 2
temperature_PCB[1] = 1
temperature_PCB[2] = 3
temperature_PCB[3] = 2
temperature_PCB[4] = 2
temperature_PCB[5] = 2
temperature_PCB[6] = 2
temperature_PCB[7] = 1


SoC noisy, teilweise Sprünge von 8%
Zellspannungen haben 100Hz Ripple überlagert, werden aber vor der SoC Schätzung nicht gemittelt.
-> 3 Zellen mit hohem Ri durch extra Zellen supporten
-> Kalman Filter mehr Trust auf Coulomb counter und nicht SIGMA_I = 0.1;  // 100mA
   weniger Trust auf Zellspannungen, wegen 100Hz ripple
-> Sigma extern einstellen über Batteryparameter
.V_CELL_MIN_PROTECT_mV      = 3100 -> 3300
.V_CELL_MIN_POWER_REDUCE_mV = 3300 -> 3500 oder SoC Grenzen von Energymanagement beachten

Platinen Wasserdicht einsprühen

Stromsensor Temperaturkalibrierung

Voltage deviation sum of cells (battery voltage) vs Vbus check also in PVBatteryInverter

12V Spannungsabfall prüfen und Gatesignale abschalten, bevor Schütze trennen. Evtl. nicht nur mit comparator, sondern genauen Bereich überwachen.

Netzabfall testen. Ausschaltzeit notieren.

Limit DCvoltage inital controller dutycycle for PV-EMulator

Iref_amp has 100Hz component without ripplecompensation in simulation

24kHz i_ac ripple when 20% direct Vac feedforward. LCL resonance=4.4kHz  fswitch=20kHz

no Antiwindup for PCC controler to provide matching integral or little antiwindup to match
the energy meters 100mWh

current feedforward in PV2AC mode in DC_VOLTAGE_CONTROL with P_ac = P_PV-P_bat
controller for i_ac_ref_amp in PAC_CONTROL

Vac und iac calibrieren
v_ac_100mV min=-0.200 avg=0.139 max=0.800  -> geht nicht sinnvoll mit 12bit adc_raw
i_ac_10mA  min=0.020 avg=0.062 max=0.110  -> nun fast 0

2.2.2025: ac offset egal ob N oder L vertauscht
: v_ac_100mV min=-314.600 avg=0.745 max=316.100 
-> 0.75V Offset -> should reduce 0.75/0.2824951171875=2.65 LSBs

Mit Batterie
Amprobe 147.6V  Unit-T 148.4V
v_ac_100mV min=-147.700 avg=-146.780 max=-146.000 
v_ac_100mV min=-147.700 avg=-146.759 max=-146.000 

v_ac_100mV min=147.700 avg=148.518 max=149.400 
v_ac_100mV min=147.700 avg=148.508 max=149.400
avg = (148,5+146,77)÷2 = 147,635V -> Matches value of Amprobe
Offset= 0.87V -> should reduce 0.87/0.2824951171875=3 LSBs

-> reduce 3 LSBs

max voltage change @2kW; Irms=8.7 Iamp=12.3A
Capacitor at 340V only using grid caps 2x390uF: 
1/20kHz * (12.3-8,7)A * 340V = 0.0612J
√(0,0612 ÷ (0,5×2×390×10^−6) − 350^2) = 349,78V


Blindleistungskompensation für eigenen Filter

power regler der mit p_ac den Strom nachführt. Evtl anpassung immer bei Nulldurchgang, da dann weniger DC entsteht als bei kontiniuierlicher Stromerhöhung?

nF an AC current sensor Fault, da Störung. SChlecht wenn auch interrupts ausgelöst werden, die Rechenzeit kosten. Erst bei wiederholung .Auch an DC Sensor?

autobalance
CSC Errors auswerten
MPPT Stromlimit

MPPT scheint im Winter bei Schwachlicht zu hohe Spannug zu machen

STM32 Code, avoid single CSC not read an causing an undervoltage error:
 Vc85 0.000 balancing
Vc86 0.000 balancing
Vc87 0.000 balancing
Vc88 0.000 balancing


bevor ans 230V Netz:

Falls DC-Kreis Kurzschluss hat, muss PV Booster GaN aktiv schalten sonst hohe Verlustleistung durch PV
P_highside = 4,5Voff * 10A = 45Watt!
-> Booster auch bei Ipv einschalten. Z.B: Ipv >= 0,7A  P_highside = 4,5Voff * 0,6A = 2,7Watt

seltsam: Bootnachricht bei Abschaltung von PV Emulatorgleichrichter, beim Versuch davor 5V supply error. Evtl Überspannung durch Trafodrossel, die 5V-USB Netzteil kurz stört? AC mode geht gleich wieder an weil noch genug DC Spannung?
S 1
DC 4 Bat 0
AC 6; 3
Ppcc 0 connErr
Pac 33  v_filt50Hz 30.5  rI 2.08
vdc b 320.2  g 320.0
PVBattery Inverter  Jan 17 2025 22:03:49
Print ENABLE

S 1
DC 1 Bat 0
AC 6; 3
Ppcc 0 connErr
Pac -3  v_filt50Hz 30.1  rI 0.00
vdc b 255.8  g 255.6


Oszi 14.01
mit 150W
DFF10%
Dann ab ca. 9:35 Uhr D20% Oszi Name D20 
etwas mehr DC Anteil mit 20%

Todo:

 deadtime der Grid-FB reduzieren 60ns -> 40ns für weniger Common Mode in bipolar switching?
GaN inactive with -3Vgs has -4.5V -> HS + LS means +9V voltage drop -> DC+ vs PE potential jumps with 9V/2=4.5V between active state and inactive state. -> No, inductor potential jumps but DC+ and DC- do not!
60ns deadtime with 1mH common mode choke:
I = 4.5V * 60ns / 1mH = 0.27mA -> much better than ~1V backward conduction loss for the whole period with IGBTs and diodes.
-> Deadtime reduction not that important. Unbalanced inductance values probably have more impact.

Second argument: minimize time*voltage error due to voltage drop. -> Not relevant in 20kHz application and 40ns vs 60ns deadtime.

Third (contra) argument: Tolerance in datasheet of gatedriver, including temperature:
Rdt=15kOhm  min 60ns  typ 95ns  max 130ns -> -47% +37%
Rdt= 6kOhm  min 23ns  typ 40ns  max 57ns  -> -43% +43%

Chosen 10kOhm value assuming -47% +43%:
Rdt=10kOhm  min 34ns  typ 64ns max 92ns

-> Measure deadtime and switching behaviour at full current before reducing the deadtime resistor!
-> At low current, natural commutation may be more efficient, than shorter deadtime. See Osci screenshots. 

6k8 -> (6,06×6.8k+3,84 = 45,0ns)
10k||15k = 6k -> (6,06×6.0k+3,84 = 40,2ns)

current Fault interrupt höher als DMA und Control-ISR; Wobei 20us für die Schütze keine Rolle spielen, nur bei Gatetreiber, aber Strom steigt durch Drossel nicht so schnell. Brückenkurzschluss ist sowieso fatal.

Temperatursensoren:
FB_Grid:
Vdc=4.81V von 5V Versorgung
NTC Widerstand bei ~20°C Zimmertemperatur ausgebaut gemessen: 11,93kOhm
Vcomparator- 0.492V -> Poti ist auf 1,14kOhm eingestellt. Vermutlich Auslösung bei ~78°C. Poti in Schaltung am 2.7.2024: 1,03kOhm
Tabelle in MA_Gorski.pdf Seite 73/87 scheint falsch zu sein.
Problem: Bei abgerissenem NTC Kabel gibt es keine Overtemp. Buchsen für NTC brechen leicht ab.
-> Langfristig ADC nutzen: Grid FB hängt an PB11 und ist sowohl an ADC1 und ADC2 auswertbar. ADC3,4,5 wäre besser, da noch frei. Stabile Referenzspannung wird benötigt.
oder event statt interrupt auslösen um direkt PWM und Contactor zu deaktivieren 

FB_Boost:
Vdc=4.80V von 5V Versorgung
Vcomparator- 0.465V -> Poti ist auf ? kOhm eingestellt.

Platine mit einer Stufe und blauem Kabel als Kommutierungskreis:
Poti in Schaltung am 2.7.2024: 0,52kOhm -> Löst bei ca. 36°C aus ohne DC Betrieb, nur durch Umgebungstemperatur -> Auf 127Ohm eingestellt, da R401 1k statt 10k. -> Doch R401=10k eingebaut, Poti 970 Ohm, ca. 83°C



AC Control DC offset fix for remanent inductor 

Warum funktioniert aktives setzen bzw. Abfragestop des shipmodes nicht direkt?

23.04.2024 10:12: Battery random turnoff
23.04.2024 19:36: influx push does not work. read still ok
25.04.2024      : BMS reset no repsonse
05.05.2024 07:41: battery random turnof at Pbat 9W. Dann später Iac_Rms error weil nicht erkannt, dass Batterie weg ist.
Möglicher Gründe:
1. Reboot des BMS. logPVINVERTER.txt auswerten
2. shipmode timer läuft ab. Kann aber nicht sein, da noch Daten gelesen werden.

# Optional ToDo

32 bit ID statt 16bit = System_uptime_seconds

DC spannung vorgeben. Nicht nur im MPPT

if (temperature_cell_min >= 10) {  // if temperature to low, prefer pulse charging -> use shorter pulses in internal inverter controller 

CAN timeout implementieren

Analog watchdog direkt mit HRTIM PWM oder Gatetreiber Enable Pin verbinden

DC stage:
Halbe Schaltfrequenz, aber dafür immer beide Stages? -> Lohnt erst, wenn Idc>Iripple. Schaltfrequenz umschalten.

I_ac has 60kHz bandwidth, which equals 4.17us group delay@60kHz. todo: check
Extra C in lowpass für Vac?
v_Ac measurement  1/(2*PI*2*470k*2.7k/(2*470k+2.7k)*(470p+6p)) = 124kHz -> 1.28us group delay @50Hz  2.0us@124kHz
AMC3330 has about -30° phase at 60kHz which is 1.4us group delay. todo: check
assume 1.5us+1.4us=2.9us group delay for AMC3330 vs 4.14us group delay for i_ac @ 60kHz.
-> Fast Vac measurment is necessary for direct feedforward. But what would Nyquist say :D

# PV-Sicherung
Phoenix Contact 25A 1200V DC I1=30kA gPV 3062774

MC-4 Fuse CF-10PV 15A 1000V DC I1=20kA gPV


# Mod
## RS-485
Main PCB:           seriell Termination 120Ohm + 2.2nF 
Batterie RS485 PCB: seriell Termination 120Ohm + 1nF 

## Voltage Measurement
Vdc
2*18nF 1206
2*270k 1206 (Komponenten in milchiger Kugelschreiberdose)
Bandbreite schon modifiziert??

Vdc ohne Sigmadelta
4*200K ( 180k SMD + 220k THT) (Komponenten in milchiger Kugelschreiberdose)
1*6.2K
1*3.3nF

# Fails
## 22.02.2025: GaNs von PV Boosterstage 1 explodiert
git commit: 7fbf3b03283dd8bbe38427fd84f86422936e39fd "increased Vdc limit; increased Vcell max; ..."
PV-Emulator war aktiv, "Aufgehende Sonne sollte gestestet werden"
stateDC laut log immer 1, aber da Vdc teilweise angestiegen ist, muss VOLTAGE_CONTROL oder MPPT aktiv gewesen sein.
stateAC laut log immer 0 -> Netz Vollbrücke hat nicht getaktet.
Batterie mit ca. 355V. Hat Batterie bei ca. 395Vdc zugeschaltet? -> Unwahrscheinlich, außerdem hätte der Strom mit ca. 1 Ohm Batterieimpedanz maximal 40V/1Ohm = 40A betragen. Da VOLTAGE_CONTROL noch nicht die Spannung reduzieren kann, war die Zuschaltbedingung nicht erfüllt. Batterie 25A Fuse intakt?
Also wurde die GaN Halbrücke vermutlich ohne Batterie zerstört.
"EC_I_DC_SENSOR : DC current sensor 60A overcurrent fault. 5V reset required." Vermutliche Ursache: Brückenkurzschluss und dadurch Strom von AC Fullbridge zur PV Fullbridge.
Vdc laut log beim Fehler 166V, also auch mit Software Sigma-Delta deutlich geringer als gleichgerichtete Netzspannung vom PV-Emulator.
Weiteres siehe Readme zur GaN Vollbrücke: Fehler war 15% dutycycle in Software, das die 320V am 2x12.5uF Eingangskondensator in einen Drosselstrom von über 40A gewandelt hat!

## 17.2.2024
4A in Trafo 45V eingespeist
ST link in USB Buchse eingesteckt.
Hat wohl den Controller angehalten und die PWM laufen lassen.
Folge: Trafo brummt, Rauch steigt auf bei GaN Highside von Mid1
1. Notaus -> bringt nichts da am Controller
2. PV Emulation ausgesteckt -> bringt nichts, war schon aus
3. Batterie ausgesteckt
4. Trafo ausgesteckt
In Netz-FB
Mid-1 : Vdiode,LS = 1,18V HS = 1,32  -> Vlt kam Rauch auch von LS. Highside sieht nicht so sauber verlötet aus wie die anderen in der Grid-FB. Sieht aus, als sei etwas Lötzinn zwischen Drain und Source.
Beide GaN haben leichte Unebenheit auf der Oberfläche beim S von GS66508B.
Mid-2 : LS = 1,34V HS = 1,35
Overshoot bei Mid-1 79V und Mid-2 65V bei Vdc~54V -> bei neuen GaNs überprüfen

In Boost-FB (Mids und DC+ noch angeschlossen)
Mid-1 : LS = 1,44V HS = 1,40
Mid-2 : LS = 1,42V HS = 1,36
Overshoot bei Mid-1 58V

Abschätzung Verlustleistung
R_Gan = 200mOhm
R_resistive = 0.6Ohm + 2*R_GaN = 1.0 Ohm
Z_inductive = 2*pi*50 * 3mH = 0.94 Ohm 
Z = 1.37 Ohm
Ipeak = 45V/1.37Ohm = 32.8A
GaN datasheet
Pulse Drain Current GaN 72A
Continuous 25°C 30A 100°C 25A

Ptrafo=200W Annahme, da Ptrafo max Fuse 460W primär
P_gan = 0.2/1.37*200W = 29.2W

-> Flussmittel verdampft und leichte Erhebung bei zwei Mid-1 GaNs entstanden. GaN funktionieren noch. GaN-PX Package hat "high-temperature fiber glass", das hat sich vermutlich ausgedehnt und ein Teil der Kupferfolie angehoben.
-> Hätte auch schlimmer kommen können, wenn aktiver Booster Überspannung erzeugt

# Bugs
## 11.3.2024 AC schaltet zu weil Zellspannung kurz zu 65V gemessen wird
S 3
DC 4; 2
AC 0; 0
Vbat 50.5  3.369  3.387
vdc 50.7
iac 0.00
Ppcc -5032
Pac -1
Pb 23
SoC 99

S 3
DC 1; 2
AC 0; 0
Vbat 50.5  0.252  65.472
vdc 50.5
iac 0.00
Ppcc -5032
Pac 0
Pb 23
SoC 99

S 1
DC 1; 1
AC 1; 3
Vbat 50.5  3.369  3.387
vdc 49.5
iac 0.00
Ppcc -5032
Pac 0
Pb 23
SoC 99

S 1
DC 1; 1
AC 1; 3
Vbat 50.5  3.369  3.387
vdc 48.2
iac 0.00
Ppcc -5032
Pac -1
Pb 23
SoC 99

S 1
DC 1; 1
AC 6; 3
Vbat 50.5  3.369  3.387
vdc 41.5
iac 0.00
Ppcc -5032
Pac -2
Pb 23
SoC 99


## 11.3.2024 18:33 AC macht keine Leistung (Ppcc aktualisiert nicht) aber bleibt an AC. Wenn iac=0.0 wird nach einiger Zeit doch wieder ein gültiger Ppcc Wert gelesen und für ein paar Sekunden eingespeist. -> Rs485 Termination wieder einlöten als AC Termination. Hat sich auch ohne Termination wieder gefangen
Problem besteht auch mit AC RC-Termination (2.2nF+120Ohm)
S 3
DC 1; 2
AC 6; 5
Vbat 49.7  3.317  3.322
vdc 49.7
iac 0.00
Ppcc 308
Pac 0
Pb -1
SoC 99

S 3
DC 3; 2
AC 6; 5
Vbat 49.7  3.317  3.322
vdc 49.7
iac 0.00
Ppcc 308
Pac 1
Pb -1
SoC 99


## 31.03.2024
1. Bus unten an mit 1k an 5V aber nich an GND. Oben nur AC Termination. GND über PC-UART an PE.
kleine Datenlücken. Ppcc immermal wieder für 20min unterbrochen. Irgendwann Bat comm fail -> Problem war UART Ausgabe und calc_and_wait function, die am Anfang zu lange gewartet hat
2. bei 80W kurz Pbat 0W -> Regler für konstante Pbat funktioniert noch nicht. Evtl weil nur kurz GMPP gefunden wurde -> GMPP aktivieren:todo, slow P_const control with highr update of Pbat:done
3. Vpv 6-12V obwohl kein Lückbetrieb. Gemessen 41V. Eine Reihe verschattet -> duty war LS statt HS 
4. Bat comm Fail auch am Tag, danach keine Influx Daten mehr, obwohl STM32 Ppcc aktualisiert
-> possible fix avoid RS485 overlap -> no UART + faster polling RS485

