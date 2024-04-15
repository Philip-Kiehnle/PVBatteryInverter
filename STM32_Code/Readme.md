# Howto

## Bootloader

In GCC preprocessor define USE_FULL_LL_DRIVER

Enter bootloader by pressing emergency stop and the reset button.  
In bootloader both LEDs (red and green) are lit.

```
philip@792G:~/Downloads/stm32flash-0.7$ ./stm32flash /dev/ttyUSB1 -b 115200
stm32flash 0.7

http://stm32flash.sourceforge.net/

Interface serial_posix: 115200 8E1
Version      : 0x31
Option 1     : 0x00
Option 2     : 0x00
Device ID    : 0x0469 (STM32G47xxx/48xxx)
- RAM        : Up to 96KiB  (16384b reserved by bootloader)
- Flash      : Up to 512KiB (size first sector: 1x2048)
- Option RAM : 48b
- System RAM : 28KiB

```

Write with verify and start application:
```
./stm32flash -w firmware.bin -v -g 0x8008000 -S 0x8008000 /dev/ttyUSB1 -b 115200
```

### Problems:
#### Data corruption can lead to execution of incomplete program
Todo: protect flash of bootloader and implement CRC (in bootloader seems more robust than in image. Bootloader should check CRC before starting the application.)