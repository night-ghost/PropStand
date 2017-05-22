# PropStand

measuring stand for propellers 
modes:

* force & torque (with voltage and current)
* vibrations


based on that works - http://www.parkflyer.ru/ru/blogs/view_entry/8857/  and http://www.parkflyer.ru/ru/blogs/view_entry/9651/

but on STM32 instead of Arduino. This replacement allowed to pack all functions into one board, improve each function and add new ones.

features:

* native USB, not slow Serial
* uses internal ADCs, not I2C ones
* all calibration can be set when working, not at compile time
* all calculations in float 
* debug modes for all measurement modes and motor debug mode
* much more data in Vibration - 1 second of measurement
* axis of Vibration measure can be selected
* n-point filters on all data
* command-line interface to change mode and to get/set calibration data
* HX711 interface is fully rewritten

console help:

Commands:
Main commands:
 f - measure force
 b - balance

Debug commands:
 g - force debug
 c - balance debug
 m - motor debug

Misc commands:
 r - recalibrate
 s - print all registers
 w - write registers to EEPROM
 z - clear EEPROM
 ! - reboot

 Rn=val - set value to register
 Rn? - show register value

Registers:
 R0 - vibration coordinate
 R1 - voltage factor * 1000
 R2 - current factor * 1000
 R3 - force tenzo factor * 1000
 R4 - torque tenzo factor * 1000
 R5 - maximal motor speed, RPM
 R6 - enable debug messages, 1/0
 
 
 use with my STM32duino fork - https://github.com/night-ghost/Arduino_STM32
 