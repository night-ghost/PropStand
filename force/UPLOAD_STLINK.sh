#!/bin/sh

/usr/local/stlink/st-flash  --reset write /mnt/disk_d/src/arduino/build/force.ino.bin 0x08002000 && /usr/local/stlink/st-util -m
