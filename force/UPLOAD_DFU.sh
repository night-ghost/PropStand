#!/bin/sh

dfu-util -v -a 1 --dfuse-address 0x08002000:unprotect:force -D /mnt/disk_d/src/arduino/build/force.ino.bin
