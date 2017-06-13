#!/bin/sh


#bare metal binary
/usr/local/stlink/st-flash  --reset write $1 0x08000000 && /usr/local/stlink/st-util -m
