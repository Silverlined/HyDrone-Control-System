#!/bin/bash

arduino-cli upload -p /dev/ttyACM0 -b arduino:avr:nano:cpu=atmega328 ~/main/main.ino
