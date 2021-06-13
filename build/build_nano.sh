#!/bin/bash

arduino-cli compile -v -b arduino:avr:nano:cpu=atmega328 ~/main/main.ino
