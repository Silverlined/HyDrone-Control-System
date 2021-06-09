EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 7400 7500 0    50   ~ 0
Arduino - FrSky Receivers - ESCs\n
Text Notes 8150 7650 0    50   ~ 0
24/03/2021
Text Label 2700 1200 0    50   ~ 0
5V
$Comp
L Connector:USB_B J?
U 1 1 605B88FC
P 1350 1400
F 0 "J?" H 1407 1867 50  0000 C CNN
F 1 "USB_B" H 1407 1776 50  0000 C CNN
F 2 "" H 1500 1350 50  0001 C CNN
F 3 " ~" H 1500 1350 50  0001 C CNN
	1    1350 1400
	1    0    0    -1  
$EndComp
NoConn ~ 1250 1800
Text Label 2400 1800 0    50   ~ 0
PWM_right
Text Label 1650 1400 0    50   ~ 0
PWM_left
Text Label 1650 1500 0    50   ~ 0
Control_mode
NoConn ~ 2300 1800
$Comp
L Connector:USB_B J?
U 1 1 605BBBFB
P 2400 1400
F 0 "J?" H 2457 1867 50  0000 C CNN
F 1 "USB_B" H 2457 1776 50  0000 C CNN
F 2 "" H 2550 1350 50  0001 C CNN
F 3 " ~" H 2550 1350 50  0001 C CNN
	1    2400 1400
	1    0    0    -1  
$EndComp
Text Label 1650 1200 0    50   ~ 0
BLDC_left
Text Label 1350 1800 0    50   ~ 0
BLDC_right
Text Notes 1000 5325 0    50   ~ 0
FrSky 8 CH 2.4GHz \nRadio Receiver
$Comp
L Connector:Conn_01x01_Male J?
U 1 1 605F5F32
P 1750 5575
F 0 "J?" H 1900 5775 50  0000 C CNN
F 1 "GND_Male" H 1900 5675 50  0000 C CNN
F 2 "" H 1750 5575 50  0001 C CNN
F 3 "~" H 1750 5575 50  0001 C CNN
	1    1750 5575
	1    0    0    -1  
$EndComp
Text Label 1550 5575 0    50   ~ 0
5V
Text Label 1100 5975 0    50   ~ 0
GoPro
Text Label 1100 5875 0    50   ~ 0
Control_mode
Text Label 1100 5775 0    50   ~ 0
PWM_right
Text Label 1100 5675 0    50   ~ 0
PWM_left
$Comp
L Connector:Conn_01x01_Male J?
U 1 1 605F4831
P 1350 5575
F 0 "J?" H 1450 5775 50  0000 C CNN
F 1 "5V_Male" H 1450 5675 50  0000 C CNN
F 2 "" H 1350 5575 50  0001 C CNN
F 3 "~" H 1350 5575 50  0001 C CNN
	1    1350 5575
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x08_Male J?
U 1 1 605EB076
P 900 5875
F 0 "J?" H 1000 6375 50  0000 C CNN
F 1 "PWM_Male" H 1000 6275 50  0000 C CNN
F 2 "" H 900 5875 50  0001 C CNN
F 3 "~" H 900 5875 50  0001 C CNN
	1    900  5875
	1    0    0    -1  
$EndComp
Text Label 1950 5575 0    50   ~ 0
GND
Text Label 2700 1500 0    50   ~ 0
GoPro
Text Label 2700 1400 0    50   ~ 0
GND
NoConn ~ 1100 5575
NoConn ~ 1100 6075
NoConn ~ 1100 6175
NoConn ~ 1100 6275
NoConn ~ 9400 2050
NoConn ~ 9300 850 
NoConn ~ 8700 1750
NoConn ~ 8700 1350
NoConn ~ 8700 1250
NoConn ~ 8700 1150
NoConn ~ 9900 1250
NoConn ~ 9900 1150
Text Label 9300 2050 3    50   ~ 0
GND
Text Label 9400 850  1    50   ~ 0
3.3V
Text Label 8700 1550 2    50   ~ 0
SDA
Text Label 8700 1650 2    50   ~ 0
SCL
$Comp
L Sensor_Motion:LSM6DS3 U?
U 1 1 60657CFF
P 9300 1450
F 0 "U?" H 9944 1496 50  0000 L CNN
F 1 "LSM6DS3" H 9944 1405 50  0000 L CNN
F 2 "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y" H 8900 750 50  0001 L CNN
F 3 "www.st.com/resource/en/datasheet/lsm6ds3.pdf" H 9400 800 50  0001 C CNN
	1    9300 1450
	1    0    0    -1  
$EndComp
NoConn ~ 5250 850 
NoConn ~ 5850 1250
NoConn ~ 5850 1350
NoConn ~ 5850 1650
Text Label 4850 2550 2    50   ~ 0
LED
NoConn ~ 4850 2450
NoConn ~ 5850 2550
NoConn ~ 5850 2250
NoConn ~ 5850 2150
NoConn ~ 5850 2050
NoConn ~ 5850 1950
NoConn ~ 5850 1850
NoConn ~ 5450 2850
Text Label 5450 850  1    50   ~ 0
3.3V
Text Label 5850 2450 0    50   ~ 0
SDA
Text Label 5850 2350 0    50   ~ 0
SCL
NoConn ~ 4850 1950
NoConn ~ 4850 1650
NoConn ~ 4850 1550
NoConn ~ 4850 1450
NoConn ~ 4850 1350
NoConn ~ 4850 1250
Text Label 4850 1850 2    50   ~ 0
BLDC_right
Text Label 4850 1750 2    50   ~ 0
BLDC_left
Text Label 5550 850  0    50   ~ 0
5V
Text Label 5350 2850 3    50   ~ 0
GND
$Comp
L MCU_Module:Arduino_Nano_v3.x A?
U 1 1 605B4D78
P 5350 1850
F 0 "A?" H 4950 2950 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 4800 2850 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 5350 1850 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 5350 1850 50  0001 C CNN
	1    5350 1850
	1    0    0    -1  
$EndComp
Text Label 4850 2350 2    50   ~ 0
GoPro
Text Label 4850 2250 2    50   ~ 0
Control_mode
Text Label 4850 2150 2    50   ~ 0
PWM_right
Text Label 4850 2050 2    50   ~ 0
PWM_left
Wire Notes Line
	4325 4325 4325 7800
Text Notes 625  4600 0    129  Italic 0
HyDrone Inside
Wire Notes Line
	6300 4325 6300 475 
Wire Notes Line
	475  4325 6300 4325
Text Notes 675  800  0    129  Italic 0
Electric Box\n
Wire Notes Line
	6325 4325 11225 4325
Text Notes 6625 775  0    129  Italic 0
Sensors\n
$Comp
L Sensor_Distance:VL53L1CXV0FY1 U?
U 1 1 606CA444
P 7325 3200
F 0 "U?" H 7655 3246 50  0000 L CNN
F 1 "VL53L1CXV0FY1" H 7655 3155 50  0000 L CNN
F 2 "Sensor_Distance:ST_VL53L1x" H 8000 2650 50  0001 C CNN
F 3 "https://www.st.com/resource/en/datasheet/vl53l1x.pdf" H 7425 3200 50  0001 C CNN
	1    7325 3200
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Distance:VL53L1CXV0FY1 U?
U 1 1 606CDF38
P 8700 3200
F 0 "U?" H 9030 3246 50  0000 L CNN
F 1 "VL53L1CXV0FY1" H 9030 3155 50  0000 L CNN
F 2 "Sensor_Distance:ST_VL53L1x" H 9375 2650 50  0001 C CNN
F 3 "https://www.st.com/resource/en/datasheet/vl53l1x.pdf" H 8800 3200 50  0001 C CNN
	1    8700 3200
	1    0    0    -1  
$EndComp
Text Notes 3100 5450 0    129  ~ 0
ESCs\n
Text Label 2650 5900 2    50   ~ 0
BLDC_left
Text Label 2650 5800 0    50   ~ 0
GND
Text Label 2650 5700 0    50   ~ 0
5V
$Comp
L Connector:Conn_01x03_Female J?
U 1 1 605F3893
P 2850 5800
F 0 "J?" H 2800 6100 50  0000 L CNN
F 1 "Conn_01x03_Female" H 2500 6000 50  0000 L CNN
F 2 "" H 2850 5800 50  0001 C CNN
F 3 "~" H 2850 5800 50  0001 C CNN
	1    2850 5800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Female J?
U 1 1 605FE3D6
P 3800 5800
F 0 "J?" H 3750 6100 50  0000 L CNN
F 1 "Conn_01x03_Female" H 3450 6000 50  0000 L CNN
F 2 "" H 3800 5800 50  0001 C CNN
F 3 "~" H 3800 5800 50  0001 C CNN
	1    3800 5800
	1    0    0    -1  
$EndComp
NoConn ~ 3600 5700
Text Label 3600 5800 0    50   ~ 0
GND
Text Label 3600 5900 2    50   ~ 0
BLDC_right
$EndSCHEMATC
