EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:MITEVT_ANALOG
LIBS:MITEVT_CONTACTORS
LIBS:MITEVT_interface
LIBS:MITEVT_mcontrollers
LIBS:MITEVT_OPTO
LIBS:MITEVT_power
LIBS:MITEVT_REG
LIBS:opel_velocity_interface-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCP1755 U501
U 1 1 563E6C0D
P 4850 3350
F 0 "U501" H 5500 3350 60  0000 C CNN
F 1 "MCP1755-33" H 5250 3950 60  0000 C CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23-5" H 4850 3350 60  0001 C CNN
F 3 "" H 4850 3350 60  0000 C CNN
	1    4850 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 563E6C39
P 5350 3700
F 0 "#PWR019" H 5350 3450 50  0001 C CNN
F 1 "GND" H 5350 3550 50  0000 C CNN
F 2 "" H 5350 3700 60  0000 C CNN
F 3 "" H 5350 3700 60  0000 C CNN
	1    5350 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3500 5350 3700
NoConn ~ 5900 3100
Wire Wire Line
	2900 2900 4850 2900
Wire Wire Line
	4600 2900 4600 3100
Wire Wire Line
	4600 3100 4850 3100
Connection ~ 4600 2900
Text HLabel 6150 2900 2    60   Output ~ 0
+3V3
Wire Wire Line
	6150 2900 5900 2900
$Comp
L MCP1755 U502
U 1 1 5650E5B1
P 1850 3350
F 0 "U502" H 2500 3350 60  0000 C CNN
F 1 "MCP1755-5" H 2250 3950 60  0000 C CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23-5" H 1850 3350 60  0001 C CNN
F 3 "" H 1850 3350 60  0000 C CNN
	1    1850 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 5650E5B7
P 2350 3700
F 0 "#PWR020" H 2350 3450 50  0001 C CNN
F 1 "GND" H 2350 3550 50  0000 C CNN
F 2 "" H 2350 3700 60  0000 C CNN
F 3 "" H 2350 3700 60  0000 C CNN
	1    2350 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 3500 2350 3700
NoConn ~ 2900 3100
Text HLabel 1300 2900 0    60   Input ~ 0
+12V
Wire Wire Line
	1300 2900 1850 2900
Wire Wire Line
	1600 2900 1600 3100
Wire Wire Line
	1600 3100 1850 3100
Connection ~ 1600 2900
Text HLabel 3850 2600 2    60   Output ~ 0
+5V
Wire Wire Line
	3850 2600 3550 2600
Wire Wire Line
	3550 2600 3550 2900
Connection ~ 3550 2900
$EndSCHEMATC
