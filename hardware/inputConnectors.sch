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
Sheet 4 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 5000 3000 2    60   Output ~ 0
RXD
Text HLabel 5000 2900 2    60   Input ~ 0
TXD
Wire Wire Line
	5000 2900 4800 2900
Wire Wire Line
	4800 3000 5000 3000
Wire Wire Line
	4800 3100 5100 3100
$Comp
L GND #PWR021
U 1 1 55FDBFE6
P 5050 2600
F 0 "#PWR021" H 5050 2350 50  0001 C CNN
F 1 "GND" H 5050 2450 50  0000 C CNN
F 2 "" H 5050 2600 60  0000 C CNN
F 3 "" H 5050 2600 60  0000 C CNN
	1    5050 2600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4800 2700 5050 2700
Wire Wire Line
	4800 2800 5050 2800
Text HLabel 5050 2800 2    60   Output ~ 0
MCU_RESET_N
Text HLabel 5050 2700 2    60   Output ~ 0
MCU_PROG_N
Wire Wire Line
	4800 2600 5050 2600
Text HLabel 5100 3100 2    60   Output ~ 0
+12V
Text HLabel 5100 3700 2    60   Output ~ 0
WVS-
Text HLabel 5100 3600 2    60   Output ~ 0
WVS+
$Comp
L CONN_01X23 P401
U 1 1 563D6C61
P 4600 3700
F 0 "P401" H 4600 4900 50  0000 C CNN
F 1 "CONN_01X23" V 4700 3700 50  0000 C CNN
F 2 "MITEVT_autoConnectors:1-776087-2" H 4600 3700 60  0001 C CNN
F 3 "" H 4600 3700 60  0000 C CNN
	1    4600 3700
	-1   0    0    1   
$EndComp
$EndSCHEMATC
