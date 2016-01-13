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
LIBS:final_outline-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 6
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
L CONN_02X03 P100
U 1 1 56353C28
P 3550 2050
F 0 "P100" H 3550 2250 50  0000 C CNN
F 1 "ISP" H 3550 1850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03" H 3550 850 60  0001 C CNN
F 3 "" H 3550 850 60  0000 C CNN
	1    3550 2050
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X08 P101
U 1 1 56353C4E
P 3000 3700
F 0 "P101" H 3000 4150 50  0000 C CNN
F 1 "DEBUG_LED" V 3000 3700 50  0000 C CNN
F 2 "LEDs:LED-0805" H 3000 2500 60  0001 C CNN
F 3 "" H 3000 2500 60  0000 C CNN
	1    3000 3700
	1    0    0    -1  
$EndComp
Text HLabel 2800 2050 0    60   Output ~ 0
+5V
Text HLabel 4300 2050 2    60   Output ~ 0
RXD
Text HLabel 2800 1800 0    60   Input ~ 0
TXD
Text HLabel 2800 2300 0    60   Output ~ 0
MCU_RESET_N
Text HLabel 4300 2300 2    60   Output ~ 0
MCU_PROG_N
$Comp
L GND #PWR07
U 1 1 563E4869
P 5300 1900
F 0 "#PWR07" H 5300 1650 50  0001 C CNN
F 1 "GND" H 5300 1750 50  0000 C CNN
F 2 "" H 5300 1900 60  0000 C CNN
F 3 "" H 5300 1900 60  0000 C CNN
	1    5300 1900
	1    0    0    -1  
$EndComp
$Comp
L R R305
U 1 1 563E43BF
P 2200 3950
F 0 "R305" V 2280 3950 50  0000 C CNN
F 1 "100" V 2200 3950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2130 3950 30  0001 C CNN
F 3 "" H 2200 3950 30  0000 C CNN
	1    2200 3950
	0    1    1    0   
$EndComp
Text HLabel 4000 3750 2    60   Output ~ 0
LED1
$Comp
L GND #PWR08
U 1 1 563E4469
P 2200 3200
F 0 "#PWR08" H 2200 2950 50  0001 C CNN
F 1 "GND" H 2200 3050 50  0000 C CNN
F 2 "" H 2200 3200 60  0000 C CNN
F 3 "" H 2200 3200 60  0000 C CNN
	1    2200 3200
	1    0    0    -1  
$EndComp
$Comp
L R R301
U 1 1 563E4AFE
P 2200 3750
F 0 "R301" V 2280 3750 50  0000 C CNN
F 1 "100" V 2200 3750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2130 3750 30  0001 C CNN
F 3 "" H 2200 3750 30  0000 C CNN
	1    2200 3750
	0    -1   -1   0   
$EndComp
Text HLabel 1900 3750 0    60   Output ~ 0
LED0
$Comp
L R R306
U 1 1 563E4B25
P 3800 3950
F 0 "R306" V 3880 3950 50  0000 C CNN
F 1 "100" V 3800 3950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3730 3950 30  0001 C CNN
F 3 "" H 3800 3950 30  0000 C CNN
	1    3800 3950
	0    1    1    0   
$EndComp
Text HLabel 4000 3850 2    60   Output ~ 0
LED3
$Comp
L R R302
U 1 1 563E4B30
P 3800 3750
F 0 "R302" V 3880 3750 50  0000 C CNN
F 1 "100" V 3800 3750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3730 3750 30  0001 C CNN
F 3 "" H 3800 3750 30  0000 C CNN
	1    3800 3750
	0    -1   -1   0   
$EndComp
Text HLabel 1900 3850 0    60   Output ~ 0
LED2
$Comp
L R R303
U 1 1 563E4C9B
P 2500 3850
F 0 "R303" V 2580 3850 50  0000 C CNN
F 1 "100" V 2500 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2430 3850 30  0001 C CNN
F 3 "" H 2500 3850 30  0000 C CNN
	1    2500 3850
	0    1    1    0   
$EndComp
$Comp
L R R307
U 1 1 563E4CC0
P 2500 4050
F 0 "R307" V 2580 4050 50  0000 C CNN
F 1 "100" V 2500 4050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2430 4050 30  0001 C CNN
F 3 "" H 2500 4050 30  0000 C CNN
	1    2500 4050
	0    1    1    0   
$EndComp
$Comp
L R R304
U 1 1 563E4CE8
P 3500 3850
F 0 "R304" V 3580 3850 50  0000 C CNN
F 1 "100" V 3500 3850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3430 3850 30  0001 C CNN
F 3 "" H 3500 3850 30  0000 C CNN
	1    3500 3850
	0    1    1    0   
$EndComp
$Comp
L R R308
U 1 1 563E4D23
P 3500 4050
F 0 "R308" V 3580 4050 50  0000 C CNN
F 1 "100" V 3500 4050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3430 4050 30  0001 C CNN
F 3 "" H 3500 4050 30  0000 C CNN
	1    3500 4050
	0    1    1    0   
$EndComp
Text HLabel 1900 3950 0    60   Output ~ 0
LED4
Text HLabel 4000 3950 2    60   Output ~ 0
LED5
Text HLabel 1900 4050 0    60   Output ~ 0
LED6
Text HLabel 4000 4050 2    60   Output ~ 0
LED7
Wire Wire Line
	3300 1950 3100 1950
Wire Wire Line
	3100 1950 3100 1800
Wire Wire Line
	3100 1800 2800 1800
Wire Wire Line
	3300 2050 2800 2050
Wire Wire Line
	3300 2150 3100 2150
Wire Wire Line
	3100 2150 3100 2300
Wire Wire Line
	3100 2300 2800 2300
Wire Wire Line
	3800 2150 4000 2150
Wire Wire Line
	4000 2150 4000 2300
Wire Wire Line
	4000 2300 4300 2300
Wire Wire Line
	3800 2050 4300 2050
Wire Wire Line
	3800 1950 4000 1950
Wire Wire Line
	4000 1950 4000 1800
Wire Wire Line
	4000 1800 5300 1800
Wire Wire Line
	2350 3750 2750 3750
Wire Wire Line
	3250 3750 3650 3750
Wire Wire Line
	1900 3850 2350 3850
Wire Wire Line
	3650 4050 4000 4050
Wire Wire Line
	2650 3350 2750 3350
Wire Wire Line
	2650 3450 2750 3450
Connection ~ 2650 3450
Wire Wire Line
	2650 3550 2750 3550
Connection ~ 2650 3550
Wire Wire Line
	2650 3650 2750 3650
Connection ~ 2650 3650
Wire Wire Line
	2350 3950 2750 3950
Wire Wire Line
	2650 4050 2750 4050
Wire Wire Line
	4000 3750 3950 3750
Wire Wire Line
	2200 3200 3350 3200
Connection ~ 2650 3350
Wire Wire Line
	3350 3200 3350 3650
Wire Wire Line
	3350 3350 3250 3350
Wire Wire Line
	3350 3450 3250 3450
Connection ~ 3350 3350
Wire Wire Line
	3350 3550 3250 3550
Connection ~ 3350 3450
Wire Wire Line
	3350 3650 3250 3650
Connection ~ 3350 3550
Wire Wire Line
	2650 3200 2650 3650
Wire Wire Line
	3350 4050 3250 4050
Wire Wire Line
	3250 3950 3650 3950
Wire Wire Line
	3950 3950 4000 3950
Wire Wire Line
	4000 3850 3650 3850
Wire Wire Line
	3350 3850 3250 3850
Wire Wire Line
	1900 3750 2050 3750
Wire Wire Line
	2650 3850 2750 3850
Wire Wire Line
	2050 3950 1900 3950
Wire Wire Line
	1900 4050 2350 4050
Connection ~ 2650 3200
Wire Wire Line
	5300 1800 5300 1900
$EndSCHEMATC
