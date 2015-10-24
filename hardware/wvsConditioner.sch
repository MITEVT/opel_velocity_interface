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
Sheet 6 6
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
L +12V #PWR?
U 1 1 562C1B14
P 5600 1900
F 0 "#PWR?" H 5600 1750 50  0001 C CNN
F 1 "+12V" H 5600 2040 50  0000 C CNN
F 2 "" H 5600 1900 60  0000 C CNN
F 3 "" H 5600 1900 60  0000 C CNN
	1    5600 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 1900 5600 2800
Wire Wire Line
	5600 3600 5600 4100
$Comp
L GND #PWR?
U 1 1 562C1B38
P 5600 4100
F 0 "#PWR?" H 5600 3850 50  0001 C CNN
F 1 "GND" H 5600 3950 50  0000 C CNN
F 2 "" H 5600 4100 60  0000 C CNN
F 3 "" H 5600 4100 60  0000 C CNN
	1    5600 4100
	1    0    0    -1  
$EndComp
$Comp
L C_Small C?
U 1 1 562C1B52
P 5850 2100
F 0 "C?" H 5860 2170 50  0000 L CNN
F 1 "0.1uF" H 5860 2020 50  0000 L CNN
F 2 "" H 5850 2100 60  0000 C CNN
F 3 "" H 5850 2100 60  0000 C CNN
	1    5850 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 1950 5850 1950
Wire Wire Line
	5850 1950 5850 2000
Connection ~ 5600 1950
Wire Wire Line
	5850 2200 5850 2300
$Comp
L GND #PWR?
U 1 1 562C1BEE
P 5850 2300
F 0 "#PWR?" H 5850 2050 50  0001 C CNN
F 1 "GND" H 5850 2150 50  0000 C CNN
F 2 "" H 5850 2300 60  0000 C CNN
F 3 "" H 5850 2300 60  0000 C CNN
	1    5850 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3100 5200 3100
Wire Wire Line
	4300 3300 5200 3300
$Comp
L R_Small R?
U 1 1 562C1C7E
P 4650 3600
F 0 "R?" H 4680 3620 50  0000 L CNN
F 1 "R_Small" H 4680 3560 50  0000 L CNN
F 2 "" H 4650 3600 60  0000 C CNN
F 3 "" H 4650 3600 60  0000 C CNN
	1    4650 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 3300 4650 3500
Connection ~ 4650 3300
$Comp
L D_Schottky D?
U 1 1 562C1CED
P 5200 3800
F 0 "D?" H 5200 3900 50  0000 C CNN
F 1 "D_Schottky" H 5200 3700 50  0000 C CNN
F 2 "" H 5200 3800 60  0000 C CNN
F 3 "" H 5200 3800 60  0000 C CNN
	1    5200 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 3300 5000 4100
Wire Wire Line
	5000 4100 5200 4100
Wire Wire Line
	5200 3950 5200 4200
Connection ~ 5000 3300
$Comp
L +12V #PWR?
U 1 1 562C1D76
P 5200 3550
F 0 "#PWR?" H 5200 3400 50  0001 C CNN
F 1 "+12V" H 5200 3690 50  0000 C CNN
F 2 "" H 5200 3550 60  0000 C CNN
F 3 "" H 5200 3550 60  0000 C CNN
	1    5200 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 3550 5200 3650
$Comp
L D_Schottky D?
U 1 1 562C1E35
P 5200 4350
F 0 "D?" H 5200 4450 50  0000 C CNN
F 1 "D_Schottky" H 5200 4250 50  0000 C CNN
F 2 "" H 5200 4350 60  0000 C CNN
F 3 "" H 5200 4350 60  0000 C CNN
	1    5200 4350
	0    1    1    0   
$EndComp
Connection ~ 5200 4100
$Comp
L GND #PWR?
U 1 1 562C1E74
P 5200 4600
F 0 "#PWR?" H 5200 4350 50  0001 C CNN
F 1 "GND" H 5200 4450 50  0000 C CNN
F 2 "" H 5200 4600 60  0000 C CNN
F 3 "" H 5200 4600 60  0000 C CNN
	1    5200 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4500 5200 4600
Wire Wire Line
	4650 3700 4650 3900
$Comp
L LM358 U?
U 1 1 562C1EF8
P 4000 5350
F 0 "U?" H 3950 5550 60  0000 L CNN
F 1 "LM358" H 3950 5100 60  0000 L CNN
F 2 "" H 4000 5350 60  0000 C CNN
F 3 "" H 4000 5350 60  0000 C CNN
	1    4000 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5450 3400 5450
Wire Wire Line
	3400 5450 3400 6100
Wire Wire Line
	3400 6100 4700 6100
Wire Wire Line
	4700 6100 4700 5350
Wire Wire Line
	4500 5350 5000 5350
$Comp
L GND #PWR?
U 1 1 562C2006
P 3900 5800
F 0 "#PWR?" H 3900 5550 50  0001 C CNN
F 1 "GND" H 3900 5650 50  0000 C CNN
F 2 "" H 3900 5800 60  0000 C CNN
F 3 "" H 3900 5800 60  0000 C CNN
	1    3900 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 5750 3900 5800
Wire Wire Line
	3500 5250 3100 5250
$Comp
L R_Small R?
U 1 1 562C20C7
P 3100 5000
F 0 "R?" H 3130 5020 50  0000 L CNN
F 1 "R_Small" H 3130 4960 50  0000 L CNN
F 2 "" H 3100 5000 60  0000 C CNN
F 3 "" H 3100 5000 60  0000 C CNN
	1    3100 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 5100 3100 5400
$Comp
L +12V #PWR?
U 1 1 562C2135
P 3100 4750
F 0 "#PWR?" H 3100 4600 50  0001 C CNN
F 1 "+12V" H 3100 4890 50  0000 C CNN
F 2 "" H 3100 4750 60  0000 C CNN
F 3 "" H 3100 4750 60  0000 C CNN
	1    3100 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 4750 3100 4900
$Comp
L R_Small R?
U 1 1 562C21BF
P 3100 5500
F 0 "R?" H 3130 5520 50  0000 L CNN
F 1 "R_Small" H 3130 5460 50  0000 L CNN
F 2 "" H 3100 5500 60  0000 C CNN
F 3 "" H 3100 5500 60  0000 C CNN
	1    3100 5500
	1    0    0    -1  
$EndComp
Connection ~ 3100 5250
Wire Wire Line
	3100 5600 3100 5700
$Comp
L GND #PWR?
U 1 1 562C2257
P 3100 5700
F 0 "#PWR?" H 3100 5450 50  0001 C CNN
F 1 "GND" H 3100 5550 50  0000 C CNN
F 2 "" H 3100 5700 60  0000 C CNN
F 3 "" H 3100 5700 60  0000 C CNN
	1    3100 5700
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR?
U 1 1 562C23D8
P 3900 4450
F 0 "#PWR?" H 3900 4300 50  0001 C CNN
F 1 "+12V" H 3900 4590 50  0000 C CNN
F 2 "" H 3900 4450 60  0000 C CNN
F 3 "" H 3900 4450 60  0000 C CNN
	1    3900 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4450 3900 4950
$Comp
L C_Small C?
U 1 1 562C23DF
P 4150 4650
F 0 "C?" H 4160 4720 50  0000 L CNN
F 1 "0.1uF" H 4160 4570 50  0000 L CNN
F 2 "" H 4150 4650 60  0000 C CNN
F 3 "" H 4150 4650 60  0000 C CNN
	1    4150 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4500 4150 4500
Wire Wire Line
	4150 4500 4150 4550
Connection ~ 3900 4500
$Comp
L GND #PWR?
U 1 1 562C23FF
P 4150 4850
F 0 "#PWR?" H 4150 4600 50  0001 C CNN
F 1 "GND" H 4150 4700 50  0000 C CNN
F 2 "" H 4150 4850 60  0000 C CNN
F 3 "" H 4150 4850 60  0000 C CNN
	1    4150 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 4750 4150 4850
Text Label 5000 5350 0    60   ~ 0
vref
Text Label 4650 3900 3    60   ~ 0
vref
Connection ~ 4700 5350
$Comp
L D_Schottky D?
U 1 1 562C2C4D
P 5200 1650
F 0 "D?" H 5200 1750 50  0000 C CNN
F 1 "D_Schottky" H 5200 1550 50  0000 C CNN
F 2 "" H 5200 1650 60  0000 C CNN
F 3 "" H 5200 1650 60  0000 C CNN
	1    5200 1650
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 1950 5200 1950
Wire Wire Line
	5200 1800 5200 2050
$Comp
L D_Schottky D?
U 1 1 562C2C57
P 5200 2200
F 0 "D?" H 5200 2300 50  0000 C CNN
F 1 "D_Schottky" H 5200 2100 50  0000 C CNN
F 2 "" H 5200 2200 60  0000 C CNN
F 3 "" H 5200 2200 60  0000 C CNN
	1    5200 2200
	0    1    1    0   
$EndComp
Connection ~ 5200 1950
$Comp
L GND #PWR?
U 1 1 562C2C5E
P 5200 2500
F 0 "#PWR?" H 5200 2250 50  0001 C CNN
F 1 "GND" H 5200 2350 50  0000 C CNN
F 2 "" H 5200 2500 60  0000 C CNN
F 3 "" H 5200 2500 60  0000 C CNN
	1    5200 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1950 5000 3100
Connection ~ 5000 3100
$Comp
L +12V #PWR?
U 1 1 562C2E24
P 5200 1350
F 0 "#PWR?" H 5200 1200 50  0001 C CNN
F 1 "+12V" H 5200 1490 50  0000 C CNN
F 2 "" H 5200 1350 60  0000 C CNN
F 3 "" H 5200 1350 60  0000 C CNN
	1    5200 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1350 5200 1500
$Comp
L R_Small R?
U 1 1 562C2E92
P 4650 2800
F 0 "R?" H 4680 2820 50  0000 L CNN
F 1 "R_Small" H 4680 2760 50  0000 L CNN
F 2 "" H 4650 2800 60  0000 C CNN
F 3 "" H 4650 2800 60  0000 C CNN
	1    4650 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2900 4650 3100
Connection ~ 4650 3100
Text Label 4650 2500 1    60   ~ 0
vref
Wire Wire Line
	4650 2500 4650 2700
Text HLabel 4300 3300 0    60   Input ~ 0
WVS-
Text HLabel 4300 3100 0    60   Input ~ 0
WVS+
Wire Wire Line
	6200 3300 6300 3300
Wire Wire Line
	6300 3300 6300 3450
$Comp
L GND #PWR?
U 1 1 562C326E
P 6300 3450
F 0 "#PWR?" H 6300 3200 50  0001 C CNN
F 1 "GND" H 6300 3300 50  0000 C CNN
F 2 "" H 6300 3450 60  0000 C CNN
F 3 "" H 6300 3450 60  0000 C CNN
	1    6300 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3200 7000 3200
Wire Wire Line
	6500 3200 6500 3050
$Comp
L R_Small R?
U 1 1 562C32FD
P 6500 2950
F 0 "R?" H 6530 2970 50  0000 L CNN
F 1 "10k" H 6530 2910 50  0000 L CNN
F 2 "" H 6500 2950 60  0000 C CNN
F 3 "" H 6500 2950 60  0000 C CNN
	1    6500 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2850 6500 2700
$Comp
L +3.3V #PWR?
U 1 1 562C33E1
P 6500 2700
F 0 "#PWR?" H 6500 2550 50  0001 C CNN
F 1 "+3.3V" H 6500 2840 50  0000 C CNN
F 2 "" H 6500 2700 60  0000 C CNN
F 3 "" H 6500 2700 60  0000 C CNN
	1    6500 2700
	1    0    0    -1  
$EndComp
$Comp
L LM311N U?
U 1 1 562C19AC
P 5700 3200
F 0 "U?" H 5900 3500 70  0000 C CNN
F 1 "LM311N" H 5900 3400 70  0000 C CNN
F 2 "" H 5700 3200 60  0000 C CNN
F 3 "" H 5700 3200 60  0000 C CNN
	1    5700 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 2350 5200 2500
$Comp
L R_Small R?
U 1 1 562C3805
P 5900 2750
F 0 "R?" H 5930 2770 50  0000 L CNN
F 1 "87k" H 5930 2710 50  0000 L CNN
F 2 "" H 5900 2750 60  0000 C CNN
F 3 "" H 5900 2750 60  0000 C CNN
	1    5900 2750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6400 3200 6400 2750
Wire Wire Line
	6400 2750 6000 2750
Connection ~ 6400 3200
Wire Wire Line
	5800 2750 5000 2750
Connection ~ 5000 2750
Connection ~ 6500 3200
Text HLabel 7000 3200 2    60   Output ~ 0
WVS_PULSE
$EndSCHEMATC
