EESchema Schematic File Version 2
LIBS:final_outline-rescue
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
L LM358 U601
U 1 1 5650E22C
P 3600 5500
F 0 "U601" H 3550 5700 60  0000 L CNN
F 1 "LM358" H 3550 5250 60  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 3600 5500 60  0001 C CNN
F 3 "" H 3600 5500 60  0000 C CNN
	1    3600 5500
	1    0    0    -1  
$EndComp
$Comp
L LM311N U602
U 1 1 5650E272
P 6900 2850
AR Path="/5650E272" Ref="U602"  Part="1" 
AR Path="/5650E01C/5650E272" Ref="U602"  Part="1" 
F 0 "U602" H 7100 3150 70  0000 C CNN
F 1 "LM311N" H 7100 3050 70  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6900 2850 60  0001 C CNN
F 3 "" H 6900 2850 60  0000 C CNN
	1    6900 2850
	1    0    0    -1  
$EndComp
$Comp
L R R601
U 1 1 5650E2D4
P 2700 5100
F 0 "R601" V 2780 5100 50  0000 C CNN
F 1 "1k" V 2700 5100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2630 5100 30  0001 C CNN
F 3 "" H 2700 5100 30  0000 C CNN
	1    2700 5100
	1    0    0    -1  
$EndComp
$Comp
L R R602
U 1 1 5650E304
P 2700 5700
F 0 "R602" V 2780 5700 50  0000 C CNN
F 1 "1k" V 2700 5700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2630 5700 30  0001 C CNN
F 3 "" H 2700 5700 30  0000 C CNN
	1    2700 5700
	1    0    0    -1  
$EndComp
$Comp
L C_Small C601
U 1 1 5650E351
P 3900 4900
F 0 "C601" H 3910 4970 50  0000 L CNN
F 1 "0.1uF" H 3910 4820 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3900 4900 60  0001 C CNN
F 3 "" H 3900 4900 60  0000 C CNN
	1    3900 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 5650E391
P 3500 6150
F 0 "#PWR022" H 3500 5900 50  0001 C CNN
F 1 "GND" H 3500 6000 50  0000 C CNN
F 2 "" H 3500 6150 60  0000 C CNN
F 3 "" H 3500 6150 60  0000 C CNN
	1    3500 6150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 5650E3B4
P 3900 5150
F 0 "#PWR023" H 3900 4900 50  0001 C CNN
F 1 "GND" H 3900 5000 50  0000 C CNN
F 2 "" H 3900 5150 60  0000 C CNN
F 3 "" H 3900 5150 60  0000 C CNN
	1    3900 5150
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR024
U 1 1 5650E3E1
P 3500 4650
F 0 "#PWR024" H 3500 4500 50  0001 C CNN
F 1 "+12V" H 3500 4790 50  0000 C CNN
F 2 "" H 3500 4650 60  0000 C CNN
F 3 "" H 3500 4650 60  0000 C CNN
	1    3500 4650
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR025
U 1 1 5650E402
P 2700 4800
F 0 "#PWR025" H 2700 4650 50  0001 C CNN
F 1 "+12V" H 2700 4940 50  0000 C CNN
F 2 "" H 2700 4800 60  0000 C CNN
F 3 "" H 2700 4800 60  0000 C CNN
	1    2700 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5800 3500 6150
Wire Wire Line
	3000 5600 3300 5600
Wire Wire Line
	3000 5600 3000 6350
Wire Wire Line
	3000 6350 4350 6350
Wire Wire Line
	3900 5500 4600 5500
Wire Wire Line
	4350 6350 4350 5500
Connection ~ 4350 5500
Wire Wire Line
	2700 5250 2700 5550
Wire Wire Line
	2700 5400 3300 5400
Connection ~ 2700 5400
Wire Wire Line
	2700 4800 2700 4950
$Comp
L GND #PWR026
U 1 1 5650E4A0
P 2700 6000
F 0 "#PWR026" H 2700 5750 50  0001 C CNN
F 1 "GND" H 2700 5850 50  0000 C CNN
F 2 "" H 2700 6000 60  0000 C CNN
F 3 "" H 2700 6000 60  0000 C CNN
	1    2700 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 6000 2700 5850
Wire Wire Line
	3500 4650 3500 5200
Wire Wire Line
	3900 4800 3900 4750
Wire Wire Line
	3900 4750 3500 4750
Connection ~ 3500 4750
Wire Wire Line
	3900 5000 3900 5150
Text Label 4600 5500 0    60   ~ 0
vref
$Comp
L GND #PWR027
U 1 1 5650E751
P 6800 3500
F 0 "#PWR027" H 6800 3250 50  0001 C CNN
F 1 "GND" H 6800 3350 50  0000 C CNN
F 2 "" H 6800 3500 60  0000 C CNN
F 3 "" H 6800 3500 60  0000 C CNN
	1    6800 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 3150 6800 3500
Text HLabel 8000 2850 2    60   Output ~ 0
WVS_PULSE
Wire Wire Line
	7200 2850 8000 2850
$Comp
L R R606
U 1 1 5650E7BC
P 7700 2550
F 0 "R606" V 7780 2550 50  0000 C CNN
F 1 "1k" V 7700 2550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7630 2550 30  0001 C CNN
F 3 "" H 7700 2550 30  0000 C CNN
	1    7700 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2700 7700 2850
Connection ~ 7700 2850
$Comp
L +3.3V #PWR028
U 1 1 5650E818
P 7700 2200
F 0 "#PWR028" H 7700 2050 50  0001 C CNN
F 1 "+3.3V" H 7700 2340 50  0000 C CNN
F 2 "" H 7700 2200 60  0000 C CNN
F 3 "" H 7700 2200 60  0000 C CNN
	1    7700 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 2200 7700 2400
$Comp
L C_Small C602
U 1 1 5650E87B
P 7150 1450
F 0 "C602" H 7160 1520 50  0000 L CNN
F 1 "0.1uF" H 7160 1370 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7150 1450 60  0001 C CNN
F 3 "" H 7150 1450 60  0000 C CNN
	1    7150 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 5650E8BE
P 7150 1700
F 0 "#PWR029" H 7150 1450 50  0001 C CNN
F 1 "GND" H 7150 1550 50  0000 C CNN
F 2 "" H 7150 1700 60  0000 C CNN
F 3 "" H 7150 1700 60  0000 C CNN
	1    7150 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1700 7150 1550
Wire Wire Line
	7150 1350 7150 1300
Wire Wire Line
	7150 1300 6800 1300
Wire Wire Line
	6800 1150 6800 2550
$Comp
L +12V #PWR030
U 1 1 5650E924
P 6800 1150
F 0 "#PWR030" H 6800 1000 50  0001 C CNN
F 1 "+12V" H 6800 1290 50  0000 C CNN
F 2 "" H 6800 1150 60  0000 C CNN
F 3 "" H 6800 1150 60  0000 C CNN
	1    6800 1150
	1    0    0    -1  
$EndComp
Connection ~ 6800 1300
Text HLabel 5550 2750 0    60   Input ~ 0
WVS+
Text HLabel 5550 2950 0    60   Input ~ 0
WVS-
Wire Wire Line
	5550 2750 6600 2750
Wire Wire Line
	5550 2950 6600 2950
$Comp
L R R603
U 1 1 5650EA6A
P 5850 2400
F 0 "R603" V 5930 2400 50  0000 C CNN
F 1 "1k" V 5850 2400 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5780 2400 30  0001 C CNN
F 3 "" H 5850 2400 30  0000 C CNN
	1    5850 2400
	1    0    0    -1  
$EndComp
$Comp
L R R604
U 1 1 5650EAA2
P 5850 3300
F 0 "R604" V 5930 3300 50  0000 C CNN
F 1 "1k" V 5850 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5780 3300 30  0001 C CNN
F 3 "" H 5850 3300 30  0000 C CNN
	1    5850 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3150 5850 2950
Connection ~ 5850 2950
Wire Wire Line
	5850 2750 5850 2550
Connection ~ 5850 2750
Wire Wire Line
	5850 3450 5850 3700
Wire Wire Line
	5850 2250 5850 2000
Text Label 5850 2000 1    60   ~ 0
vref
Text Label 5850 3700 3    60   ~ 0
vref
$Comp
L D_Schottky D602
U 1 1 5650EDA8
P 6400 1950
F 0 "D602" H 6400 2050 50  0000 C CNN
F 1 "CUS08F30H3FCT-ND" H 6400 1850 50  0000 C CNN
F 2 "Diodes_SMD:SOD-323" H 6400 1950 60  0001 C CNN
F 3 "" H 6400 1950 60  0000 C CNN
	1    6400 1950
	0    1    1    0   
$EndComp
$Comp
L D_Schottky D601
U 1 1 5650EE57
P 6400 1300
F 0 "D601" H 6400 1400 50  0000 C CNN
F 1 "CUS08F30H3FCT-ND" H 6400 1200 50  0000 C CNN
F 2 "Diodes_SMD:SOD-323" H 6400 1300 60  0001 C CNN
F 3 "" H 6400 1300 60  0000 C CNN
	1    6400 1300
	0    1    1    0   
$EndComp
$Comp
L R R605
U 1 1 5650EEA9
P 7000 2300
F 0 "R605" V 7080 2300 50  0000 C CNN
F 1 "87k" V 7000 2300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6930 2300 30  0001 C CNN
F 3 "" H 7000 2300 30  0000 C CNN
	1    7000 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	7150 2300 7450 2300
Wire Wire Line
	7450 2300 7450 2850
Connection ~ 7450 2850
Wire Wire Line
	6850 2300 6350 2300
Wire Wire Line
	6350 2300 6350 2750
Connection ~ 6350 2750
$Comp
L +12V #PWR031
U 1 1 5650F105
P 6450 3250
F 0 "#PWR031" H 6450 3100 50  0001 C CNN
F 1 "+12V" H 6450 3390 50  0000 C CNN
F 2 "" H 6450 3250 60  0000 C CNN
F 3 "" H 6450 3250 60  0000 C CNN
	1    6450 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 1450 6400 1800
Wire Wire Line
	6400 1650 6100 1650
Wire Wire Line
	6100 1650 6100 2750
Connection ~ 6100 2750
Connection ~ 6400 1650
$Comp
L GND #PWR032
U 1 1 5650F1ED
P 6400 2150
F 0 "#PWR032" H 6400 1900 50  0001 C CNN
F 1 "GND" H 6400 2000 50  0000 C CNN
F 2 "" H 6400 2150 60  0000 C CNN
F 3 "" H 6400 2150 60  0000 C CNN
	1    6400 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 2150 6400 2100
Wire Wire Line
	6150 3850 6150 2950
Connection ~ 6150 2950
$Comp
L D_Schottky D603
U 1 1 5650F5B7
P 6450 3500
F 0 "D603" H 6450 3600 50  0000 C CNN
F 1 "CUS08F30H3FCT-ND" H 6450 3400 50  0000 C CNN
F 2 "Diodes_SMD:SOD-323" H 6450 3500 60  0001 C CNN
F 3 "" H 6450 3500 60  0000 C CNN
	1    6450 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	6400 1000 6400 1150
$Comp
L +12V #PWR033
U 1 1 5650F6E0
P 6400 1000
F 0 "#PWR033" H 6400 850 50  0001 C CNN
F 1 "+12V" H 6400 1140 50  0000 C CNN
F 2 "" H 6400 1000 60  0000 C CNN
F 3 "" H 6400 1000 60  0000 C CNN
	1    6400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3250 6450 3350
$Comp
L D_Schottky D604
U 1 1 5650F786
P 6450 4100
F 0 "D604" H 6450 4200 50  0000 C CNN
F 1 "CUS08F30H3FCT-ND" H 6450 4000 50  0000 C CNN
F 2 "Diodes_SMD:SOD-323" H 6450 4100 60  0001 C CNN
F 3 "" H 6450 4100 60  0000 C CNN
	1    6450 4100
	0    1    1    0   
$EndComp
Wire Wire Line
	6450 3650 6450 3950
Wire Wire Line
	6150 3850 6450 3850
Connection ~ 6450 3850
$Comp
L GND #PWR034
U 1 1 5650F87C
P 6450 4400
F 0 "#PWR034" H 6450 4150 50  0001 C CNN
F 1 "GND" H 6450 4250 50  0000 C CNN
F 2 "" H 6450 4400 60  0000 C CNN
F 3 "" H 6450 4400 60  0000 C CNN
	1    6450 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 4400 6450 4250
Wire Wire Line
	2950 5500 2950 5400
Connection ~ 2950 5400
$Comp
L GND #PWR035
U 1 1 56634664
P 2950 5850
F 0 "#PWR035" H 2950 5600 50  0001 C CNN
F 1 "GND" H 2950 5700 50  0000 C CNN
F 2 "" H 2950 5850 50  0000 C CNN
F 3 "" H 2950 5850 50  0000 C CNN
	1    2950 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 5850 2950 5700
Wire Wire Line
	6800 3350 6900 3350
Wire Wire Line
	6900 3350 6900 3150
Connection ~ 6800 3350
$Comp
L C_Small C603
U 1 1 56635020
P 2950 5600
F 0 "C603" H 2960 5670 50  0000 L CNN
F 1 "1uF" H 2960 5520 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2950 5600 50  0001 C CNN
F 3 "" H 2950 5600 50  0000 C CNN
	1    2950 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 3150 7200 3150
Wire Wire Line
	7200 3150 7200 2950
$EndSCHEMATC
