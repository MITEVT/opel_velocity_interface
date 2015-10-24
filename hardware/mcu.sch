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
Sheet 2 6
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
L LPC11C14 U2
U 1 1 55FD8CC8
P 4900 3850
F 0 "U2" H 3750 5700 60  0000 C CNN
F 1 "LPC11C14" H 5700 2050 60  0000 C CNN
F 2 "Housings_QFP:LQFP-48_7x7mm_Pitch0.5mm" H 5050 2650 60  0001 C CNN
F 3 "" H 5050 2650 60  0000 C CNN
	1    4900 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 5800 4700 6000
Wire Wire Line
	4700 6000 4900 6000
Wire Wire Line
	4900 6000 4900 5800
Wire Wire Line
	4800 6000 4800 6200
Connection ~ 4800 6000
$Comp
L GND #PWR03
U 1 1 55FD8CD4
P 4800 6200
F 0 "#PWR03" H 4800 5950 50  0001 C CNN
F 1 "GND" H 4800 6050 50  0000 C CNN
F 2 "" H 4800 6200 60  0000 C CNN
F 3 "" H 4800 6200 60  0000 C CNN
	1    4800 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1850 4600 1700
Wire Wire Line
	4600 1700 4800 1700
Wire Wire Line
	4800 1150 4800 1850
Connection ~ 4800 1700
$Comp
L C_Small C5
U 1 1 55FD8CDE
P 5000 1500
F 0 "C5" H 5010 1570 50  0000 L CNN
F 1 "0.1uF" H 5010 1420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 5000 1500 60  0001 C CNN
F 3 "" H 5000 1500 60  0000 C CNN
	1    5000 1500
	1    0    0    -1  
$EndComp
$Comp
L C_Small C6
U 1 1 55FD8CE5
P 5400 1500
F 0 "C6" H 5410 1570 50  0000 L CNN
F 1 "1=uF" H 5410 1420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5400 1500 60  0001 C CNN
F 3 "" H 5400 1500 60  0000 C CNN
	1    5400 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 1300 5400 1300
Wire Wire Line
	5400 1300 5400 1400
Wire Wire Line
	5000 1300 5000 1400
Connection ~ 5000 1300
Wire Wire Line
	5000 1700 5400 1700
Wire Wire Line
	5400 1600 5400 1800
Wire Wire Line
	5000 1600 5000 1700
$Comp
L GND #PWR04
U 1 1 55FD8CF3
P 5400 1800
F 0 "#PWR04" H 5400 1550 50  0001 C CNN
F 1 "GND" H 5400 1650 50  0000 C CNN
F 2 "" H 5400 1800 60  0000 C CNN
F 3 "" H 5400 1800 60  0000 C CNN
	1    5400 1800
	1    0    0    -1  
$EndComp
Connection ~ 5400 1700
Connection ~ 4800 1300
$Comp
L +3.3V #PWR05
U 1 1 55FD8CFB
P 4800 1150
F 0 "#PWR05" H 4800 1000 50  0001 C CNN
F 1 "+3.3V" H 4800 1290 50  0000 C CNN
F 2 "" H 4800 1150 60  0000 C CNN
F 3 "" H 4800 1150 60  0000 C CNN
	1    4800 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4950 2400 4950
Wire Wire Line
	3500 5050 2400 5050
$Comp
L ISO1050 U1
U 1 1 55FD8DD3
P 1800 5150
F 0 "U1" H 1450 5500 60  0000 C CNN
F 1 "ISO1050" H 2100 5500 60  0000 C CNN
F 2 "Housings_SSOP:MSOP-8_3x3mm_Pitch0.65mm" H 2100 5500 60  0001 C CNN
F 3 "" H 2100 5500 60  0000 C CNN
	1    1800 5150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1200 5050 1000 5050
Wire Wire Line
	1200 5250 1000 5250
Text HLabel 1000 5050 0    60   BiDi ~ 0
CAN1L
Text HLabel 1000 5250 0    60   BiDi ~ 0
CAN1H
$Comp
L +3.3V #PWR06
U 1 1 55FD9041
P 1900 3850
F 0 "#PWR06" H 1900 3700 50  0001 C CNN
F 1 "+3.3V" H 1900 3990 50  0000 C CNN
F 2 "" H 1900 3850 60  0000 C CNN
F 3 "" H 1900 3850 60  0000 C CNN
	1    1900 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 3850 1900 4650
$Comp
L C_Small C3
U 1 1 55FD90AA
P 2100 4250
F 0 "C3" H 2110 4320 50  0000 L CNN
F 1 "0.1uF" H 2110 4170 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 2100 4250 60  0001 C CNN
F 3 "" H 2100 4250 60  0000 C CNN
	1    2100 4250
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 55FD90FE
P 2400 4250
F 0 "C4" H 2410 4320 50  0000 L CNN
F 1 "1uF" H 2410 4170 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2400 4250 60  0001 C CNN
F 3 "" H 2400 4250 60  0000 C CNN
	1    2400 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 3950 2400 3950
Wire Wire Line
	2100 3950 2100 4150
Wire Wire Line
	2400 3950 2400 4150
Connection ~ 2100 3950
Connection ~ 1900 3950
Wire Wire Line
	2100 4350 2100 4500
Wire Wire Line
	2400 4350 2400 4550
$Comp
L GND #PWR07
U 1 1 55FD9221
P 2400 4550
F 0 "#PWR07" H 2400 4300 50  0001 C CNN
F 1 "GND" H 2400 4400 50  0000 C CNN
F 2 "" H 2400 4550 60  0000 C CNN
F 3 "" H 2400 4550 60  0000 C CNN
	1    2400 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 4500 2400 4500
Connection ~ 2400 4500
$Comp
L C_Small C2
U 1 1 55FD94B7
P 1450 4250
F 0 "C2" H 1460 4320 50  0000 L CNN
F 1 "0.1uF" H 1460 4170 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1450 4250 60  0001 C CNN
F 3 "" H 1450 4250 60  0000 C CNN
	1    1450 4250
	-1   0    0    -1  
$EndComp
$Comp
L C_Small C1
U 1 1 55FD94BD
P 1150 4250
F 0 "C1" H 1160 4320 50  0000 L CNN
F 1 "1uF" H 1160 4170 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1150 4250 60  0001 C CNN
F 3 "" H 1150 4250 60  0000 C CNN
	1    1150 4250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1150 3950 1700 3950
Wire Wire Line
	1450 3950 1450 4150
Wire Wire Line
	1150 3950 1150 4150
Connection ~ 1450 3950
Wire Wire Line
	1450 4350 1450 4500
Wire Wire Line
	1150 4350 1150 4600
Wire Wire Line
	1700 3750 1700 4650
Connection ~ 1700 3950
$Comp
L GND #PWR08
U 1 1 55FD9599
P 1150 4600
F 0 "#PWR08" H 1150 4350 50  0001 C CNN
F 1 "GND" H 1150 4450 50  0000 C CNN
F 2 "" H 1150 4600 60  0000 C CNN
F 3 "" H 1150 4600 60  0000 C CNN
	1    1150 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 4500 1150 4500
Connection ~ 1150 4500
Wire Wire Line
	1900 5650 1900 5850
$Comp
L GND #PWR09
U 1 1 55FD9670
P 1900 5850
F 0 "#PWR09" H 1900 5600 50  0001 C CNN
F 1 "GND" H 1900 5700 50  0000 C CNN
F 2 "" H 1900 5850 60  0000 C CNN
F 3 "" H 1900 5850 60  0000 C CNN
	1    1900 5850
	1    0    0    -1  
$EndComp
$Comp
L GNDD #PWR010
U 1 1 55FD969C
P 1700 5850
F 0 "#PWR010" H 1700 5600 50  0001 C CNN
F 1 "GNDD" H 1700 5700 50  0000 C CNN
F 2 "" H 1700 5850 60  0000 C CNN
F 3 "" H 1700 5850 60  0000 C CNN
	1    1700 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5650 1700 5850
Wire Wire Line
	6150 2550 6550 2550
Wire Wire Line
	6150 2650 6550 2650
Text HLabel 6550 2550 2    60   Input ~ 0
RXD
Text HLabel 6550 2650 2    60   Output ~ 0
TXD
Wire Wire Line
	1950 2550 3500 2550
$Comp
L R_Small R1
U 1 1 55FD98BA
P 2150 2350
F 0 "R1" H 2180 2370 50  0000 L CNN
F 1 "10k" H 2180 2310 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 2150 2350 60  0001 C CNN
F 3 "" H 2150 2350 60  0000 C CNN
	1    2150 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 2550 2150 2450
Wire Wire Line
	2150 2250 2150 2100
Wire Wire Line
	3050 2450 3500 2450
Wire Wire Line
	3200 2450 3200 2350
$Comp
L R_Small R5
U 1 1 55FD99AE
P 3200 2250
F 0 "R5" H 3230 2270 50  0000 L CNN
F 1 "10k" H 3230 2210 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 3200 2250 60  0001 C CNN
F 3 "" H 3200 2250 60  0000 C CNN
	1    3200 2250
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR011
U 1 1 55FD9A20
P 2150 2100
F 0 "#PWR011" H 2150 1950 50  0001 C CNN
F 1 "+3.3V" H 2150 2240 50  0000 C CNN
F 2 "" H 2150 2100 60  0000 C CNN
F 3 "" H 2150 2100 60  0000 C CNN
	1    2150 2100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR012
U 1 1 55FD9A49
P 3200 2100
F 0 "#PWR012" H 3200 1950 50  0001 C CNN
F 1 "+3.3V" H 3200 2240 50  0000 C CNN
F 2 "" H 3200 2100 60  0000 C CNN
F 3 "" H 3200 2100 60  0000 C CNN
	1    3200 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2100 3200 2150
Connection ~ 2150 2550
Text HLabel 1950 2550 0    60   Input ~ 0
MCU_RESET_N
Text HLabel 3050 2450 0    60   Input ~ 0
MCU_PROG_N
Connection ~ 3200 2450
$Comp
L LED D3
U 1 1 56198BE4
P 3100 6300
F 0 "D3" H 3100 6400 50  0000 C CNN
F 1 "LED" H 3100 6200 50  0000 C CNN
F 2 "LEDs:LED-0805" H 3100 6300 60  0001 C CNN
F 3 "" H 3100 6300 60  0000 C CNN
	1    3100 6300
	0    -1   -1   0   
$EndComp
$Comp
L LED D4
U 1 1 56198CE9
P 3400 6300
F 0 "D4" H 3400 6400 50  0000 C CNN
F 1 "LED" H 3400 6200 50  0000 C CNN
F 2 "LEDs:LED-0805" H 3400 6300 60  0001 C CNN
F 3 "" H 3400 6300 60  0000 C CNN
	1    3400 6300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3500 5450 3400 5450
Wire Wire Line
	3100 5350 3500 5350
$Comp
L LED D2
U 1 1 56198FDC
P 2800 6300
F 0 "D2" H 2800 6400 50  0000 C CNN
F 1 "LED" H 2800 6200 50  0000 C CNN
F 2 "LEDs:LED-0805" H 2800 6300 60  0001 C CNN
F 3 "" H 2800 6300 60  0000 C CNN
	1    2800 6300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3500 5250 2800 5250
$Comp
L LED D1
U 1 1 5619916F
P 2500 6300
F 0 "D1" H 2500 6400 50  0000 C CNN
F 1 "LED" H 2500 6200 50  0000 C CNN
F 2 "LEDs:LED-0805" H 2500 6300 60  0001 C CNN
F 3 "" H 2500 6300 60  0000 C CNN
	1    2500 6300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3500 5150 2500 5150
$Comp
L GND #PWR019
U 1 1 5619924D
P 2500 6600
F 0 "#PWR019" H 2500 6350 50  0001 C CNN
F 1 "GND" H 2500 6450 50  0000 C CNN
F 2 "" H 2500 6600 60  0000 C CNN
F 3 "" H 2500 6600 60  0000 C CNN
	1    2500 6600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 56199294
P 2800 6600
F 0 "#PWR020" H 2800 6350 50  0001 C CNN
F 1 "GND" H 2800 6450 50  0000 C CNN
F 2 "" H 2800 6600 60  0000 C CNN
F 3 "" H 2800 6600 60  0000 C CNN
	1    2800 6600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 561992DB
P 3100 6600
F 0 "#PWR021" H 3100 6350 50  0001 C CNN
F 1 "GND" H 3100 6450 50  0000 C CNN
F 2 "" H 3100 6600 60  0000 C CNN
F 3 "" H 3100 6600 60  0000 C CNN
	1    3100 6600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 56199322
P 3400 6600
F 0 "#PWR022" H 3400 6350 50  0001 C CNN
F 1 "GND" H 3400 6450 50  0000 C CNN
F 2 "" H 3400 6600 60  0000 C CNN
F 3 "" H 3400 6600 60  0000 C CNN
	1    3400 6600
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 561994AC
P 2500 5800
F 0 "R2" V 2580 5800 50  0000 C CNN
F 1 "100" V 2500 5800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2430 5800 30  0001 C CNN
F 3 "" H 2500 5800 30  0000 C CNN
	1    2500 5800
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 56199538
P 2800 5800
F 0 "R3" V 2880 5800 50  0000 C CNN
F 1 "100" V 2800 5800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 2730 5800 30  0001 C CNN
F 3 "" H 2800 5800 30  0000 C CNN
	1    2800 5800
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 561995C8
P 3100 5800
F 0 "R4" V 3180 5800 50  0000 C CNN
F 1 "100" V 3100 5800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3030 5800 30  0001 C CNN
F 3 "" H 3100 5800 30  0000 C CNN
	1    3100 5800
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5619962D
P 3400 5800
F 0 "R6" V 3480 5800 50  0000 C CNN
F 1 "100" V 3400 5800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3330 5800 30  0001 C CNN
F 3 "" H 3400 5800 30  0000 C CNN
	1    3400 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 5450 3400 5650
Wire Wire Line
	3100 5350 3100 5650
Wire Wire Line
	2800 5250 2800 5650
Wire Wire Line
	2500 5150 2500 5650
Wire Wire Line
	2500 5950 2500 6100
Wire Wire Line
	2800 5950 2800 6100
Wire Wire Line
	3100 5950 3100 6100
Wire Wire Line
	3400 5950 3400 6100
Wire Wire Line
	2500 6500 2500 6600
Wire Wire Line
	2800 6500 2800 6600
Wire Wire Line
	3100 6500 3100 6600
Wire Wire Line
	3400 6500 3400 6600
$Comp
L SPST SW2
U 1 1 5619B003
P 2850 2950
F 0 "SW2" H 2850 3050 50  0000 C CNN
F 1 "SPST" H 2850 2850 50  0000 C CNN
F 2 "Buttons_Switches_ThroughHole:SW_Micro_SPST" H 2850 2950 60  0001 C CNN
F 3 "" H 2850 2950 60  0000 C CNN
	1    2850 2950
	-1   0    0    1   
$EndComp
$Comp
L SPST SW1
U 1 1 5619B1B9
P 1550 2950
F 0 "SW1" H 1550 3050 50  0000 C CNN
F 1 "SPST" H 1550 2850 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_EVPBF" H 1550 2950 60  0001 C CNN
F 3 "" H 1550 2950 60  0000 C CNN
	1    1550 2950
	-1   0    0    1   
$EndComp
Wire Wire Line
	3350 2450 3350 2950
Connection ~ 3350 2450
Wire Wire Line
	2350 2950 2250 2950
Wire Wire Line
	2250 2950 2250 3100
$Comp
L GND #PWR028
U 1 1 5619B5FE
P 2250 3100
F 0 "#PWR028" H 2250 2850 50  0001 C CNN
F 1 "GND" H 2250 2950 50  0000 C CNN
F 2 "" H 2250 3100 60  0000 C CNN
F 3 "" H 2250 3100 60  0000 C CNN
	1    2250 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 5619B65D
P 950 3050
F 0 "#PWR029" H 950 2800 50  0001 C CNN
F 1 "GND" H 950 2900 50  0000 C CNN
F 2 "" H 950 3050 60  0000 C CNN
F 3 "" H 950 3050 60  0000 C CNN
	1    950  3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  3050 950  2950
Wire Wire Line
	950  2950 1050 2950
Wire Wire Line
	2050 2950 2050 2550
Connection ~ 2050 2550
Wire Wire Line
	3500 3550 3150 3550
Wire Wire Line
	3150 3650 3500 3650
Wire Wire Line
	3500 3750 3150 3750
Wire Wire Line
	3150 3850 3500 3850
Wire Wire Line
	3500 3950 3150 3950
Wire Wire Line
	3150 4050 3500 4050
Wire Wire Line
	3500 4150 3150 4150
Wire Wire Line
	3150 4250 3500 4250
Text Label 3150 3550 2    60   ~ 0
PIO0_11
Text Label 3150 3650 2    60   ~ 0
PIO1_0
Text Label 3150 3750 2    60   ~ 0
PIO1_1
Text Label 3150 3850 2    60   ~ 0
PIO1_2
Text Label 3150 3950 2    60   ~ 0
PIO1_3
Text Label 3150 4050 2    60   ~ 0
PIO1_4
Text Label 3150 4150 2    60   ~ 0
PIO1_10
Text Label 3150 4250 2    60   ~ 0
PIO1_11
Wire Wire Line
	6150 4550 6600 4550
Wire Wire Line
	6150 4650 6600 4650
Wire Wire Line
	6150 4750 6600 4750
Wire Wire Line
	6150 4850 6600 4850
Wire Wire Line
	6150 4950 6600 4950
Wire Wire Line
	6150 5150 6600 5150
Wire Wire Line
	6150 5250 6600 5250
Wire Wire Line
	6150 5350 6600 5350
Wire Wire Line
	6150 5450 6600 5450
Text Label 6600 4650 0    60   ~ 0
PIO1_8
Text Label 6600 4750 0    60   ~ 0
PIO2_6
Text Label 6600 4850 0    60   ~ 0
PIO2_7
Text Label 6600 4950 0    60   ~ 0
PIO2_8
Text Label 6600 5150 0    60   ~ 0
PIO3_0
Text Label 6600 5250 0    60   ~ 0
PIO3_1
Text Label 6600 5350 0    60   ~ 0
PIO3_2
Text Label 6600 5450 0    60   ~ 0
PIO3_3
NoConn ~ 3500 2650
NoConn ~ 3500 2750
Wire Wire Line
	1700 3750 950  3750
Text HLabel 950  3750 0    60   Input ~ 0
CAN1P
Wire Wire Line
	6150 4100 6600 4100
Text Label 6600 4100 0    60   ~ 0
PIO0_5
Wire Wire Line
	6150 4000 6600 4000
Text Label 6600 4000 0    60   ~ 0
PIO0_4
Wire Wire Line
	6150 4350 6600 4350
Text Label 6600 4350 0    60   ~ 0
PIO0_3
Wire Wire Line
	6150 3700 6600 3700
Text Label 6600 3700 0    60   ~ 0
PIO2_1
Text Label 3500 4850 2    60   ~ 0
PIO2_4
Text Label 3500 4750 2    60   ~ 0
PIO1_9
Wire Wire Line
	6150 3800 6600 3800
Text Label 6600 3800 0    60   ~ 0
PIO0_1
Wire Wire Line
	6150 3500 6600 3500
Text Label 6600 3500 0    60   ~ 0
PIO2_2
Wire Wire Line
	6150 3600 6600 3600
Text Label 6600 3600 0    60   ~ 0
PIO2_3
NoConn ~ 6150 2800
NoConn ~ 6150 2900
NoConn ~ 6150 3000
NoConn ~ 6150 3100
NoConn ~ 6150 3300
NoConn ~ 6150 5050
Text HLabel 6600 4550 2    60   Input ~ 0
WVS_PULSE
$EndSCHEMATC
