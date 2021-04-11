EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 8
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
L power:GND #PWR0117
U 1 1 5ECD9143
P 4050 3800
F 0 "#PWR0117" H 4050 3550 50  0001 C CNN
F 1 "GND" H 4055 3627 50  0000 C CNN
F 2 "" H 4050 3800 50  0001 C CNN
F 3 "" H 4050 3800 50  0001 C CNN
	1    4050 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal_GND24 Y601
U 1 1 5ECDCBB7
P 4050 2950
F 0 "Y601" V 3950 3150 50  0000 L CNN
F 1 "Crystal_GND24" V 3850 3150 50  0000 L CNN
F 2 "bbcar_stw_adc:Crystal_SMD_Abracon_ABMM2-4Pin_6.0x3.6mm" H 4050 2950 50  0001 C CNN
F 3 "~" H 4050 2950 50  0001 C CNN
F 4 "815-ABMM2-8-E2T" V 4050 2950 50  0001 C CNN "Mouser Nr"
	1    4050 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 3700 4050 3800
Wire Wire Line
	3850 2950 3750 2950
Wire Wire Line
	3750 2950 3750 3700
Wire Wire Line
	3750 3700 4050 3700
Wire Wire Line
	4250 2950 4350 2950
Wire Wire Line
	4350 2950 4350 3700
Wire Wire Line
	4350 3700 4050 3700
Connection ~ 4050 3700
Wire Wire Line
	4050 2800 4050 2700
Wire Wire Line
	4050 3100 4050 3200
Wire Wire Line
	4050 3200 3400 3200
Wire Wire Line
	3750 3700 3400 3700
Connection ~ 3750 3700
$Comp
L Device:C C602
U 1 1 5ECE0095
P 3400 3450
F 0 "C602" H 3515 3496 50  0000 L CNN
F 1 "18p" H 3515 3405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3438 3300 50  0001 C CNN
F 3 "~" H 3400 3450 50  0001 C CNN
F 4 "80-C0603C180J5GAUTO" H 3400 3450 50  0001 C CNN "Mouser Nr"
	1    3400 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 3300 3400 3200
Wire Wire Line
	3400 3600 3400 3700
Connection ~ 3400 3700
Wire Wire Line
	3400 3700 2950 3700
Wire Wire Line
	2950 2700 4050 2700
$Comp
L Device:C C601
U 1 1 5ECE0DF9
P 2950 3450
F 0 "C601" H 3065 3496 50  0000 L CNN
F 1 "18p" H 3065 3405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2988 3300 50  0001 C CNN
F 3 "~" H 2950 3450 50  0001 C CNN
F 4 "80-C0603C180J5GAUTO" H 2950 3450 50  0001 C CNN "Mouser Nr"
	1    2950 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 2700 2950 3300
Wire Wire Line
	2950 3600 2950 3700
Text HLabel 2750 2700 0    50   UnSpc ~ 0
XTAL1
Wire Wire Line
	2750 2700 2950 2700
Connection ~ 2950 2700
Text HLabel 2750 3200 0    50   UnSpc ~ 0
XTAL2
Wire Wire Line
	2750 3200 3400 3200
Connection ~ 3400 3200
$EndSCHEMATC
