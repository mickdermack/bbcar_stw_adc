EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 8
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
L Connector_Generic:Conn_01x03 J702
U 1 1 5E540163
P 7100 3950
F 0 "J702" H 7180 3992 50  0000 L CNN
F 1 "Conn_01x03" H 7180 3901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 7100 3950 50  0001 C CNN
F 3 "~" H 7100 3950 50  0001 C CNN
F 4 "~" H 7100 3950 50  0001 C CNN "Mouser Nr"
	1    7100 3950
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J701
U 1 1 5E540169
P 7100 2950
F 0 "J701" H 7180 2992 50  0000 L CNN
F 1 "Conn_01x03" H 7180 2901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 7100 2950 50  0001 C CNN
F 3 "~" H 7100 2950 50  0001 C CNN
F 4 "~" H 7100 2950 50  0001 C CNN "Mouser Nr"
	1    7100 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 2850 6650 2850
Wire Wire Line
	6650 2850 6650 3650
Wire Wire Line
	6900 3850 6650 3850
Connection ~ 6650 3850
Wire Wire Line
	6800 3050 6900 3050
Wire Wire Line
	6800 3050 6800 4050
Wire Wire Line
	6800 4050 6900 4050
Wire Wire Line
	6900 2950 6250 2950
Wire Wire Line
	6900 3950 6250 3950
Connection ~ 5500 2950
Wire Wire Line
	5500 4650 5850 4650
Connection ~ 6650 4650
Connection ~ 5500 3950
$Comp
L Device:D_Zener D701
U 1 1 5E5401A7
P 5850 3300
F 0 "D701" V 5804 3380 50  0000 L CNN
F 1 "3.3V" V 5895 3380 50  0000 L CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 5850 3300 50  0001 C CNN
F 3 "~" H 5850 3300 50  0001 C CNN
F 4 "771-PESD3V3Y1BSFYL" V 5850 3300 50  0001 C CNN "Mouser Nr"
	1    5850 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	5850 2950 5850 3150
Connection ~ 5850 2950
Wire Wire Line
	5850 2950 5500 2950
Wire Wire Line
	5850 3450 5850 3650
$Comp
L Device:D_Zener D702
U 1 1 5E5401B2
P 5850 4300
F 0 "D702" V 5804 4380 50  0000 L CNN
F 1 "3.3V" V 5895 4380 50  0000 L CNN
F 2 "Diode_SMD:D_0603_1608Metric" H 5850 4300 50  0001 C CNN
F 3 "~" H 5850 4300 50  0001 C CNN
F 4 "771-PESD3V3Y1BSFYL" V 5850 4300 50  0001 C CNN "Mouser Nr"
	1    5850 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	5850 3950 5850 4150
Connection ~ 5850 3950
Wire Wire Line
	5850 3950 5500 3950
Wire Wire Line
	5850 4450 5850 4650
Wire Wire Line
	6650 3850 6650 4650
Wire Wire Line
	5500 3650 5850 3650
Connection ~ 5850 3650
Connection ~ 5850 4650
$Comp
L Device:C C701
U 1 1 5E5401C1
P 6250 3300
F 0 "C701" H 6365 3346 50  0000 L CNN
F 1 "1u" H 6365 3255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6288 3150 50  0001 C CNN
F 3 "~" H 6250 3300 50  0001 C CNN
F 4 "963-UMF212B7105KGHT" H 6250 3300 50  0001 C CNN "Mouser Nr"
	1    6250 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 2950 6250 3150
Connection ~ 6250 2950
Wire Wire Line
	6250 2950 5850 2950
Wire Wire Line
	6250 3450 6250 3650
Wire Wire Line
	6250 3650 5850 3650
$Comp
L Device:C C702
U 1 1 5E5401CC
P 6250 4300
F 0 "C702" H 6365 4346 50  0000 L CNN
F 1 "1u" H 6365 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6288 4150 50  0001 C CNN
F 3 "~" H 6250 4300 50  0001 C CNN
F 4 "963-UMF212B7105KGHT" H 6250 4300 50  0001 C CNN "Mouser Nr"
	1    6250 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 3950 6250 4150
Connection ~ 6250 3950
Wire Wire Line
	6250 3950 5850 3950
Wire Wire Line
	6250 4450 6250 4650
Wire Wire Line
	6250 4650 5850 4650
Wire Wire Line
	4350 2950 5500 2950
Wire Wire Line
	4350 3950 5500 3950
Text HLabel 4350 2950 0    50   Output ~ 0
POTI1
Text HLabel 4350 3950 0    50   Output ~ 0
POTI2
Wire Wire Line
	6650 4650 6650 4850
Connection ~ 6250 4650
Wire Wire Line
	6250 4650 6650 4650
Wire Wire Line
	6250 3650 6650 3650
Connection ~ 6250 3650
Connection ~ 6650 3650
Wire Wire Line
	6650 3650 6650 3850
$Comp
L Device:R R701
U 1 1 5F015AE8
P 5500 3300
F 0 "R701" H 5570 3346 50  0000 L CNN
F 1 "47k" H 5570 3255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5430 3300 50  0001 C CNN
F 3 "~" H 5500 3300 50  0001 C CNN
F 4 "667-ERJ-3EKF4702V" H 5500 3300 50  0001 C CNN "Mouser Nr"
	1    5500 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R702
U 1 1 5F015C60
P 5500 4300
F 0 "R702" H 5570 4346 50  0000 L CNN
F 1 "47k" H 5570 4255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5430 4300 50  0001 C CNN
F 3 "~" H 5500 4300 50  0001 C CNN
F 4 "667-ERJ-3EKF4702V" H 5500 4300 50  0001 C CNN "Mouser Nr"
	1    5500 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 3450 5500 3650
Wire Wire Line
	5500 2950 5500 3150
Wire Wire Line
	5500 3950 5500 4150
Wire Wire Line
	5500 4450 5500 4650
$Comp
L power:GND #PWR0118
U 1 1 5F05C57B
P 6650 4850
F 0 "#PWR0118" H 6650 4600 50  0001 C CNN
F 1 "GND" H 6655 4677 50  0000 C CNN
F 2 "" H 6650 4850 50  0001 C CNN
F 3 "" H 6650 4850 50  0001 C CNN
	1    6650 4850
	1    0    0    -1  
$EndComp
Connection ~ 6800 3050
Wire Wire Line
	6800 2500 6800 3050
$Comp
L power:+3V3 #PWR0119
U 1 1 5F10EA61
P 6800 2500
F 0 "#PWR0119" H 6800 2350 50  0001 C CNN
F 1 "+3V3" H 6815 2673 50  0000 C CNN
F 2 "" H 6800 2500 50  0001 C CNN
F 3 "" H 6800 2500 50  0001 C CNN
	1    6800 2500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
