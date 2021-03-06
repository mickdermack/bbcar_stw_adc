EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 8
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
L power:+3V3 #PWR0129
U 1 1 5F014058
P 3450 2400
F 0 "#PWR0129" H 3450 2250 50  0001 C CNN
F 1 "+3V3" H 3465 2573 50  0000 C CNN
F 2 "" H 3450 2400 50  0001 C CNN
F 3 "" H 3450 2400 50  0001 C CNN
	1    3450 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2400 3450 2450
$Comp
L power:+3V3 #PWR0130
U 1 1 5F014DF2
P 3000 1250
F 0 "#PWR0130" H 3000 1100 50  0001 C CNN
F 1 "+3V3" H 3015 1423 50  0000 C CNN
F 2 "" H 3000 1250 50  0001 C CNN
F 3 "" H 3000 1250 50  0001 C CNN
	1    3000 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1250 3000 1300
Wire Wire Line
	3000 1300 3400 1300
$Comp
L Device:C C401
U 1 1 5F015504
P 3000 1500
F 0 "C401" H 3115 1546 50  0000 L CNN
F 1 "0.1u" H 3115 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3038 1350 50  0001 C CNN
F 3 "~" H 3000 1500 50  0001 C CNN
F 4 "963-HMK107B7104KAHT" H 3000 1500 50  0001 C CNN "Mouser Nr"
	1    3000 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C402
U 1 1 5F015A38
P 3400 1500
F 0 "C402" H 3515 1546 50  0000 L CNN
F 1 "10u" H 3515 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3438 1350 50  0001 C CNN
F 3 "~" H 3400 1500 50  0001 C CNN
F 4 "963-LMK107BBJ106MAHT" H 3400 1500 50  0001 C CNN "Mouser Nr"
	1    3400 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1300 3400 1350
Wire Wire Line
	3000 1300 3000 1350
Connection ~ 3000 1300
Wire Wire Line
	3000 1650 3000 1700
Wire Wire Line
	3400 1650 3400 1700
$Comp
L power:GND #PWR0131
U 1 1 5F0160BA
P 3000 1750
F 0 "#PWR0131" H 3000 1500 50  0001 C CNN
F 1 "GND" H 3005 1577 50  0000 C CNN
F 2 "" H 3000 1750 50  0001 C CNN
F 3 "" H 3000 1750 50  0001 C CNN
	1    3000 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1700 3400 1700
Wire Wire Line
	3000 1700 3000 1750
Connection ~ 3000 1700
$Comp
L power:GND #PWR0132
U 1 1 5F016B86
P 3450 5750
F 0 "#PWR0132" H 3450 5500 50  0001 C CNN
F 1 "GND" H 3455 5577 50  0000 C CNN
F 2 "" H 3450 5750 50  0001 C CNN
F 3 "" H 3450 5750 50  0001 C CNN
	1    3450 5750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R401
U 1 1 5F01727E
P 2700 2700
F 0 "R401" H 2770 2746 50  0000 L CNN
F 1 "10k" H 2770 2655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2630 2700 50  0001 C CNN
F 3 "~" H 2700 2700 50  0001 C CNN
F 4 "667-ERJ-3GEYJ103V" H 2700 2700 50  0001 C CNN "Mouser Nr"
	1    2700 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 2950 2700 2850
Wire Wire Line
	2700 2550 2700 2450
Wire Wire Line
	2700 2450 3450 2450
Connection ~ 3450 2450
Text Label 2650 3150 2    50   ~ 0
IO36
Text Label 2650 3250 2    50   ~ 0
IO39
Text Label 8400 4050 2    50   ~ 0
IO36
Text Label 8400 4150 2    50   ~ 0
IO39
Wire Wire Line
	8400 4050 8450 4050
Wire Wire Line
	8450 4150 8400 4150
Wire Wire Line
	7400 5150 7400 4250
Wire Wire Line
	7400 4250 8450 4250
Wire Wire Line
	7500 5250 7500 4350
Wire Wire Line
	7500 4350 8450 4350
Wire Wire Line
	7600 4950 7600 4450
Wire Wire Line
	7600 4450 8450 4450
Wire Wire Line
	7700 5050 7700 4550
Wire Wire Line
	7700 4550 8450 4550
Text Notes 8050 2600 0    50   ~ 0
Right side breakout header
Text HLabel 4200 2600 1    50   BiDi ~ 0
IO23
Text HLabel 4300 2600 1    50   BiDi ~ 0
IO22
Text HLabel 4400 2600 1    50   BiDi ~ 0
IO21
Text HLabel 4500 2600 1    50   BiDi ~ 0
IO19
Text HLabel 4600 2600 1    50   BiDi ~ 0
IO18
Text HLabel 4700 2600 1    50   BiDi ~ 0
IO17
Wire Wire Line
	4200 2600 4200 4550
Wire Wire Line
	4300 2600 4300 4450
Wire Wire Line
	4400 2600 4400 4350
Wire Wire Line
	4500 2600 4500 4250
Wire Wire Line
	4600 2600 4600 4150
Wire Wire Line
	4700 2600 4700 4050
Text HLabel 4800 2600 1    50   BiDi ~ 0
IO16
Text HLabel 4900 2600 1    50   BiDi ~ 0
IO4
Wire Wire Line
	4800 2600 4800 3950
Wire Wire Line
	4900 2600 4900 3350
Text Notes 4150 2300 0    50   ~ 0
To display
Wire Wire Line
	4050 5050 7700 5050
Wire Wire Line
	4050 4950 7600 4950
Wire Wire Line
	4050 5250 7500 5250
Wire Wire Line
	4050 5150 7400 5150
Wire Wire Line
	2850 3250 2650 3250
Wire Wire Line
	2650 3150 2850 3150
Wire Wire Line
	4050 4550 4200 4550
Wire Wire Line
	4050 4450 4300 4450
Wire Wire Line
	4050 4350 4400 4350
Wire Wire Line
	4050 4250 4500 4250
Wire Wire Line
	4050 4150 4600 4150
Wire Wire Line
	2850 2950 2700 2950
Wire Wire Line
	4050 4050 4700 4050
Wire Wire Line
	4050 3950 4800 3950
Wire Wire Line
	4050 3350 4900 3350
NoConn ~ 2850 4250
NoConn ~ 2850 4150
NoConn ~ 2850 4550
Wire Wire Line
	3450 2450 3450 2750
NoConn ~ 2850 4650
NoConn ~ 2850 4450
NoConn ~ 2850 4350
Wire Wire Line
	4050 4850 8450 4850
Wire Wire Line
	4050 4750 8450 4750
Wire Wire Line
	4050 4650 8450 4650
Wire Wire Line
	3450 5550 3450 5750
Wire Wire Line
	8100 3850 8100 5050
Wire Wire Line
	7900 3550 7900 4950
$Comp
L RF_Module:ESP32-WROOM-32 U401
U 1 1 5F012DEB
P 3450 4150
F 0 "U401" H 2950 2750 50  0000 C CNN
F 1 "ESP32-WROOM-32" H 2950 2650 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 3450 2650 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf" H 3150 4200 50  0001 C CNN
F 4 "~" H 3450 4150 50  0001 C CNN "Mouser Nr"
	1    3450 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3850 8100 3850
Wire Wire Line
	4050 3750 5200 3750
Wire Wire Line
	4050 3650 5100 3650
Wire Wire Line
	4050 3550 7900 3550
Wire Wire Line
	6600 2700 7750 2700
Wire Wire Line
	4050 3450 6600 3450
Wire Wire Line
	6500 2600 7750 2600
Wire Wire Line
	6400 2500 7750 2500
$Comp
L Connector_Generic:Conn_01x05 J401
U 1 1 5F0DF1E7
P 5750 5850
F 0 "J401" V 5950 6100 50  0000 R CNN
F 1 "Conn_01x05" V 5850 6100 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 5750 5850 50  0001 C CNN
F 3 "~" H 5750 5850 50  0001 C CNN
F 4 "~" H 5750 5850 50  0001 C CNN "Mouser Nr"
	1    5750 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	5950 5650 5950 2950
Text Label 2450 2950 2    50   ~ 0
EN
Wire Wire Line
	2700 2950 2450 2950
Connection ~ 2700 2950
Text Label 6450 5550 0    50   ~ 0
EN
Wire Wire Line
	5850 5550 5850 5650
Wire Wire Line
	5750 5650 5750 3050
Wire Wire Line
	5650 5650 5650 3250
Wire Wire Line
	5550 5650 5550 5600
Wire Wire Line
	5550 5600 5300 5600
Wire Wire Line
	5300 5600 5300 6050
$Comp
L power:GND #PWR0133
U 1 1 5F0FF41B
P 5300 6050
F 0 "#PWR0133" H 5300 5800 50  0001 C CNN
F 1 "GND" H 5305 5877 50  0000 C CNN
F 2 "" H 5300 6050 50  0001 C CNN
F 3 "" H 5300 6050 50  0001 C CNN
	1    5300 6050
	1    0    0    -1  
$EndComp
Text Notes 5500 6200 0    50   ~ 0
Programming header
Text HLabel 5100 2600 1    50   BiDi ~ 0
IO13
Text Notes 5050 2300 0    50   ~ 0
To STM32 UART
Text Notes 8300 3450 0    50   ~ 0
The pins of the breakout headers are\nin physical order (top to bottom)
Text HLabel 5200 2600 1    50   BiDi ~ 0
IO14
Wire Wire Line
	4050 3150 6800 3150
Wire Wire Line
	5100 2600 5100 3650
Wire Wire Line
	5200 2600 5200 3750
$Comp
L Device:R R402
U 1 1 5F082CB8
P 6200 5550
F 0 "R402" V 5993 5550 50  0000 C CNN
F 1 "1k" V 6084 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6130 5550 50  0001 C CNN
F 3 "~" H 6200 5550 50  0001 C CNN
F 4 "667-ERJ-3GEYJ102V" H 6200 5550 50  0001 C CNN "Mouser Nr"
	1    6200 5550
	0    1    1    0   
$EndComp
Wire Wire Line
	5850 5550 6050 5550
Wire Wire Line
	6350 5550 6450 5550
Connection ~ 5650 3250
Wire Wire Line
	5650 3250 6500 3250
Connection ~ 5750 3050
Wire Wire Line
	5750 3050 6400 3050
Wire Wire Line
	4050 2950 5950 2950
Wire Wire Line
	4050 3250 5650 3250
Wire Wire Line
	4050 3050 5750 3050
Wire Wire Line
	8450 4950 7900 4950
Wire Wire Line
	8450 5050 8100 5050
Text Notes 8750 4450 0    50   ~ 0
Left side breakout header
$Comp
L Connector_Generic:Conn_01x11 J403
U 1 1 5F24B832
P 8650 4550
F 0 "J403" H 8730 4592 50  0000 L CNN
F 1 "Conn_01x11" H 8730 4501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x11_P2.54mm_Vertical" H 8650 4550 50  0001 C CNN
F 3 "~" H 8650 4550 50  0001 C CNN
F 4 "~" H 8650 4550 50  0001 C CNN "Mouser Nr"
	1    8650 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 2500 6400 3050
Wire Wire Line
	6500 2600 6500 3250
Wire Wire Line
	6600 2700 6600 3450
Wire Wire Line
	5950 2950 6700 2950
Wire Wire Line
	6700 2950 6700 2800
Wire Wire Line
	6700 2800 7750 2800
Connection ~ 5950 2950
Wire Wire Line
	6800 2900 7750 2900
Wire Wire Line
	6800 2900 6800 3150
$Comp
L Connector_Generic:Conn_01x05 J402
U 1 1 5F26BFBB
P 7950 2700
F 0 "J402" H 8030 2742 50  0000 L CNN
F 1 "Conn_01x05" H 8030 2651 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 7950 2700 50  0001 C CNN
F 3 "~" H 7950 2700 50  0001 C CNN
F 4 "~" H 7950 2700 50  0001 C CNN "Mouser Nr"
	1    7950 2700
	1    0    0    -1  
$EndComp
$EndSCHEMATC
