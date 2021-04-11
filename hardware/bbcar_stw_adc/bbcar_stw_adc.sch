EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 8
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 1650 1350 800  450 
U 5EFD80AF
F0 "Power3V3" 50
F1 "Power3V3.sch" 50
F2 "VIN" I L 1650 1450 50 
F3 "GND" I L 1650 1700 50 
F4 "VOUT" O R 2450 1450 50 
$EndSheet
$Comp
L power:+3V3 #PWR0110
U 1 1 5EFE20C5
P 2700 1250
F 0 "#PWR0110" H 2700 1100 50  0001 C CNN
F 1 "+3V3" H 2715 1423 50  0000 C CNN
F 2 "" H 2700 1250 50  0001 C CNN
F 3 "" H 2700 1250 50  0001 C CNN
	1    2700 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1250 2700 1450
Wire Wire Line
	2700 1450 2450 1450
$Comp
L power:GND #PWR0111
U 1 1 5EFE3FC3
P 1400 1850
F 0 "#PWR0111" H 1400 1600 50  0001 C CNN
F 1 "GND" H 1405 1677 50  0000 C CNN
F 2 "" H 1400 1850 50  0001 C CNN
F 3 "" H 1400 1850 50  0001 C CNN
	1    1400 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1700 1400 1700
Wire Wire Line
	1400 1700 1400 1850
Wire Wire Line
	1400 1450 1650 1450
$Comp
L power:+5V #PWR0112
U 1 1 5EFEA746
P 1400 1000
F 0 "#PWR0112" H 1400 850 50  0001 C CNN
F 1 "+5V" H 1415 1173 50  0000 C CNN
F 2 "" H 1400 1000 50  0001 C CNN
F 3 "" H 1400 1000 50  0001 C CNN
	1    1400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 1000 1400 1450
$Sheet
S 2550 3600 550  900 
U 5F050C1F
F0 "Display_Conn" 50
F1 "Display_Conn.sch" 50
F2 "TP_IRQ" O R 3100 3800 50 
F3 "LCD_RS" I R 3100 3900 50 
F4 "LCD_RST" I R 3100 3700 50 
F5 "SCLK" I R 3100 4000 50 
F6 "LCD_CS" I R 3100 4300 50 
F7 "TP_CS" I R 3100 4400 50 
F8 "COPI" I R 3100 4200 50 
F9 "TP_CIPO" O R 3100 4100 50 
$EndSheet
$Sheet
S 3550 3600 600  900 
U 5F0604FE
F0 "ESP32" 50
F1 "ESP32.sch" 50
F2 "IO23" B L 3550 3800 50 
F3 "IO22" B L 3550 3900 50 
F4 "IO21" B L 3550 4200 50 
F5 "IO19" B L 3550 4100 50 
F6 "IO18" B L 3550 3700 50 
F7 "IO17" B L 3550 4000 50 
F8 "IO16" B L 3550 4300 50 
F9 "IO4" B L 3550 4400 50 
F10 "IO13" B R 4150 3700 50 
F11 "IO14" B R 4150 3800 50 
$EndSheet
$Comp
L Connector_Generic:Conn_01x03 J103
U 1 1 5F132F42
P 4850 1350
F 0 "J103" H 4930 1392 50  0000 L CNN
F 1 "Conn_01x03" H 4930 1301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4850 1350 50  0001 C CNN
F 3 "~" H 4850 1350 50  0001 C CNN
F 4 "~" H 4850 1350 50  0001 C CNN "Mouser Nr"
	1    4850 1350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J101
U 1 1 5F13319A
P 3950 1350
F 0 "J101" H 4030 1392 50  0000 L CNN
F 1 "Conn_01x03" H 4030 1301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3950 1350 50  0001 C CNN
F 3 "~" H 3950 1350 50  0001 C CNN
F 4 "~" H 3950 1350 50  0001 C CNN "Mouser Nr"
	1    3950 1350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J102
U 1 1 5F133CFF
P 3950 1750
F 0 "J102" H 4030 1792 50  0000 L CNN
F 1 "Conn_01x03" H 4030 1701 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3950 1750 50  0001 C CNN
F 3 "~" H 3950 1750 50  0001 C CNN
F 4 "~" H 3950 1750 50  0001 C CNN "Mouser Nr"
	1    3950 1750
	1    0    0    -1  
$EndComp
Text Notes 6350 2250 2    50   ~ 0
Power headers
$Comp
L power:+3V3 #PWR0101
U 1 1 5F138FEE
P 3700 1150
F 0 "#PWR0101" H 3700 1000 50  0001 C CNN
F 1 "+3V3" H 3715 1323 50  0000 C CNN
F 2 "" H 3700 1150 50  0001 C CNN
F 3 "" H 3700 1150 50  0001 C CNN
	1    3700 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1150 3700 1250
Wire Wire Line
	3700 1250 3750 1250
Wire Wire Line
	3700 1250 3700 1350
Wire Wire Line
	3700 1350 3750 1350
Connection ~ 3700 1250
Wire Wire Line
	3700 1350 3700 1450
Wire Wire Line
	3700 1450 3750 1450
Connection ~ 3700 1350
Wire Wire Line
	4650 1450 4600 1450
Wire Wire Line
	4600 1450 4600 1350
Wire Wire Line
	4650 1250 4600 1250
Connection ~ 4600 1250
Wire Wire Line
	4600 1250 4600 1100
Wire Wire Line
	4650 1350 4600 1350
Connection ~ 4600 1350
Wire Wire Line
	4600 1350 4600 1250
$Comp
L power:+5V #PWR0102
U 1 1 5F14AA38
P 4600 1050
F 0 "#PWR0102" H 4600 900 50  0001 C CNN
F 1 "+5V" H 4615 1223 50  0000 C CNN
F 2 "" H 4600 1050 50  0001 C CNN
F 3 "" H 4600 1050 50  0001 C CNN
	1    4600 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5F14AD90
P 3700 2000
F 0 "#PWR0103" H 3700 1750 50  0001 C CNN
F 1 "GND" H 3705 1827 50  0000 C CNN
F 2 "" H 3700 2000 50  0001 C CNN
F 3 "" H 3700 2000 50  0001 C CNN
	1    3700 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2000 3700 1850
Wire Wire Line
	3700 1650 3750 1650
Wire Wire Line
	3750 1750 3700 1750
Connection ~ 3700 1750
Wire Wire Line
	3700 1750 3700 1650
Wire Wire Line
	3750 1850 3700 1850
Connection ~ 3700 1850
Wire Wire Line
	3700 1850 3700 1750
$Comp
L Connector_Generic:Conn_01x03 J104
U 1 1 5F15B604
P 5750 1350
F 0 "J104" H 5830 1392 50  0000 L CNN
F 1 "Conn_01x03" H 5830 1301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5750 1350 50  0001 C CNN
F 3 "~" H 5750 1350 50  0001 C CNN
F 4 "~" H 5750 1350 50  0001 C CNN "Mouser Nr"
	1    5750 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1450 5500 1450
Wire Wire Line
	5500 1450 5500 1350
Wire Wire Line
	5550 1250 5500 1250
Connection ~ 5500 1250
Wire Wire Line
	5550 1350 5500 1350
Connection ~ 5500 1350
Wire Wire Line
	5500 1350 5500 1250
$Comp
L power:VBUS #PWR0104
U 1 1 5F16DB0F
P 5500 850
F 0 "#PWR0104" H 5500 700 50  0001 C CNN
F 1 "VBUS" H 5515 1023 50  0000 C CNN
F 2 "" H 5500 850 50  0001 C CNN
F 3 "" H 5500 850 50  0001 C CNN
	1    5500 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 850  5500 1250
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5F17C96D
P 4950 1050
F 0 "#FLG0102" H 4950 1125 50  0001 C CNN
F 1 "PWR_FLAG" H 4950 1223 50  0000 C CNN
F 2 "" H 4950 1050 50  0001 C CNN
F 3 "~" H 4950 1050 50  0001 C CNN
	1    4950 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 1050 4950 1100
Wire Wire Line
	4950 1100 4600 1100
Connection ~ 4600 1100
Wire Wire Line
	4600 1100 4600 1050
Text Notes 4100 800  0    50   ~ 0
+5V will be powered from VBUS\nvia an external regulator board
Wire Wire Line
	3100 3700 3550 3700
Wire Wire Line
	3550 3800 3100 3800
Wire Wire Line
	3100 3900 3550 3900
Wire Wire Line
	3550 4000 3100 4000
Wire Wire Line
	3100 4100 3550 4100
Wire Wire Line
	3550 4200 3100 4200
Wire Wire Line
	3100 4300 3550 4300
Wire Wire Line
	3550 4400 3100 4400
$Comp
L Device:D_Zener D?
U 1 1 5F06C8DB
P 4600 1800
AR Path="/5EFD80AF/5F06C8DB" Ref="D?"  Part="1" 
AR Path="/5F06C8DB" Ref="D101"  Part="1" 
F 0 "D101" V 4554 1880 50  0000 L CNN
F 1 "5V" V 4645 1880 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-128" H 4600 1800 50  0001 C CNN
F 3 "~" H 4600 1800 50  0001 C CNN
F 4 "771-PTVS5V0P1UP115" V 4600 1800 50  0001 C CNN "Mouser Nr"
	1    4600 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	4600 1650 4600 1450
Wire Wire Line
	4600 1950 4600 2000
$Comp
L power:GND #PWR0105
U 1 1 5F071B56
P 4600 2000
F 0 "#PWR0105" H 4600 1750 50  0001 C CNN
F 1 "GND" H 4605 1827 50  0000 C CNN
F 2 "" H 4600 2000 50  0001 C CNN
F 3 "" H 4600 2000 50  0001 C CNN
	1    4600 2000
	1    0    0    -1  
$EndComp
Connection ~ 4600 1450
Wire Notes Line
	3450 600  6400 600 
Wire Notes Line
	6400 600  6400 2300
Wire Notes Line
	6400 2300 3450 2300
Wire Notes Line
	3450 2300 3450 600 
Text Notes 4700 1700 0    50   ~ 0
TVS
$Sheet
S 4800 3600 500  900 
U 5F05568E
F0 "STM32" 50
F1 "STM32.sch" 50
F2 "PA0" B R 5300 3700 50 
F3 "PA1" B R 5300 3800 50 
F4 "PA11" B R 5300 4000 50 
F5 "PA12" B R 5300 4100 50 
F6 "PA2" B L 4800 3700 50 
F7 "PA3" B L 4800 3800 50 
$EndSheet
$Sheet
S 5650 3600 550  300 
U 5F05CFA3
F0 "Potis" 50
F1 "Potis.sch" 50
F2 "POTI2" O L 5650 3800 50 
F3 "POTI1" O L 5650 3700 50 
$EndSheet
$Sheet
S 6350 3900 550  300 
U 5F076809
F0 "CAN" 50
F1 "CAN.sch" 50
F2 "CAN_TX" I L 6350 4100 50 
F3 "CAN_RX" O L 6350 4000 50 
$EndSheet
Wire Wire Line
	5300 3700 5650 3700
Wire Wire Line
	5650 3800 5300 3800
Wire Wire Line
	5300 4000 6350 4000
Wire Wire Line
	6350 4100 5300 4100
Wire Wire Line
	4800 3700 4150 3700
Wire Wire Line
	4800 3800 4150 3800
$EndSCHEMATC