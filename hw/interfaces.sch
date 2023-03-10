EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 5
Title ""
Date ""
Rev ""
Comp "JECC"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CAN-transceiver:SN65HVD232 U12
U 1 1 607D7E47
P 8200 3900
F 0 "U12" H 8725 4787 60  0000 C CNN
F 1 "SN65HVD232" H 8725 4681 60  0000 C CNN
F 2 "SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 9050 3850 60  0001 C CNN
F 3 "" H 8200 4050 60  0000 C CNN
	1    8200 3900
	1    0    0    -1  
$EndComp
$Comp
L MODULE_compute:QWIIC CN1
U 1 1 6065FBFD
P 9650 2000
F 0 "CN1" H 9742 1613 60  0000 C CNN
F 1 "QWIIC" H 9742 1719 60  0000 C CNN
F 2 "connectors_JST_SH:Connectors_JST_SM04B-SRSS-TB" H 9650 2150 60  0001 C CNN
F 3 "" H 9650 2000 60  0001 C CNN
	1    9650 2000
	1    0    0    1   
$EndComp
$Comp
L mechanical-connectors:USB_B P4
U 1 1 606CB3F3
P 9800 5300
F 0 "P4" V 9782 5488 50  0000 L CNN
F 1 "USB_B" V 9873 5488 50  0000 L CNN
F 2 "mechanical-connectors:USB_B" H 9828 5533 50  0001 C CNN
F 3 "" V 9750 5200 50  0000 C CNN
	1    9800 5300
	0    1    1    0   
$EndComp
$Comp
L mechanical-connectors:CONN_01X03 P2
U 1 1 606C45C2
P 9850 3600
F 0 "P2" H 9928 3641 50  0000 L CNN
F 1 "CONN_01X03" H 9928 3550 50  0000 L CNN
F 2 "CON_wuerth:WR-TBL_691322310003" H 9928 3504 50  0001 L CNN
F 3 "" H 9850 3600 50  0000 C CNN
	1    9850 3600
	1    0    0    1   
$EndComp
$Comp
L mechanical-connectors:CONN_02X03 P10
U 1 1 606D2AEA
P 5650 6350
F 0 "P10" H 5650 6665 50  0000 C CNN
F 1 "CONN_02X03" H 5650 6574 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Horizontal" H 5650 6573 50  0001 C CNN
F 3 "" H 5650 5150 50  0000 C CNN
	1    5650 6350
	1    0    0    -1  
$EndComp
$Comp
L mechanical-connectors:CONN_01X03 P1
U 1 1 606DB288
P 4450 1050
F 0 "P1" H 4528 1091 50  0000 L CNN
F 1 "CONN_01X03" H 4528 1000 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 4528 954 50  0001 L CNN
F 3 "" H 4450 1050 50  0000 C CNN
	1    4450 1050
	1    0    0    -1  
$EndComp
$Comp
L mechanical-connectors:CONN_01X04 P3
U 1 1 606DD690
P 9700 1200
F 0 "P3" H 9617 825 50  0000 C CNN
F 1 "CONN_01X04" H 9617 916 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 9778 1104 50  0001 L CNN
F 3 "" H 9700 1200 50  0000 C CNN
	1    9700 1200
	1    0    0    1   
$EndComp
$Comp
L mechanical-connectors:CONN_01X06 P9
U 1 1 606DF1DA
P 4500 2200
F 0 "P9" H 4578 2241 50  0000 L CNN
F 1 "CONN_01X06" H 4578 2150 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 4578 2104 50  0001 L CNN
F 3 "" H 4500 2200 50  0000 C CNN
	1    4500 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 3500 9650 3500
Wire Wire Line
	9250 3600 9650 3600
Text Label 9250 3500 0    50   ~ 0
nCANH
Text Label 9250 3600 0    50   ~ 0
nCANL
$Comp
L power-supply:+3V3 #PWR0112
U 1 1 60705341
P 7750 2850
F 0 "#PWR0112" H 7750 2700 50  0001 C CNN
F 1 "+3V3" H 7765 3023 50  0000 C CNN
F 2 "" H 7750 2850 50  0000 C CNN
F 3 "" H 7750 2850 50  0000 C CNN
	1    7750 2850
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0113
U 1 1 607068BB
P 9550 3900
F 0 "#PWR0113" H 9550 3650 50  0001 C CNN
F 1 "GND" H 9555 3727 50  0000 C CNN
F 2 "" H 9550 3900 50  0000 C CNN
F 3 "" H 9550 3900 50  0000 C CNN
	1    9550 3900
	1    0    0    -1  
$EndComp
NoConn ~ 9250 3350
NoConn ~ 9250 3750
Wire Wire Line
	9550 3900 9550 3700
Wire Wire Line
	9550 3700 9650 3700
$Comp
L power-supply:GND #PWR0115
U 1 1 607075C6
P 8000 3850
F 0 "#PWR0115" H 8000 3600 50  0001 C CNN
F 1 "GND" H 8005 3677 50  0000 C CNN
F 2 "" H 8000 3850 50  0000 C CNN
F 3 "" H 8000 3850 50  0000 C CNN
	1    8000 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3350 8000 3350
Wire Wire Line
	8200 3750 8000 3750
Wire Wire Line
	8000 3750 8000 3850
$Comp
L devices:C_0603 C60
U 1 1 60708223
P 7750 3050
F 0 "C60" H 7842 3096 50  0000 L CNN
F 1 "100nF" H 7842 3005 50  0000 L CNN
F 2 "capacitors:C_0603" H 7750 2900 50  0001 C CNN
F 3 "" H 7750 3050 50  0000 C CNN
	1    7750 3050
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0116
U 1 1 6070A2B2
P 7750 3200
F 0 "#PWR0116" H 7750 2950 50  0001 C CNN
F 1 "GND" H 7755 3027 50  0000 C CNN
F 2 "" H 7750 3200 50  0000 C CNN
F 3 "" H 7750 3200 50  0000 C CNN
	1    7750 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3150 7750 3200
Wire Wire Line
	7750 2850 7750 2900
Wire Wire Line
	8000 2900 7750 2900
Wire Wire Line
	8000 2900 8000 3350
Connection ~ 7750 2900
Wire Wire Line
	7750 2900 7750 2950
$Comp
L interface:MAX3232 U13
U 1 1 6063577D
P 4100 5950
F 0 "U13" H 3600 7000 50  0000 C CNN
F 1 "MAX3232" H 4550 7000 50  0000 C CNN
F 2 "SOIC:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 4150 4900 50  0001 L CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX3222-MAX3241.pdf" H 4100 6050 50  0001 C CNN
	1    4100 5950
	1    0    0    -1  
$EndComp
Text GLabel 8000 3500 0    50   Input ~ 0
CAN_TX
Text GLabel 8000 3600 0    50   Input ~ 0
CAN_RX
Wire Wire Line
	8200 3500 8000 3500
Wire Wire Line
	8000 3600 8200 3600
Wire Wire Line
	9550 2050 9300 2050
Wire Wire Line
	9300 2050 9300 1350
Wire Wire Line
	9300 1350 9500 1350
Wire Wire Line
	9200 2150 9200 1250
Wire Wire Line
	9200 1250 9500 1250
Wire Wire Line
	9550 2150 9200 2150
Wire Wire Line
	9550 1950 9400 1950
Wire Wire Line
	9400 1950 9400 1150
Wire Wire Line
	9400 1150 9500 1150
Wire Wire Line
	9550 1850 9450 1850
Wire Wire Line
	9450 1850 9450 1050
Wire Wire Line
	9450 1050 9500 1050
$Comp
L power-supply:GND #PWR0117
U 1 1 60721809
P 9200 2250
F 0 "#PWR0117" H 9200 2000 50  0001 C CNN
F 1 "GND" H 9205 2077 50  0000 C CNN
F 2 "" H 9200 2250 50  0000 C CNN
F 3 "" H 9200 2250 50  0000 C CNN
	1    9200 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 2250 9200 2150
Connection ~ 9200 2150
$Comp
L power-supply:+3V3 #PWR0118
U 1 1 6072449A
P 9300 1200
F 0 "#PWR0118" H 9300 1050 50  0001 C CNN
F 1 "+3V3" H 9315 1373 50  0000 C CNN
F 2 "" H 9300 1200 50  0000 C CNN
F 3 "" H 9300 1200 50  0000 C CNN
	1    9300 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 1200 9300 1350
Connection ~ 9300 1350
Text GLabel 9050 1950 0    50   BiDi ~ 0
SDA1
Text GLabel 9050 1850 0    50   Input ~ 0
SCL1
Wire Wire Line
	9050 1850 9450 1850
Connection ~ 9450 1850
Wire Wire Line
	9050 1950 9400 1950
Connection ~ 9400 1950
$Comp
L devices:R_0603 R8
U 1 1 60726680
P 9200 5300
F 0 "R8" V 9250 5150 50  0000 C CNN
F 1 "20" V 9250 5450 50  0000 C CNN
F 2 "resistors:R_0603" H 9200 5150 50  0001 C CNN
F 3 "" H 9200 5300 50  0000 C CNN
	1    9200 5300
	0    1    1    0   
$EndComp
$Comp
L devices:R_0603 R7
U 1 1 60727AD5
P 9200 5200
F 0 "R7" V 9250 5050 50  0000 C CNN
F 1 "20" V 9250 5350 50  0000 C CNN
F 2 "resistors:R_0603" H 9200 5050 50  0001 C CNN
F 3 "" H 9200 5200 50  0000 C CNN
	1    9200 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	9500 5300 9300 5300
Wire Wire Line
	9500 5200 9300 5200
Wire Wire Line
	9500 5400 9450 5400
Wire Wire Line
	9450 5700 9900 5700
Wire Wire Line
	9900 5700 9900 5600
Wire Wire Line
	9450 5400 9450 5700
$Comp
L power-supply:GND #PWR0119
U 1 1 607299B0
P 9900 5800
F 0 "#PWR0119" H 9900 5550 50  0001 C CNN
F 1 "GND" H 9905 5627 50  0000 C CNN
F 2 "" H 9900 5800 50  0000 C CNN
F 3 "" H 9900 5800 50  0000 C CNN
	1    9900 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 5800 9900 5700
Connection ~ 9900 5700
$Comp
L power-supply:+5V #PWR0120
U 1 1 6072B149
P 8900 4800
F 0 "#PWR0120" H 8900 4650 50  0001 C CNN
F 1 "+5V" H 8915 4973 50  0000 C CNN
F 2 "" H 8900 4800 50  0000 C CNN
F 3 "" H 8900 4800 50  0000 C CNN
	1    8900 4800
	1    0    0    -1  
$EndComp
$Comp
L devices:R_0603 R6
U 1 1 6072C838
P 8900 5050
F 0 "R6" H 8959 5096 50  0000 L CNN
F 1 "4k7" H 8959 5005 50  0000 L CNN
F 2 "resistors:R_0603" H 8900 4900 50  0001 C CNN
F 3 "" H 8900 5050 50  0000 C CNN
	1    8900 5050
	1    0    0    -1  
$EndComp
Text GLabel 8800 5300 0    50   BiDi ~ 0
USB+
Text GLabel 8800 5200 0    50   BiDi ~ 0
USB-
Wire Wire Line
	9100 5200 8800 5200
Wire Wire Line
	8800 5300 8900 5300
Wire Wire Line
	9500 5100 9150 5100
Wire Wire Line
	9150 5100 9150 4850
Wire Wire Line
	9150 4850 8900 4850
Wire Wire Line
	8900 4850 8900 4950
Wire Wire Line
	8900 4800 8900 4850
Connection ~ 8900 4850
Wire Wire Line
	8900 5150 8900 5300
Connection ~ 8900 5300
Wire Wire Line
	8900 5300 9100 5300
$Comp
L power-supply:+5V #PWR0121
U 1 1 60739D7E
P 3500 1000
F 0 "#PWR0121" H 3500 850 50  0001 C CNN
F 1 "+5V" H 3515 1173 50  0000 C CNN
F 2 "" H 3500 1000 50  0000 C CNN
F 3 "" H 3500 1000 50  0000 C CNN
	1    3500 1000
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0123
U 1 1 6073AF2A
P 3500 1450
F 0 "#PWR0123" H 3500 1200 50  0001 C CNN
F 1 "GND" H 3505 1277 50  0000 C CNN
F 2 "" H 3500 1450 50  0000 C CNN
F 3 "" H 3500 1450 50  0000 C CNN
	1    3500 1450
	1    0    0    -1  
$EndComp
$Comp
L devices:C_1210 C55
U 1 1 6073C1E7
P 3500 1200
F 0 "C55" H 3592 1246 50  0000 L CNN
F 1 "10uF" H 3592 1155 50  0000 L CNN
F 2 "capacitors:C_1210" H 3500 1050 50  0001 C CNN
F 3 "" H 3500 1200 50  0000 C CNN
	1    3500 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 1150 4050 1150
Wire Wire Line
	4050 1150 4050 1350
Wire Wire Line
	4050 1350 3500 1350
Wire Wire Line
	3500 1350 3500 1450
Wire Wire Line
	3500 1300 3500 1350
Connection ~ 3500 1350
Wire Wire Line
	4250 1050 3500 1050
Wire Wire Line
	3500 1050 3500 1000
Wire Wire Line
	3500 1100 3500 1050
Connection ~ 3500 1050
Text GLabel 4100 950  0    50   Input ~ 0
SERVO
Wire Wire Line
	4250 950  4100 950 
$Comp
L devices:Jumper_NC_Small JP1
U 1 1 60746A2C
P 10250 3300
F 0 "JP1" H 10250 3421 50  0000 C CNN
F 1 "Jumper_NC_Small" H 10260 3240 50  0001 C CNN
F 2 "wire_pads:SolderJumper_2mm" H 10250 3421 50  0001 C CNN
F 3 "" H 10250 3300 50  0000 C CNN
	1    10250 3300
	1    0    0    -1  
$EndComp
$Comp
L devices:R_0603 R9
U 1 1 60747BA7
P 9900 3300
F 0 "R9" V 9704 3300 50  0000 C CNN
F 1 "120" V 9795 3300 50  0000 C CNN
F 2 "resistors:R_0603" H 9900 3150 50  0001 C CNN
F 3 "" H 9900 3300 50  0000 C CNN
	1    9900 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	10000 3300 10150 3300
Wire Wire Line
	10350 3300 10700 3300
Wire Wire Line
	9800 3300 9500 3300
Text Label 9500 3300 0    50   ~ 0
nCANH
Text Label 10450 3300 0    50   ~ 0
nCANL
$Comp
L power-supply:+3V3 #PWR0124
U 1 1 6075032A
P 4000 1850
F 0 "#PWR0124" H 4000 1700 50  0001 C CNN
F 1 "+3V3" H 4015 2023 50  0000 C CNN
F 2 "" H 4000 1850 50  0000 C CNN
F 3 "" H 4000 1850 50  0000 C CNN
	1    4000 1850
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0125
U 1 1 6075189C
P 4000 2550
F 0 "#PWR0125" H 4000 2300 50  0001 C CNN
F 1 "GND" H 4005 2377 50  0000 C CNN
F 2 "" H 4000 2550 50  0000 C CNN
F 3 "" H 4000 2550 50  0000 C CNN
	1    4000 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 1950 4000 1950
Wire Wire Line
	4000 1950 4000 1850
Wire Wire Line
	4300 2450 4000 2450
Wire Wire Line
	4000 2450 4000 2550
Wire Wire Line
	4300 2050 4000 2050
Wire Wire Line
	4300 2150 4000 2150
Wire Wire Line
	4300 2250 4000 2250
Wire Wire Line
	4300 2350 4000 2350
Text GLabel 4000 2050 0    50   Output ~ 0
MISO2
Text GLabel 4000 2150 0    50   Input ~ 0
MOSI2
Text GLabel 4000 2250 0    50   Input ~ 0
SCK2
Text GLabel 4000 2350 0    50   Input ~ 0
CS
$Comp
L power-supply:+3V3 #PWR0126
U 1 1 607ED76B
P 3700 4250
F 0 "#PWR0126" H 3700 4100 50  0001 C CNN
F 1 "+3V3" H 3715 4423 50  0000 C CNN
F 2 "" H 3700 4250 50  0000 C CNN
F 3 "" H 3700 4250 50  0000 C CNN
	1    3700 4250
	1    0    0    -1  
$EndComp
$Comp
L devices:C_0603 C56
U 1 1 607ED771
P 3700 4450
F 0 "C56" H 3792 4496 50  0000 L CNN
F 1 "100nF" H 3792 4405 50  0000 L CNN
F 2 "capacitors:C_0603" H 3700 4300 50  0001 C CNN
F 3 "" H 3700 4450 50  0000 C CNN
	1    3700 4450
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0127
U 1 1 607ED777
P 3700 4600
F 0 "#PWR0127" H 3700 4350 50  0001 C CNN
F 1 "GND" H 3705 4427 50  0000 C CNN
F 2 "" H 3700 4600 50  0000 C CNN
F 3 "" H 3700 4600 50  0000 C CNN
	1    3700 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 4550 3700 4600
Wire Wire Line
	3700 4250 3700 4300
Connection ~ 3700 4300
Wire Wire Line
	3700 4300 3700 4350
Wire Wire Line
	4100 4750 4100 4300
Wire Wire Line
	3700 4300 4100 4300
$Comp
L power-supply:GND #PWR0185
U 1 1 607F371A
P 4100 7250
F 0 "#PWR0185" H 4100 7000 50  0001 C CNN
F 1 "GND" H 4105 7077 50  0000 C CNN
F 2 "" H 4100 7250 50  0000 C CNN
F 3 "" H 4100 7250 50  0000 C CNN
	1    4100 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 7250 4100 7150
$Comp
L devices:C_0603 C54
U 1 1 607FB3A1
P 2950 5200
F 0 "C54" H 3042 5246 50  0000 L CNN
F 1 "100nF" H 3042 5155 50  0000 L CNN
F 2 "capacitors:C_0603" H 2950 5050 50  0001 C CNN
F 3 "" H 2950 5200 50  0000 C CNN
	1    2950 5200
	1    0    0    -1  
$EndComp
$Comp
L devices:C_0603 C57
U 1 1 607FD5BE
P 5150 5200
F 0 "C57" H 5242 5246 50  0000 L CNN
F 1 "100nF" H 5242 5155 50  0000 L CNN
F 2 "capacitors:C_0603" H 5150 5050 50  0001 C CNN
F 3 "" H 5150 5200 50  0000 C CNN
	1    5150 5200
	1    0    0    -1  
$EndComp
$Comp
L devices:C_0603 C58
U 1 1 607FF5B4
P 5200 5550
F 0 "C58" V 5150 5450 50  0000 C CNN
F 1 "100nF" V 5150 5700 50  0000 C CNN
F 2 "capacitors:C_0603" H 5200 5400 50  0001 C CNN
F 3 "" H 5200 5550 50  0000 C CNN
	1    5200 5550
	0    1    1    0   
$EndComp
$Comp
L devices:C_0603 C59
U 1 1 6080543C
P 5200 5850
F 0 "C59" V 5250 5900 50  0000 L CNN
F 1 "100nF" V 5250 5600 50  0000 L CNN
F 2 "capacitors:C_0603" H 5200 5700 50  0001 C CNN
F 3 "" H 5200 5850 50  0000 C CNN
	1    5200 5850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4900 5550 5100 5550
Wire Wire Line
	4900 5850 5100 5850
Wire Wire Line
	4900 5350 5150 5350
Wire Wire Line
	5150 5350 5150 5300
Wire Wire Line
	5150 5100 5150 5050
Wire Wire Line
	5150 5050 4900 5050
Wire Wire Line
	3300 5350 2950 5350
Wire Wire Line
	2950 5350 2950 5300
Wire Wire Line
	3300 5050 2950 5050
Wire Wire Line
	2950 5050 2950 5100
Wire Wire Line
	4900 6250 5400 6250
Wire Wire Line
	4900 6650 5100 6650
Wire Wire Line
	5100 6650 5100 6350
Wire Wire Line
	5100 6350 5400 6350
Wire Wire Line
	4900 6450 5050 6450
Wire Wire Line
	5050 6450 5050 6750
Wire Wire Line
	5050 6750 6100 6750
Wire Wire Line
	6100 6750 6100 6350
Wire Wire Line
	6100 6350 5900 6350
Wire Wire Line
	4900 6050 5400 6050
Wire Wire Line
	5400 6050 5400 5950
Wire Wire Line
	5400 5950 6100 5950
Wire Wire Line
	6100 5950 6100 6250
Wire Wire Line
	6100 6250 5900 6250
$Comp
L power-supply:GND #PWR0187
U 1 1 6082323D
P 5950 6900
F 0 "#PWR0187" H 5950 6650 50  0001 C CNN
F 1 "GND" H 5955 6727 50  0000 C CNN
F 2 "" H 5950 6900 50  0000 C CNN
F 3 "" H 5950 6900 50  0000 C CNN
	1    5950 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 6450 5350 6450
Wire Wire Line
	5350 6450 5350 6600
Wire Wire Line
	5350 6600 5950 6600
Wire Wire Line
	5950 6600 5950 6450
Wire Wire Line
	5950 6450 5900 6450
Wire Wire Line
	5950 6900 5950 6600
Connection ~ 5950 6600
Wire Wire Line
	5300 5850 5500 5850
Wire Wire Line
	5500 5850 5500 5550
Wire Wire Line
	5500 5550 5300 5550
$Comp
L power-supply:GND #PWR0197
U 1 1 6083804F
P 5650 5650
F 0 "#PWR0197" H 5650 5400 50  0001 C CNN
F 1 "GND" H 5655 5477 50  0000 C CNN
F 2 "" H 5650 5650 50  0000 C CNN
F 3 "" H 5650 5650 50  0000 C CNN
	1    5650 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 5650 5650 5550
Wire Wire Line
	5650 5550 5500 5550
Connection ~ 5500 5550
Text GLabel 3100 6050 0    50   Input ~ 0
TX1
Text GLabel 3100 6250 0    50   Input ~ 0
TX2
Text GLabel 3100 6450 0    50   Output ~ 0
RX1
Text GLabel 3100 6650 0    50   Output ~ 0
RX2
Wire Wire Line
	3100 6050 3300 6050
Wire Wire Line
	3100 6250 3300 6250
Wire Wire Line
	3100 6450 3300 6450
Wire Wire Line
	3100 6650 3300 6650
$Comp
L mechanical-connectors:CONN_01X04 P12
U 1 1 60850D14
P 4100 3400
F 0 "P12" H 4178 3486 50  0000 L CNN
F 1 "CONN_01X04" H 4178 3395 50  0000 L CNN
F 2 "connectors_JST_SH:Connectors_JST_SM04B-SRSS-TB" H 4178 3304 50  0000 L CNN
F 3 "" H 4100 3400 50  0000 C CNN
	1    4100 3400
	1    0    0    -1  
$EndComp
$Comp
L power-supply:+3V3 #PWR0198
U 1 1 60856971
P 3700 3150
F 0 "#PWR0198" H 3700 3000 50  0001 C CNN
F 1 "+3V3" H 3715 3323 50  0000 C CNN
F 2 "" H 3700 3150 50  0000 C CNN
F 3 "" H 3700 3150 50  0000 C CNN
	1    3700 3150
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0199
U 1 1 60857223
P 3600 3750
F 0 "#PWR0199" H 3600 3500 50  0001 C CNN
F 1 "GND" H 3605 3577 50  0000 C CNN
F 2 "" H 3600 3750 50  0000 C CNN
F 3 "" H 3600 3750 50  0000 C CNN
	1    3600 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3350 3700 3350
Wire Wire Line
	3700 3350 3700 3150
Wire Wire Line
	3900 3250 3600 3250
Wire Wire Line
	3600 3250 3600 3750
Text GLabel 3250 3550 0    50   Output ~ 0
RX3
Text GLabel 3250 3450 0    50   Input ~ 0
TX3
Wire Wire Line
	3900 3450 3250 3450
Wire Wire Line
	3900 3550 3250 3550
Text Label 5000 6050 0    50   ~ 0
nTX1
Text Label 5000 6250 0    50   ~ 0
nTX2
Text Label 5150 6350 0    50   ~ 0
nRX2
Text Label 5900 6350 0    50   ~ 0
nRX1
$EndSCHEMATC
