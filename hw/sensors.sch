EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 5
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
L MEMS:BNO055 U2
U 1 1 607D6898
P 2300 4000
F 0 "U2" H 3050 6187 60  0000 C CNN
F 1 "BNO055" H 3050 6081 60  0000 C CNN
F 2 "LGA:LGA-28-5832" H 2900 3950 60  0001 C CNN
F 3 "" H 2300 4150 60  0000 C CNN
	1    2300 4000
	1    0    0    -1  
$EndComp
$Comp
L power-supply:+3V3 #PWR0203
U 1 1 6063A7E4
P 1950 2000
F 0 "#PWR0203" H 1950 1850 50  0001 C CNN
F 1 "+3V3" H 1965 2173 50  0000 C CNN
F 2 "" H 1950 2000 50  0000 C CNN
F 3 "" H 1950 2000 50  0000 C CNN
	1    1950 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 2200 1950 2200
Wire Wire Line
	1950 2200 1950 2100
Wire Wire Line
	2300 2100 1950 2100
Connection ~ 1950 2100
Wire Wire Line
	1950 2100 1950 2000
NoConn ~ 3800 2100
NoConn ~ 3800 2600
$Comp
L devices:R_0603 R11
U 1 1 6063B2E3
P 1950 2650
F 0 "R11" H 2009 2696 50  0000 L CNN
F 1 "10k" H 2009 2605 50  0000 L CNN
F 2 "resistors:R_0603" H 1950 2500 50  0001 C CNN
F 3 "" H 1950 2650 50  0000 C CNN
	1    1950 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2550 1950 2200
Connection ~ 1950 2200
Wire Wire Line
	1950 2750 1950 2850
Wire Wire Line
	1950 2850 2300 2850
Wire Wire Line
	2300 2950 1800 2950
Text GLabel 1800 2950 0    50   Input ~ 0
RESET
$Comp
L devices:C_0603 C62
U 1 1 6064D505
P 1900 3300
F 0 "C62" H 1992 3346 50  0000 L CNN
F 1 "100nF" H 1992 3255 50  0000 L CNN
F 2 "capacitors:C_0603" H 1900 3150 50  0001 C CNN
F 3 "" H 1900 3300 50  0000 C CNN
	1    1900 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3150 1900 3150
Wire Wire Line
	1900 3150 1900 3200
$Comp
L power-supply:GND #PWR0204
U 1 1 6064E0B5
P 1900 4200
F 0 "#PWR0204" H 1900 3950 50  0001 C CNN
F 1 "GND" H 1905 4027 50  0000 C CNN
F 2 "" H 1900 4200 50  0000 C CNN
F 3 "" H 1900 4200 50  0000 C CNN
	1    1900 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3450 1900 3450
Connection ~ 1900 3450
Wire Wire Line
	1900 3450 1900 3400
Wire Wire Line
	2300 3550 1900 3550
Connection ~ 1900 3550
Wire Wire Line
	1900 3550 1900 3450
Wire Wire Line
	2300 3650 1900 3650
Wire Wire Line
	1900 3550 1900 3650
Connection ~ 1900 3650
Wire Wire Line
	1900 3650 1900 3750
Wire Wire Line
	1900 3750 2300 3750
Connection ~ 1900 3750
Wire Wire Line
	1900 3750 1900 3850
Wire Wire Line
	2300 3850 1900 3850
Connection ~ 1900 3850
Wire Wire Line
	1900 3850 1900 4200
NoConn ~ 3800 3200
$Comp
L power-supply:GND #PWR0205
U 1 1 6064F751
P 4000 4100
F 0 "#PWR0205" H 4000 3850 50  0001 C CNN
F 1 "GND" H 4005 3927 50  0000 C CNN
F 2 "" H 4000 4100 50  0000 C CNN
F 3 "" H 4000 4100 50  0000 C CNN
	1    4000 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 2850 4000 2850
Wire Wire Line
	4000 2850 4000 2950
Wire Wire Line
	3800 2950 4000 2950
Connection ~ 4000 2950
Wire Wire Line
	4000 2950 4000 3750
Wire Wire Line
	3800 3550 4500 3550
Wire Wire Line
	3800 3650 4500 3650
Wire Wire Line
	3800 3750 4000 3750
Connection ~ 4000 3750
Wire Wire Line
	4000 3750 4000 4100
Text GLabel 4500 3550 2    50   BiDi ~ 0
SDA1
Text GLabel 4500 3650 2    50   Input ~ 0
SCL1
$Comp
L devices:R_0603 R?
U 1 1 60654D6E
P 5050 3700
AR Path="/60654D6E" Ref="R?"  Part="1" 
AR Path="/60664788/60654D6E" Ref="R12"  Part="1" 
F 0 "R12" H 5109 3746 50  0000 L CNN
F 1 "0R0" H 5109 3655 50  0000 L CNN
F 2 "resistors:R_0603" H 5050 3550 50  0001 C CNN
F 3 "" H 5050 3700 50  0000 C CNN
	1    5050 3700
	1    0    0    -1  
$EndComp
$Comp
L devices:R_0603 R?
U 1 1 60654D74
P 5050 4000
AR Path="/60654D74" Ref="R?"  Part="1" 
AR Path="/60664788/60654D74" Ref="R13"  Part="1" 
F 0 "R13" H 5109 4046 50  0000 L CNN
F 1 "0R0" H 5109 3955 50  0000 L CNN
F 2 "resistors:R_0603" H 5050 3850 50  0001 C CNN
F 3 "" H 5050 4000 50  0000 C CNN
	1    5050 4000
	1    0    0    -1  
$EndComp
$Comp
L power-supply:+3V3 #PWR?
U 1 1 60654D7A
P 5050 3500
AR Path="/60654D7A" Ref="#PWR?"  Part="1" 
AR Path="/60664788/60654D7A" Ref="#PWR0206"  Part="1" 
F 0 "#PWR0206" H 5050 3350 50  0001 C CNN
F 1 "+3V3" H 5065 3673 50  0000 C CNN
F 2 "" H 5050 3500 50  0000 C CNN
F 3 "" H 5050 3500 50  0000 C CNN
	1    5050 3500
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR?
U 1 1 60654D80
P 5050 4200
AR Path="/60654D80" Ref="#PWR?"  Part="1" 
AR Path="/60664788/60654D80" Ref="#PWR0207"  Part="1" 
F 0 "#PWR0207" H 5050 3950 50  0001 C CNN
F 1 "GND" H 5055 4027 50  0000 C CNN
F 2 "" H 5050 4200 50  0000 C CNN
F 3 "" H 5050 4200 50  0000 C CNN
	1    5050 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3600 5050 3500
Wire Wire Line
	5050 4100 5050 4200
Wire Wire Line
	5050 3800 5050 3850
Wire Wire Line
	3800 3850 5050 3850
Connection ~ 5050 3850
Wire Wire Line
	5050 3850 5050 3900
$Comp
L devices:C_0603 C61
U 1 1 60656022
P 1600 2250
F 0 "C61" H 1692 2296 50  0000 L CNN
F 1 "100nF" H 1692 2205 50  0000 L CNN
F 2 "capacitors:C_0603" H 1600 2100 50  0001 C CNN
F 3 "" H 1600 2250 50  0000 C CNN
	1    1600 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2100 1600 2100
Wire Wire Line
	1600 2100 1600 2150
Wire Wire Line
	1600 2350 1600 2400
$Comp
L power-supply:GND #PWR0208
U 1 1 60657D51
P 1600 2400
F 0 "#PWR0208" H 1600 2150 50  0001 C CNN
F 1 "GND" H 1605 2227 50  0000 C CNN
F 2 "" H 1600 2400 50  0000 C CNN
F 3 "" H 1600 2400 50  0000 C CNN
	1    1600 2400
	1    0    0    -1  
$EndComp
$Comp
L mechanical-connectors:CONN_02X03 P11
U 1 1 606596CB
P 8100 3250
F 0 "P11" H 8100 3655 50  0000 C CNN
F 1 "CONN_02X03" H 8100 3564 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Horizontal" H 8100 3473 50  0000 C CNN
F 3 "" H 8100 2050 50  0000 C CNN
	1    8100 3250
	1    0    0    -1  
$EndComp
$Comp
L power-supply:+5V #PWR0209
U 1 1 6065A147
P 8100 2650
F 0 "#PWR0209" H 8100 2500 50  0001 C CNN
F 1 "+5V" H 8115 2823 50  0000 C CNN
F 2 "" H 8100 2650 50  0000 C CNN
F 3 "" H 8100 2650 50  0000 C CNN
	1    8100 2650
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0210
U 1 1 6065A443
P 8100 3650
F 0 "#PWR0210" H 8100 3400 50  0001 C CNN
F 1 "GND" H 8105 3477 50  0000 C CNN
F 2 "" H 8100 3650 50  0000 C CNN
F 3 "" H 8100 3650 50  0000 C CNN
	1    8100 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 3150 7750 3150
Wire Wire Line
	7750 3150 7750 2850
Wire Wire Line
	7750 2850 8100 2850
Wire Wire Line
	8100 2850 8100 2650
Wire Wire Line
	8350 3150 8450 3150
Wire Wire Line
	8450 3150 8450 2850
Wire Wire Line
	8450 2850 8100 2850
Connection ~ 8100 2850
Wire Wire Line
	7850 3350 7750 3350
Wire Wire Line
	7750 3350 7750 3550
Wire Wire Line
	7750 3550 8100 3550
Wire Wire Line
	8100 3550 8100 3650
Wire Wire Line
	8350 3350 8450 3350
Wire Wire Line
	8450 3350 8450 3550
Wire Wire Line
	8450 3550 8100 3550
Connection ~ 8100 3550
Text GLabel 7750 3250 0    50   Output ~ 0
ENCODER0
Text GLabel 8450 3250 2    50   Output ~ 0
ENCODER1
Wire Wire Line
	8350 3250 8450 3250
Wire Wire Line
	7850 3250 7750 3250
$Comp
L devices:R_0603 R55
U 1 1 607BB7DA
P 6150 4850
F 0 "R55" H 6209 4896 50  0000 L CNN
F 1 "150k" H 6209 4805 50  0000 L CNN
F 2 "resistors:R_0603" H 6150 4700 50  0001 C CNN
F 3 "" H 6150 4850 50  0000 C CNN
	1    6150 4850
	1    0    0    -1  
$EndComp
$Comp
L devices:R_0603 R56
U 1 1 607BC157
P 6150 5300
F 0 "R56" H 6209 5346 50  0000 L CNN
F 1 "10k" H 6209 5255 50  0000 L CNN
F 2 "resistors:R_0603" H 6150 5150 50  0001 C CNN
F 3 "" H 6150 5300 50  0000 C CNN
	1    6150 5300
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0212
U 1 1 607BCD79
P 6150 5600
F 0 "#PWR0212" H 6150 5350 50  0001 C CNN
F 1 "GND" H 6155 5427 50  0000 C CNN
F 2 "" H 6150 5600 50  0000 C CNN
F 3 "" H 6150 5600 50  0000 C CNN
	1    6150 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 4950 6150 5050
Wire Wire Line
	6150 5400 6150 5600
Wire Wire Line
	6150 4600 6150 4750
Wire Wire Line
	6150 5050 6300 5050
Connection ~ 6150 5050
Wire Wire Line
	6150 5050 6150 5200
Text GLabel 6300 5050 2    50   Output ~ 0
VIN_MEAS
$Comp
L power-supply:+BATT #PWR0213
U 1 1 607CC3D4
P 6150 4600
F 0 "#PWR0213" H 6150 4450 50  0001 C CNN
F 1 "+BATT" H 6165 4773 50  0000 C CNN
F 2 "" H 6150 4600 50  0000 C CNN
F 3 "" H 6150 4600 50  0000 C CNN
	1    6150 4600
	1    0    0    -1  
$EndComp
$EndSCHEMATC
