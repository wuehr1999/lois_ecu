EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 6
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
L mechanical-connectors:CONN_01X03 P?
U 1 1 6072D290
P 5650 2400
AR Path="/6072D290" Ref="P?"  Part="1" 
AR Path="/60663BD3/6072D290" Ref="P1"  Part="1" 
F 0 "P1" H 5728 2441 50  0000 L CNN
F 1 "CONN_01X03" H 5728 2350 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Horizontal" H 5728 2304 50  0001 L CNN
F 3 "" H 5650 2400 50  0000 C CNN
	1    5650 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2300 5000 2300
Wire Wire Line
	5450 2400 5300 2400
Wire Wire Line
	5300 2400 5300 2150
$Comp
L power-supply:+5V #PWR?
U 1 1 6072D299
P 5300 2150
AR Path="/6072D299" Ref="#PWR?"  Part="1" 
AR Path="/60663BD3/6072D299" Ref="#PWR0105"  Part="1" 
F 0 "#PWR0105" H 5300 2000 50  0001 C CNN
F 1 "+5V" H 5315 2323 50  0000 C CNN
F 2 "" H 5300 2150 50  0000 C CNN
F 3 "" H 5300 2150 50  0000 C CNN
	1    5300 2150
	1    0    0    -1  
$EndComp
$Comp
L devices:C_1206 C?
U 1 1 6072D29F
P 5300 2550
AR Path="/6072D29F" Ref="C?"  Part="1" 
AR Path="/60663BD3/6072D29F" Ref="C51"  Part="1" 
F 0 "C51" H 5150 2600 50  0000 L CNN
F 1 "22uF" H 5050 2500 50  0000 L CNN
F 2 "capacitors:C_1206" H 5300 2400 50  0001 C CNN
F 3 "" H 5300 2550 50  0000 C CNN
	1    5300 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2450 5300 2400
Connection ~ 5300 2400
Wire Wire Line
	5450 2500 5450 2750
Wire Wire Line
	5450 2750 5300 2750
Wire Wire Line
	5300 2750 5300 2650
Wire Wire Line
	5450 2750 5450 2850
Connection ~ 5450 2750
$Comp
L power-supply:GND #PWR?
U 1 1 6072D2AC
P 5450 2850
AR Path="/6072D2AC" Ref="#PWR?"  Part="1" 
AR Path="/60663BD3/6072D2AC" Ref="#PWR0106"  Part="1" 
F 0 "#PWR0106" H 5450 2600 50  0001 C CNN
F 1 "GND" H 5455 2677 50  0000 C CNN
F 2 "" H 5450 2850 50  0000 C CNN
F 3 "" H 5450 2850 50  0000 C CNN
	1    5450 2850
	1    0    0    -1  
$EndComp
$Comp
L devices:R_0603 R?
U 1 1 6072D2DA
P 4900 2300
AR Path="/6072D2DA" Ref="R?"  Part="1" 
AR Path="/60663BD3/6072D2DA" Ref="R1"  Part="1" 
F 0 "R1" V 4800 2300 50  0000 C CNN
F 1 "0R0" V 5000 2300 50  0000 C CNN
F 2 "resistors:R_0603" H 4900 2150 50  0001 C CNN
F 3 "" H 4900 2300 50  0000 C CNN
	1    4900 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 2300 4800 2300
Text GLabel 4300 2300 0    50   Input ~ 0
SERVO0
$EndSCHEMATC
