EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 5
Title ""
Date "2019-12-29"
Rev ""
Comp "JECC"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L devices:FUSE F1
U 1 1 5ED360E7
P 2250 1500
AR Path="/5F5F9720/5ED360E7" Ref="F1"  Part="1" 
AR Path="/5F5F9C85/5ED360E7" Ref="F?"  Part="1" 
F 0 "F1" H 2250 1830 50  0000 C CNN
F 1 "15A" H 2250 1739 50  0000 C CNN
F 2 "fuse_holders_and_fuses:Fuseholder_ATO_Blade_littlefuse_8_Pin" H 2250 1648 50  0001 C CNN
F 3 "" H 2250 1500 50  0000 C CNN
	1    2250 1500
	1    0    0    -1  
$EndComp
$Comp
L devices:C_0603 C12
U 1 1 5ED3911A
P 3250 1800
AR Path="/5F5F9720/5ED3911A" Ref="C12"  Part="1" 
AR Path="/5F5F9C85/5ED3911A" Ref="C?"  Part="1" 
F 0 "C12" H 3342 1846 50  0000 L CNN
F 1 "100nF" H 3342 1755 50  0000 L CNN
F 2 "capacitors:C_1206" H 3250 1650 50  0001 C CNN
F 3 "" H 3250 1800 50  0000 C CNN
	1    3250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1500 2700 1500
Wire Wire Line
	2000 1500 1900 1500
Wire Wire Line
	1900 1500 1900 1750
Wire Wire Line
	1900 1850 1900 2150
Wire Wire Line
	3250 1700 3250 1500
Connection ~ 3250 1500
$Comp
L power-supply:+BATT #PWR0128
U 1 1 5ED5D5AA
P 4400 1450
AR Path="/5F5F9720/5ED5D5AA" Ref="#PWR0128"  Part="1" 
AR Path="/5F5F9C85/5ED5D5AA" Ref="#PWR?"  Part="1" 
F 0 "#PWR0128" H 4400 1300 50  0001 C CNN
F 1 "+BATT" H 4415 1623 50  0000 C CNN
F 2 "" H 4400 1450 50  0000 C CNN
F 3 "" H 4400 1450 50  0000 C CNN
	1    4400 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2250 3250 2150
$Comp
L voltage-regulators:TPS54335A U4
U 1 1 5EDCA4F6
P 3350 4300
AR Path="/5F5F9720/5EDCA4F6" Ref="U4"  Part="1" 
AR Path="/5F5F9C85/5EDCA4F6" Ref="U?"  Part="1" 
F 0 "U4" H 3350 5140 50  0000 C CNN
F 1 "TPS54335A" H 3350 5049 50  0000 C CNN
F 2 "SOIC:SOIC-8-1EP_3.9x4.9mm_Pitch1.27mm" H 3250 4200 50  0001 C CNN
F 3 "" H 3350 4300 50  0001 C CNN
	1    3350 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3800 2450 3800
$Comp
L devices:C_1206 C9
U 1 1 5EDCC31F
P 2150 3600
AR Path="/5F5F9720/5EDCC31F" Ref="C9"  Part="1" 
AR Path="/5F5F9C85/5EDCC31F" Ref="C?"  Part="1" 
F 0 "C9" H 2242 3646 50  0000 L CNN
F 1 "4.7uF 50V" H 2242 3555 50  0000 L CNN
F 2 "capacitors:C_1206" H 2150 3450 50  0001 C CNN
F 3 "" H 2150 3600 50  0000 C CNN
	1    2150 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3450 2150 3450
$Comp
L devices:C_1206 C8
U 1 1 5EDCD909
P 1950 3600
AR Path="/5F5F9720/5EDCD909" Ref="C8"  Part="1" 
AR Path="/5F5F9C85/5EDCD909" Ref="C?"  Part="1" 
F 0 "C8" H 2042 3646 50  0000 L CNN
F 1 "4.7uF 50V" H 2042 3555 50  0000 L CNN
F 2 "capacitors:C_1206" H 1950 3450 50  0001 C CNN
F 3 "" H 1950 3600 50  0000 C CNN
	1    1950 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3500 1950 3450
Wire Wire Line
	2150 3500 2150 3450
Connection ~ 2150 3450
Wire Wire Line
	2150 3450 2450 3450
Wire Wire Line
	1950 3700 1950 3750
Wire Wire Line
	1950 3750 2150 3750
Wire Wire Line
	2150 3750 2150 3700
Wire Wire Line
	1950 3750 1950 3800
Connection ~ 1950 3750
$Comp
L power-supply:GND #PWR0129
U 1 1 5EDD00B0
P 1950 3800
AR Path="/5F5F9720/5EDD00B0" Ref="#PWR0129"  Part="1" 
AR Path="/5F5F9C85/5EDD00B0" Ref="#PWR?"  Part="1" 
F 0 "#PWR0129" H 1950 3550 50  0001 C CNN
F 1 "GND" H 1955 3627 50  0000 C CNN
F 2 "" H 1950 3800 50  0000 C CNN
F 3 "" H 1950 3800 50  0000 C CNN
	1    1950 3800
	1    0    0    -1  
$EndComp
Connection ~ 2450 3450
Wire Wire Line
	2450 3450 2450 3400
Wire Wire Line
	2450 3450 2450 3800
$Comp
L power-supply:+BATT #PWR0130
U 1 1 5EDD1FFB
P 2450 3400
AR Path="/5F5F9720/5EDD1FFB" Ref="#PWR0130"  Part="1" 
AR Path="/5F5F9C85/5EDD1FFB" Ref="#PWR?"  Part="1" 
F 0 "#PWR0130" H 2450 3250 50  0001 C CNN
F 1 "+BATT" H 2465 3573 50  0000 C CNN
F 2 "" H 2450 3400 50  0000 C CNN
F 3 "" H 2450 3400 50  0000 C CNN
	1    2450 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 4050 1500 4050
Wire Wire Line
	1500 4050 1500 4000
Wire Wire Line
	1500 4050 1500 4100
Connection ~ 1500 4050
$Comp
L devices:R_0603 R16
U 1 1 5EDD3F74
P 1500 3900
AR Path="/5F5F9720/5EDD3F74" Ref="R16"  Part="1" 
AR Path="/5F5F9C85/5EDD3F74" Ref="R?"  Part="1" 
F 0 "R16" H 1559 3946 50  0000 L CNN
F 1 "220k" H 1559 3855 50  0000 L CNN
F 2 "resistors:R_0603" H 1500 3750 50  0001 C CNN
F 3 "" H 1500 3900 50  0000 C CNN
	1    1500 3900
	1    0    0    -1  
$EndComp
$Comp
L devices:R_0603 R17
U 1 1 5EDD4333
P 1500 4200
AR Path="/5F5F9720/5EDD4333" Ref="R17"  Part="1" 
AR Path="/5F5F9C85/5EDD4333" Ref="R?"  Part="1" 
F 0 "R17" H 1559 4246 50  0000 L CNN
F 1 "43.2k" H 1559 4155 50  0000 L CNN
F 2 "resistors:R_0603" H 1500 4050 50  0001 C CNN
F 3 "" H 1500 4200 50  0000 C CNN
	1    1500 4200
	1    0    0    -1  
$EndComp
$Comp
L power-supply:+BATT #PWR0131
U 1 1 5EDD7C1A
P 1500 3750
AR Path="/5F5F9720/5EDD7C1A" Ref="#PWR0131"  Part="1" 
AR Path="/5F5F9C85/5EDD7C1A" Ref="#PWR?"  Part="1" 
F 0 "#PWR0131" H 1500 3600 50  0001 C CNN
F 1 "+BATT" H 1515 3923 50  0000 C CNN
F 2 "" H 1500 3750 50  0000 C CNN
F 3 "" H 1500 3750 50  0000 C CNN
	1    1500 3750
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0132
U 1 1 5EDD7ECB
P 1500 4350
AR Path="/5F5F9720/5EDD7ECB" Ref="#PWR0132"  Part="1" 
AR Path="/5F5F9C85/5EDD7ECB" Ref="#PWR?"  Part="1" 
F 0 "#PWR0132" H 1500 4100 50  0001 C CNN
F 1 "GND" H 1505 4177 50  0000 C CNN
F 2 "" H 1500 4350 50  0000 C CNN
F 3 "" H 1500 4350 50  0000 C CNN
	1    1500 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3750 1500 3800
Wire Wire Line
	1500 4300 1500 4350
Wire Wire Line
	2600 4250 1850 4250
Wire Wire Line
	1850 4250 1850 4300
Text Label 2200 4050 0    50   ~ 0
EN
$Comp
L devices:R_0603 R18
U 1 1 5EDDAA24
P 1850 4400
AR Path="/5F5F9720/5EDDAA24" Ref="R18"  Part="1" 
AR Path="/5F5F9C85/5EDDAA24" Ref="R?"  Part="1" 
F 0 "R18" H 1909 4446 50  0000 L CNN
F 1 "143k" H 1909 4355 50  0000 L CNN
F 2 "resistors:R_0603" H 1850 4250 50  0001 C CNN
F 3 "" H 1850 4400 50  0000 C CNN
	1    1850 4400
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0133
U 1 1 5EDDAE67
P 1850 4550
AR Path="/5F5F9720/5EDDAE67" Ref="#PWR0133"  Part="1" 
AR Path="/5F5F9C85/5EDDAE67" Ref="#PWR?"  Part="1" 
F 0 "#PWR0133" H 1850 4300 50  0001 C CNN
F 1 "GND" H 1855 4377 50  0000 C CNN
F 2 "" H 1850 4550 50  0000 C CNN
F 3 "" H 1850 4550 50  0000 C CNN
	1    1850 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 4550 1850 4500
$Comp
L devices:C_0603 C11
U 1 1 5EDDC5EE
P 2400 4600
AR Path="/5F5F9720/5EDDC5EE" Ref="C11"  Part="1" 
AR Path="/5F5F9C85/5EDDC5EE" Ref="C?"  Part="1" 
F 0 "C11" H 2492 4646 50  0000 L CNN
F 1 "12nF" H 2492 4555 50  0000 L CNN
F 2 "capacitors:C_0603" H 2400 4450 50  0001 C CNN
F 3 "" H 2400 4600 50  0000 C CNN
	1    2400 4600
	1    0    0    -1  
$EndComp
$Comp
L devices:C_0603 C10
U 1 1 5EDDCBDA
P 2150 4750
AR Path="/5F5F9720/5EDDCBDA" Ref="C10"  Part="1" 
AR Path="/5F5F9C85/5EDDCBDA" Ref="C?"  Part="1" 
F 0 "C10" H 2242 4796 50  0000 L CNN
F 1 "120pF" H 1900 4700 50  0000 L CNN
F 2 "capacitors:C_0603" H 2150 4600 50  0001 C CNN
F 3 "" H 2150 4750 50  0000 C CNN
	1    2150 4750
	1    0    0    -1  
$EndComp
$Comp
L devices:R_0603 R19
U 1 1 5EDDD09D
P 2400 4900
AR Path="/5F5F9720/5EDDD09D" Ref="R19"  Part="1" 
AR Path="/5F5F9C85/5EDDD09D" Ref="R?"  Part="1" 
F 0 "R19" H 2459 4946 50  0000 L CNN
F 1 "3.74k" H 2459 4855 50  0000 L CNN
F 2 "resistors:R_0603" H 2400 4750 50  0001 C CNN
F 3 "" H 2400 4900 50  0000 C CNN
	1    2400 4900
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0134
U 1 1 5EDDD61E
P 2400 5150
AR Path="/5F5F9720/5EDDD61E" Ref="#PWR0134"  Part="1" 
AR Path="/5F5F9C85/5EDDD61E" Ref="#PWR?"  Part="1" 
F 0 "#PWR0134" H 2400 4900 50  0001 C CNN
F 1 "GND" H 2405 4977 50  0000 C CNN
F 2 "" H 2400 5150 50  0000 C CNN
F 3 "" H 2400 5150 50  0000 C CNN
	1    2400 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 4450 2400 4450
Wire Wire Line
	2150 4450 2150 4650
Wire Wire Line
	2400 4500 2400 4450
Connection ~ 2400 4450
Wire Wire Line
	2400 4450 2150 4450
Wire Wire Line
	2400 4700 2400 4800
Wire Wire Line
	2150 4850 2150 5100
Wire Wire Line
	2150 5100 2400 5100
Wire Wire Line
	2550 5100 2550 4950
Wire Wire Line
	2550 4800 2600 4800
Wire Wire Line
	2600 4950 2550 4950
Connection ~ 2550 4950
Wire Wire Line
	2550 4950 2550 4800
Wire Wire Line
	2400 5000 2400 5100
Connection ~ 2400 5100
Wire Wire Line
	2400 5100 2550 5100
Wire Wire Line
	2400 5100 2400 5150
Text Label 2200 4250 0    50   ~ 0
RT
Text Label 2150 4450 0    50   ~ 0
COMP
$Comp
L devices:C_0603 C16
U 1 1 5EDE7565
P 4350 3800
AR Path="/5F5F9720/5EDE7565" Ref="C16"  Part="1" 
AR Path="/5F5F9C85/5EDE7565" Ref="C?"  Part="1" 
F 0 "C16" V 4121 3800 50  0000 C CNN
F 1 "100nF" V 4212 3800 50  0000 C CNN
F 2 "capacitors:C_0603" H 4350 3650 50  0001 C CNN
F 3 "" H 4350 3800 50  0000 C CNN
	1    4350 3800
	0    1    1    0   
$EndComp
$Comp
L devices:C_1206 C19
U 1 1 5EDE7C21
P 5200 4400
AR Path="/5F5F9720/5EDE7C21" Ref="C19"  Part="1" 
AR Path="/5F5F9C85/5EDE7C21" Ref="C?"  Part="1" 
F 0 "C19" H 5292 4446 50  0000 L CNN
F 1 "22uF" H 5292 4355 50  0000 L CNN
F 2 "capacitors:C_1206" H 5200 4250 50  0001 C CNN
F 3 "" H 5200 4400 50  0000 C CNN
	1    5200 4400
	1    0    0    -1  
$EndComp
$Comp
L devices:C_1206 C20
U 1 1 5EDE81BD
P 5700 4400
AR Path="/5F5F9720/5EDE81BD" Ref="C20"  Part="1" 
AR Path="/5F5F9C85/5EDE81BD" Ref="C?"  Part="1" 
F 0 "C20" H 5792 4446 50  0000 L CNN
F 1 "22uF" H 5792 4355 50  0000 L CNN
F 2 "capacitors:C_1206" H 5700 4250 50  0001 C CNN
F 3 "" H 5700 4400 50  0000 C CNN
	1    5700 4400
	1    0    0    -1  
$EndComp
$Comp
L L_power:74437346100 L1
U 1 1 5EDEACA6
P 4850 4200
AR Path="/5F5F9720/5EDEACA6" Ref="L1"  Part="1" 
AR Path="/5F5F9C85/5EDEACA6" Ref="L?"  Part="1" 
F 0 "L1" H 4850 4505 50  0000 C CNN
F 1 "74437324150" H 4850 4414 50  0000 C CNN
F 2 "L_power:WE-LHMI40xx" H 4850 4323 50  0000 C CNN
F 3 "" H 4850 4200 50  0000 C CNN
	1    4850 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3800 4250 3800
Wire Wire Line
	4450 3800 4500 3800
Wire Wire Line
	4500 3800 4500 4200
Wire Wire Line
	4500 4200 4600 4200
Wire Wire Line
	4100 4200 4500 4200
Connection ~ 4500 4200
Wire Wire Line
	5100 4200 5200 4200
Wire Wire Line
	5200 4300 5200 4200
Connection ~ 5200 4200
Wire Wire Line
	5200 4200 5700 4200
Wire Wire Line
	5700 4300 5700 4200
Connection ~ 5700 4200
Wire Wire Line
	5700 4200 6100 4200
Wire Wire Line
	5200 4500 5200 4550
Wire Wire Line
	5200 4550 5700 4550
Wire Wire Line
	5700 4550 5700 4500
Wire Wire Line
	5200 4550 5200 4600
Connection ~ 5200 4550
$Comp
L power-supply:GND #PWR0135
U 1 1 5EDFB36D
P 5200 4600
AR Path="/5F5F9720/5EDFB36D" Ref="#PWR0135"  Part="1" 
AR Path="/5F5F9C85/5EDFB36D" Ref="#PWR?"  Part="1" 
F 0 "#PWR0135" H 5200 4350 50  0001 C CNN
F 1 "GND" H 5205 4427 50  0000 C CNN
F 2 "" H 5200 4600 50  0000 C CNN
F 3 "" H 5200 4600 50  0000 C CNN
	1    5200 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4100 6100 4200
Connection ~ 6100 4200
Wire Wire Line
	6100 4200 6100 4350
$Comp
L power-supply:+5V #PWR0136
U 1 1 5EDFD5F4
P 6100 4100
AR Path="/5F5F9720/5EDFD5F4" Ref="#PWR0136"  Part="1" 
AR Path="/5F5F9C85/5EDFD5F4" Ref="#PWR?"  Part="1" 
F 0 "#PWR0136" H 6100 3950 50  0001 C CNN
F 1 "+5V" H 6115 4273 50  0000 C CNN
F 2 "" H 6100 4100 50  0000 C CNN
F 3 "" H 6100 4100 50  0000 C CNN
	1    6100 4100
	1    0    0    -1  
$EndComp
$Comp
L devices:R_0603 R21
U 1 1 5EDFD987
P 6100 4450
AR Path="/5F5F9720/5EDFD987" Ref="R21"  Part="1" 
AR Path="/5F5F9C85/5EDFD987" Ref="R?"  Part="1" 
F 0 "R21" H 6159 4496 50  0000 L CNN
F 1 "100k" H 6159 4405 50  0000 L CNN
F 2 "resistors:R_0603" H 6100 4300 50  0001 C CNN
F 3 "" H 6100 4450 50  0000 C CNN
	1    6100 4450
	1    0    0    -1  
$EndComp
$Comp
L devices:R_0603 R22
U 1 1 5EDFDF17
P 6100 4900
AR Path="/5F5F9720/5EDFDF17" Ref="R22"  Part="1" 
AR Path="/5F5F9C85/5EDFDF17" Ref="R?"  Part="1" 
F 0 "R22" H 6159 4946 50  0000 L CNN
F 1 "19.1k" H 6159 4855 50  0000 L CNN
F 2 "resistors:R_0603" H 6100 4750 50  0001 C CNN
F 3 "" H 6100 4900 50  0000 C CNN
	1    6100 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4550 6100 4700
Wire Wire Line
	4100 4700 5000 4700
Wire Wire Line
	5000 4700 5000 4850
Wire Wire Line
	5000 4850 5450 4850
Wire Wire Line
	5450 4850 5450 4700
Wire Wire Line
	5450 4700 6100 4700
Connection ~ 6100 4700
Wire Wire Line
	6100 4700 6100 4800
Wire Wire Line
	6100 5000 6100 5050
$Comp
L power-supply:GND #PWR0137
U 1 1 5EE04652
P 6100 5050
AR Path="/5F5F9720/5EE04652" Ref="#PWR0137"  Part="1" 
AR Path="/5F5F9C85/5EE04652" Ref="#PWR?"  Part="1" 
F 0 "#PWR0137" H 6100 4800 50  0001 C CNN
F 1 "GND" H 6105 4877 50  0000 C CNN
F 2 "" H 6100 5050 50  0000 C CNN
F 3 "" H 6100 5050 50  0000 C CNN
	1    6100 5050
	1    0    0    -1  
$EndComp
Text Label 4050 3800 0    50   ~ 0
BOOT
Text Label 4150 4200 0    50   ~ 0
PH
Text Label 4150 4700 0    50   ~ 0
SENSE5V
$Comp
L voltage-regulators:LT1117 U3
U 1 1 5EE092E2
P 2550 6800
AR Path="/5F5F9720/5EE092E2" Ref="U3"  Part="1" 
AR Path="/5F5F9C85/5EE092E2" Ref="U?"  Part="1" 
F 0 "U3" H 2550 7187 60  0000 C CNN
F 1 "LT1117" H 2550 7081 60  0000 C CNN
F 2 "SOT_TO:SOT-223" H 2500 6500 60  0001 C CNN
F 3 "" H 2550 6800 60  0000 C CNN
	1    2550 6800
	1    0    0    -1  
$EndComp
$Comp
L devices:C_1206 C7
U 1 1 5EE0A0E4
P 1400 6950
AR Path="/5F5F9720/5EE0A0E4" Ref="C7"  Part="1" 
AR Path="/5F5F9C85/5EE0A0E4" Ref="C?"  Part="1" 
F 0 "C7" H 1492 6996 50  0000 L CNN
F 1 "10uF" H 1492 6905 50  0000 L CNN
F 2 "capacitors:C_1206" H 1400 6800 50  0001 C CNN
F 3 "" H 1400 6950 50  0000 C CNN
	1    1400 6950
	1    0    0    -1  
$EndComp
$Comp
L devices:C_1206 C14
U 1 1 5EE0A47C
P 3450 7050
AR Path="/5F5F9720/5EE0A47C" Ref="C14"  Part="1" 
AR Path="/5F5F9C85/5EE0A47C" Ref="C?"  Part="1" 
F 0 "C14" H 3542 7096 50  0000 L CNN
F 1 "22uF" H 3542 7005 50  0000 L CNN
F 2 "capacitors:C_1206" H 3450 6900 50  0001 C CNN
F 3 "" H 3450 7050 50  0000 C CNN
	1    3450 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 6700 1400 6700
Wire Wire Line
	1400 6700 1400 6850
Connection ~ 1400 6700
Wire Wire Line
	1400 7050 1400 7200
Wire Wire Line
	1400 7200 1750 7200
Wire Wire Line
	3450 7200 3450 7150
Wire Wire Line
	1900 6850 1750 6850
Wire Wire Line
	1750 6850 1750 7200
Connection ~ 1750 7200
Wire Wire Line
	1750 7200 3450 7200
Wire Wire Line
	1750 7200 1750 7300
Wire Wire Line
	3200 6700 3450 6700
Wire Wire Line
	3450 6700 3450 6600
Wire Wire Line
	3200 6850 3450 6850
Wire Wire Line
	3450 6850 3450 6700
Connection ~ 3450 6700
Wire Wire Line
	3450 6950 3450 6850
Connection ~ 3450 6850
Wire Wire Line
	1400 6600 1400 6700
$Comp
L power-supply:+5V #PWR0138
U 1 1 5EE285F2
P 1400 6600
AR Path="/5F5F9720/5EE285F2" Ref="#PWR0138"  Part="1" 
AR Path="/5F5F9C85/5EE285F2" Ref="#PWR?"  Part="1" 
F 0 "#PWR0138" H 1400 6450 50  0001 C CNN
F 1 "+5V" H 1415 6773 50  0000 C CNN
F 2 "" H 1400 6600 50  0000 C CNN
F 3 "" H 1400 6600 50  0000 C CNN
	1    1400 6600
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0139
U 1 1 5EE28C71
P 1750 7300
AR Path="/5F5F9720/5EE28C71" Ref="#PWR0139"  Part="1" 
AR Path="/5F5F9C85/5EE28C71" Ref="#PWR?"  Part="1" 
F 0 "#PWR0139" H 1750 7050 50  0001 C CNN
F 1 "GND" H 1755 7127 50  0000 C CNN
F 2 "" H 1750 7300 50  0000 C CNN
F 3 "" H 1750 7300 50  0000 C CNN
	1    1750 7300
	1    0    0    -1  
$EndComp
$Comp
L power-supply:+3V3 #PWR0140
U 1 1 5EE29197
P 3450 6600
AR Path="/5F5F9720/5EE29197" Ref="#PWR0140"  Part="1" 
AR Path="/5F5F9C85/5EE29197" Ref="#PWR?"  Part="1" 
F 0 "#PWR0140" H 3450 6450 50  0001 C CNN
F 1 "+3V3" H 3465 6773 50  0000 C CNN
F 2 "" H 3450 6600 50  0000 C CNN
F 3 "" H 3450 6600 50  0000 C CNN
	1    3450 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 1900 3250 2150
Wire Wire Line
	1900 2150 2700 2150
Connection ~ 3250 2150
$Comp
L power-supply:GND #PWR0141
U 1 1 5EE99C6D
P 3250 2250
AR Path="/5F5F9720/5EE99C6D" Ref="#PWR0141"  Part="1" 
AR Path="/5F5F9C85/5EE99C6D" Ref="#PWR?"  Part="1" 
F 0 "#PWR0141" H 3250 2000 50  0001 C CNN
F 1 "GND" H 3255 2077 50  0000 C CNN
F 2 "" H 3250 2250 50  0000 C CNN
F 3 "" H 3250 2250 50  0000 C CNN
	1    3250 2250
	1    0    0    -1  
$EndComp
$Comp
L devices:CP_Small C13
U 1 1 5F63C7AE
P 3850 1800
F 0 "C13" H 3938 1846 50  0000 L CNN
F 1 "47uF " H 3938 1755 50  0000 L CNN
F 2 "capacitors:TantalC_SizeD_EIA-7343_Wave" H 3938 1709 50  0001 L CNN
F 3 "" H 3850 1800 50  0000 C CNN
	1    3850 1800
	1    0    0    -1  
$EndComp
$Comp
L devices:CP_Small C15
U 1 1 5F63D2B5
P 4400 1800
F 0 "C15" H 4488 1846 50  0000 L CNN
F 1 "47uF" H 4488 1755 50  0000 L CNN
F 2 "capacitors:TantalC_SizeD_EIA-7343_Wave" H 4400 1800 50  0001 C CNN
F 3 "" H 4400 1800 50  0000 C CNN
	1    4400 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 1500 4400 1700
Wire Wire Line
	3850 1700 3850 1500
Wire Wire Line
	3250 1500 3850 1500
Connection ~ 3850 1500
Wire Wire Line
	3850 1500 4400 1500
Wire Wire Line
	3850 1900 3850 2150
Wire Wire Line
	3250 2150 3850 2150
Wire Wire Line
	3850 2150 4400 2150
Wire Wire Line
	4400 2150 4400 1900
Connection ~ 3850 2150
$Comp
L devices:CP_Small C21
U 1 1 5F64AE53
P 7400 4600
F 0 "C21" H 7488 4691 50  0000 L CNN
F 1 "47uF" H 7488 4600 50  0000 L CNN
F 2 "capacitors:c_elec_8x6.7" H 7488 4509 50  0000 L CNN
F 3 "" H 7400 4600 50  0000 C CNN
	1    7400 4600
	1    0    0    -1  
$EndComp
$Comp
L devices:CP_Small C17
U 1 1 5F64D021
P 4550 6950
F 0 "C17" H 4638 7041 50  0000 L CNN
F 1 "47uF" H 4638 6950 50  0000 L CNN
F 2 "capacitors:c_elec_8x6.7" H 4638 6859 50  0000 L CNN
F 3 "" H 4550 6950 50  0000 C CNN
	1    4550 6950
	1    0    0    -1  
$EndComp
$Comp
L power-supply:+5V #PWR0143
U 1 1 5F659644
P 7400 4350
AR Path="/5F5F9720/5F659644" Ref="#PWR0143"  Part="1" 
AR Path="/5F5F9C85/5F659644" Ref="#PWR?"  Part="1" 
F 0 "#PWR0143" H 7400 4200 50  0001 C CNN
F 1 "+5V" H 7415 4523 50  0000 C CNN
F 2 "" H 7400 4350 50  0000 C CNN
F 3 "" H 7400 4350 50  0000 C CNN
	1    7400 4350
	1    0    0    -1  
$EndComp
$Comp
L power-supply:+3V3 #PWR0144
U 1 1 5F65BD8A
P 4550 6700
AR Path="/5F5F9720/5F65BD8A" Ref="#PWR0144"  Part="1" 
AR Path="/5F5F9C85/5F65BD8A" Ref="#PWR?"  Part="1" 
F 0 "#PWR0144" H 4550 6550 50  0001 C CNN
F 1 "+3V3" H 4565 6873 50  0000 C CNN
F 2 "" H 4550 6700 50  0000 C CNN
F 3 "" H 4550 6700 50  0000 C CNN
	1    4550 6700
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0145
U 1 1 5F65E1BB
P 4550 7150
AR Path="/5F5F9720/5F65E1BB" Ref="#PWR0145"  Part="1" 
AR Path="/5F5F9C85/5F65E1BB" Ref="#PWR?"  Part="1" 
F 0 "#PWR0145" H 4550 6900 50  0001 C CNN
F 1 "GND" H 4555 6977 50  0000 C CNN
F 2 "" H 4550 7150 50  0000 C CNN
F 3 "" H 4550 7150 50  0000 C CNN
	1    4550 7150
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0146
U 1 1 5F660574
P 7400 4850
AR Path="/5F5F9720/5F660574" Ref="#PWR0146"  Part="1" 
AR Path="/5F5F9C85/5F660574" Ref="#PWR?"  Part="1" 
F 0 "#PWR0146" H 7400 4600 50  0001 C CNN
F 1 "GND" H 7405 4677 50  0000 C CNN
F 2 "" H 7400 4850 50  0000 C CNN
F 3 "" H 7400 4850 50  0000 C CNN
	1    7400 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4350 7400 4500
Wire Wire Line
	7400 4700 7400 4850
Wire Wire Line
	4550 6850 4550 6700
Wire Wire Line
	4550 7050 4550 7150
$Comp
L devices:Led_Small D2
U 1 1 5FE922A4
P 8750 4700
F 0 "D2" V 8841 4632 50  0000 R CNN
F 1 "Led_Small" V 8750 4632 50  0000 R CNN
F 2 "LEDs:LED_0603" V 8659 4632 50  0000 R CNN
F 3 "" V 8750 4700 50  0000 C CNN
	1    8750 4700
	0    -1   -1   0   
$EndComp
$Comp
L devices:R_0603 R23
U 1 1 5FE9396D
P 8750 4450
F 0 "R23" H 8809 4496 50  0000 L CNN
F 1 "220" H 8809 4405 50  0000 L CNN
F 2 "resistors:R_0603" H 8750 4300 50  0001 C CNN
F 3 "" H 8750 4450 50  0000 C CNN
	1    8750 4450
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0148
U 1 1 5FE99D2A
P 8750 4900
F 0 "#PWR0148" H 8750 4650 50  0001 C CNN
F 1 "GND" H 8755 4727 50  0000 C CNN
F 2 "" H 8750 4900 50  0000 C CNN
F 3 "" H 8750 4900 50  0000 C CNN
	1    8750 4900
	1    0    0    -1  
$EndComp
$Comp
L power-supply:+5V #PWR0149
U 1 1 5FE9A1B0
P 8750 4250
F 0 "#PWR0149" H 8750 4100 50  0001 C CNN
F 1 "+5V" H 8765 4423 50  0000 C CNN
F 2 "" H 8750 4250 50  0000 C CNN
F 3 "" H 8750 4250 50  0000 C CNN
	1    8750 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 4350 8750 4250
Wire Wire Line
	8750 4600 8750 4550
Wire Wire Line
	8750 4800 8750 4900
$Comp
L devices:Led_Small D1
U 1 1 5FEA57C7
P 5850 7050
F 0 "D1" V 5941 6982 50  0000 R CNN
F 1 "Led_Small" V 5850 6982 50  0000 R CNN
F 2 "LEDs:LED_0603" V 5759 6982 50  0000 R CNN
F 3 "" V 5850 7050 50  0000 C CNN
	1    5850 7050
	0    -1   -1   0   
$EndComp
$Comp
L devices:R_0603 R20
U 1 1 5FEA57CD
P 5850 6800
F 0 "R20" H 5909 6846 50  0000 L CNN
F 1 "220" H 5909 6755 50  0000 L CNN
F 2 "resistors:R_0603" H 5850 6650 50  0001 C CNN
F 3 "" H 5850 6800 50  0000 C CNN
	1    5850 6800
	1    0    0    -1  
$EndComp
$Comp
L power-supply:GND #PWR0150
U 1 1 5FEA57D3
P 5850 7250
F 0 "#PWR0150" H 5850 7000 50  0001 C CNN
F 1 "GND" H 5855 7077 50  0000 C CNN
F 2 "" H 5850 7250 50  0000 C CNN
F 3 "" H 5850 7250 50  0000 C CNN
	1    5850 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 6700 5850 6600
Wire Wire Line
	5850 6950 5850 6900
Wire Wire Line
	5850 7150 5850 7250
$Comp
L power-supply:+3V3 #PWR0151
U 1 1 5FEA841C
P 5850 6600
AR Path="/5F5F9720/5FEA841C" Ref="#PWR0151"  Part="1" 
AR Path="/5F5F9C85/5FEA841C" Ref="#PWR?"  Part="1" 
F 0 "#PWR0151" H 5850 6450 50  0001 C CNN
F 1 "+3V3" H 5865 6773 50  0000 C CNN
F 2 "" H 5850 6600 50  0000 C CNN
F 3 "" H 5850 6600 50  0000 C CNN
	1    5850 6600
	1    0    0    -1  
$EndComp
Wire Notes Line
	11200 2700 500  2700
Wire Notes Line
	500  5800 11200 5800
Wire Wire Line
	1500 1850 1900 1850
Wire Wire Line
	1900 1750 1500 1750
$Comp
L mechanical-connectors:WR-TBL_691313710002 P5
U 1 1 5ED2F897
P 1300 1800
AR Path="/5F5F9720/5ED2F897" Ref="P5"  Part="1" 
AR Path="/5F5F9C85/5ED2F897" Ref="P?"  Part="1" 
F 0 "P5" H 1217 1525 50  0000 C CNN
F 1 "WR-TBL_691313710002" H 1217 1616 50  0000 C CNN
F 2 "mechanical-connectors:WR-TBL_691313710002" H 1200 1850 50  0001 C CNN
F 3 "" H 1300 1800 50  0000 C CNN
	1    1300 1800
	-1   0    0    1   
$EndComp
Wire Wire Line
	4400 1500 4400 1450
Connection ~ 4400 1500
$Comp
L devices:D_Small D4
U 1 1 5F669B2D
P 2700 1800
F 0 "D4" V 2654 1868 50  0000 L CNN
F 1 "D_Small" V 2745 1868 50  0000 L CNN
F 2 "diodes:DO-214AB" V 2791 1868 50  0001 L CNN
F 3 "" V 2700 1800 50  0000 C CNN
	1    2700 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	2700 1700 2700 1500
Connection ~ 2700 1500
Wire Wire Line
	2700 1500 3250 1500
Wire Wire Line
	2700 1900 2700 2150
Connection ~ 2700 2150
Wire Wire Line
	2700 2150 3250 2150
$EndSCHEMATC
