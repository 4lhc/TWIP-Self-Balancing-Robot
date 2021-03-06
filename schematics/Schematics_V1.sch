EESchema Schematic File Version 4
LIBS:Schematics_V1-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Robot Control"
Date "2020-02-04"
Rev "1.4"
Comp "Amrita University - Robotics and Automation"
Comment1 "Authors: Sreejith S"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Sensor_Motion:MPU-6050 U?
U 1 1 5DC76829
P 1950 3100
F 0 "U?" H 1950 3100 50  0000 C CNN
F 1 "MPU-6050" H 2250 2550 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm" H 1950 2300 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 1950 2950 50  0001 C CNN
	1    1950 3100
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M?
U 1 1 5DC78305
P 10300 2150
F 0 "M?" H 10458 2146 50  0000 L CNN
F 1 "Motor_DC" H 10458 2055 50  0000 L CNN
F 2 "" H 10300 2060 50  0001 C CNN
F 3 "~" H 10300 2060 50  0001 C CNN
	1    10300 2150
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW?
U 1 1 5DC78CEA
P 1650 6600
F 0 "SW?" H 1650 6835 50  0000 C CNN
F 1 "SW_SPST" H 1650 6744 50  0000 C CNN
F 2 "" H 1650 6600 50  0001 C CNN
F 3 "~" H 1650 6600 50  0001 C CNN
	1    1650 6600
	1    0    0    -1  
$EndComp
$Comp
L Schematics_V1-rescue:TM4C123G-MCU_Texas U?
U 1 1 5DC7863C
P 5450 3400
F 0 "U?" H 5450 1211 50  0000 C CNN
F 1 "TM4C123G" H 5450 1120 50  0000 C CNN
F 2 "Package_QFP:LQFP-64_10x10mm_P0.5mm" H 6650 1300 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tm4c1231c3pm.pdf" H 5450 3600 50  0001 C CNN
	1    5450 3400
	1    0    0    -1  
$EndComp
$Comp
L Schematics_V1-rescue:TB6612FNG-Driver_Motor U?
U 1 1 5DC73E7C
P 8800 2850
F 0 "U?" H 8800 4031 50  0000 C CNN
F 1 "TB6612FNG" H 8800 3940 50  0000 C CNN
F 2 "Package_SO:SSOP-24_5.3x8.2mm_P0.65mm" H 9250 3450 50  0001 C CNN
F 3 "https://toshiba.semicon-storage.com/us/product/linear/motordriver/detail.TB6612FNG.html" H 9250 3450 50  0001 C CNN
	1    8800 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2450 9900 2450
Wire Wire Line
	9900 2450 9900 1800
Wire Wire Line
	9900 1800 10300 1800
Wire Wire Line
	9400 2650 10300 2650
$Comp
L Motor:Motor_DC M?
U 1 1 5DC7F23B
P 10350 3300
F 0 "M?" H 10508 3296 50  0000 L CNN
F 1 "Motor_DC" H 10508 3205 50  0000 L CNN
F 2 "" H 10350 3210 50  0001 C CNN
F 3 "~" H 10350 3210 50  0001 C CNN
	1    10350 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2950 10350 2950
Wire Wire Line
	9400 3150 9900 3150
Wire Wire Line
	9900 3150 9900 3800
Wire Wire Line
	9900 3800 10350 3800
Wire Wire Line
	10300 2650 10300 2450
Wire Wire Line
	10300 1800 10300 1950
Wire Wire Line
	10350 3100 10350 2950
Wire Wire Line
	10350 3600 10350 3800
$Comp
L power:GND #PWR?
U 1 1 5DC8440A
P 9300 3950
F 0 "#PWR?" H 9300 3700 50  0001 C CNN
F 1 "GND" H 9305 3777 50  0000 C CNN
F 2 "" H 9300 3950 50  0001 C CNN
F 3 "" H 9300 3950 50  0001 C CNN
	1    9300 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 3850 9100 3950
Wire Wire Line
	9100 3950 9300 3950
$Comp
L power:GND #PWR?
U 1 1 5DC85B05
P 8600 3950
F 0 "#PWR?" H 8600 3700 50  0001 C CNN
F 1 "GND" H 8605 3777 50  0000 C CNN
F 2 "" H 8600 3950 50  0001 C CNN
F 3 "" H 8600 3950 50  0001 C CNN
	1    8600 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3850 8500 3950
Wire Wire Line
	8500 3950 8600 3950
$Comp
L power:+3.3V #PWR?
U 1 1 5DC87561
P 8400 1800
F 0 "#PWR?" H 8400 1650 50  0001 C CNN
F 1 "+3.3V" H 8415 1973 50  0000 C CNN
F 2 "" H 8400 1800 50  0001 C CNN
F 3 "" H 8400 1800 50  0001 C CNN
	1    8400 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 1800 8400 1850
Wire Wire Line
	8400 1850 8500 1850
$Comp
L power:+12V #PWR?
U 1 1 5DC888A7
P 9200 1800
F 0 "#PWR?" H 9200 1650 50  0001 C CNN
F 1 "+12V" H 9215 1973 50  0000 C CNN
F 2 "" H 9200 1800 50  0001 C CNN
F 3 "" H 9200 1800 50  0001 C CNN
	1    9200 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 1800 9200 1850
Wire Wire Line
	9200 1850 9100 1850
$Comp
L Schematics_V1-rescue:KR8085-Sensor U?
U 1 1 5DC8CCF2
P 10350 4750
F 0 "U?" H 10628 4821 50  0000 L CNN
F 1 "KR8085" H 10628 4730 50  0000 L CNN
F 2 "" H 10350 4750 50  0001 C CNN
F 3 "" H 10350 4750 50  0001 C CNN
	1    10350 4750
	1    0    0    -1  
$EndComp
$Comp
L Schematics_V1-rescue:KR8085-Sensor U?
U 1 1 5DC8ECD0
P 10350 5700
F 0 "U?" H 10628 5771 50  0000 L CNN
F 1 "KR8085" H 10628 5680 50  0000 L CNN
F 2 "" H 10350 5700 50  0001 C CNN
F 3 "" H 10350 5700 50  0001 C CNN
	1    10350 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DC8F42E
P 9750 5200
F 0 "#PWR?" H 9750 4950 50  0001 C CNN
F 1 "GND" H 9755 5027 50  0000 C CNN
F 2 "" H 9750 5200 50  0001 C CNN
F 3 "" H 9750 5200 50  0001 C CNN
	1    9750 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 5200 9950 5200
Wire Wire Line
	9950 5200 9950 5550
$Comp
L power:GND #PWR?
U 1 1 5DC90FF2
P 9850 4300
F 0 "#PWR?" H 9850 4050 50  0001 C CNN
F 1 "GND" H 9855 4127 50  0000 C CNN
F 2 "" H 9850 4300 50  0001 C CNN
F 3 "" H 9850 4300 50  0001 C CNN
	1    9850 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 4300 9950 4300
Wire Wire Line
	9950 4300 9950 4600
$Comp
L power:+3.3V #PWR?
U 1 1 5DC91B85
P 9850 5000
F 0 "#PWR?" H 9850 4850 50  0001 C CNN
F 1 "+3.3V" V 9865 5128 50  0000 L CNN
F 2 "" H 9850 5000 50  0001 C CNN
F 3 "" H 9850 5000 50  0001 C CNN
	1    9850 5000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9900 4900 9950 4900
Wire Wire Line
	9900 5000 9850 5000
Wire Wire Line
	9900 4900 9900 5000
$Comp
L power:+3.3V #PWR?
U 1 1 5DC951A5
P 9950 6050
F 0 "#PWR?" H 9950 5900 50  0001 C CNN
F 1 "+3.3V" V 9965 6178 50  0000 L CNN
F 2 "" H 9950 6050 50  0001 C CNN
F 3 "" H 9950 6050 50  0001 C CNN
	1    9950 6050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9950 5850 9950 6050
Wire Wire Line
	7900 4900 7900 5650
Wire Wire Line
	7900 5650 9950 5650
Wire Wire Line
	9950 5750 7800 5750
Wire Wire Line
	7800 5750 7800 5000
$Comp
L Schematics_V1-rescue:MP1584-Regulator_Switching U?
U 1 1 5DCB46E1
P 2800 6700
F 0 "U?" H 2800 7067 50  0000 C CNN
F 1 "MP1584" H 2800 6976 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TSOT-23-6" H 2800 7050 50  0001 C CNN
F 3 "https://www.monolithicpower.com/pub/media/document/MP1470_r1.02.pdf" H 2800 6700 50  0001 C CNN
	1    2800 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 2100 1250 2800
Wire Wire Line
	1250 2100 4050 2100
Wire Wire Line
	1250 2900 1150 2900
Wire Wire Line
	1150 2900 1150 2000
Wire Wire Line
	1150 2000 4050 2000
Wire Wire Line
	2650 2800 2750 2800
Wire Wire Line
	2750 2800 2750 1900
Wire Wire Line
	2750 1900 4050 1900
$Comp
L power:+12V #PWR?
U 1 1 5DCC90A1
P 1350 6550
F 0 "#PWR?" H 1350 6400 50  0001 C CNN
F 1 "+12V" H 1365 6723 50  0000 C CNN
F 2 "" H 1350 6550 50  0001 C CNN
F 3 "" H 1350 6550 50  0001 C CNN
	1    1350 6550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DCC94A6
P 1350 7200
F 0 "#PWR?" H 1350 6950 50  0001 C CNN
F 1 "GND" H 1355 7027 50  0000 C CNN
F 2 "" H 1350 7200 50  0001 C CNN
F 3 "" H 1350 7200 50  0001 C CNN
	1    1350 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 6900 1350 7200
Wire Wire Line
	1350 6550 1350 6600
Wire Wire Line
	1350 6600 1450 6600
$Comp
L power:+3.3V #PWR?
U 1 1 5DCD39CA
P 3900 6100
F 0 "#PWR?" H 3900 5950 50  0001 C CNN
F 1 "+3.3V" H 3915 6273 50  0000 C CNN
F 2 "" H 3900 6100 50  0001 C CNN
F 3 "" H 3900 6100 50  0001 C CNN
	1    3900 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DCD6181
P 3900 7150
F 0 "#PWR?" H 3900 6900 50  0001 C CNN
F 1 "GND" H 3905 6977 50  0000 C CNN
F 2 "" H 3900 7150 50  0001 C CNN
F 3 "" H 3900 7150 50  0001 C CNN
	1    3900 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D?
U 1 1 5DCDE223
P 3900 6450
F 0 "D?" V 3939 6332 50  0000 R CNN
F 1 "GREEN" V 3848 6332 50  0000 R CNN
F 2 "" H 3900 6450 50  0001 C CNN
F 3 "~" H 3900 6450 50  0001 C CNN
	1    3900 6450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 5DCDF893
P 3900 6850
F 0 "R?" H 3970 6896 50  0000 L CNN
F 1 "330" H 3970 6805 50  0000 L CNN
F 2 "" V 3830 6850 50  0001 C CNN
F 3 "~" H 3900 6850 50  0001 C CNN
	1    3900 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 6600 3900 6700
Wire Wire Line
	3200 6600 3500 6600
Wire Wire Line
	3900 6100 3900 6300
Wire Wire Line
	3900 7000 3900 7150
Connection ~ 1950 6600
Wire Wire Line
	1950 6600 2400 6600
Wire Wire Line
	2400 6800 2350 6800
Wire Wire Line
	2350 6800 2350 6900
Wire Wire Line
	2350 6900 1950 6900
Connection ~ 1950 6900
Wire Wire Line
	1350 6900 1950 6900
Wire Wire Line
	1850 6600 1950 6600
$Comp
L Device:C C?
U 1 1 5DCF198D
P 1950 6750
F 0 "C?" H 2065 6796 50  0000 L CNN
F 1 "0.33uF" H 2065 6705 50  0000 L CNN
F 2 "" H 1988 6600 50  0001 C CNN
F 3 "~" H 1950 6750 50  0001 C CNN
	1    1950 6750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5DD08959
P 3500 6750
F 0 "C?" H 3615 6796 50  0000 L CNN
F 1 "0.1uF" H 3615 6705 50  0000 L CNN
F 2 "" H 3538 6600 50  0001 C CNN
F 3 "~" H 3500 6750 50  0001 C CNN
	1    3500 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 6600 3700 6600
Connection ~ 3500 6600
Wire Wire Line
	3700 6300 3700 6600
Wire Wire Line
	3700 6300 3900 6300
Wire Wire Line
	3200 6800 3200 6900
Wire Wire Line
	3200 6900 3500 6900
Wire Wire Line
	3500 7000 3500 6900
Wire Wire Line
	3500 7000 3900 7000
Connection ~ 3500 6900
Connection ~ 3900 6300
Connection ~ 3900 7000
$Comp
L power:+3.3V #PWR?
U 1 1 5DD24D16
P 2050 2250
F 0 "#PWR?" H 2050 2100 50  0001 C CNN
F 1 "+3.3V" H 2065 2423 50  0000 C CNN
F 2 "" H 2050 2250 50  0001 C CNN
F 3 "" H 2050 2250 50  0001 C CNN
	1    2050 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 2250 2050 2400
$Comp
L power:GND #PWR?
U 1 1 5DD28229
P 1950 3900
F 0 "#PWR?" H 1950 3650 50  0001 C CNN
F 1 "GND" H 1955 3727 50  0000 C CNN
F 2 "" H 1950 3900 50  0001 C CNN
F 3 "" H 1950 3900 50  0001 C CNN
	1    1950 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3900 1950 3800
Wire Wire Line
	6850 4900 7900 4900
Wire Wire Line
	7800 5000 6850 5000
Wire Wire Line
	6850 3900 8000 3900
Wire Wire Line
	8000 3900 8000 4700
Wire Wire Line
	7900 4800 7900 4000
Wire Wire Line
	7900 4000 6850 4000
Text Notes 10450 5250 0    50   ~ 0
Quadrature Hall \nEffect Sensors
Text Notes 1500 4350 0    50   ~ 0
Inertial Measurement Unit\nGY-521 Breakout
Text Notes 8450 1450 0    50   ~ 0
Dual DC Motor \nDriver Breakout
Text Notes 2450 7250 0    50   ~ 0
DC-DC Buck Converter
Wire Wire Line
	7900 4800 9950 4800
Wire Wire Line
	8000 4700 9950 4700
$Comp
L power:+3.3V #PWR?
U 1 1 5DD12D8C
P 8100 2200
F 0 "#PWR?" H 8100 2050 50  0001 C CNN
F 1 "+3.3V" H 8115 2373 50  0000 C CNN
F 2 "" H 8100 2200 50  0001 C CNN
F 3 "" H 8100 2200 50  0001 C CNN
	1    8100 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 2200 8100 2450
Wire Wire Line
	8100 2450 8200 2450
Wire Wire Line
	6850 2500 7600 2500
Wire Wire Line
	7600 2500 7600 2650
Wire Wire Line
	7600 2650 8200 2650
Wire Wire Line
	6850 2600 7500 2600
Wire Wire Line
	7500 2600 7500 2750
Wire Wire Line
	7500 2750 8200 2750
Wire Wire Line
	8200 2850 7400 2850
Wire Wire Line
	7400 2850 7400 2700
Wire Wire Line
	7400 2700 6850 2700
Wire Wire Line
	6850 2800 7300 2800
Wire Wire Line
	7300 2800 7300 2950
Wire Wire Line
	7300 2950 8200 2950
Wire Wire Line
	6850 2900 7200 2900
Wire Wire Line
	7200 2900 7200 3200
Wire Wire Line
	7200 3200 8200 3200
Wire Wire Line
	6850 3100 7100 3100
Wire Wire Line
	7100 3100 7100 3300
Wire Wire Line
	7100 3300 8200 3300
$Comp
L Device:Voltmeter_DC MES?
U 1 1 5DE9A68A
P 4700 6750
F 0 "MES?" H 4853 6796 50  0000 L CNN
F 1 "Voltmeter_DC" H 4853 6705 50  0000 L CNN
F 2 "" V 4700 6850 50  0001 C CNN
F 3 "~" V 4700 6850 50  0001 C CNN
	1    4700 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 6300 4700 6300
Wire Wire Line
	4700 6300 4700 6550
Wire Wire Line
	4700 6950 4700 7000
Wire Wire Line
	4700 7000 3900 7000
$Comp
L Schematics_V1-rescue:CP2102_USB_TTL-Interface_CAN_LIN U?
U 1 1 5DED52F0
P 2150 1250
F 0 "U?" H 2192 685 50  0000 C CNN
F 1 "CP2102_USB_TTL" H 2192 776 50  0000 C CNN
F 2 "" H 2050 1350 50  0001 C CNN
F 3 "" H 2050 1350 50  0001 C CNN
	1    2150 1250
	-1   0    0    1   
$EndComp
Wire Wire Line
	2850 1100 3750 1100
Wire Wire Line
	3750 1100 3750 1700
Wire Wire Line
	3750 1700 4050 1700
Wire Wire Line
	2850 1200 3850 1200
Wire Wire Line
	3850 1200 3850 1600
Wire Wire Line
	3850 1600 4050 1600
$Comp
L power:GND #PWR?
U 1 1 5DEE6568
P 3350 1500
F 0 "#PWR?" H 3350 1250 50  0001 C CNN
F 1 "GND" H 3355 1327 50  0000 C CNN
F 2 "" H 3350 1500 50  0001 C CNN
F 3 "" H 3350 1500 50  0001 C CNN
	1    3350 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 1500 3350 1300
Wire Wire Line
	2850 1300 3350 1300
$Comp
L power:+3.3V #PWR?
U 1 1 5DEEE593
P 3100 1650
F 0 "#PWR?" H 3100 1500 50  0001 C CNN
F 1 "+3.3V" H 3115 1823 50  0000 C CNN
F 2 "" H 3100 1650 50  0001 C CNN
F 3 "" H 3100 1650 50  0001 C CNN
	1    3100 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1650 2950 1650
Wire Wire Line
	2950 1650 2950 1400
Wire Wire Line
	2950 1400 2850 1400
$EndSCHEMATC
