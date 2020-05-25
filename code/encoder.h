/***********************************************************************
File Name          : encoder.h
Project            : Self Balancing Robot
Author             : Sreejith S, Karthik
Date               : 30 Nov 2019
Platform           : Tiva Development Kit EK-TM4C123GXL
Descirption        : Code for Qudrature HallEffect Sensor
***********************************************************************/



#ifndef QEI_H
#define QEI_H

/*
PPR: 495


MotorA C1 -> PC5/PhA1
MotorA C2 -> PC6/PhB1

MotorB C1 -> PD6/PhA0
MotorB C2 -> PD7/PhB0
*/


#define QEILOAD 1979

#include <stdint.h>
#include "gpio.h"
#include "tm4c123gh6pm.h"

void qei_init(void);
int32_t qei_get_pos(void);
#endif

