/***********************************************************************
File Name          : time.h
Project            : Self Balancing Robot
Author             : Sreejith S, Karthik
Date               : 30 Nov 2019
Platform           : Tiva Development Kit EK-TM4C123GXL
Descirption        : Code for clock related functions
***********************************************************************/


#ifndef TIME_H
#define TIME_H

#include <stdint.h>
#include "gpio.h"
#include "tm4c123gh6pm.h"

void pll_init(uint8_t sysdiv);
void systick_init(uint32_t reload);
void delay_ms(uint32_t delay);
#endif

