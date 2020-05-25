/***********************************************************************
File Name            : gpio.h
Project                : Self Balancing Robot
Author                : Sreejith S, Karthik
Date                    : 10 Nov 2019
Platform            : Tiva Development Kit EK-TM4C123GXL
Descirption        : Code for gpio functionalities
***********************************************************************/
#ifndef GPIO_H
#define GPIO_H


#include <stdint.h>
#include "tm4c123gh6pm.h"    




extern volatile uint32_t* SYS_CTL;
extern volatile uint32_t* CORE_P;

extern uint8_t CLK_MOSC;
extern uint8_t CLK_PIOSC;
extern uint8_t CLK_PLL_ON;
extern uint8_t CLK_PLL_OFF;

extern volatile uint32_t* GPIO_A;
extern volatile uint32_t* GPIO_B;
extern volatile uint32_t* GPIO_C;
extern volatile uint32_t* GPIO_D;
extern volatile uint32_t* GPIO_E;
extern volatile uint32_t* GPIO_F;

extern uint8_t GPIO_OUT;
extern uint8_t GPIO_IN;
extern uint8_t GPIO_TRI;
extern uint8_t GPIO_PUR;
extern uint8_t GPIO_PDR;
extern uint8_t GPIO_ODR;
extern uint8_t GPIO_DEN;
extern uint8_t GPIO_NONE;


void gpio_init(volatile uint32_t* port, uint8_t pins, uint8_t dirs);
void gpio_clock(volatile uint32_t* port);
void gpio_enable_afsel(volatile uint32_t* port, uint8_t pins); 
void gpio_unlock(volatile uint32_t* port, uint8_t pins);
void gpio_pur(volatile uint32_t* port, uint8_t pins);
void gpio_pdr(volatile uint32_t* port, uint8_t pins);
void gpio_den(volatile uint32_t* port, uint8_t pins);
void gpio_write(volatile uint32_t* port, uint8_t pin, uint8_t value);
uint8_t gpio_read(volatile uint32_t* port, uint8_t pin);

#endif

