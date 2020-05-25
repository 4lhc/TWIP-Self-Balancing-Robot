/***********************************************************************
File Name            : gpio.c
Project                : Self Balancing Robot
Author                : Sreejith S, Karthik
Date                    : 10 Nov 2019
Platform            : Tiva Development Kit EK-TM4C123GXL
Descirption        : Code for gpio functionalities
***********************************************************************/

#include "gpio.h"

volatile uint32_t* SYS_CTL = (uint32_t*) 0x400FE000;     // pg232: refer System Control Register Map for offsets
volatile uint32_t* CORE_P  = (uint32_t*) 0xE000E000;     // pg134: refer Peripherals Register Map for offsets

uint8_t CLK_MOSC    = 0x00;
uint8_t CLK_PIOSC    = 0x01;
uint8_t CLK_PLL_ON    = 0x01;
uint8_t CLK_PLL_OFF    = 0x00;

volatile uint32_t* GPIO_A = (uint32_t*) 0x40004000;
volatile uint32_t* GPIO_B = (uint32_t*) 0x40005000;
volatile uint32_t* GPIO_C = (uint32_t*) 0x40006000;
volatile uint32_t* GPIO_D = (uint32_t*) 0x40007000;
volatile uint32_t* GPIO_E = (uint32_t*) 0x40024000;
volatile uint32_t* GPIO_F = (uint32_t*) 0x40025000;

uint8_t GPIO_OUT = 0xFF;
uint8_t GPIO_IN  = 0x00;
uint8_t GPIO_TRI = 0x00;
uint8_t GPIO_PUR = 0x01;
uint8_t GPIO_PDR = 0x02;
uint8_t GPIO_ODR = 0x04;
uint8_t GPIO_DEN = 0x10;
uint8_t GPIO_NONE= 0x20;

void gpio_clock(volatile uint32_t* port)
{
            uint8_t mask;
            if (port == GPIO_A) mask = 0x01;
            else if (port == GPIO_B) mask = 0x02;
            else if (port == GPIO_C) mask = 0x04;
            else if (port == GPIO_D) mask = 0x08;
            else if (port == GPIO_E) mask = 0x10;
            else if (port == GPIO_F) mask = 0x20;
            SYS_CTL[0x608/4] |= mask;                      //RCGCGPIO (RCGC2) 
        
}

void gpio_init(volatile uint32_t* port, uint8_t pins, uint8_t dir)
{
        int8_t delay;
        gpio_clock(port);                      // Enable clock
        gpio_unlock(port, pins);               // Protection provided for the GPIO pins that can be used as the four JTAG/SWD pins and the NMI pin
                                               // PD7 need unlocking
//        while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};  //Wait till PortE peripheral ready
         delay = 100;                          // Two NOP delay to stabilize clock (Mixed ASM C Programming)
        
        port[0x400/4] |= (pins & dir);         // Set direction: GPIODIR Register offset 0x400
                                               // Set bit for output: All pins are cleared on reset ie; input by default 
        port[0x420/4] &= ~((uint32_t) pins);   // GPIOAFSEL disable alternate functions: Clear for GPIO: Pg671 GPIOPCTL to select the alt function
        
        gpio_den(port, pins);
}

void gpio_enable_afsel(volatile uint32_t* port, uint8_t pins)
{
        port[0x420/4] |= (uint32_t) pins;   // GPIOAFSEL enable alternate functions: Pg671 GPIOPCTL to select the alt function
}

void gpio_unlock(volatile uint32_t* port, uint8_t pins) {
                                               // This step is only needed for pins PC0-3, PD7 and PF0 on TM4C123GXL LaunchPad.
        port[0x520/4] = 0x4C4F434B;            // GPIOLOCK), offset 0x520 - enables write access to GPIOCR
        port[0x524/4] |= pins;                 // unlock pins on that port
}

void gpio_pur(volatile uint32_t* port, uint8_t pins) {
        port[0x510/4] |= pins;                 // Set PUR
}    

void gpio_pdr(volatile uint32_t* port, uint8_t pins) {
        port[0x514/4] |= pins;                 // Set PDR
}


void gpio_den(volatile uint32_t* port, uint8_t pins) {
        port[0x51C/4] |= pins;                              // Set DEN
}

void gpio_write(volatile uint32_t* port, uint8_t pin, uint8_t value) {
        port[(1 << pin << 2)/4] = value << pin;
}


uint8_t gpio_read(volatile uint32_t* port, uint8_t pin) {
        return (port[0x3FC/4] >> pin) & 1;
}
