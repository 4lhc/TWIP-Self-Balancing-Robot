/***********************************************************************
File Name          : time.c
Project            : Self Balancing Robot
Author             : Sreejith S, Karthik
Date               : 30 Nov 2019
Platform           : Tiva Development Kit EK-TM4C123GXL
Descirption        : Code for clock related functions
***********************************************************************/

#include "time.h"


void pll_init(uint8_t sysdiv)
    // Initialize PLL to generate 25MHz system clock from 16MHz crystal
{
    SYSCTL_RCC_R |= (0x01<<11);                     //Set BYPASS bit
    SYSCTL_RCC_R &= ~(0x01<<22);                    //Clear USESSYDIV
    
    SYSCTL_RCC_R &= ~(0x1F<<6);                     //Clear XTAL bits 10:6
    SYSCTL_RCC_R |= (0x15<<6);                      //Set XTAL to 16Mhz 
    
    SYSCTL_RCC_R &= ~(0x03<<4);                     //Clear OSCSRC to select external crystal
    SYSCTL_RCC_R &= ~(0x01<<13);                    //Clear PWRDWN bit to activate PLL

    SYSCTL_RCC_R &= ~(0x08<<23);                    //Clear SYSDIV bits 26:23
    SYSCTL_RCC_R |= ((sysdiv)<<23);                 //Set SYSDIV
    
    SYSCTL_RCC_R |= (0x01<<22);                     //Set USESSYDIV

    while(!(SYSCTL_RIS_R & 0x40)){}                 //Wait for PLL to stabilize - PLLLRIS 
    SYSCTL_RCC_R &= ~(0x01<<11);                    //Clear BYPASS bit
    
}

void systick_init(uint32_t reload)
{

    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_RELOAD_R = reload-1;
}

void delay_ms(uint32_t delay)
    //delay - in ms
{
    while (delay)
    {
            NVIC_ST_CURRENT_R = 0;                          //Clear the current register
            NVIC_ST_CTRL_R |= 0x05;                         //Start timer with no interrupt
    
            while(!(NVIC_ST_CTRL_R & 0x10000));            //check if timer reached 0
            NVIC_ST_CTRL_R = 0;
            delay--;
    }

}
