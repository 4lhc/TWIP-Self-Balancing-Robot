/***********************************************************************
File Name           : uart.h
Project             : Self Balancing Robot
Author              : Sreejith S, Karthik
Date                : 29 Nov 2019
Platform            : Tiva Development Kit EK-TM4C123GXL
Descirption         : Uart communication
***********************************************************************/

#ifndef UART_H
#define UART_H



/*
PA0/U0Rx
PA1/U0Tx
*/


#include <stdint.h>
#include "gpio.h"
#include "tm4c123gh6pm.h"


void uart_init(uint8_t module, uint32_t baud, uint32_t sys_clock);
uint8_t uart_receive(uint8_t module);
void uart_send(uint8_t module, uint8_t data);
void uart_send_string(uint8_t module, char* string);
void uart_send_itoa(uint8_t module, int num);
void uart_readln_str(uint8_t module, char* string);


void test_uart(uint8_t module);



#endif
