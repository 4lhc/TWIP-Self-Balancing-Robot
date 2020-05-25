/***********************************************************************
File Name            : i2c.h
Project              : Self Balancing Robot
Author               : Sreejith S, Karthik
Date                 : 06 Dec 2019
Platform             : Tiva Development Kit EK-TM4C123GXL
Descirption          : Code for I2C communication
***********************************************************************/


#ifndef I2C_H
#define I2C_H
#include <stdint.h>
#include "gpio.h"
#include "tm4c123gh6pm.h"


#define I2C_PORT GPIO_E
#define I2C_CLK_MASK 0x04



void i2c_init(uint32_t sys_clock, uint32_t speed);
void i2c_read_single(uint8_t slave_addr, uint8_t* data);
void i2c_write_single(uint8_t slave_addr, uint8_t data);
void i2c_read_burst(uint8_t slave_addr, uint8_t mpu_reg_addr, uint8_t* data, uint8_t size);
void i2c_write_burst(uint8_t slave_addr, uint8_t* data, uint8_t size);
uint8_t i2c_is_busy(void);
uint8_t i2c_is_busbusy(void);
void i2c_write_2(uint8_t slave_addr, uint8_t data1, uint8_t data2);



#endif
