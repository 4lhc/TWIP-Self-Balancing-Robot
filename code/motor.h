/***********************************************************************
File Name            : motor.h
Project                : Self Balancing Robot
Author                : Sreejith S, Karthik
Date                : 10 Nov 2019
Platform            : Tiva Development Kit EK-TM4C123GXL
Descirption            : Code for motor control
***********************************************************************/
/*
Pin Assignment:
---------------

PB0 -> AIN1
PB1 -> AIN2
PB2 -> BIN1
PB3 -> BIN2
PB4/M0PWM2 -> PWMA [Using PWM0 module]
PB6/M0PWM0 -> PWMB

Control:
-----------|-----------------
IN1 IN2 PWM|    Mode        |
-----------|-----------------
H   H   H/L|    Short Brake |
L   H   H  |    CCW         |
H   L   H  |    CW          |       
L   L   H  |    Stop        |
-----------------------------

*/

  
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "gpio.h"
#include "tm4c123gh6pm.h"  
#include "uart.h"


#define MOTOR_PINS 0x0F
#define PWM_PINS 0x50
#define AIN1 0
#define AIN2 1
#define BIN1 2
#define BIN2 3
#define PWMA 4 
#define PWMB 6

extern uint32_t* PWM_BASE;
extern uint32_t* PWM0;
extern uint32_t* PWM1;

void motor_init(uint32_t clk_speed, uint16_t freq);
void motor_speed(float speed);
void motor_stop(void);
void motor_freq(uint32_t clk_speed, uint16_t freq);
void pwm_init(uint32_t clk_speed, uint16_t freq);
void pwm_set_freq(uint32_t* pwm, uint32_t clk_speed, uint16_t freq);
void pwm_set_duty(uint32_t* pwm, float duty);
void test_motors(uint32_t clk_speed);

#endif
