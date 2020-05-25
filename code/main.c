/***********************************************************************
File Name    : main.c
Project      : Self Balancing Robot
Author       : Sreejith S, Karthik
Date         : 10 Nov 2019
Platform     : Tiva Development Kit EK-TM4C123GXL
Descirption  : Course project for Embedded Systems Design - Amrita Robotics and Automation
***********************************************************************/


#define UART0 0
#define UART7 7
#define BAUDRATE 115200
#define CLK_SPEED_MHz 25
#define SYSDIV (200/CLK_SPEED_MHz)-1
#define SYSTICK_RELOAD_MS CLK_SPEED_MHz*1000
#define I2C_SPEED 100000
#define LPF_ALPHA 0.8f
#define SETPOINT 0.0f
#define KP 10.0f
#define KI 0.1f
#define KD 1.0f

#include <stdio.h>
#include "gpio.h"
#include "motor.h"
#include "pid.h"
#include "mpu6050.h"
#include "time.h"
#include "encoder.h"
#include "uart.h"
#include "helpers.h"

   SPid PID = { 0, 0, 0.3, -0.3, KI, KP, KD};


int main(void)
{
   volatile unsigned int position = 0x0;
   float m_speed = 0.0;
   char string[20];    // = "0000000000";
   volatile float pitch_angle_0 = 0;
   volatile float pitch_angle_1 = 0;
   
   pll_init(SYSDIV);
   systick_init(SYSTICK_RELOAD_MS);
   motor_init(CLK_SPEED_MHz*1000000, 500);
   
   uart_init(UART0, BAUDRATE, CLK_SPEED_MHz*1000000);       //init module 0
   uart_init(UART7, BAUDRATE, CLK_SPEED_MHz*1000000);       //init module 6
   
   qei_init();
   i2c_init(CLK_SPEED_MHz*1000000, I2C_SPEED);
   mpu_init();
   //test_motors(CLK_SPEED_MHz*1000000);
   
//   i2c_write_burst(MPU6050_I2C_ADDR, 0x8, 5);
    //i2c_write_burst(MPU6050_I2C_ADDR, data, 1);
    //i2c_read_burst(MPU6050_I2C_ADDR, MPU6050_O_WHO_AM_I, data, 2);

   motor_speed(m_speed);
   while(1)
   {
       //uart_readln_str(UART7, string);
       //pitch_angle_0 = (float)qei_get_pos()/QEILOAD;
       pitch_angle_0 = get_gyro_x();
       //pitch_angle_0 = str_to_float(string);
       //Low pass filter to smooth out the angle value
       //pitch_angle_1 = LPF_ALPHA*pitch_angle_1 + (1.0f - LPF_ALPHA)*pitch_angle_0;
       //m_speed = UpdatePID(&PID, SETPOINT-pitch_angle_1, pitch_angle_1);
       m_speed = UpdatePID(&PID, SETPOINT-pitch_angle_0, pitch_angle_0);
       //uart_send_itoa(UART0, pitch_angle_0);
       motor_speed(-m_speed);
       uart_send(UART0, '.');
   }

}




