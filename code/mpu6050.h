/***********************************************************************
File Name            : mpu6050.h
Project              : Self Balancing Robot
Author               : Sreejith S, Karthik
Date                 : 06 Dec 2019
Platform             : Tiva Development Kit EK-TM4C123GXL
Descirption          : Code for MPU6050 IMU
***********************************************************************/


#ifndef MPU6050_H
#define MPU6050_H

#define MPU6050_I2C_ADDR 0x68

#include "hw_mpu6050.h"
#include <stdint.h>
#include "i2c.h"




void mpu_init(void);
int16_t get_gyro_x(void);
int16_t get_gyro_y(void);
int16_t get_gyro_z(void);
int16_t get_accel_x(void);
int16_t get_accel_y(void);
int16_t get_accel_z(void);



#endif
