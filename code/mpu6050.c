/***********************************************************************
File Name            : mpu6050.c
Project              : Self Balancing Robot
Author               : Sreejith S, Karthik
Date                 : 06 Dec 2019
Platform             : Tiva Development Kit EK-TM4C123GXL
Descirption          : Code for MPU6050 IMU
***********************************************************************/

#include "mpu6050.h"


void mpu_init(void)
{

    //Clock Source - PLL with X-Axis Gyro
    i2c_write_2(MPU6050_I2C_ADDR, MPU6050_O_PWR_MGMT_1, 0x00); 
    //Set Gyro FSR +/- 2000 deg/s - (Incresing the FSR reduce sensitivity? -- Verify)
    i2c_write_2(MPU6050_I2C_ADDR, MPU6050_O_GYRO_CONFIG, MPU6050_GYRO_CONFIG_FS_SEL_2000);
    //Set Accel FSR - +/- 2G
    i2c_write_2(MPU6050_I2C_ADDR, MPU6050_O_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_AFS_SEL_2G);
    //Set Sleep mode status
    //i2c_write_2(MPU6050_I2C_ADDR, MPU6050_O_PWR_MGMT_1, 0x00); 
    return;
}

int16_t get_gyro_x(void)
{
    uint8_t data[2];
    i2c_read_burst(MPU6050_I2C_ADDR, MPU6050_O_GYRO_XOUT_H, data, 2);
    return (int16_t) (data[0]<<8 | data[1]);
}

int16_t get_gyro_y(void)
{
    uint8_t data[2];
    i2c_read_burst(MPU6050_I2C_ADDR, MPU6050_O_GYRO_YOUT_H, data, 2);
    return (int16_t) (data[0]<<8 | data[1]);
}

int16_t get_gyro_z(void)
{
    uint8_t data[2];
    i2c_read_burst(MPU6050_I2C_ADDR, MPU6050_O_GYRO_ZOUT_H, data, 2);
    return (int16_t) (data[0]<<8 | data[1]);
}

int16_t get_accel_x(void)
{
    uint8_t data[2];
    i2c_read_burst(MPU6050_I2C_ADDR, MPU6050_O_ACCEL_XOUT_H, data, 2);
    return (int16_t) (data[0]<<8 | data[1]);
}

int16_t get_accel_y(void)
{
    uint8_t data[2];
    i2c_read_burst(MPU6050_I2C_ADDR, MPU6050_O_ACCEL_YOUT_H, data, 2);
    return (int16_t) (data[0]<<8 | data[1]);
}

int16_t get_accel_z(void)
{
    uint8_t data[2];
    i2c_read_burst(MPU6050_I2C_ADDR, MPU6050_O_ACCEL_ZOUT_H, data, 2);
    return (int16_t) (data[0]<<8 | data[1]);
}

