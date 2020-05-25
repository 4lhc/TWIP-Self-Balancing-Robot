/***********************************************************************
File Name            : motor.h
Project              : Self Balancing Robot
Author               : Sreejith S, Karthik
Date                 : 17 Nov 2019
Platform            : Tiva Development Kit EK-TM4C123GXL
Descirption            : Code for motor control
***********************************************************************/

#include "motor.h"

uint32_t* PWM_BASE = (uint32_t*) 0x40028000;
uint32_t* PWM0       = (uint32_t*) 0x40028040;
uint32_t* PWM1        = (uint32_t*) 0x40028080;

void motor_init(uint32_t clk_speed, uint16_t freq)
{
    gpio_init(GPIO_B, MOTOR_PINS, GPIO_OUT);
    pwm_init(clk_speed, freq);
    gpio_write(GPIO_B, AIN1, 1);
    gpio_write(GPIO_B, AIN2, 1);
    gpio_write(GPIO_B, BIN1, 1);
    gpio_write(GPIO_B, BIN2, 1);

}

void motor_speed(float speed)
{
    if (speed > 1 || speed < -1) return;
    if (speed == 0)
    {
        //Short Brake
        gpio_write(GPIO_B, AIN1, 1);
        gpio_write(GPIO_B, AIN2, 1);
        gpio_write(GPIO_B, BIN1, 1);
        gpio_write(GPIO_B, BIN2, 1);
    }
    else if (speed > 0)
    {
        //CCW
        gpio_write(GPIO_B, AIN1, 0);
        gpio_write(GPIO_B, AIN2, 1);
        gpio_write(GPIO_B, BIN1, 0);
        gpio_write(GPIO_B, BIN2, 1);

    }
    else if (speed < 0)
    {
        //CW
        gpio_write(GPIO_B, AIN1, 1);
        gpio_write(GPIO_B, AIN2, 0);
        gpio_write(GPIO_B, BIN1, 1);
        gpio_write(GPIO_B, BIN2, 0);
        speed *= -1;

    }
    pwm_set_duty(PWM0, speed);
    pwm_set_duty(PWM1, speed);
}

void motor_stop(void)
{
    //Board is drawing power through GPIO
    //So turning off motor GPIOS
   gpio_write(GPIO_B, AIN1, 0);
   gpio_write(GPIO_B, AIN2, 0);
   gpio_write(GPIO_B, BIN1, 0);
   gpio_write(GPIO_B, BIN2, 0);

}

void motor_freq(uint32_t clk_speed, uint16_t freq) {
    pwm_set_freq(PWM0, clk_speed, freq);
}


void pwm_init(uint32_t clk_speed, uint16_t freq)
{
    SYSCTL_RCGC0_R = 0x00100000;                                            //Enable pwm clock
    gpio_init(GPIO_B, PWM_PINS, GPIO_OUT);                                  //Enable Clock for portB, DEN
    gpio_enable_afsel(GPIO_B, PWM_PINS);                                    //Enable alternate function
    GPIO_PORTB_PCTL_R  = (GPIO_PORTB_PCTL_R  & 0xF0F0FFFF) |0x04040000;     //PORT mux control for PB6 & PB4 - Select PWM
    SYSCTL_RCC_R = (SYSCTL_RCC_R & 0xFFE1FFFF)|0x00120000;                  //Set USEPWMDIV & Set PMWDIV = 4

    PWM0_0_CTL_R = 0x00000000;                                              //Disable PWM
    PWM0_1_CTL_R = 0x00000000;


    PWM0_0_GENA_R |= 0x000000C2;                                            //ACTCMPAD = Drive PWMA HIGH (0x3); ACTZERO = Drive PWMA LOW (0x2)
    PWM0_0_GENB_R = 0x00000000;                                             //Disable PWMB output
    PWM0_1_GENA_R |= 0x000000C2;
    PWM0_1_GENB_R = 0x00000000;

    PWM0_0_LOAD_R = (clk_speed >> 2) / freq - 1;                           // set frequency
    PWM0_1_LOAD_R = (clk_speed >> 2) / freq - 1;

    PWM0_0_CMPA_R  = 0;                                                    //duty cycle 0%
    PWM0_1_CMPA_R  = 0;

    PWM0_0_CTL_R = 0x00000001;                                             //The PWM generation block is enabled and produces PWM signals
    PWM0_1_CTL_R = 0x00000001;

    PWM0_ENABLE_R  = 0x05;                                                 //Set PWM0EN & PWM2EN to enable PWM output to pins PB6 & PB4

}

void pwm_set_freq(uint32_t* pwm, uint32_t clk_speed, uint16_t freq)
{
    //offsets
    //PWMnLOAD - 0x010
    //PWMnCMPA - 0x018

    float duty = pwm[0x018/4] / (float) pwm[0x010/4];    // duty cycle
    pwm[0x010/4] = (clk_speed >> 2) / freq - 1;            // new frequency
    pwm[0x018/4] = pwm[0x010/4] * duty;                    // new cmpA value, preserving duty cycle

}

void pwm_set_duty(uint32_t* pwm, float duty)
{
    if (duty == 1.0f || duty == -1.0f)
    {
        pwm[0x018/4] = pwm[0x010/4] - 1;
    }
    else
    {
        pwm[0x018/4] = pwm[0x010/4] * duty - 1;
    }

}

void msleep(uint32_t milliseconds) {
    uint32_t max, ms;
    for (ms = milliseconds; ms > 0; --ms) {
        for (max=4000; max > 0; --max);    // this loops for 1 ms
                                        // calculated from the disassembly
    }
}

void test_motors(uint32_t clk_speed)
{
    float speed = 0;
    float speed_inc = 0.1;
    uint16_t freq = 200;

    int16_t freq_inc = 500;

    motor_init(clk_speed, freq);
    motor_speed(0.5);

    for(speed=-0.99; speed<=1; speed += speed_inc)
    {
        motor_speed(speed);
        msleep(500);
    }
    motor_speed(0.02);

    for(freq=-0; freq<=4000; freq += freq_inc)
    {
        motor_freq(clk_speed, freq);
        msleep(500);
    }
    motor_speed(0);
     motor_stop();

}
