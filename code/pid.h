/***********************************************************************
File Name            : pid.h
Project              : Self Balancing Robot
Author               : Sreejith S, Karthik
Date                 : 08 Dec 2019
Platform             : Tiva Development Kit EK-TM4C123GXL
Descirption          : Code for pid controller
***********************************************************************/
//Ref: [Implementing PID controller in software](https://www.youtube.com/watch?v=JVqJ7uRGwoA)

#ifndef PID_H
#define PID_H

#define DEADZONE 

#include <stdint.h>

typedef struct
{
        float derState;         // Last position input
        float integratState;    // Integrator state
        float integratMax,      // Maximum and minimum
              integratMin;      // allowable integrator state
        float integratGain,     // integral gain
              propGain,         // proportional gain
              derGain;          // derivative gain
} SPid;


float UpdatePID(SPid * pid, float error, float position);

#endif
