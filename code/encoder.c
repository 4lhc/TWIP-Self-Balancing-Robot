/***********************************************************************
File Name          : encoder.c
Project            : Self Balancing Robot
Author             : Sreejith S, Karthik
Date               : 30 Nov 2019
Platform           : Tiva Development Kit EK-TM4C123GXL
Descirption        : Code for Qudrature HallEffect Sensor
***********************************************************************/

#include "encoder.h"

void qei_init(void)
{
    //Enable the QEI clock using the RCGCQEI register in the System Control module (see page 355).
    //Using QEI Module 0 & Module1
    SYSCTL_RCGCQEI_R |= 0x02;                
                                             
    //Enable the clock to the appropriate GPIO module via the RCGCGPIO (see page 340).
    //PORT C & D
    SYSCTL_RCGCGPIO_R |= 0x0C;                
                                             
    //AFSEL PC5, PC6, PD7 & PD6
    GPIO_PORTC_AFSEL_R |= (1<<6) | (1<<5);     
    GPIO_PORTD_AFSEL_R |= (1<<6) | (1<<7);   

    GPIO_PORTC_PDR_R |= 0x60;
    GPIO_PORTC_DEN_R |= 0x60;
    
    GPIO_PORTD_PDR_R |= 0xC0;
    GPIO_PORTD_DEN_R |= 0xC0;
    
    //Configure the PMCn fields in the GPIOPCTL register to assign the QEI signals
    //to the appropriate pins (see page 688 and Table 23-5 on page 1351).
    GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R  & 0xFFFFFFFF) | 0x06600000;
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R  & 0xFFFFFFFF) | 0x66000000;
    
    //Configure the quadrature encoder to capture edges on both signals and maintain
    //an absolute position by resetting when it reaches the maximum as
    //defined by the MAXPOS
    //QEI1_CTL_R = 0;             //DISBLE - Needed?
    //STALLEN - The QEI module does not stall when the microcontroller is stopped by a debugger.
  
    QEI1_CTL_R |= (0<<12);      
    QEI1_CTL_R |= (0x00<<6);    //VELDIV - Predivide Velocity
    QEI1_CTL_R |= (1<<5);       //VELEN - Capture Velocity
    
    //CAPMODE - 1 = The PhA and PhB edges are counted, providing twice the
    //positional resolution but half the range.   
    QEI1_CTL_R |= (1<<3);       
    
    //MAXPOS: 495*4 - 1 = 1979 = 0x7BB
    QEI1_MAXPOS_R  = QEILOAD;
    QEI1_CTL_R |= (1<<0);       //ENABLE QEI
}


int32_t qei_get_pos(void)
{
    int32_t qei_pos = QEI1_POS_R;
    if (qei_pos > (QEI1_MAXPOS_R/2))
    {
        return (qei_pos - QEI1_MAXPOS_R);
    }
    return qei_pos;

}



