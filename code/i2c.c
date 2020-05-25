/***********************************************************************
File Name            : i2c.c
Project              : Self Balancing Robot
Author               : Sreejith S, Karthik
Date                 : 06 Dec 2019
Platform             : Tiva Development Kit EK-TM4C123GXL
Descirption          : Code for I2C communication
***********************************************************************/

#include "i2c.h"


void i2c_init(uint32_t sys_clock, uint32_t speed)
{

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;    //Enable clock for Port E    
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};  //Wait till PortE peripheral ready      
    GPIO_PORTE_AFSEL_R |= (1<<5)|(1<<4);
    GPIO_PORTE_PCTL_R &= ~(0xFF<<16);
    GPIO_PORTE_PCTL_R |= (3<<20)|(3<<16);
    //GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFF00FFFF)+0x00330000;
    GPIO_PORTE_ODR_R |= (1<<5);
    GPIO_PORTE_DEN_R |= (1<<5)|(1<<4);
    //GPIO_PORTE_AMSEL_R &= ~0x30;                            //Disable analog functionality on PE4,5
    
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R2; while ((SYSCTL_PRI2C_R&SYSCTL_PRI2C_R2) == 0){};
          
    
    I2C2_MCR_R = I2C_MCR_MFE;               //master mode
    I2C2_MTPR_R = (sys_clock/(20*speed))-1;            //11 for 100kbps@25MHz sysclock  //configure for speed


}

uint8_t i2c_is_busy(void)
{
    return (I2C2_MCS_R&I2C_MCS_BUSY);
}

uint8_t i2c_is_busbusy(void)
{
    return (I2C2_MCS_R&I2C_MCS_BUSBSY);
}

uint8_t i2c_is_write_error(void)
{
    return (I2C2_MCS_R&I2C_MCS_ERROR);
}



void i2c_read_single(uint8_t slave_addr, uint8_t* data)
{
    return;
}

void i2c_write_single(uint8_t slave_addr, uint8_t data)
{
    //See page 1007
    ///Not working!

    volatile uint32_t readback;
    I2C2_MSA_R = (slave_addr<<1);
    I2C2_MDR_R = data;
//    readback = I2C2_MDR_R;// Ensure write has been drained from the write buffer
    //---0-111
    I2C2_MCS_R &= ~(I2C_MCS_HS);
    I2C2_MCS_R |=  (I2C_MCS_STOP|I2C_MCS_START|I2C_MCS_RUN);
    //readback = I2C2_MCS_R; // Ensure write has been drained from the write buffer
    while(!i2c_is_busy()){};
    while(i2c_is_busy()){};

}




void i2c_read_burst(uint8_t slave_addr, uint8_t mpu_reg_addr, uint8_t* data, uint8_t size)
{
    //write mpu_reg_addr and read recieved bytes into data
    uint8_t i;
    /*while(i2c_is_busy()){};                     // wait for I2C ready
    //Transmit MPU register address
    I2C2_MSA_R = (slave_addr<<1);               // MSA[7:1] is slave address
    I2C2_MDR_R = mpu_reg_addr;                  // Write MPU Register Address
    I2C2_MCS_R = (I2C_MCS_START|                // generate start/restart
                  I2C_MCS_RUN);                 // master enable
    while(i2c_is_busy()){};
    */
    i2c_write_burst(slave_addr, &mpu_reg_addr, 1);
    //Recieve bytes
    I2C2_MSA_R |= 0x01;                         // MSA[0] is 1 for receive
    if (size == 1)
    {

        //When only one byte is read generate both Start and Stop bits, negative ack
        I2C2_MCS_R = ((~I2C_MCS_ACK)|                 // No Ack
                        I2C_MCS_STOP|                 // Stop bit
                        I2C_MCS_START|                // Start
                        I2C_MCS_RUN);                 // Master Enable
        data[0] = I2C2_MDR_R;
        //I2C2_MCS_R =  I2C_MCS_STOP;
        while(i2c_is_busy()){};

        return;
    }

    //0th byte
    I2C2_MCS_R = (  I2C_MCS_ACK|                  // Positive Ack
                    I2C_MCS_START|                // Repeated-start
                    I2C_MCS_RUN);                 // Master Enable
    while(i2c_is_busy()){};
    data[0] = I2C2_MDR_R;

    //1:(size-2) bytes
    for (i=1; i<size-1; ++i)
    {
        I2C2_MCS_R = (  I2C_MCS_ACK|                  // Ack for 2nd till (n-1)th byte
                        I2C_MCS_RUN);                 // Master Enable
        while(i2c_is_busbusy()){}; //Not needed!
        while(i2c_is_busy()){};
        data[i] = I2C2_MDR_R;
    }

    //Last Byte
    I2C2_MCS_R =((~I2C_MCS_ACK)|              // No Ack
                I2C_MCS_STOP|                 // Stop bit
                I2C_MCS_RUN);                 // Master Enable
    while(i2c_is_busy()){};
    data[size-1] = I2C2_MDR_R;


    while(i2c_is_busy()){};                   // wait for transmission done
    //return (I2C2_MDR_R&0xFF);               // usually returns 0xFF on error

}



void i2c_write_burst(uint8_t slave_addr, uint8_t* data, uint8_t size)
{
    //Pg 1009
    uint8_t i;
    while(i2c_is_busy()){};;                   // wait for I2C ready
    I2C2_MSA_R = (slave_addr<<1);              // MSA[7:1] is slave address
    //I2C2_MSA_R &= ~0x01;                     // MSA[0] is 0 - Master transmits data to slave
    I2C2_MDR_R = data[0];
    while(i2c_is_busy()){};;
    if (size == 1)
    {
        I2C2_MCS_R = (  I2C_MCS_STOP|                 // Stop bit
                        I2C_MCS_START|                // start
                        I2C_MCS_RUN);                 // Master Enable

        while(i2c_is_busy()){};     // [Read Only] wait for transmission done
        return;
    }

    for (i=0; i<size; ++i)
    {
        I2C2_MDR_R = data[i];
        if (i == size-1)
        {
             I2C2_MCS_R = (  I2C_MCS_STOP|                 // Stop bit
                             I2C_MCS_RUN);                 // Master Enable
             while(i2c_is_busy()){};     // [Read Only] wait for transmission done
             return;
        }
        if (i == 0)
        {   I2C2_MCS_R = (  I2C_MCS_START|                // start
                            I2C_MCS_RUN);                 // Master Enable
            while(i2c_is_busy()){};     // [Read Only] wait for transmission done
        }
        else
        {
            I2C2_MCS_R = (I2C_MCS_RUN);                 // Master Enable
            while(i2c_is_busy()){};

        }



    }
    i2c_is_write_error();
     // return error bits
    //return (I2C2_MCS_R&(I2C_MCS_DATACK|I2C_MCS_ADRACK|I2C_MCS_ERROR));
}

void i2c_write_2(uint8_t slave_addr, uint8_t data1, uint8_t data2)
{
    uint8_t data[2];
    data[0] = data1;
    data[1] = data2;
    i2c_write_burst(slave_addr, data, 2);
}
