/***********************************************************************
File Name           : uart.c
Project             : Self Balancing Robot
Author              : Sreejith S, Karthik
Date                : 29 Nov 2019
Platform            : Tiva Development Kit EK-TM4C123GXL
Descirption         : Uart communication
***********************************************************************/

#include "uart.h"


void uart_init(uint8_t module, uint32_t baud, uint32_t sys_clock)
{
    //only module0 & 6 for now - write better code later
    uint16_t IBRD;
    float BRD;
    uint8_t FBRD;
    
    //Baud Rate settings
    
    BRD = (float)sys_clock/(baud<<4);
    IBRD = BRD;
    FBRD = 64*(BRD-IBRD) + 0.5f;
    
    if (module == 0)
    {
        //see page 902 - Section 14.4
        SYSCTL_RCGCUART_R |= (1<<0);                        //Enable the UART module using the RCGCUART register (see page 344)
        SYSCTL_RCGCGPIO_R |= (1<<0);                        //Enable the clock to the appropriate GPIO module (GPIOA) via the RCGCGPIO register (see page 340).
                                                            //To find out which GPIO port to enable, refer to Table 23-5 on page 1351.
        GPIO_PORTA_AFSEL_R |= (1<<1)|(1<<0);
        GPIO_PORTA_DEN_R |= (1<<1)|(1<<0);
        // GPIO_PORTA_AMSEL_R &= ~0x03;
        
        GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R  & 0xFFFFFF00) |0x0000011;
                                                            //Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate
                                                            //pins (see page 688 and Table 23-5 on page 1351).

        
        UART0_CTL_R &= ~(1<<0);                             //Disable the UART by clearing the UARTEN bit in the UARTCTL register
        UART0_IBRD_R = IBRD;                                //Write the integer portion of the BRD to the UARTIBRD register.        
        UART0_FBRD_R = FBRD;                                //Write the fractional portion of the BRD to the UARTFBRD register.
        
        UART0_LCRH_R = (0x3<<5)|(1<<4);                     //Write the desired serial parameters to the UARTLCRH register
                                                            //8-bit word length, 1 stopbit
                                                            //FIFO Enable FEN bit4
                                                            
        UART0_CC_R = 0x00;                                  //Configure the UART clock source by writing to the UARTCC register.
                                                            //Use system clock
        UART0_CTL_R |= (1<<0)|(1<<8)|(1<<9);                //Enable the UART by setting the UARTEN bit in the UARTCTL register
    }
    else
    {
        //Module 7
        SYSCTL_RCGCUART_R |= (1<<7);                        //Enable the UART module using the RCGCUART register (see page 344)
        SYSCTL_RCGCGPIO_R |= (1<<4);                        //Enable the clock to the appropriate GPIO module (GPIOD) via the RCGCGPIO register (see page 340).
                                                            //To find out which GPIO port to enable, refer to Table 23-5 on page 1351.
        GPIO_PORTE_AFSEL_R |= (1<<0)|(1<<1);
        GPIO_PORTE_DEN_R |= (1<<0)|(1<<1);
        // GPIO_PORTA_AMSEL_R &= ~0x03;
        
        GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R  & 0xFFFFFF00) |0x00000011;
                                                            //Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate
                                                            //pins (see page 688 and Table 23-5 on page 1351).

        
        UART7_CTL_R &= ~(1<<0);                             //Disable the UART by clearing the UARTEN bit in the UARTCTL register
        UART7_IBRD_R = IBRD;                                //Write the integer portion of the BRD to the UARTIBRD register.        
        UART7_FBRD_R = FBRD;                                //Write the fractional portion of the BRD to the UARTFBRD register.
        
        UART7_LCRH_R = (0x3<<5)|(1<<4);                     //Write the desired serial parameters to the UARTLCRH register
                                                            //8-bit word length, 1 stopbit
                                                            //FIFO Enable FEN bit4
                                                            
        UART7_CC_R = 0x00;                                  //Configure the UART clock source by writing to the UARTCC register.
                                                            //Use system clock
        UART7_CTL_R |= (1<<0)|(1<<8)|(1<<9);                //Enable the UART by setting the UARTEN bit in the UARTCTL register
    }
    

}

void uart_send(uint8_t module, uint8_t data)
{
    if (module == 0)
    {
        while((UART0_FR_R&(1<<5)));
        UART0_DR_R = data;
    }
    else
    {
        while((UART7_FR_R&(1<<5)));
        UART7_DR_R = data;
    } 
    
}

void uart_send_string(uint8_t module, char* string)
{

    while(*string)
    {
        uart_send(module, *(string++));
    }        

}

uint8_t uart_receive(uint8_t module)
{
    if (module == 0)
    {
        while((UART0_FR_R&(1<<4)));
        return (uint8_t)(UART0_DR_R&0xFF);
    }
    else
    {
        while((UART7_FR_R&(1<<4)));
        return (uint8_t)(UART7_DR_R&0xFF);
    } 
}


void test_uart(uint8_t module)
{
    uint8_t data;
    uart_send_string(module, "Enter a char:\r\n");
    data = uart_receive(module);
    uart_send_string(module, "Recieved ");
    uart_send(module, data);
    uart_send_string(module, "\r\n\r\n");
    
}


void uart_send_itoa(uint8_t module, int num)
{   
    uint8_t num_str[12];
    uint8_t pos = 10;
    char sign = '0';
    if (num<0)  sign = '-';
    num_str[11] = '\0';
    while(num)
    {
        num_str[pos] = (num%10)+48;
        num /= 10;
        pos--;
    }
    num_str[pos] = sign;
    
    uart_send_string(module, (char*)(&num_str[pos]));
}


void uart_readln_str(uint8_t module, char* string)
{
    uint8_t c = '\0';
    uint8_t i = 0;
    while(1)
    {
        
        c = uart_receive(module);
        if(c == 0x0D || c == 0x0A)
        {
            string[i] = '\0';
            break;
        }
        string[i] = c;
        i++;
        
    }
    
}

