/*
 * File:		spimasterslave.c
 * Purpose:		Main process
 *
 */

#include "common.h"
#include "teensy31_spi.h"

dspi_ctl dspi;      // The struct which contains the SPI peripheral settings

#define  LED_ON     GPIOC_PSOR=(1<<5)
#define  LED_OFF    GPIOC_PCOR=(1<<5)

#define  LED2_ON     GPIOC_PSOR=(1<<6)
#define  LED2_OFF    GPIOC_PCOR=(1<<6)

/********************************************************************/
int main (void)
{
    unsigned char u8vData = 0xAA;
    unsigned char u8vRxData = 0;
    
    PORTC_PCR5 = PORT_PCR_MUX(0x1);                         // LED is on PC5 (pin 13), config as GPIO (alt = 1)
    GPIOC_PDDR = (1<<5);                                    // make this an output pin
    
    vfnDSPIMaster_Init();                                   // Init the SPI Master
   
    /*Setting some additonal SPI variables*/
    dspi.br     = 0x00;
    dspi.cpha   = 0x00000000;
    dspi.cpol   = 0x00000000;

    volatile uint32_t n;
    while(1){ 
        for (n=0; n<8000000; n++)  ;                        // dumb delay
        //vfnDSPISlave_Init();                              // Init the SPI Slave
        u8vData = 0xAA;                                     // Send another byte
        (void)u32fnDPSIMaster_SendByte(u8vData,&dspi);      // Send byte

        u8vData = 0x55;                                     // Send another byte
        (void)u32fnDPSIMaster_SendByte(u8vData,&dspi);      

        u8vData = 0xEE;                                     // Send another byte
        (void)u32fnDPSIMaster_SendByte(u8vData,&dspi);      
    }
    return 0;
		
}

