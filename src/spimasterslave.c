/*
 * File:		spimasterslave.c
 * Purpose:		Main process
 *
 */

#include "common.h"

typedef struct{
    uint32_t mstr;
    uint32_t pcs;
    uint32_t hlt;
    uint32_t fmsz;
    uint32_t cpol;
    uint32_t cpha;
    uint32_t lsbfe;
    uint32_t br;
    uint32_t tcf;
    uint32_t eoqf;
    uint32_t cont;
    uint32_t ctas;
    uint32_t delay;
    uint32_t eoq;
    uint32_t pcsx;
    uint32_t txdata;
    uint32_t buffer;
} dspi_ctl;

dspi_ctl dspi;

#define  LED_ON     GPIOC_PSOR=(1<<5)
#define  LED_OFF    GPIOC_PCOR=(1<<5)

#define  LED2_ON     GPIOC_PSOR=(1<<6)
#define  LED2_OFF    GPIOC_PCOR=(1<<6)

void vfnDSPIMaster_Init(void);
void vfnDSPISlave_Init(void);

unsigned long u32fnDPSIMaster_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value);
unsigned long u32fnDPSIMaster_RxByte(unsigned char u8lvData, dspi_ctl * dspi_value);

unsigned long u32fnDPSISlave_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value);

/********************************************************************/
int main (void)
{
    unsigned char u8vData = 0xAA;
    unsigned char u8vRxData = 0;
    
    PORTC_PCR5 = PORT_PCR_MUX(0x1);                         // LED is on PC5 (pin 13), config as GPIO (alt = 1)
    GPIOC_PDDR = (1<<5);                                    // make this an output pin
    
    vfnDSPIMaster_Init();                                   // Init the SPI Master
   
    volatile uint32_t n;
    while(1){ 
        for (n=0; n<8000000; n++)  ;                        // dumb delay
        //vfnDSPISlave_Init();                              // Init the SPI Slave
        u8vData = 0xAA;                                     // Send another byte
        (void)u32fnDPSIMaster_SendByte(u8vData,&dspi);      // Send byte

        u8vData = 0x55;                                     // Send another byte
        (void)u32fnDPSIMaster_SendByte(u8vData,&dspi);      
    }

    // Never gets down here
	while(1)
	{
          u8vData++;
          (void)u32fnDPSISlave_SendByte(u8vData,&dspi);   
          u8vRxData = u32fnDPSIMaster_RxByte(u8vData,&dspi);
          if(u8vData == u8vRxData)
          {
              LED_OFF;
          }
          else
          {
              LED_ON;   // Error - Turn on LED
              while(1);
          }
	} 
    return 0;
		
}
/********************************************************************/
void vfnDSPIMaster_Init(void)
{
 /* clock gate */
  //SIM_SCGC6 |= SIM_SCGC6_DSPI0_MASK; BV 5-21
  SIM_SCGC6 |= SIM_SCGC6_SPI0_MASK; // System Clock Gating Control Register 6. Set SPI0 Bit

  /* pin mux */
  PORTD_PCR0 &= ~PORT_PCR_MUX_MASK;
  PORTD_PCR0 |= PORT_PCR_MUX(2);    //SPI0_PCS0 - Alt Function 2. PTD0 - Teensy 2
  PORTD_PCR1 &= ~PORT_PCR_MUX_MASK;
  PORTD_PCR1 |= PORT_PCR_MUX(2);    //SPI0_SCK - Alt Function 2 - PTD1 - Teensy 14
  PORTD_PCR2 &= ~PORT_PCR_MUX_MASK;
  PORTD_PCR2 |= PORT_PCR_MUX(2);    //SPI0_SOUT - Alt Function 2 - PTD2 - Teensy 7
  PORTD_PCR3 &= ~PORT_PCR_MUX_MASK;
  PORTD_PCR3 |= PORT_PCR_MUX(2);    //SPI0_SIN - Alt Function 2 - PTD3 - Teensy 8

  //SPI0_CTAR0 = SPI_CTAR_FMSZ(0xF) | SPI_CTAR_CPOL_MASK;                     // Clock Transfer and Attributes Register. Frame Size and CPOL
  SPI0_CTAR0 = SPI_CTAR_FMSZ(0x7) | SPI_CTAR_CPOL_MASK;                     // Clock Transfer and Attributes Register. Frame Size and CPOL
  SPI0_CTAR0 |= (1<<2) | 1; // 0b0101 in BR bits to divide SPI SCK by 36 
  SPI0_CTAR0 |= (1<<14);     // 0b0100 in CSSCK bits to scale delay by 32

  /*Set SPI mode Master, Halt and Incoming data is shifted into the shift register.  */
  SPI0_MCR = SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0x1) | SPI_MCR_HALT_MASK;    // Module Configuration Register
  
  /*Setting some variables*/
  dspi.br     = 0x00;
  dspi.cpha   = 0x00000000;
  dspi.cpol   = 0x00000000;  
}

void vfnDSPISlave_Init(void)
{
  SIM_SCGC6 |=SIM_SCGC6_SPI1_MASK;  // System Clock Gating Control Register 6 - Set SPI1 Bit
 
  //// Made For K60/K70 Family - Need to find Equivalent for K20 Family, with Teensy Access 
  //PORTE_PCR1 &= ~PORT_PCR_MUX_MASK; 
  //PORTE_PCR1 |= PORT_PCR_MUX(2);    // SPI1_SOUT
  //PORTE_PCR2 &= ~PORT_PCR_MUX_MASK; 
  //PORTE_PCR2 |= PORT_PCR_MUX(2);    // SPI1 SCLK
  //PORTE_PCR3 &= ~PORT_PCR_MUX_MASK;
  //PORTE_PCR3 |= PORT_PCR_MUX(2);    // SPI1 SIN
 //// PORTE_PCR4 &= ~PORT_PCR_MUX_MASK;
 //// PORTE_PCR4 |= PORT_PCR_MUX(2); 
 
  // Made For K60/K70 Family - Need to find Equivalent for K20 Family, with Teensy Access 
  PORTB_PCR16 &= ~PORT_PCR_MUX_MASK; 
  PORTB_PCR16 |= PORT_PCR_MUX(2);    // SPI1_SOUT - PTB16 (Teensy 0) (Alt 2)
  //PORTB_PCR2 &= ~PORT_PCR_MUX_MASK; 
  //PORTB_PCR2 |= PORT_PCR_MUX(2);    // SPI1 SCLK - 
  PORTB_PCR17 &= ~PORT_PCR_MUX_MASK;
  PORTB_PCR17 |= PORT_PCR_MUX(2);    // SPI1 SIN - PTB17 (Teensy 1) (Alt 2)
  
  SPI1_MCR = SPI_MCR_PCSIS(0x1) | SPI_MCR_DIS_TXF_MASK | SPI_MCR_DIS_RXF_MASK ; //  
}

unsigned long u32fnDPSIMaster_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value)
{
  unsigned long u32ID;
 /*Commands to Flash Memory*/
  //SPI0_CTAR0 = SPI_CTAR_FMSZ(0xF) | dspi_value->br | dspi_value->cpha | SPI_CTAR_CPOL_MASK;
  //SPI0_CTAR0 = SPI_CTAR_FMSZ(0x7) | dspi_value->br | dspi_value->cpha | dspi_value->cpol | SPI_CTAR_CPOL_MASK;
  SPI0_CTAR0 = SPI_CTAR_FMSZ(0x7) | dspi_value->br | dspi_value->cpha | dspi_value->cpol;
  SPI0_CTAR0 |= (1<<2) | 1;     // 0101 to divide by 36     TODO - USE BIT MASKS DEFINED IN .h files
  SPI0_CTAR0 |= (1<<14);        // 0b0100 in CSSCK bits to scale delay by 32
  SPI0_PUSHR = SPI_PUSHR_EOQ_MASK | SPI_PUSHR_PCS(0x1) | (u8lvData);  
  
  /*Start transmition*/
  SPI0_MCR &= ~SPI_MCR_HALT_MASK;       // Clear  the HALT Bit - Start Transfers
  
  while( !(SPI0_SR & SPI_SR_EOQF_MASK)) //EOQF - End of queue FIFO
  {}
  u32ID = SPI0_POPR ; 
  SPI0_SR |=  SPI_SR_EOQF_MASK | SPI_SR_TCF_MASK | SPI_SR_RFDF_MASK ;   // Writing a 1 clears bits in the SR
                                                                        // Clear the end of queue, 
  SPI0_MCR |= SPI_MCR_HALT_MASK;    // Clear the HALT bit - End Transfers
  
  return u32ID;
}

unsigned long u32fnDPSIMaster_RxByte(unsigned char u8lvData, dspi_ctl * dspi_value)
{
  return u32fnDPSIMaster_SendByte(u8lvData,dspi_value);
}

unsigned long u32fnDPSISlave_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value)
{
  SPI1_PUSHR_SLAVE = SPI_PUSHR_SLAVE_TXDATA(u8lvData);  
}
