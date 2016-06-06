#include "teensy31_spi.h"

/** @brief: a function to initialize the mk20 SPI0 instance
*/
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
    
}

/** @brief: a function to send one byte "u8lvData" over SPI
*/
unsigned long u32fnDPSIMaster_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value)
{
  unsigned long u32ID;
 /*Commands to Flash Memory*/
  SPI0_CTAR0 = SPI_CTAR_FMSZ(0x7) | dspi_value->br | dspi_value->cpha | dspi_value->cpol;
  SPI0_CTAR0 |= (1<<2) | 1;             // 0101 to divide by 36     TODO - USE BIT MASKS DEFINED IN .h files
  SPI0_CTAR0 |= (1<<14);                // 0b0100 in CSSCK bits to scale delay by 32
  SPI0_PUSHR = SPI_PUSHR_EOQ_MASK | SPI_PUSHR_PCS(0x1) | (u8lvData);

  /*Start transmition*/
  SPI0_MCR &= ~SPI_MCR_HALT_MASK;       // Clear  the HALT Bit - Start Transfers

  while( !(SPI0_SR & SPI_SR_EOQF_MASK)) //EOQF - End of queue FIFO
  {}
  u32ID = SPI0_POPR ;
  SPI0_SR |=  SPI_SR_EOQF_MASK | SPI_SR_TCF_MASK | SPI_SR_RFDF_MASK ;   // Writing a 1 clears bits in the SR
                                                                        // Clear the end of queue, 
  SPI0_MCR |= SPI_MCR_HALT_MASK;        // Clear the HALT bit - End Transfers

  return u32ID;
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

unsigned long u32fnDPSIMaster_RxByte(unsigned char u8lvData, dspi_ctl * dspi_value)
{
  return u32fnDPSIMaster_SendByte(u8lvData,dspi_value);
}

unsigned long u32fnDPSISlave_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value)
{
  SPI1_PUSHR_SLAVE = SPI_PUSHR_SLAVE_TXDATA(u8lvData);
}

