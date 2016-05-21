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


void vfnDSPIMaster_Init(void);
void vfnDSPISlave_Init(void);

unsigned long u32fnDPSIMaster_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value);
unsigned long u32fnDPSIMaster_RxByte(unsigned char u8lvData, dspi_ctl * dspi_value);

unsigned long u32fnDPSISlave_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value);

/********************************************************************/
int main (void)
{
	unsigned char u8vData = 0x01;
        unsigned char u8vRxData = 0;
        printf("  ******************************************** \r\n");
        printf("  *              DSPI Master Slave           * \r\n");
  	printf("  ******************************************** \r\n");


        printf(" Connect PTE2 -> PTD1 \r\n"); 
        printf(" Connect PTE4 -> PTD0 \r\n");
        printf(" Connect PTE1 -> PTD3 \r\n");
        printf(" Connect PTE3 -> PTD2 \r\n");
        
        vfnDSPIMaster_Init();
        vfnDSPISlave_Init();
        (void)u32fnDPSIMaster_SendByte(u8vData,&dspi);
	while(1)
	{
          u8vData++;
         (void)u32fnDPSISlave_SendByte(u8vData,&dspi);   
          u8vRxData = u32fnDPSIMaster_RxByte(u8vData,&dspi);
          if(u8vData == u8vRxData)
            printf(" Data sent and received %d \r\n" ,u8vRxData);
          else
          {
            printf(" ERROR No match with sent data!!!! \r\n");
            while(1);
          }
	} 
    return 0;
		
}
/********************************************************************/
void vfnDSPIMaster_Init(void)
{
 /* clock gate */
  SIM_SCGC6 |= SIM_SCGC6_DSPI0_MASK;
  /* pin mux */

  PORTD_PCR0 &= ~PORT_PCR_MUX_MASK;
  PORTD_PCR0 |= PORT_PCR_MUX(2); //SPI0_PCS0
  PORTD_PCR1 &= ~PORT_PCR_MUX_MASK;
  PORTD_PCR1 |= PORT_PCR_MUX(2); //SPI0_SCK
  PORTD_PCR2 &= ~PORT_PCR_MUX_MASK;
  PORTD_PCR2 |= PORT_PCR_MUX(2); //SPI0_SOUT
  PORTD_PCR3 &= ~PORT_PCR_MUX_MASK;
  PORTD_PCR3 |= PORT_PCR_MUX(2); //SPI0_SIN

  
   SPI0_CTAR0 = SPI_CTAR_FMSZ(0xF) | SPI_CTAR_CPOL_MASK;
  /*Set SPI mode Master, Halt and Incoming data is shifted into the shift register.  */
  SPI0_MCR = SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0x1) | SPI_MCR_HALT_MASK;
  
  /*Setting some variables*/
  dspi.br     = 0x00;
  dspi.cpha   = 0x00000000;
  dspi.cpol   = 0x00000000;  
}

void vfnDSPISlave_Init(void)
{
  SIM_SCGC6 |=SIM_SCGC6_SPI1_MASK;
  
  PORTE_PCR1 &= ~PORT_PCR_MUX_MASK;
  PORTE_PCR1 |= PORT_PCR_MUX(2);
  PORTE_PCR2 &= ~PORT_PCR_MUX_MASK;
  PORTE_PCR2 |= PORT_PCR_MUX(2);
  PORTE_PCR3 &= ~PORT_PCR_MUX_MASK;
  PORTE_PCR3 |= PORT_PCR_MUX(2);
 // PORTE_PCR4 &= ~PORT_PCR_MUX_MASK;
 // PORTE_PCR4 |= PORT_PCR_MUX(2); 
  
  SPI1_MCR = SPI_MCR_PCSIS(0x1) | SPI_MCR_DIS_TXF_MASK | SPI_MCR_DIS_RXF_MASK ; //  
}

unsigned long u32fnDPSIMaster_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value)
{
  unsigned long u32ID;
 /*Commands to Flash Memory*/
  SPI0_CTAR0 = SPI_CTAR_FMSZ(0xF) | dspi_value->br | dspi_value->cpha | SPI_CTAR_CPOL_MASK;
  SPI0_PUSHR = SPI_PUSHR_EOQ_MASK | SPI_PUSHR_PCS(0x1) | u8lvData;  
  
  /*Start transmition*/
  SPI0_MCR &= ~SPI_MCR_HALT_MASK;
  
  while( !(SPI0_SR & SPI_SR_EOQF_MASK)) //EOQF
  {}
  u32ID = SPI0_POPR ; 
  SPI0_SR |=  SPI_SR_EOQF_MASK | SPI_SR_TCF_MASK | SPI_SR_RFDF_MASK ;
  SPI0_MCR |= SPI_MCR_HALT_MASK;
  
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
