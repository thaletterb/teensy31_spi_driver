#ifndef _TEENSY31_SPI_DRIVER__
#define _TEENSY31_SPI_DRIVER__

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

/* 
*   Function Prototypes
*/

// A function to initialize the SPI Peripheral
void vfnDSPIMaster_Init(void);

// A function to send a byte over SPI
unsigned long u32fnDPSIMaster_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value);

// TODO - Functions which haven't been implemented yet
void vfnDSPISlave_Init(void);
unsigned long u32fnDPSIMaster_RxByte(unsigned char u8lvData, dspi_ctl * dspi_value);
unsigned long u32fnDPSISlave_SendByte(unsigned char u8lvData, dspi_ctl *dspi_value);


#endif  //  

