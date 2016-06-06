#####################################################
#Teensy 3.1 SPI Driver                              #
#####################################################

# Introduction
A simple SPI driver targeted for the freescale mk20dx256vlh7 found on the Teensy 3.1 board 

# HW Connections

| SPI           | MK20          | Teensy  |
| -------------:|--------------:| -------:|
| /CS           | PTD0          | 2       |
| SCLK          | PTD1          | 14      |
| MOSI          | PTD2          | 7       |
| MISO          | PTD3          | 8       |

# Todo
1. Configuring the SPI as slave
2. Using the SPI FIFO buffer to increase throughput 

# Credits
Original Sample Source Code: https://community.freescale.com/thread/303923#comment-314671
