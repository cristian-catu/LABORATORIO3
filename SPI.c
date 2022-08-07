 /*
 * File            : spi.c
 * Author          : Ligo George
 * Company         : electroSome
 * Project         : SPI Library for MPLAB XC8
 * Microcontroller : PIC 16F877A
 * Created on April 15, 2017, 5:59 PM
 */

#include "SPI.h"

void spiInit(Spi_Type sType, Spi_Data_Sample sDataSample, Spi_Clock_Idle sClockIdle, Spi_Transmit_Edge sTransmitEdge)
{
    TRISC5 = 0;
    if(sType & 0b00000100)
    {
        SSPSTAT = sTransmitEdge;
        TRISC3 = 1;
    }
    else
    {
        SSPSTAT = sDataSample | sTransmitEdge;
        TRISC3 = 0;
    }
    
    SSPCON = sType | sClockIdle;
}

static void spiReceiveWait()
{
    while ( !SSPSTATbits.BF );
}

void spiWrite(char dat)
{
    SSPBUF = dat;
}

unsigned spiDataReady()
{
    if(SSPSTATbits.BF)
        return 1;
    else
        return 0;
}

char spiRead()
{
    spiReceiveWait(); 
    return(SSPBUF);
}

