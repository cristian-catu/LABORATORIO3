#include <xc.h>
#include "oscilador.h"

//freq            0 ---> 1MHz, 1 ---> 2MHz, 2 ---> 4MHz, 3 ---> 8MHz, 4 ---> 500kHz, default ---> 4MHz
void init_osc_MHz(uint8_t freq){
    OSCCONbits.SCS = 1;
    switch(freq){
        case 0:
            OSCCONbits.IRCF = 0b100;
            break;
        case 1:
            OSCCONbits.IRCF = 0b101;
            break;
        case 2:
            OSCCONbits.IRCF = 0b110;
            break;
        case 3:
            OSCCONbits.IRCF = 0b111;
            break;
        case 4:
            OSCCONbits.IRCF = 0b011;
            break;
        default:
            OSCCONbits.IRCF = 0b110;
            break;
    }
}

