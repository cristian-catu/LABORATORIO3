#include <xc.h>
#include "adc.h"
#define _XTAL_FREQ 4000000

//adc_cs       0 ---> Fosc/2, 1 ---> Fosc/8, 2 ---> Fosc/32, 3 ---> Frc
//vref_plus     0 ---> VDD, 1 ---> Fref
//vref_minus    0 ---> VSS, 1 ---> Fref

void adc_init(uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus){
    // Configuración ADC
    switch(adc_cs){
        case 0:
            ADCON0bits.ADCS = 0b00;     // Fosc/2
            break;
        case 1:
            ADCON0bits.ADCS = 0b01;     // Fosc/8
            break;
        case 2:
            ADCON0bits.ADCS = 0b10;     // Fosc/32
            break;
        case 3:
            ADCON0bits.ADCS = 0b11;     // Frc
            break;
        default:
            break;
    }
    ADCON1bits.VCFG0 = vref_plus;       // VDD
    ADCON1bits.VCFG1 = vref_minus;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
}

void adc_start(uint8_t channel){
    // Configuración canal
    if(channel < 14){
        ADCON0bits.CHS = channel;
    }
    else{
        ADCON0bits.CHS = 0; //canal 0 default
    }
    __delay_us(40);             // Sample time
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO){}
}
int adc_read(void){
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    return (ADRESH);            //Retorna lectura de ADC en ADRESH
}
