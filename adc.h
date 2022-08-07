/* 
 * File:   oscilador.h
 * Author: CATU
 *
 * Created on 22 de julio de 2022, 08:02 AM
 */

#ifndef ADC_H
#define	ADC_H


#include <xc.h>
#include <stdint.h>

//Definimos funciones
void adc_init(uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus);
void adc_start(uint8_t channel);
int adc_read(void);
#endif	/* OSCILADOR_H */