/* 
 * File:   oscilador.h
 * Author: CATU
 *
 * Created on 22 de julio de 2022, 08:02 AM
 */

#ifndef TMR0_H
#define	TMR0_H


#include <xc.h>
#include <stdint.h>

//Definimos las funciones
void tmr0_init(uint16_t prescaler);
void tmr0_reload(void);
void spiReceiveWait();
#endif	/* OSCILADOR_H */