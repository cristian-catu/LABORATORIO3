    /*
 * File:
 * Author: Cristian Catú
 * LAB 3
 *
 * Created on 7 august 2022, 19:04
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#define _XTAL_FREQ 4000000
#define FLAG_SPI 0x0F

/*------------------------------------------------------------------------------
 * LIBRERIAS
 ------------------------------------------------------------------------------*/
#include "SPI.h"
#include "oscilador.h"
#include "tmr0.h"
#include "LCD.h"
#include "adc.h"


void setup(void);
void setup2(void);

uint8_t contador = 0;
uint8_t bandera = 0;
uint8_t cont_master = 0;
uint8_t val_temporal = 0;
uint8_t canal_ADC = 0;
uint8_t valor_recibido = 0;
uint8_t POT1 = 0;
uint8_t POT2 = 0;
uint8_t POT1_ENVIO = 0;
uint8_t POT2_ENVIO = 0;
uint8_t variable = 0;
uint8_t init_POT_1 = 0;
uint8_t dec_POT_1 = 0;
uint8_t init_POT_2 = 0;
uint8_t dec_POT_2 = 0;
unsigned short VOLTAJE_1 = 0;
unsigned short VOLTAJE_2 = 0;
char s[];

unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, //Función del mapeo
            unsigned short out_min, unsigned short out_max);

void __interrupt() isr (void){
    //INTERRUPCIÓN DEL ESCLAVO
    if(PORTCbits.RC0 == 0){
        if(PIR1bits.SSPIF){             // ¿Recibió datos el esclavo?
            if (bandera == 1){         //Enviamos POT2
                spiWrite(POT1_ENVIO);
                bandera = 0;
            }
            else if (bandera == 0){ //Enviamos POT1
                spiWrite(POT2_ENVIO);
                bandera = 1;
            }
            PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción
        }
        if(PIR1bits.ADIF){
            if(canal_ADC == 0){
                POT1 = adc_read();//Leemos el POT1
            }
            else if(canal_ADC == 1){
                POT2 = adc_read();//Leemos el POT2
            }
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    TRISC = 0xFF;
    
    //CODIGO DEL MAESTRO
    if(PORTCbits.RC0 == 1){
        setup();
        Lcd_Set_Cursor(1,1); //escojo la fila
        Lcd_Write_String("  POT1    POT2");//escribimos en el LCD
        while(1){
            spiWrite(FLAG_SPI); //Se envía un valor para que el esclavo responda
            spiReceiveWait(); //Se espera el envio
            
            PORTCbits.RC2 = 1;  //Se habilita y desabilita el esclavo por que es necesario en los PICS
            __delay_ms(10);  
            PORTCbits.RC2 = 0; 
            __delay_ms(10);

            spiReceiveWait(); //Se espera lo que recibr
            valor_recibido = spiRead();    //se guarda lo que se recibio
            __delay_ms(10);
            //La variable puede ser 0 o 1 segun lo que se envio del esclavo
            variable = 0b00000001 & valor_recibido;
            if (variable == 1){
                POT1 = valor_recibido;       //Guardamos POT1 en maestro
                VOLTAJE_1 = map(POT1, 1, 255, 0, 500); //Conseguimos los enteros y decimales de POT1
                init_POT_1 = VOLTAJE_1/100;
                dec_POT_1 = VOLTAJE_1-init_POT_1*100;
            }
            else if (variable == 0) {
                POT2 = valor_recibido;      //Guardamos POT2 en maestro
                VOLTAJE_2 = map(POT2, 0, 254, 0, 500);// Conseguimos los enteros y decimales de POT2
                init_POT_2 = VOLTAJE_2/100;
                dec_POT_2 = VOLTAJE_2-init_POT_2*100;
            }
            
            sprintf(s, "  %d.%d    %d.%d ", init_POT_1, dec_POT_1, init_POT_2, dec_POT_2); //Colocamos valores finales en cadena
            Lcd_Set_Cursor(2,1); //escojo la fila
            Lcd_Write_String(s); //Escribimos en LCD

        }
    }
    
    //CODIGO DEL ESCLAVO
    else if(PORTCbits.RC0 == 0){
        setup2();
        while(1){
            if (canal_ADC == 0){ //primer canal
                adc_start(0);
                canal_ADC = 1;
            }
            else if (canal_ADC == 1){//segundo canal
                adc_start(1);
                canal_ADC = 0;
            }
            POT1_ENVIO = POT1 & 0b11111110;//se codifican los valores a enviar
            POT2_ENVIO = POT2 | 0b00000001;//se codifican los valores a enviar
        }
    }
}

//SETUP maestro
void setup(void){
    ANSEL = 0;
    TRISA = 0;
    PORTA = 0;
    TRISB = 0;
    PORTB = 0;
    TRISD = 0;
    PORTD = 0;
    
    TRISC = 0b00010001;
    init_osc_MHz(2);
    spiInit(SPI_MASTER_OSC_DIV4, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);//inicializamos el SPI del maestro
    Lcd_Init();
}

//SETUP Esclavo
void setup2(void){
    ANSELH = 0;
    ANSEL = 0b00000011;
    TRISA = 0b00100000;
    TRISB = 0;
    PORTB = 0;
    TRISD = 0;
    PORTD = 0;
    TRISC = 0b00011001;
    init_osc_MHz(2);
    adc_init(1,0,0);
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIR1bits.SSPIF = 0; 
    PIE1bits.SSPIE = 1;  
    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_ACTIVE_2_IDLE);//inicalizamos el SPI del esclavo
    
}
// funcion de mapeo
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}