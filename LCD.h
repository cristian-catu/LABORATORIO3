/*
 * File: LCD.h
 * Se utilizó y se adaptaron las librerías de Ligo George
 * de la página www.electrosome.com
 * Enlace: https://electrosome.com/lcd-pic-mplab-xc8/
 * Revision history:
 */

// This is a guard condition so that contents of this file are not included
// more than once.
#ifndef LCD_H
#define	LCD_H

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000
#endif

#ifndef RS
#define RS PORTDbits.RD2
#endif

#ifndef EN
#define EN RD3
#endif

#ifndef D4
#define D4 RD4
#endif

#ifndef D5
#define D5 RD5
#endif

#ifndef D6
#define D6 RD6
#endif

#ifndef D7
#define D7 RD7
#endif

#include <xc.h> // include processor files - each processor file is guarded.

//LCD Functions Developed by electroSome

void Lcd_Port(char a);

void Lcd_Cmd(char a);

void Lcd_Clear(void);

void Lcd_Set_Cursor(char a, char b);

void Lcd_Init(void);

void Lcd_Write_Char(char a);

void Lcd_Write_String(char *a);

void Lcd_Shift_Right(void);

void Lcd_Shift_Left(void);

#endif	/* LCD_H */
