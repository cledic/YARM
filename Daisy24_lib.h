/*
 * Simple library for Daisy24 LCD froma ACME Systems
 * http://www.acmesystems.it/DAISY-24
*/

#ifndef DAISY24_LIB_H_
#define DAISY24_LIB_H_

#define RIGA_1		0
#define RIGA_2		1

#define PB_UP_LEFT		1
#define PB_UP_RIGHT		2
#define PB_DOWN_LEFT	8
#define PB_DOWN_RIGHT	4

uint32_t Daisy24_LCD_Init(void);
uint32_t Daisy24_Expander_Init(void);
uint32_t Daisy24_LCD_Clear(void);
uint32_t Daisy24_LCD_SetCursor( uint8_t x, uint8_t y);
uint32_t Daisy24_LCD_WriteChar( char c);
uint32_t Daisy24_LCD_WriteString( char*s);
uint32_t Daisy24_LCD_BkLightOff(void);
uint32_t Daisy24_LCD_BkLightOn(void);
uint32_t Daisy24_LCD_MonitorInit( char*s1, char*s2, uint32_t m);
uint32_t Daisy24_LCD_MonitorUpdate( uint32_t*);

uint32_t Daisy24_PB_Status( uint32_t*p);

#endif
