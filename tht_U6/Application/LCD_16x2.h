/*
 * LCD_16x2.h
 *
 * Created: 17-01-2022 01:19:42
 *  Author: HP
 */ 


#ifndef LCD_16X2_H_
#define LCD_16X2_H_

/*#define F_CPU 16000000*/
#include <avr/io.h>
#include <util/delay.h>

#define LCD_Data_Dir DDRC
#define LCD_Command_Dir DDRD
#define LCD_Data_Port PORTC
#define LCD_Command_Port PORTD
#define RS 6
#define EN 7

void LCD_Init(void);
void LCD_float(float);
void LCD_Pos_xy (char row, char pos);
void LCD_String_xy (char , char, char *);
void LCD_location(uint8_t, uint8_t);
void LCD_Char(unsigned char);
void LCD_write_string(char *);
void LCD_write(uint8_t);
void LCD_showvalue(uint16_t);
void LCD_Command(unsigned char);
void LCD_Clear(void);


#endif /* LCD_16X2_H_ */