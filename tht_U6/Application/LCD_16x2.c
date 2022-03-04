/*
 * LCD_16x2.c
 *
 * Created: 17-01-2022 01:19:42
 *  Author: HP
 */

#include "LCD_16x2.h"

void LCD_Command(unsigned char cmnd)
{
	/* send command to LCD */
	LCD_Data_Port = cmnd;
	
	/* make RS zero to send command */
	LCD_Command_Port &= ~(1<<RS);
	
	/* High to low transition on EN pin */
	LCD_Command_Port |= (1<<EN);	
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	
	_delay_ms(3);
}

void LCD_Char (unsigned char char_data)
{
	/* send character to LCD */
	LCD_Data_Port= char_data;
	
	/* make RS one to send data */
	LCD_Command_Port |= (1<<RS);
	
	/* High to low transition on EN pin */	
	LCD_Command_Port |= (1<<EN);	
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	
	_delay_ms(1);
}
void LCD_write(uint8_t data)
{
	/* send character to LCD */
	LCD_Data_Port= data;
	
	/* make RS one to send data */
	LCD_Command_Port |= (1<<RS);
	
	/* High to low transition on EN pin */
	LCD_Command_Port |= (1<<EN);
	_delay_us(1);
	LCD_Command_Port &= ~(1<<EN);
	
	_delay_ms(1);
}

void LCD_Init (void)			
{
	/* Make RS and EN pin as output */
	LCD_Command_Dir |= ((1<<RS) | (1<<EN));
	
	/* Make data port as output */
	LCD_Data_Dir = 0xFF;		
	_delay_ms(20);
	
	/* 8 bit 2 line 5x7 mode */
	LCD_Command (0x38);
	
	/* General delay */
	_delay_us(1);
	
	/* clear screen */		
	LCD_Command (0x01);
	
	/* General delay */
	_delay_us(1);
	
	/* Display on cursor on */
	LCD_Command (0x0E);
	
	/* General delay */
	_delay_us(1);
	
	/* Cursor at first row first column */		
	LCD_Command (0x80);	
	
	/* give small delay */
	_delay_us(50);	
}

void LCD_String (char *str)		
{
	int i;
	for(i=0;str[i]!=0;i++)		
	{
		LCD_Char(str[i]);
	}
}
void LCD_write_string(char *str)
{
	int i=0;
	while(str[i] != '\0')
	{
		LCD_write(str[i]);
		i++;
	}
}

void LCD_String_xy (char row, char pos, char *str)
{
	if (row == 0 && pos<16)
		LCD_Command((pos & 0x0F)|0x80);	
	else if (row == 1 && pos<16)
		LCD_Command((pos & 0x0F)|0xC0);	
		LCD_String(str);			
}
void LCD_Pos_xy (char row, char pos)
{
	if (row == 0 && pos<16)
	LCD_Command((pos & 0x0F)|0x80);
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);
}

void LCD_location(uint8_t x, uint8_t y)
{
	uint8_t firstadress[] = {0x80, 0xC0, 0x94, 0xD4};
	LCD_Command(firstadress[x-1] + y - 1);
	_delay_ms(10);
}

void LCD_Clear(void)
{
	LCD_Command (0x01);		
	LCD_Command (0x80);		
}

void LCD_float(float num)
{
	int value = num * 100;

	LCD_Char((value / 1000) + 48); // '1'
	
	LCD_Char((value / 100) % 10 + 48);

	LCD_Char('.'); // '.'

	LCD_Char((value / 10) % 10 + 48); //'8'

	LCD_Char((value) % 10 + 48); //'5'
}

void LCD_showvalue(uint16_t data)
{
	LCD_write(((data / 100) % 10) + 0x30);
	LCD_write(((data / 10) % 10) + 0x30);
	LCD_Char('.');
	LCD_write(((data / 1) % 10) + 0x30);
}