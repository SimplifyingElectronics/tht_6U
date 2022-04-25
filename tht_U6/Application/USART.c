/*
 * USART.c
 *
 * Created: 16-01-2022 18:00:41
 *  Author: HP
 */ 

#include "USART.h"
#include "LCD_16x2.h"

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

unsigned char USART_init(uint32_t baud_rate)
{
	/* Initialize Q */
	UQFront = UQEnd = -1;
	
	uint16_t ubrrvalue = ((F_CPU/(baud_rate * 8UL)) - 1);
	if(ubrrvalue <= 0)
	{
		return USART_ERROR;
	}
	
	else
	{		
		UCSRB |= (1 << RXEN) | (1 << TXEN) | (1<<RXCIE);/* Turn on transmission and reception */
		UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);/* Use 8-bit character sizes */
		UCSRA |= ((1<<U2X));

		UBRRL = ubrrvalue;		/* Load lower 8-bits of the baud rate value */
		UBRRH = (ubrrvalue >> 8);	/* Load upper 8-bits*/
		
		sei();
		
		return USART_OK;
	}
}
void UWriteData(char data)
{
	/* wait till data register is not empty */
	while(!(UCSRA & (1<<UDRE)));
	
	/* Write data to the data register of UART */
	UDR = data;
}

void UWriteData_string(char *str)
{
	while(*str != '\0')
	{
		UWriteData(*str);
		str++;	
	}
	UWriteData('\0');
}
void UWriteData_Nbytes(uint8_t *str, uint8_t NBytes)
{
	uint8_t i = 0;
	while(i<NBytes)
	{
		UWriteData(str[i]);
		i++;
	}
	
}

ISR(USART_RXC_vect)
{
	char data = UDR;
	
	if(((UQEnd == (RECEIVE_BUFFER_SIZE - 1)) && (UQFront == 0)) || ((UQEnd+1)==UQFront))
	{
		UQFront++;
		
		if(UQFront==RECEIVE_BUFFER_SIZE) UQFront = 0;
	}
	
	if(UQEnd==(RECEIVE_BUFFER_SIZE-1)) UQEnd = 0;
	
	else
	UQEnd++;
	
	URbuff[UQEnd] = data;
	
	if(UQFront == -1) UQFront = 0;
	
}

char UReadData(void)
{
	char data;
	
	if(UQFront == -1)
	UQFront = 0;
	
	data = URbuff[UQFront];
	
	if(UQFront == UQEnd)
	UQFront = UQEnd = -1;
	
	else
	{
		UQFront++;
		
		if(UQFront == RECEIVE_BUFFER_SIZE)
		UQFront = 0;
	}
	return data;	
}

uint8_t UAvailableData(void)
{
	if(UQFront == -1) return 0;
	if(UQFront < UQEnd) return (UQEnd - UQFront + 1);
	else if (UQFront > UQEnd) return (RECEIVE_BUFFER_SIZE - UQFront + UQEnd + 1);
	else return 1;
}