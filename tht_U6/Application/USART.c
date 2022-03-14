/*
 * USART.c
 *
 * Created: 16-01-2022 18:00:41
 *  Author: HP
 */ 

#include "USART.h"

unsigned char USART_init(uint32_t baud_rate)
{
	/* Initialize Q */
	UQFront = UQEnd = -1;
	
	uint16_t ubrrvalue = (F_CPU/(baud_rate * 8) - 1);
	if(ubrrvalue <= 0)
	{
		return USART_ERROR;
	}
	
	else
	{		
		UBRRH = (unsigned char)(ubrrvalue >> 8);
		UBRRL = (unsigned char)(ubrrvalue);
	
		UCSRC |= (1<<URSEL);
		UCSRC |= ((3<<UCSZ0));
		UCSRB |= ((1<<RXCIE) | (1<<RXEN) | (1 << TXEN));
		UCSRA |= ((1<<U2X));
		
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

char UReadData(void)
{
	char data;
	
	if(UQFront == -1)
	return 0;
	
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
	else if (UQFront > UQEnd) return (RECEIVE_BUFFER_SIZE - UQFront + UQEnd - 1);
	else return 1;
}