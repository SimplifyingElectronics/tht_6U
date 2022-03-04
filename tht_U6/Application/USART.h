/*
 * USART.h
 *
 * Created: 16-01-2022 18:00:26
 *  Author: HP
 */ 


#ifndef USART_H_
#define USART_H_

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>

#define USART_OK					1
#define USART_ERROR					0
#define RECEIVE_BUFFER_SIZE			90

int8_t UQFront, UQEnd;

uint16_t baud_rate;

volatile char URbuff[RECEIVE_BUFFER_SIZE];

unsigned char USART_init(uint32_t);

void UWriteData(char);

void UWriteData_string(char *);

void UWriteData_Nbytes(uint8_t *, uint8_t);

char UReadData(void);

uint8_t UAvailableData(void);

#endif /* USART_H_ */