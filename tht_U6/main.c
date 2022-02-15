/*
 * tht_U6.c
 *
 * Created: 16-01-2022 17:08:04
 * Author : HP
 */ 

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "USART.h"
#include "eeprom.h"
#include "Timer.h"
#include "LCD_16x2.h"

#define INC_KEY					2
#define DEC_KEY					3
#define PROG_KEY				1
#define ENTER_KEY				4
#define KEY_INIT				DDRA &= (~((1<<INC_KEY) | (1<<DEC_KEY) | (1<<PROG_KEY) | (1<<ENTER_KEY)))
#define KEY_PULLUP_INIT			PORTA |= ((1<<INC_KEY) | (1<<DEC_KEY) | (1<<PROG_KEY) | (1<<ENTER_KEY))

#include <avr/eeprom.h>

#define EEPROM_CHECKSUM						7621
#define EEPROM_CHECKSUM_ADD					(uint16_t *)110
#define EEPROM_TEMP_ADD						(uint16_t *)130
#define EEPROM_K_P_ADD						(uint16_t *)150
#define EEPROM_K_I_ADD						(uint16_t *)170
#define EEPROM_K_D_ADD						(uint16_t *)190
#define EEPROM_FLAGDEBUG_ADD				(uint16_t *)210

#define TEMP_LOW							150
#define TEMP_HIGH							450
#define TEMP_DEFAULT						285

#define REC_TEMP_MAX_THRESHOLD				550
#define REC_TEMP_MIN_THRESHOLD				250

#define K_P_LOW								0
#define K_P_HIGH							750
#define K_P_DEFAULT							10

#define K_I_LOW								50
#define K_I_HIGH							500
#define K_I_DEFAULT							0

#define K_D_LOW								0
#define K_D_HIGH							500
#define K_D_DEFAULT							0

#define FLAGDEBUG_DEFAULT					0

#define INTERLOCK_KEY_INIT					DDRB |= (1<<0)
#define SET_INTERLOCK_KEY					PORTB |= (1<<0)
#define RESET_INTERLOCK_KEY					PORTB &= (~(1<<0))

#define RED_LED_INIT						DDRB |= (1<<1)
#define RED_LED_ON							PORTB |= (1<<1)
#define RED_LED_OFF							PORTB &= (~(1<<1))

#define GREEN_LED_INIT						DDRB |= (1<<2)
#define GREEN_LED_ON						PORTB |= (1<<2)
#define GREEN_LED_OFF						PORTB &= (~(1<<2))
#define PID_UPDATE_TIME						50

void callback (void);
void processTempUpdate(void);
void keyEventUpdate(void);

uint8_t flagUpdateTime = 0, operationStatus;

uint16_t setTemp, setKp, setKi, setKd, currTemp, lastTempUpdate, fcntfilter, prevTemp;

uint16_t baud_rate, flagDebugMode = 1, OCR_value_1, OCR_value_2 = 0;

float sumError= 0.00, Interlock_Temp_Range = 2.00;

int main(void)
{
	_delay_ms(100);
	KEY_INIT;
	KEY_PULLUP_INIT;

 	eeprom_init();
	 
	 while(USART_init(115200) == USART_ERROR);
	 
	 LCD_Init();
	 
	 _delay_ms(50);
	 
	 if(flagDebugMode)
	 {
		LCD_String_xy(0,0,"          = ");
		LCD_String_xy(0,0,"Curr Temp");
		LCD_Pos_xy(0,11);
		LCD_float(((float) setTemp/10));
	 }
	 else
	 {
		 LCD_String_xy(0,0,"          = ");
		 LCD_String_xy(0,0,"Curr Temp");
		 LCD_Pos_xy(0,11);
		 LCD_float(((float) setTemp/10));
		 
		 LCD_String_xy(1,0,"          = ");
		 LCD_String_xy(1,0,"Set Temp");
		 LCD_Pos_xy(1,11);
		 LCD_float(((float) setTemp/10));
	 }
	 
	 timer0_init();
	 timer1_init();
	 timer2_init();
	 
	 INTERLOCK_KEY_INIT;
	 RED_LED_INIT;
	 GREEN_LED_INIT;
	 RESET_INTERLOCK_KEY;
	 
	 currTemp = setTemp;
	 
	 long pidUpdateTimeout = milli();
	
    /* Replace with your application code */
    while (1) 
    {
		if(milli() > pidUpdateTimeout + PID_UPDATE_TIME)
		{
			pidUpdateTimeout = milli();
			if(flagUpdateTime)
			{
				flagUpdateTime = 0;
				/*processTempUpdate();*/
				LCD_String_xy(0,15,"''");
			}
			keyEventUpdate();
		}	
    }
}
void processTempUpdate(void)
{
	uint16_t recTemp = lastTempUpdate;

	if(recTemp == 0x3030) // this is error and print it on lcd
	{
		if(flagDebugMode)
		{
			flagDebugMode = 0;
			LCD_String_xy(0,15,"E");
		}
 		return;
 	}

	if((recTemp > REC_TEMP_MAX_THRESHOLD) || (recTemp > REC_TEMP_MIN_THRESHOLD))// this is error and print it on lcd
	{
		if(flagDebugMode)
		{
			flagDebugMode = 0;
			LCD_String_xy(0,15,"R");
		}
		return;
	}

	if((prevTemp != 0) && (((prevTemp -20) > recTemp) || ((prevTemp + 20) < recTemp)))// this is error and print it on lcd
	{
		if(flagDebugMode)
		{
			flagDebugMode = 0;
			LCD_String_xy(0,15,"F");
		}
	
		fcntfilter++;
		
		if(fcntfilter > 10)
		{
			fcntfilter = 0;
			prevTemp = recTemp;
		}
		return;
	}
else
{
	fcntfilter = 0;
}

// // if()
// // {
// // 	print K at 16	
// // }

	prevTemp = recTemp;
	currTemp = recTemp;
	LCD_Pos_xy(1,11);
	LCD_float(((float) currTemp/10));


}

void keyEventUpdate(void)
{
	
}

void displayDebudInfo(float data)
{
	LCD_location(1,1);
	
	if((operationStatus == 1) || (operationStatus == 3))
	{ 
		LCD_Char((OCR0 / 100) % 10 + 0x30);
		LCD_Char((OCR0 / 10) % 10 + 0x30);
		LCD_Char((OCR0 / 1) % 10 + 0x30);
	}
	else
	{
		LCD_Char((OCR1A / 100) % 10 + 0x30);
		LCD_Char((OCR1A / 10) % 10 + 0x30);
		LCD_Char((OCR1A / 1) % 10 + 0x30);
	}
	
	LCD_Char(',');
	LCD_location(1,6);
	LCD_Char('>');
	
	if(operationStatus == 1)
	{
		LCD_Char('F');
		LCD_Char('C');
	}
	
	else if(operationStatus == 2)
	{
		LCD_Char('F');
		LCD_Char('H');
	}
	
	else if(operationStatus == 3)
	{
		LCD_Char('P');
		LCD_Char('C');
	}
	
	else if(operationStatus == 4)
	{
		LCD_Char('P');
		LCD_Char('H');
	}
	
	else if(operationStatus == 5)
	{
		LCD_Char(' ');
		LCD_Char('S');
	}
	
	LCD_location(2,1);
	char tampError[16];
	sprintf(tampError, "%3.4f", (double) sumError);
	LCD_Char(tampError);
}

void pid_Controller(float setPoint, float currentPoint, float Kp, float Ki, float Kd)
{
	float error = 0;

	Ki = Ki * (PID_UPDATE_TIME/1000);
	Kd = Kd / (PID_UPDATE_TIME/1000);

	error = ((float)(setPoint - currentPoint));

	if((error < ((float)Interlock_Temp_Range)) || (error > ((float)-Interlock_Temp_Range)))
	{
		SET_INTERLOCK_KEY;
		RED_LED_OFF;
		GREEN_LED_ON;
	}

	else
	{
		RESET_INTERLOCK_KEY;
		RED_LED_ON;
		GREEN_LED_OFF;
	}

	if(error < (-1))
	{
		timer1_stop();
		OCR_value_2 = 255;
		OCR0 = OCR_value_2;
		timer0_start();
		sumError = 0;
		operationStatus = 1;
	}

	if(error > 1)
	{
		timer0_stop();
		OCR_value_1 = 255;
		OCR1A = OCR_value_1;
		timer1_start();
		sumError = 0;
		operationStatus = 2;

	}
}

void eeprom_init(void)
{
	if((eeprom_read_word(EEPROM_CHECKSUM_ADD)) == EEPROM_CHECKSUM)
	{
		setTemp = eeprom_read_word(EEPROM_TEMP_ADD);
		setKp = eeprom_read_word(EEPROM_K_P_ADD);
		setKi = eeprom_read_word(EEPROM_K_I_ADD);
		setKd = eeprom_read_word(EEPROM_K_D_ADD);
		flagDebugMode = eeprom_read_word(EEPROM_FLAGDEBUG_ADD);

		if((setTemp < TEMP_LOW) || (setTemp > TEMP_HIGH))
		{
			setTemp = TEMP_DEFAULT;
			eeprom_write_word(EEPROM_TEMP_ADD, setTemp);
		}
		
		if((setKp < K_P_LOW) || (setKp > K_P_HIGH))
		{
			setKp = K_P_DEFAULT;
			eeprom_write_word(EEPROM_K_P_ADD, setKp);
		}
		
		if((setKi < K_I_LOW) || (setKi > K_I_HIGH))
		{
			setKi = K_I_DEFAULT;
			eeprom_write_word(EEPROM_K_I_ADD, setKi);
		}
		
		if((setKd < K_D_LOW) || (setKd > K_D_HIGH))
		{
			setKd = K_D_DEFAULT;
			eeprom_write_word(EEPROM_K_D_ADD, setKd);
		}
		
		if((flagDebugMode != 0) && (flagDebugMode != 1))
		{
			flagDebugMode = 0;
			eeprom_write_word(EEPROM_FLAGDEBUG_ADD, flagDebugMode);
		}
		
	}
	
	else
	{
		setTemp = TEMP_DEFAULT;
		setKp = K_P_DEFAULT;
		setKi = K_I_DEFAULT;
		setKd = K_D_DEFAULT;
		flagDebugMode = 0;
		eeprom_write_word(EEPROM_TEMP_ADD, TEMP_DEFAULT);
		eeprom_write_word(EEPROM_K_P_ADD, K_P_DEFAULT);
		eeprom_write_word(EEPROM_K_I_ADD, K_I_DEFAULT);
		eeprom_write_word(EEPROM_K_D_ADD, K_D_DEFAULT);
		eeprom_write_word(EEPROM_FLAGDEBUG_ADD, FLAGDEBUG_DEFAULT);
		
		eeprom_write_word(EEPROM_CHECKSUM_ADD, EEPROM_CHECKSUM);
	}
}

void callback (void)
{
	
}

