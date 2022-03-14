/*
 * tht_U6.c
 *
 * Created: 16-01-2022 17:08:04
 * Author : HP
 */ 

#include <avr/io.h>
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
#define EEPROM_DEBUG_ADD					(uint16_t *)210

#define TEMP_LOW							150
#define TEMP_HIGH							450
#define TEMP_DEFAULT						285

#define CUR_TEMP_HIGH_THRESHOLD				550
#define CUR_TEMP_LOW_THRESHOLD				250

#define K_P_LOW								10
#define K_P_HIGH							7500
#define K_P_DEFAULT							1500

#define K_I_LOW								0
#define K_I_HIGH							500
#define K_I_DEFAULT							0

#define K_D_LOW								0
#define K_D_HIGH							500
#define K_D_DEFAULT							5

uint16_t setKp = 1500;
uint16_t setKi = 0;
uint16_t setKd = 5;

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

/*#define IS_KEY_INC_PRESSED					((PINA & (1<<INC_KEY) == 0))*/
// #define IS_KEY_DEC_PRESSED
// #define IS_KEY_PROG_PRESSED
// #define IS_KEY_ENTER_PRESSED
// #define IS_KEY_INC_RELEASED
// #define IS_KEY_DEC_RELEASED
// #define IS_KEY_PROG_RELEASED
// #define IS_KEY_ENTER_RELEASED

void callback (void);
void processTempUpdate(void);
void keyEventUpdate(void);
void displayUserInfo(uint16_t);
void displayDebugInfo(float);

uint8_t operationStatus;

uint16_t setTemp = 285, currTemp, fcntfilter, prevTemp;

uint16_t baud_rate, OCR_value_1, OCR_value_2 = 0;

volatile uint16_t flagDebugMode = 0;
volatile uint16_t lastUpdatedTemp = 0;
volatile uint16_t flagTempUpdate = 0;

float sumError= 0.00, Interlock_Temp_Range = 2.00;
float lastcurrentPoint = 0.00;

int main(void)
{
	_delay_ms(50);
	KEY_INIT;
	KEY_PULLUP_INIT;

	/* EEPROM write */
	eeprom_init();
 
	 while(USART_init(115200) == USART_ERROR);
  	 
	 LCD_Init();
	 
	 
	 if(flagDebugMode)
	 {
		 LCD_location(1,1);
		 LCD_write_string("         = ");
		 LCD_showvalue(setTemp);
		 LCD_location(2,1);
		 LCD_write_string("         = ");
		 LCD_showvalue(setTemp);
	 }
	 else
	 {
		LCD_location(1,1);
		LCD_write_string("         =");
		LCD_location(1,1);
		LCD_write_string("Cur Temp");
		LCD_location(1,12);
		LCD_showvalue(((float) setTemp));
		LCD_location(2,1);
		LCD_write_string("         =");
		LCD_location(2,1);
		LCD_write_string("Set Temp");
		LCD_location(2,12);
		LCD_showvalue(((float) setTemp));
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
			if(flagTempUpdate)
			{
				flagTempUpdate = 0;
				processTempUpdate();
				LCD_location(1,16);
				LCD_write(' ');
			}
		}
// 		keyEventUpdate();	
   }
}
void processTempUpdate(void)
{
	lastUpdatedTemp = 310;
	prevTemp = 280;
	uint16_t recTempData = lastUpdatedTemp;

	if(recTempData == 0x3030) // this is error and print it on lcd
	{
		if(flagDebugMode)
		{
			LCD_location(1,16);
			LCD_write('E');
		}
 		return;
 	}

	if((recTempData > CUR_TEMP_HIGH_THRESHOLD) || (recTempData < CUR_TEMP_LOW_THRESHOLD))// this is error and print it on lcd
	{
		if(flagDebugMode)
		{
			LCD_location(1,16);
			LCD_write('R');
		}
		return;
	}

	if((prevTemp != 0) && (((prevTemp -20) > recTempData) || ((prevTemp + 20) < recTempData)))// this is error and print it on lcd
	{
		if(flagDebugMode)
		{
			LCD_location(1,16);
			LCD_write('F');
		}
	
		fcntfilter++;
		
		if(fcntfilter > 10)
		{
			fcntfilter = 0;
			prevTemp = recTempData;
		}
		return;
	}
else
{
	fcntfilter = 0;
}

	if(flagDebugMode)
	{
		LCD_location(1,16);
		LCD_write('K');
	}

	prevTemp = recTempData;
	currTemp = recTempData;
	LCD_location(2,12);
	LCD_showvalue(((float) currTemp));

}
// 
// void keyEventUpdate(void)
// {
// 	
// }
// 
void displayDebugInfo(float data)
{
	LCD_location(1,1);
	
	if((operationStatus == 1) || (operationStatus == 3))
	{ 
		LCD_write((OCR0 / 100) % 10 + 0x30);
		LCD_write((OCR0 / 10) % 10 + 0x30);
		LCD_write((OCR0 / 1) % 10 + 0x30);
	}
	else
	{
		LCD_write((OCR1A / 100) % 10 + 0x30);
		LCD_write((OCR1A / 10) % 10 + 0x30);
		LCD_write((OCR1A / 1) % 10 + 0x30);
	}
	
	LCD_write(',');
	LCD_location(1,6);
	LCD_write('>');
	
	if(operationStatus == 1)
	{
		LCD_write('F');
		LCD_write('C');
	}
	
	else if(operationStatus == 2)
	{
		LCD_write('F');
		LCD_write('H');
	}
	
	else if(operationStatus == 3)
	{
		LCD_write('P');
		LCD_write('C');
	}
	
	else if(operationStatus == 4)
	{
		LCD_write('P');
		LCD_write('H');
	}
	
	else if(operationStatus == 5)
	{
		LCD_write(' ');
		LCD_write('S');
	}
	
	LCD_location(2,1);
	char tempError[10];
	sprintf(tempError, "%3.4f", (double) sumError);
	LCD_write_string(tempError);
}
 
// float pid_Controller(float setPoint, float currentPoint, float Kp, float Ki, float Kd)
// {
// 	float error = 0;
// 
// 	Ki = Ki * (PID_UPDATE_TIME/1000);
// 	Kd = Kd / (PID_UPDATE_TIME/1000);
// 
// 	error = ((float)(setPoint - currentPoint));
// 
// 	if((error < ((float)Interlock_Temp_Range)) && (error > ((float)-Interlock_Temp_Range)))
// 	{
// 		SET_INTERLOCK_KEY;
// 		RED_LED_OFF;
// 		GREEN_LED_ON;
// 	}
// 
// 	else
// 	{
// 		RESET_INTERLOCK_KEY;
// 		RED_LED_ON;
// 		GREEN_LED_OFF;
// 	}
// 
// 	if(error < (-1))
// 	{
// 		timer1_stop();
// 		OCR_value_2 = 255;
// 		OCR0 = OCR_value_2;
// 		timer0_start();
// 		sumError = 0;
// 		operationStatus = 1;
// 	}
// 
// 	if(error > 1)
// 	{
// 		timer0_stop();
// 		OCR_value_1 = 255;
// 		OCR1A = OCR_value_1;
// 		timer1_start();
// 		sumError = 0;
// 		operationStatus = 2;
// 
// 	}
// 	else
// 	{
// 		float absError = error;
// 		if (absError < 0)
// 		absError *= -1;
// 		sumError += (Ki * absError);
// 		if(sumError > ((2*Kp)/10))
// 		sumError = ((2*Kp)/10);
// 		else if(sumError < 0)
// 		sumError = 0;
// 		if(Ki == 0)
// 		sumError =0;
// 
// 		double output;
// 		float pointDiff = currentPoint - lastcurrentPoint;
// 		if(pointDiff < 0)
// 		pointDiff *= -1;
// 		output = (Kp * absError + sumError - (Ki * pointDiff));
// 		if(output > 255)
// 		output = 255;
// 		else if(output < -255)
// 		output = -255;
// 
// 		output = abs(output);
// 		
// 		if(error > 0)
// 		{
// 			timer0_stop();
// 			OCR_value_1 = output;
// 			OCR1A = OCR_value_1;
// 			timer1_start();
// 			operationStatus = 4;		
// 		}
// 		
// 		else if(error < 0)
// 		{
// 			timer1_stop();
// 			OCR_value_2 = output;
// 			OCR0 = OCR_value_2;
// 			timer0_start();
// 			operationStatus = 3;
// 		}
// 		
// 		else
// 		{
// 			
// 			OCR_value_2 = 0;
// 			OCR0 = OCR_value_2;
// 			OCR_value_1 = 0;
// 			OCR1A = OCR_value_1;
// 			timer0_stop();
// 			timer1_stop();
// 			sumError = 0;
// 			operationStatus = 5;
// 		}
// 	}
// }
void displayUserInfo(uint16_t data)
{
	if(data>999)
	{
		LCD_Char(((data / 1000) % 10) + 0x30);
	}
	
	LCD_write(((data / 100) % 10) + 0x30);
	LCD_write(((data / 10) % 10) + 0x30);
	LCD_Char('.');
	LCD_write(((data / 1) % 10) + 0x30);
	LCD_Char(' ');
}
// void keyEventUpdate(void)
// {
// 	if((IS_INC_KEY_PRESSED) && (IS_DEC_KEY_PRESSED))
// 	{
// 		timer0_stop();
// 		timer1_stop();
// 		LCD_Clear();
// 		_delay_ms(50);
// 		
// 		while((!IS_INC_KEY_RELEASED) && (!IS_INC_KEY_RELEASED));
// 		
// 		LCD_location(2,1);
// 		LCD_String("V - ");
// 		LCD_Char(((VERSION / 100) % 10) + 0x30);
// 		LCD_Char(".");
// 		LCD_Char(((VERSION / 100) % 10) + 0x30);
// 		LCD_Char(".");
// 		LCD_Char(((VERSION / 100) % 10) + 0x30);
// 		LCD_location(1,1);
// 		LCD_String("Gain P = ");
// 		displayUserInfo(setKp);
// 	}
// 	
// 	uint8_t fcntSpeedInc = 0, fcntSpeedDec = 0;
// 	
// 	while(IS_KEY_PROGRAM_RELEASED)
// 	{
// 		if(IS_KEY_INC_PRESSED)
// 		{
// 			fcntSpeedDec = 0;
// 			setKp = setKp + 1 + fcntSpeedInc++;
// 			
// 			if(setKp > K_P_HIGH)
// 				setKp = K_P_HIGH;
// 			
// 			LCD_location(1,10);
// 			displayUserInfo(setKp);
// 			_delay_ms(250);
// 		}
// 		
// 		else if(IS_KEY_DEC_PRESSED)
// 		{
// 			fcntSpeedInc = 0;
// 			if(setKp > (K_P_LOW + 1 + fcntSpeedInc))
// 			setKp = setKp - 1 - fcntSpeedInc++;
// 			
// 			else
// 			setKp = K_P_LOW;
// 			
// 			LCD_location(1,10);
// 			displayUserInfo(setKp);
// 			_delay_ms(250);
// 		}
// 		
// 		if(IS_KEY_INC_PRESSED)
// 		fcntSpeedInc = 0;
// 		
// 		else if(IS_KEY_DEC_PRESSED)
// 		fcntSpeedDec = 0;
// 		
// 		_delay_ms(250);
// 		
// 		while(IS_KEY_PROGRAM_PRESSED);
// 	}
// }
// 
void eeprom_init(void)
{
// 	if((eeprom_read_word(EEPROM_CHECKSUM_ADD)) == EEPROM_CHECKSUM)
// 	{
// 		setTemp = eeprom_read_word(EEPROM_TEMP_ADD);
// 		setKp = eeprom_read_word(EEPROM_K_P_ADD);
// 		setKi = eeprom_read_word(EEPROM_K_I_ADD);
// 		setKd = eeprom_read_word(EEPROM_K_D_ADD);
// 		flagDebugMode = eeprom_read_word(EEPROM_DEBUG_ADD);
// 
// 		if((setTemp < TEMP_LOW) || (setTemp > TEMP_HIGH))
// 		{
// 			setTemp = TEMP_DEFAULT;
// 			eeprom_write_word(EEPROM_TEMP_ADD, setTemp);
// 		}
// 		
// 		if((setKp < K_P_LOW) || (setKp > K_P_HIGH))
// 		{
// 			setKp = K_P_DEFAULT;
// 			eeprom_write_word(EEPROM_K_P_ADD, setKp);
// 		}
// 		
// 		if((setKi < K_I_LOW) || (setKi > K_I_HIGH))
// 		{
// 			setKi = K_I_DEFAULT;
// 			eeprom_write_word(EEPROM_K_I_ADD, setKi);
// 		}
// 		
// 		if((setKd < K_D_LOW) || (setKd > K_D_HIGH))
// 		{
// 			setKd = K_D_DEFAULT;
// 			eeprom_write_word(EEPROM_K_D_ADD, setKd);
// 		}
// 		
// 		if((flagDebugMode != 0) && (flagDebugMode != 1))
// 		{
// 			flagDebugMode = 0;
// 			eeprom_write_word(EEPROM_DEBUG_ADD, flagDebugMode);
// 		}
// 		
// 	}
// 	
// 	else
// 	{
		setTemp = TEMP_DEFAULT;
		eeprom_write_word(EEPROM_TEMP_ADD, setTemp);
		
		setKp = K_P_DEFAULT;
		eeprom_write_word(EEPROM_K_P_ADD, setKp);
		
		setKi = K_I_DEFAULT;
		eeprom_write_word(EEPROM_K_I_ADD, setKi);
		
		setKd = K_D_DEFAULT;
		eeprom_write_word(EEPROM_K_D_ADD, setKd);
		
		flagDebugMode = 0;		
		eeprom_write_word(EEPROM_DEBUG_ADD, flagDebugMode);
		
		eeprom_write_word(EEPROM_CHECKSUM_ADD, EEPROM_CHECKSUM);
// 	}
}
// 
void callback (void)
{
	
}

