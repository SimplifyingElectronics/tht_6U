/*
 * Timer.c
 *
 * Created: 31-01-2022 00:58:51
 *  Author: GUNJAN
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#include "Timer.h"
#include "LCD_16x2.h"

extern void callback (void);

long fcnt_millis;

void timer0_init(void)
{
	DDRB |= (1<<3);
	TCCR0 |= ((1<<WGM01) | (1<<WGM00) | (1<<CS01) | (1<<CS00));
	TCNT0 = 0;
	
	/* Turn on interrupt */
	sei();
}
void timer0_start(void)
{
	/* Clear OC0 on compare match, set OC0 at BOTTOM, (non-inverting mode) */
/*	TCCR0 |= (1<<COM01);*/
	TCCR0 |= (1<<COM01);
	
	/*  */
	TCNT0 = 0;
	
}
void timer0_stop(void)
{
	/* Clear OC0 on compare match, set OC0 at BOTTOM, (non-inverting mode) */
	TCCR0 &= (~((1<<COM00) | (1<<COM01)));
	
	/*  */
	PORTB &= (~(1<<3));	
}

void timer1_init(void)
{
	DDRD |= (1<<5);
	ICR1 = 255;
	TCCR1A &= (~((1<<WGM10) | (1<<CS00) | (1<<COM1A0) | (1<<COM1A1)));
	TCCR1B &= (~(1<<CS12));
	TCCR1A |= (1<<WGM11);
	
	TCCR1B |= ((1<<WGM13) | (1<<WGM12) | (1<<CS11) | (1<<CS10));
	
	OCR1A = 0;
	TCNT1 = 0;
	
	/* Turn on interrupt */
	sei();
}
void timer1_start(void)
{
	/* Clear OC0 on compare match, set OC0 at BOTTOM, (non-inverting mode) */
	TCCR1A |= (1<<COM1A1);
	
	/*  */
	TCNT1 = 0;
	
}
void timer1_stop(void)
{
	/* Clear OC0 on compare match, set OC0 at BOTTOM, (non-inverting mode) */
	TCCR1A &= (~((1<<COM1A0) | (1<<COM1A1)));
	
	/*  */
	PORTD &= (~(1<<5));
}

void timer2_init(void)
{
	/* check for ctc and 500 Hz 128 prescaler */
	/* TIMSK OCIE2 fntmilli = 0  */
	TCCR2 |= ((1<<WGM21) | (1<<CS22) | (1<<CS20));
	OCR2 = 124;
/*	OCR2 = 255;*/
	TIMSK |= (1<<OCIE2);
	TCNT2 = 0;
	fcnt_millis = 0;

	/* Turn on interrupt */
	sei();
}
void timer2_start(void)
{
	/* Clear OC0 on compare match, set OC0 at BOTTOM, (non-inverting mode) */
	TCCR1A |= (1<<COM1A1);
	
	/* clear the counter */
	TCNT1 = 0;
	
}
void timer2_stop(void)
{
	/* Clear OC0 on compare match, set OC0 at BOTTOM, (non-inverting mode) */
	TCCR0 &= (~((1<<COM1A0) | (1<<COM1A1)));
	
	/* make pwm as input */
	DDRD &= (~(1<<5));
}

ISR(TIMER2_COMP_vect)
{
	fcnt_millis++;
	callback();
}
long milli(void)
{
	return fcnt_millis;
	
}

