/*
 * Timer.h
 *
 * Created: 31-01-2022 00:59:09
 *  Author: GUNJAN
 */ 


#ifndef TIMER_H_
#define TIMER_H_

void timer0_init(void);
void timer0_start(void);

void timer1_init(void);
void timer1_start(void);

void timer2_init(void);
void timer2_start(void);

long milli(void);


#endif /* TIMER_H_ */