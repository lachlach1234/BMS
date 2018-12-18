/*
 * status_LED.c
 *
 * Created: 05.12.2018
 * Author : Lukas Frank
 *
 *
 * This file inlcudes all function, which are needed for the LED
 */ 

#include "status_LED.h"



void init_statusLED()
{
	DDRB = DDRB | (1<<DDB3);
}

void run_status_LED()
{
	PORTB = PORTB | (1<<PORTB3);
}

void toggle_status_LED()
{
	PORTB = PORTB | (1<<PORTB3);
	_delay_ms(500);
	PORTB = PORTB & ~(1<<PORTB3);
	_delay_ms(500);
}

void test_toggle_LED()
{
	PORTB = PORTB ^ (1<<PB3);
}