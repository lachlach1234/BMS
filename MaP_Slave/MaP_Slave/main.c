/*
 * MaP_Slave.c
 *
 * Created: 07.11.2018 08:07:59
 * Author : Lukas Frank
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>

#include "MaP_Slave.h"
#include "MaP_Slave.c"
#include "status_LED.c"
#include "status_LED.h"


//void init_PCINT1();

//void enable_Timer1();

void send_test_data(uint8_t data);
void stm_test_trf(uint16_t data);
void mts_test_trf(uint16_t data);
void MC_conversion_8bit(uint8_t cmd_send);


volatile unsigned char bf = FALSE;		//Flagbit
volatile unsigned char sb = TRUE;		//Startbit
volatile unsigned char send = FALSE;	//Sendbit

volatile uint16_t bit_received = 0x00;				//Received Bit
volatile uint16_t bit_main = 0x00;				//Received Bit in Main
volatile uint16_t counter = 0x00;		//BitCounter for receiving

volatile static uint16_t mts_send;		// Hilfsvariable für senden des Frame

volatile static int8_t bitindex;		// Bit-Index wird darin abgespeichert

volatile static int8_t mts_type;		//Gibt an was der Frame für ein Type ist



int main(void)
{
	CLKPR = 0x80;
	CLKPR = 0x00;
	
	init_MaP();
	init_statusLED();
	//enable_clock();
	//TCCR0B = TCCR0B | (1<<CS01);	// clock Prescaler = 8
	//init_PCINT1();
	
	M_DDR_Input;
	M_OUT_L;
	//M_OUT_H;
	
	//enable_Timer1();
	
	//M_OUT_H;
	//enable_clock();
	
		
	init_PCINT1();
	enable_Timer1();
	
	//uint8_t test = 0b10110110;
	//uint16_t test = 0b1011001010111001;
	//uint16_t test2 = 0b1001100110101011;
	/*GIMSK = GIMSK & ~(1<<PCIE);
	M_DDR_Output;
	M_OUT_H;*/
	
	mts_send = 0b10110110;
	bitindex = 0x10;
	mts_type = RQTF;
	
	
	//M_DDR_Output;
	sei ();
	
	S_DDR_Output;
	S_OUT_L;
	
    while (1) 
    {	
		/*
		PORTB = PORTB ^ (1<<PB3);
		//mts_send = 0b1011011010110110;
		bit_received = 0b10100101;
		MC_conversion_8bit(bit_received);
		bitindex = 0x10;
		mts_test_trf(bit_received);
		_delay_ms(500);
		PORTB = PORTB ^ (1<<PB3);
		//mts_send = 0b1001101110011011;
		bit_received = 0b10111011;
		MC_conversion_8bit(bit_received);
		bitindex = 0x10;
		mts_test_trf(bit_received);
		_delay_ms(500);*/
		//stm_test_trf(test);
		//send_test_data(test);
		//M_OUT_L;
		//_delay_ms(1000);*/
		/*
		if(bit_main == 0b10100101)
		{
			PORTB = PORTB ^ (1<<PB3);
			bit_main = 0x00;
		}
		else if(bit_main == 0b10111011)
		{
			PORTB = PORTB ^ (1<<PB3);
			bit_main = 0x00;
		}
		*/
		
		if(bf == TRUE)
		{	
			//PORTB = PORTB ^ (1<<PB3);
			disable_Timer1();
			//enable_clock();
			GIMSK = GIMSK & ~(1<<PCIE);		// Deaktivieren des Pin Change Interrupts
			
			if(bit_main == 0b10100101)
			{
				PORTB = PORTB ^ (1<<PB3);
				MC_conversion_8bit(0b10100101);
				bitindex = 0x10;
				mts_test_trf(mts_send);
				bit_main = 0x00;
			}
			else if(bit_main == 0b10111011)
			{
				PORTB = PORTB ^ (1<<PB3);
				MC_conversion_8bit(0b10111011);
				bitindex = 0x10;
				mts_test_trf(mts_send);
				bit_main = 0x00;
			}
			bf = FALSE;
			sb = TRUE;
			//PORTB = bit_main;
			bit_main = 0x00;
			//disable_clock();
			enable_Timer1();
			GIMSK = GIMSK | (1<<PCIE);	// Aktivieren des PIN Change Interrupts
		}		
    }
}

void mts_test_trf(uint16_t data)
{
	//PORTB = PORTB ^ (1<<PB3);
	//set start bit
	//S_DDR_Output;
	//S_OUT_L;
	TCNT0 = 0x00;	//Timer reset to 0x00
	
	TIMSK = TIMSK | (1<<OCIE0A);	// COMPA wird aktiviert
	
	TCCR0A = (1<<WGM01);	// CTC Mode mit OCR0A als TOP value
	TCCR0B = TCCR0B | (1<<CS01);	// clock Prescaler = 8
	TCNT0 = 0x00;			// Zurücksetzen des Zählerstands des Timer
	OCR0A = MaP_BITRATE;	// TOP Value auf 52 --> 20kHz
	TIFR = TIFR |(1<<OCF0A);	// Interrupt Flag Bit wird zurückgestetzt
	TCNT0 = 0x00;
	//S_OUT_L;
	sei();
}

ISR(TIMER0_COMPA_vect)
{	
	//PORTB = PORTB ^ (1<<PB3);
	uint16_t data = mts_send;
	int8_t mask = bitindex;
			
	if(!mask)
	{
		TIMSK = TIMSK & ~(1<<OCIE0A);	// COMPA wird deaktiviert
		TCCR0B = TCCR0B & ~(1<<CS01);
		
		if(mts_type == RQTF)
		{
			S_OUT_H;
			return;
		}
		else
		{
			S_OUT_L;
			return;
		}
	}
	
	mask--;
		
	if(data & (1<<mask))
	{
		S_OUT_H;
	}
	
	else
	{
		S_OUT_L;	// hier liegt der fehler vom langen Startbit
	}
	bitindex = mask;
}






/*
* Funktion für das Senden von 8Bit mithilfe des
* USI des ATtiny im 3-Wire Mode
* Senden von 8Bit mit dem internen Schieberegister
* Richtung Slave -> Master
*/

void send_test_data(uint8_t data)
{	
	//TCCR0B = 0x00;	// Stop Clock
	STOP_CLOCK();
	
	// USI in 3-Wire Mode; USIWM0 für 3 Wire Mode; USICS0 für Timer/Counter0 Compare Match
	//USICR = USICR | (1<<USIWM0) | (1<<USICS0);
	 USICR = USI_WI3_CONF;
	
	// Daten in das Schieberegister laden
	USIDR = data;
	
	//Master-Pin als Output für den Transfer
	M_DDR_Output;
	
	//?????????????????????
	// USI als Bit counter auf 0x08 gesetzt -8 bit
	//USISR = (0x08 & 0xf);
	//?????????????????????
	USI_SET_COUNTER(0x08);
	
	CLOCK_RATE = MaP_BITRATE;	// reference clockrate
	RESET_CLOCK();
	//TCNT0 = 0x00;	//Reset des CLK
	// Starten des Senden
	//TCCR0B = CLK_PSC;	// Prescaler = 8, Aktivieren des Timer
	START_CLOCK();
	
	USISR = USISR | (1<<USIOIF);
	
	//Warten bis das letzte Bit gesendet wurde
	while (!(USISR & (1<<USIOIF)));
	//PORTB = PORTB ^ (1<<PB3);
	
	// Trennt die Verbindung vom TIMER0 und dem USI Shift Register
	// Keine Clock gesetzt
	//USICR = USICR & ~(1<<USICS0);
	 USI_DETACH_CLK();
	//USISR = USISR | (1<<USIOIF);
	
	//TIFR = TIFR | (1<<OCF0A);	// Clock Clear
	CLEAR_CLOCK();
	
	// Beenden und Zurücksetzen des gesamten USI
	//USICR = 0x00;
	USI_DISABLE();
}

void stm_test_trf(uint16_t data)
{
	uint16_t temp = data;
	
	uint8_t dataH = ((temp >> 8) | 0x80);
	uint8_t dataL = (temp & 0xff);
	
	// Senden der Highbyte Daten
	send_test_data(dataH);
	send_test_data(dataL);
}

void MC_conversion_8bit(uint8_t cmd_received)
{
	uint8_t i = 0;
	uint8_t MASKE = 0b10000000;	//Comparemask - MSB First
	
	//////////////////////////////////////
	// 8 Bit to 16 Bit with Manchester Code

	for(i = 0; i < 8; i++)
	{
		if(cmd_received & MASKE)
		{
			mts_send = mts_send << 2;
			mts_send = mts_send | 2;
		}
		else
		{
			mts_send = mts_send << 2;
			mts_send = mts_send | 1;
		}
		
		cmd_received = cmd_received << 1;
	}	
}

ISR(PCINT0_vect)
{	
		if(sb == TRUE && send == FALSE)
		{
			if(M_IN)	//PINB & (1<<PINB1)
			{
				//ignorieren , Ruhephase am Ende eines Empfangenen Byte
			}
			else
			{
				//PORTB = PORTB ^ (1<<PB2);
				sb = FALSE;
				TCNT1 = 0;
			}
		}
		else if(sb == FALSE && send == FALSE)
		{	
			if(TCNT1 < ((MaP_BITRATE*2)-25))	// Abfragung damit nur die richtige Flanke erkannt wird. Jede nicht richtige wird ausgelassen
			{
				// Die Flanke darf nicht genommen werden
				//PORTB = PORTB ^ (1<<PB3);
			}
			else
			{
				//PORTB = PORTB ^ (1<<PB3);
				TCNT1 = 0;
				
				if(M_IN)		//PINB & (1<<PINB1),	PD1 abfragen, positive Flanke
				{
					bit_received = bit_received << 1;			//schieben um eins nach links
					bit_received = bit_received | 0;
				}
				else						//PD0 abfragen, negative Flanke
				{
					//PORTB = PORTB ^ (1<<PB3);
					bit_received = bit_received << 1;
					bit_received = bit_received | 1;			//schreibe eins hinein
				}
				
				counter++;
								
				if(counter >= 8)
				{
					/*		if(bit_received == 0b10100101)
							{
								PORTB = PORTB ^ (1<<PB3);
							}
							else if(bit_received == 0b10111011)
							{
								PORTB = PORTB ^ (1<<PB3);
							}*/
					counter = 0;
					bf = TRUE;
					bit_main = bit_received;
					bit_received = 0x00;
					//PORTB = bit_main;
				}
			}
		}	
}