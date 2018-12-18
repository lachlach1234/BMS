/*
 * MaP.c
 *
 * Created: 01.10.2018 21:22:49
 * Author : Lukas Frank
 */ 

#define F_CPU 16000000UL



#define TRUE 1
#define FALSE 0



#include "MaP2.c"

volatile unsigned char bf = FALSE;		//Flagbit
volatile unsigned char sb = TRUE;		//Startbit
volatile unsigned char send = FALSE;	//Sendbit

volatile uint16_t bit_received = 0x00;				//Received Bit
volatile uint16_t bit_main = 0x00;				//Received Bit in Main
volatile uint16_t counter = 0x00;		//BitCounter for receiving



uint8_t i;
static uint8_t *test; 

int main(void)
{	
 CLKPR = 0x80;
 CLKPR = 0x00;						//Set clock to 16MHz
	
	// Initialisierung für LEDs
	DDRB = 0xff;
	PORTB = 0x00;
	DDRD = DDRD | (1<<DDD2);
	
	
    // Initialisierung für das Empfangen am PD0 durch INT0
	DDRD = DDRD &~ (1<<DDD0);
	PORTD = PORTD | (1<<PORTD0);
	
	TCNT3 = 0x00;	//Timer reset to 0x00
	ICR3 = MaP_BITRATE;	// MaP_BITRATE 208
	//Starts CTC-Mode
	TCCR3B = TCCR3B | (1<<WGM33) | (1<<WGM32) ;	// CTC-Mode and ICR3 as MAX-Value
	TCCR3B = TCCR3B | (1<<CS31);
	//TCCR3B = TCCR3B & ~(1<<CS30) & ~(1<<CS32);	//Clock Prescaler clk/8

	//TIMSK3 = TIMSK3 | (1<<TOIE3); //Timer_3 Overflow Interrupt freigeben
		
	 EICRA = EICRA & ~(1<<ISC01) | (1<<ISC00); 	// any Flanke am INT0
	 EIMSK = EIMSK | (1<<INT0);					// indivi. Interruptfreig. Für INT0
	 sei();										// globale Interruptfreig.
	 
	
	test = init_MaP();
	uint8_t cdrq = 0x55;
	uint8_t adress;
	//cdrq = 0b01011001;
	//adress = 0b11001011;
	cdrq = 0b11111111;
	adress = 0b11111111;
	MCUCR = MCUCR | (1<<JTD);
	MCUCR = MCUCR | (1<<JTD);	//JTAG Schnittstelle deaktivieren

	
	DDRF = DDRF | (1<<DDF6);	//PF6 als Output
	DDRF = 0xff;
	
	//sendRQT(cdrq);
	
	sb = TRUE;
	//ICR3 = 800;	// Zum Empfangen gehört der Top Value geändert
    while (1) 
    {		
		
		if(bit_main == 0b1001101010011001)
		{
			PORTD = PORTD ^ (1<<PORTD2);
			bit_main = 0x00;
		}
		if(bit_main == 0b10101010101)
		{
			PORTD = PORTD ^ (1<<PORTD2);
			bit_main = 0x00;
		}
		if(bit_main == 0b10111010)
		{
			PORTD = PORTD ^ (1<<PORTD2);
			bit_main = 0x00;
		}
		
		if(bf == TRUE)
		{
			bf = FALSE;
			sb = TRUE;
			//ICR3 = 800;	// Zum Empfangen gehört der Top Value geändert
			//PORTB = bit_main;
			bit_main = 0x00;
		}
		/*
		_delay_ms(500);
		cdrq = 0b10100101;
		sendRQT(cdrq);
		_delay_ms(500);
		cdrq = 0b10111011;
		sendRQT(cdrq);*/
		
		/*
		_delay_ms(500);
		cdrq = 0b10100101;
		sendRQT(cdrq);
		_delay_ms(500);
		cdrq = 0b10111011;
		sendGCD(cdrq);*/
		
		sendACD(cdrq,adress);
		_delay_ms(500);
		sendGCD(01010101);
		_delay_ms(500);
		//sendGCD(cdrq);
		//i = sendRQT(cdrq);
		//sendGCD(cdrq);
		/*S_OUT_H;
		_delay_ms(500);
		S_OUT_L;
		_delay_ms(500);*/
			
    }
}

ISR(INT0_vect)
{
	if(sb == TRUE && send == FALSE)
	{
		if(PIND & (1<<PIND0))
		{
			//ignorieren, Ruhephase am Ende eines Empfangenen Byte
		}
		else
		{
			PORTD = PORTD ^ (1<<PORTD2);
			sb = FALSE;
			TCNT3 = 0;
		}
	}
	else if(sb == FALSE && send == FALSE)
	{
		//if(TCNT3 < ((MaP_BITRATE*2)-70))			//etwas weniger als 112, damit er die Flanke wirklich erkennt.
		//if(TCNT3 < 150)
		//if(TCNT3 < 350)		// Richtiger Wert für eine Chiprate von 208
		if(TCNT3 < ((MaP_BITRATE*2)-70))	// Abfragung damit nur die richtige Flanke erkannt wird. Jede nicht richtige wird ausgelassen
		{
			// Die Flanke darf nicht genommen werden
		}
		else
		{
			TCNT3 = 0;
			
			if(PIND & (1<<PIND0))		//PD0 abfragen, positive Flanke
			{
				bit_received = bit_received << 1;			//schieben um eins nach links
				bit_received = bit_received | 0;
			}
			else						//PD0 abfragen, negative Flanke
			{
				bit_received = bit_received << 1;
				bit_received = bit_received | 1;			//schreibe eins hinein
			}
			
			counter++;
			
			
			if(counter >= 8)
			{
				counter = 0;
				bf = TRUE;
				bit_main = bit_received;
				bit_received = 0x00;
				PORTB = bit_main;
			}
		}
	}
	
}

/*
ISR(INT0_vect)
{
	if(PIND & (1<<PIND0))		//PD0 abfragen, positive Flanke
	{
		bit_received = bit_received << 1;			//schieben um eins nach links
		bit_received = bit_received | 1;
	}
	else						//PD0 abfragen, negative Flanke
	{
		bit_received = bit_received << 1;
		bit_received = bit_received | 0;			//schreibe eins hinein
	}
	
	counter++;
	
	if(counter >= 12)
	{
		counter = 0;
		bf = TRUE;
		bit_main = bit_received;
		bit_received = 0x00;
	}
}
*/