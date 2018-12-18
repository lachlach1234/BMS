
#include "MaP1.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>

volatile static uint8_t cdmf[2*number_Slaves] = {0};

volatile static int8_t mts_type;		//Gibt an was der Frame für ein Type ist
volatile static uint16_t mts_send;		// Hilfsvariable für senden des Frame
volatile static int8_t bitindex;		// Bit-Index wird darin abgespeichert	
volatile static int8_t f_CNT;
volatile static uint8_t stm_receive;	// Empfangsvariable für Daten vom Slave

volatile static uint8_t MaP_Bitrate_refTimer = (MaP_BITRATE -10);	// Bitrate for referenceTimer
volatile static uint8_t LiCOM_S1_dTorE = 0x00;

void send_mts()		//Master to slave
{	
	//set start bit
	S_OUT_L;
	TCNT3 = 0x00;	//Timer reset to 0x00
	ICR3 = MaP_BITRATE;
	//Starts CTC-Mode
	TCCR3B = TCCR3B | (1<<WGM33) | (1<<WGM32) ;	// CTC-Mode and ICR3 as MAX-Value
	TCCR3B = TCCR3B | (1<<CS31);
	//TCCR3B = TCCR3B & ~(1<<CS30) & ~(1<<CS32);	//Clock Prescaler clk/8	
	
	//enable output compare units
	OCR3BL = MaP_Bitrate_refTimer;
	/*if(mts_type != ACDF)
	{
		TIFR3 = TIFR3 | (1<<OCF3B);		//
		TIMSK3 = TIMSK3 | (1<<OCIE3B);	//OCIE3B - Output Compare B Match Interrupt enable
	}*/

	TIFR3 = TIFR3 | (1<<ICF3);		// ICF3 is cleared
	TIMSK3 = TIMSK3 | (1<<ICIE3);	// Input Capture Interrupt is available
	
	sei();
}

/**
Initialisierung des MaP - Managementprotocol
**/
uint8_t* init_MaP()
{
	MCUCR = MCUCR | (1<<JTD);
	MCUCR = MCUCR | (1<<JTD);	//JTAG Schnittstelle deaktivieren

	DDRF = DDRF & ~(1<<DDF7);	//PF7 als Input
	DDRF = DDRF | (1<<DDF6);	//PF6 als Output

	S_OUT_H;					//PF6 on High	

	return cdmf;
}

/**
Interrupt Routine für das versenden der Daten von Master zu Slave
Getriggered wird durch den Timer3 Capture Interrupt
Dazu wird als Flag-Bit ICF3 verwendet
Gesendet werden die bits in jeder möglichen Frame-Form
Je nach mts_type wird diejendige Frame-Form gesendet
**/
ISR(TIMER3_CAPT_vect)
{	
	uint16_t data = mts_send;
	int8_t mask = bitindex;
	
	if(!mask)
	{
		TIMSK3 = 0x00;
		
		if(mts_type == RQTF)
		{
			S_OUT_L;
			ICR3 = AQT_T_VAL;
			OCR3A = AQT_T_VAL;
			
			stm_receive = 0x00;
			bitindex = 0x08;
			f_CNT = 0x00;
			
			TIFR3 = TIFR3 | (1<<OCF3A);
			TIMSK3 = TIMSK3 | (1<<OCIE3A);
			return;
		}
	}
	
	mask--;
		
	if(data & (1<<mask))
	{
		S_OUT_H;
		//_delay_ms(500);
	}
	
	else
	{
		S_OUT_L;
		//_delay_ms(500);
		TIFR3 = TIFR3 | (1<<OCF3B);
		if((data & (1<<(mask-1))))
		{
			TIMSK3 = TIMSK3 | (1<<OCIE3B);
		}
		
		/*if()	// 
		{
			TIMSK3 = (1<<OCIE3B);
		}*/
	}
	
	bitindex = mask;
}

ISR(TIMER3_COMPA_vect)
{
	
}

ISR(TIMER3_COMPB_vect)
{
	/*S_OUT_H;
	TIMSK3 = TIMSK3 & ~(1<<OCIE3B);*/
}

ISR(TIMER3_COMPC_vect)
{
	
}


/**
Senden der verschiedenen Frames
*/
int8_t sendRQT(uint8_t cdrq)	// Senden eines RQTF Request Frame
{
	S_OUT_H;
	/*mts_send = cdrq;
	mts_send = mts_send | (1<<TYPE1);
	mts_send = mts_send & ~(1<<TYPE0);	// 10 für RQTF
	*/
	cdrq = cdrq | (1<<TYPE1);
	cdrq = cdrq & ~(1<<TYPE0);
	MC_conversion_8bit(cdrq);
	
		
	mts_type = RQTF;			// master_to_slave_type = RQTF
	
	//bitindex = 0x08;			
	bitindex = 0x10;
	
	send_mts();
	
	
	//while(MaP_Busy);	//Solange ein Interrupt enable ist im TIMSK3 Register, bleibt man in der Schleife
	
	/*while(TIMSK3 & (1<<OCIE3A));
	while(TIMSK3 & (1<<OCIE3B));
	while(TIMSK3 & (1<<OCIE3C));
	while(TIMSK3 & (1<<TOIE3));
	while(TIMSK3 & (1<<ICIE3));*/
	return(f_CNT+1);	//Was geb ich zurück???
}

void sendGCD(uint8_t cdrq)	// Senden eines Global Command Frame
{
	mts_send = cdrq;
	mts_send = mts_send | (1<<TYPE1) | (1<<TYPE0);		// 11 für GCDF
	
	mts_type = GCDF;		// master_to_slave_type = GCDF
	bitindex = 0x08;
	
	send_mts();
}

void sendACD(uint8_t cdrq, uint8_t address)
{
	cdrq = cdrq & ~(1<<TYPE1);
	cdrq = cdrq | (1<<TYPE0);
	uint16_t cdrq1 = cdrq;
	//uint16_t cdrq1 = cdrq & ~((1<<TYPE1) | (1<<TYPE0));	// 01 für ACDF
	mts_send = (cdrq1<<8);				// 8 Stellen nach links, damit 8 Stellen für die Addresse frei sind
	mts_send = mts_send | address;		// Adresse an die letzten 8 Bit
	
	mts_type = ACDF;		// master_to_slave_type = ACDF
	bitindex = 0x10;
	
	send_mts();
}


void MC_conversion_8bit(uint8_t cdrq1)
{
	uint8_t i = 0;
	uint8_t MASKE = 0b10000000;	//Comparemask - MSB First
	
	//////////////////////////////////////
	// 8 Bit to 16 Bit with Manchester Code

	for(i = 0; i < 8; i++)
	{
		if(cdrq1 & MASKE)
		{
			mts_send = mts_send << 2;
			mts_send = mts_send | 2;
		}
		else
		{
			mts_send = mts_send << 2;
			mts_send = mts_send | 1;
		}
		
		cdrq1 = cdrq1 << 1;
	}
	
}
/*
void MC_conversion_16bit(uint16_t cdrq)
{
	uint8_t i = 0;
	uint8_t MASKE = 0b1000000000000000;	//Comparemask - MSB First
	
	//////////////////////////////////////
	// 8 Bit to 16 Bit with Manchester Code

	for(i = 0; i < 16; i++)
	{
		if(cdrq & MASKE)
		{
			mts_send = mts_send << 2;
			mts_send = mts_send | 2;
		}
		else
		{
			mts_send = mts_send << 2;
			mts_send = mts_send | 1;
		}
		
		cdrq = cdrq << 1;
	}
	
}
*/