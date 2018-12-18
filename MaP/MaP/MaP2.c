
#include "MaP1.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>

volatile static uint8_t cdmf[4*number_Slaves] = {0};

volatile static int8_t mts_type;		//Gibt an was der Frame für ein Type ist
volatile static uint16_t mts_send;		// Hilfsvariable für senden des Frame

volatile static uint32_t mts_send_32bit;	// Benötigt um ACD zu senden
volatile static int16_t bitindex_16_Bit;	// Benötigt um ACD zu senden

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
Gesendet werden die Bits in jeder möglichen Frame-Form
Je nach mts_type wird diejendige Frame-Form gesendet
**/
ISR(TIMER3_CAPT_vect)
{	
	//uint16_t data = mts_send;
	//int8_t mask = bitindex;
	//uint32_t data = mts_send_32bit;
	//int16_t mask = bitindex_16_Bit;
	uint32_t data;
	int16_t mask;
	
	if (mts_type == RQTF | mts_type == GCDF)
	{
		data = mts_send;
		mask = bitindex;
	}
	else
	{
		data = mts_send_32bit;
		mask = bitindex_16_Bit;
	}
	
	if(!mask)
	{
		TIMSK3 = 0x00;
		
		if(mts_type == RQTF)
		{
			S_OUT_H;
			ICR3 = AQT_T_VAL;
			OCR3A = AQT_T_VAL;
			
			stm_receive = 0x00;
			bitindex = 0x08;
			f_CNT = 0x00;
			
			TIFR3 = TIFR3 | (1<<OCF3A);
			TIMSK3 = TIMSK3 | (1<<OCIE3A);
			return;
		}
		else
		{
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
	
	//bitindex = mask;
	//bitindex_16_Bit = mask;
	if (mts_type == RQTF | mts_type == GCDF)
	{
		bitindex = mask;
	}
	else
	{
		bitindex_16_Bit = mask;
	}
}

/*
* Interrupt Routine für das Generieren der Wartezeit
* bei einem RQT. Verwendet wird OCF3A
*
*/
ISR(TIMER3_COMPA_vect)
{
	TIMSK3 = 0x00;	// Zurücksetzen der Timer
	ICR3 = MaP_BITRATE;
	
	OCR3CL = MaP_BITRATE;
	
	TIFR3 = TIFR3 | (1<<OCF3C);
	TIMSK3 = TIMSK3 | (1 << OCIE3C);
}

ISR(TIMER3_COMPB_vect)
{
	/*S_OUT_H;
	TIMSK3 = TIMSK3 & ~(1<<OCIE3B);*/
}

/*
* Interrupt Routine für das Empfangen der Daten von den Slaves
* Empfangen des CDMF - Cell Data Main Frame
* 
* Verwendet wird OCF3C
*
* Mithilfe des ISR(TIMER3_COMPA_vect) und dem ISR(TIMER3_COMPC_vect) werden
* die Daten des CDMF empfangen.
* Bei Empfangen des 0CSD wurden alle Daten Empfangen
*/

ISR(TIMER3_COMPC_vect)
{
	int8_t mask = (bitindex-1);
	uint8_t data = stm_receive;
	
	if(mask == -1)
	{
		if(data == 0x00 || f_CNT >= (2*number_Slaves))
		{
			TIMSK3 = 0x00;
		}
		else
		{
			S_OUT_L; 
			
			ICR3 = (4*MaP_BITRATE);
			OCR3A = (4*MaP_BITRATE);
			TIFR3 = TIFR3 | (1<<OCF3A);
			TIMSK3 = TIMSK3 | (1<<OCIE3A);
			
			int8_t i = f_CNT;
			cdmf[i] = data;
			i++;
			mask = 0x08;
			data = 0x00;
			f_CNT = i;
		}
	}
	else
	{
		if(S_IN)
		{
			data = data | (1<<mask);
		}
	}
	stm_receive = data;
	bitindex = mask;
}


/**
Senden der verschiedenen Frames
*/
int8_t sendRQT(uint8_t cdrq)	// Senden eines RQTF Request Frame
{
	//S_OUT_H;
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
	/*
	mts_send = cdrq;
	mts_send = mts_send | (1<<TYPE1) | (1<<TYPE0);		// 11 für GCDF
	*/
	cdrq = cdrq | (1<<TYPE1) | (1<<TYPE0);		// 11 für GCDF
	MC_conversion_8bit(cdrq);
	
	mts_type = GCDF;		// master_to_slave_type = GCDF
	//bitindex = 0x08;
	bitindex = 0x10;
	
	send_mts();
}

void sendACD(uint8_t cdrq, uint8_t adresse)
{
	uint32_t cdrq2 = 0;
	cdrq = cdrq & ~(1<<TYPE1);
	cdrq = cdrq | (1<<TYPE0);
	//uint16_t cdrq1 = cdrq;
	//uint16_t cdrq1 = cdrq & ~((1<<TYPE1) | (1<<TYPE0));	// 01 für ACDF
	//cdrq1 = (cdrq1<<8);				// 8 Stellen nach links, damit 8 Stellen für die Addresse frei sind
	uint16_t cdrq1 = (cdrq<<8);
	cdrq1 = cdrq1 | adresse;		// Adresse an die letzten 8 Bit
	
	cdrq2 = MC_conversion_16bit(cdrq1);
	
	mts_type = ACDF;		// master_to_slave_type = ACDF
	//bitindex_16_Bit = 0x20;
	bitindex_16_Bit = 0b1000000000000000;
	mts_send_32bit = cdrq2;
		
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

uint32_t MC_conversion_16bit(uint16_t cdrq)
{
	uint8_t i = 0;
	uint16_t MASKE = 0b1000000000000000;	//Comparemask - MSB First
	uint32_t cdrq1 = 0;
	
	//////////////////////////////////////
	// 16 Bit to 32 Bit with Manchester Code

	for(i = 0; i < 16; i++)
	{
		if(cdrq & MASKE)
		{
			cdrq1 = cdrq1 << 2;
			cdrq1 = cdrq1 | 2;
		}
		else
		{
			cdrq1 = cdrq1 << 2;
			cdrq1 = cdrq1 | 1;
		}
		
		cdrq = cdrq << 1;
	}
	return cdrq1;
}
