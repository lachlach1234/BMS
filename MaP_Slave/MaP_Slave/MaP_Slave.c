/*
 * MaP_S.c
 *
 * Created: 17.10.2018 13:36:36
 * Author : Lukas Frank
 *
 * Implementierung des MaP am Slave
 */

//____Includes______________________
#include "MaP_Slave.h"

#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <avr/interrupt.h>
#include <avr/io.h>

static uint8_t MaP_bitrate;
static uint8_t MaP_syncrate;
static uint8_t MaP_deadtime;


/*	PCINT1 zum Erkennen der Flanke
 *	Bei positiver Flanke und bei negativer Flanke
*/
void init_PCINT1()
{
	//M_DDR_Input;
	//M_OUT_H;	// Auskommentieren
	//S_DDR_Output;
	//S_OUT_L;
	
	GIMSK = GIMSK | (1<<PCIE);		//PCINT ist global enable
	PCMSK = PCMSK | (1<<PCINT1);	// PCINT1 is enable
	
	sei();
}

/*
*	Timer 1 Compare A für das Empfangen
*	TOP Vlaue auf 50 --> 20kHz
*	In diesem Fall wird der Top Value höher gewählt, weil der Timer nur verwendet wird um zum Zählen
*	und die falschen Flanken beim Empfangen zu ignorieren.
*/
void enable_Timer1()
{
	//TCCR0A = (1<<WGM01);	// CTC Mode mit OCR0A als TOP value
	//TIMSK = TIMSK | (1<<OCIE1A);	// COMPA wird aktiviert
	TCCR1 = TCCR1 | (1<<CS12);	// clock Prescaler = 8 (Synchronous mode)
	TCNT1 = 0x00;			// Zurücksetzen des Zählerstands des Timer
	OCR1A = 200;	// TOP Value auf 50 --> 20kHz
	TIFR = TIFR |(1<<OCF1A);	// Interrupt Flag Bit wird zurückgestetzt
}

void disable_Timer1()
{
	//TIMSK = TIMSK & ~(1<<OCIE1A);	// COMPA wird deaktiviert
	TCCR1 = TCCR1 & ~(1<<CS12);	// clock Prescaler = 8 (Synchronous mode)
	TCCR1 = 0x00;
}


void enable_clock()
{
	TCCR0A = (1<<WGM01);	// CTC Mode mit OCR0A als TOP value
	TCCR0B = 0x00;			// reference CLK wird gestoppt
	TCNT0 = 0x00;			// Zurücksetzen des Zählerstands des Timer
	OCR0A = MaP_BITRATE;	// TOP Value auf 52 --> 20kHz
	TIFR = TIFR |(1<<OCF0A);	// Interrupt Flag Bit wird zurückgestetzt
}

void disable_clock()
{
	TCCR0A = 0x00;
	TCCR0B = 0x00;
	
	TIFR = TIFR | (1<<OCIE0A) | (1<<OCIE0B);
}

void init_MaP()
{
	//Default Values für Bitrate setzen
	MaP_bitrate = MaP_BITRATE;
	MaP_syncrate = MaP_SYNCRATE;
	MaP_deadtime = MaP_DEADTIME;
	
	//Slave-Pin auf Output und High
	S_DDR_Output;
	S_OUT_H;
	
	//Aktivieren der Start-Detection PCINT1
	//PCMSK = PCMSK | (1<<PCINT1);
	
	//Sleep Mode...
	
	//sei();
}

void sleep_until_start()
{
	//Master-Pin auf Input und Low
	//wartet auf das Startbit
	M_DDR_Input;
	M_OUT_L;
	
	//Slave-Pin auf High (zur sicherheit)
	S_OUT_H;
	
	// Aktivieren des Start detection Interrupt
	GIMSK = GIMSK | (1<<PCIE);
	
	
	
	// Deaktivieren der Start Detection
	GIMSK = GIMSK & ~(1<<PCIE);
}
/*
uint8_t start_detect()
{
	
}
*/
uint8_t mts_trf()
{
	uint8_t cmdf = 0x00;
	//uint8_t i = ??
	uint8_t i = 0;
	
	while(i)
	{
		if(TIFR & (1<<OCF0A))
		{
			i--;
			if(M_IN)
			{
				S_OUT_H;
				cmdf = cmdf | (1<<i);
			}
			else
			{
				S_OUT_L;
				
				TIFR = TIFR | (1<<OCF0A);	//löschen des Interrupt Flag
				
			}
		}
	}
	
	if(cmdf & (1<<TYPE1))	// KEIN adressed Command
	{
		while(!MaP_CLK);
		
		S_OUT_L;
		TIFR = TIFR | (1<<OCF0A);	//löschen des Interrupt Flag
		
		return cmdf;
	}
	
	uint8_t a = 0x00;
	uint8_t add_1 = 1;
	
	while (1)
	{
		if(MaP_CLK)
		{
			i--;
			if(!M_IN)
			{
				if (add_1)
				{
					S_OUT_H;
					a = a | (1<<i);
					add_1 = 0x00;
				}
				else
				{
					S_OUT_L;
				}
			}
			else
			{
				if(!add_1)
				{
					S_OUT_H;
					a = a | (1<<i);
				}
				else
				{
					S_OUT_L;
				}
			}
			TIFR = TIFR | (1<<OCF0A);
		}
	}
	//while();
	
	S_OUT_L;
	TIFR = TIFR | (1<<OCF0A);
	
	//    return a == 0xFF ? cmd : 0xFF;

}

void stm_trf(uint16_t data)
{
	uint16_t temp = data;
	
	uint8_t dataH = ((temp >> 8) | 0x80);
	uint8_t dataL = ((temp & 0xff) | 0x01);
	
	// Check Parity und setzen des bit wenn notwendig
	dataH = dataH & ~0x40;
	//dataH = dataH (check_parity(temp) ? 0x40 : 0x00); //??????????????????
	
	// Warten auf das Nächste Reverse Start Bit (RS)
	if(wait_for_RS(RS0_TIMEOUT))
	{
		return;
	}
	
	// Senden der Highbyte Daten
	uint8_t data_stm = wi3_transfer_CSD(dataH);
	
	// Warten auf das Reverse Start Bit vorbereiten
	M_DDR_Input;
	M_OUT_L;
	
	uint8_t i = 2*number_Slaves;
	
	// Transmission von den Bytes des untergeordneten Slaves
	while (data_stm && i)
	{
		// Warten auf das nächste Reverse Start Bit
		if(wait_for_RS(RSx_TIMEOUT))
		{
			return;
		}
		data_stm = wi3_transfer_CSD(data_stm);
		M_DDR_Input;
		M_OUT_L;
		i--;
	}
	
	// Keine Daten mehr vom vorherigen Slave --> High zum Slave
	S_DDR_Output;
	S_OUT_H;
	
	//
	// Warten auf das nächste Reverse Start Bit
	if(wait_for_RS(RSx_TIMEOUT))
	{
		return;
	}
	
	// Transmission vom LOW-Byte
	wi3_transfer_CSD_LOW(dataL);
	
	// Warten auf das Reverse Start Bit vorbereiten
	M_DDR_Input;
	M_OUT_L;
	
	// Warten auf das nächste Reverse Start Bit
	if(wait_for_RS(RSx_TIMEOUT))
	{
		return;
	}
	
	// Transmission von 0x00 (0CSD) als Beendugn des S->M Transfer
	wi3_transfer_CSD_LOW(0x00);
	
	M_DDR_Input;
	M_OUT_L;	
}






/*
uint8_t wait_for_RS(uint8_t rs_timeout)
{
	uint8_t bit_counter = 0x00; 
	
	// Configuration und Starten des Timer (Timer0 = MaP_CLK)
	TCCR0B = 0x00;	// Stoppen des reference CLK
	OCR0A = MaP_BITRATE;	// Setzen der Reference Clockrate
	TCNT0 = 0x00;	// Zurücksetzen des Timer
	TIFR = TIFR | (1<<OCF0A);	// Löschen des Interrupt Flags
	TCCR0B = CLK_PSC;	// Starten der Clock mit dem Prescaler CLK_PSC
	
	// Warten auf das Reverse Start Bit
	while (!M_IN)
	{
		if(MaP_CLK)
		{
			bit_counter++;
			
			
			// Falls ein Timeout zu lange dauert
			if (bit_counter >= rs_timeout)
			{
				return -1;
			}
			
			// Timer Interrupt Flag wird gelcleared
			TIFR = TIFR | (1<<OCF0A);
		}
	}
	
	// Wenn das Reverse Start Bit empfangen ist --> return 0
	return 0;	
}
*/



/*

uint8_t wi3_transfer_CSD(uint8_t data)
{
	// Reference Timer Stoppen
	TCCR0B = 0x00;
	
	uint8_t received_data;
	
	// USI in 3-Wire Mode; USIWM0 für 3 Wire Mode; USICS0 für Timer/Counter0 Compare Match
	USICR = USICR | (1<<USIWM0) | (1<<USICS0); 
	
	// Daten in das Schieberegister laden
	USIDR = data;
	
	//Master-Pin als Output für den Transfer
	M_DDR_Output;
	
	//?????????????????????
	// USI als Bit counter auf 0x08 gesetzt -8 bit
	USISR = (0x08 & 0xf);
	//?????????????????????
	
	OCR0A = MaP_BITRATE;	// reference clockrate
	TCNT0 = 0x00;	//Reset des CLK
	// Starten des Senden
	TCCR0B = CLK_PSC;
	
	// Generieren des Reverse Start Bit zum übergeordneten Slave (1/ Takt delay)
	S_DDR_Output;
	//while(TCNT0 < ......);
	S_OUT_H;
	
	// Verzögerung von 1/4 Takt
	//while(TCNT0 < ......);
	// Nach Verzögerung auf Eingang
	
	S_DDR_Input;
	S_OUT_L;
	
	//Warten bis das letzte Bit gesendet wurde
	while (!(USISR & (1<<USIOIF)));
	
	
	// Trennt die Verbindung vom TIMER0 und dem USI Shift Register
	// Keine Clock gesetzt
	USICR = USICR & ~(1<<USICS0);
	USISR = USISR | (1<<USIOIF);
	
	TIFR = TIFR | (1<<OCF0A);	// Clock Clear
	
	//	Muss bevor dem letzten Bit gesetzt werden
	 S_OUT_L;
	 S_DDR_Output;
	
	// warten bis das letzte Bit Komplett gesendet wurde
	while(!MaP_CLK); // 1 takt warten, damit der Slave genug Zeit hat
	
	received_data = USIDR;	// Empfangene Daten werden gelesen
	
	// Beenden und Zurücksetzen des gesamten USI
	USICR = 0x00;
	
	// Return der empfangenen CSD Daten
	return received_data;
}

*/





void wi3_transfer_CSD_LOW(uint8_t data)
{
	// Reference Timer Stoppen
	TCCR0B = 0x00;
	
	//uint8_t received_data;
	
	// USI in 3-Wire Mode; USIWM0 für 3 Wire Mode; USICS0 für Timer/Counter0 Compare Match
	USICR = USICR | (1<<USIWM0) | (1<<USICS0);
	
	// Daten in das Schieberegister laden
	USIDR = data;
	
	//Master-Pin als Output für den Transfer
	M_DDR_Output;
	
	//?????????????????????
	// USI als Bit counter auf 0x08 gesetzt -8 bit
	USISR = (0x08 & 0xf);
	//?????????????????????
	
	OCR0A = MaP_BITRATE;	// reference clockrate
	TCNT0 = 0x00;	//Reset des CLK
	// Starten des Senden
	TCCR0B = CLK_PSC;
	
	//Warten bis das letzte Bit gesendet wurde
	while (!(USISR & (1<<USIOIF)));
	
	// Trennt die Verbindung vom TIMER0 und dem USI Shift Register
	// Keine Clock gesetzt
	USICR = USICR & ~(1<<USICS0);
	USISR = USISR | (1<<USIOIF);
	
	TIFR = TIFR | (1<<OCF0A);	// Clock Clear
	
	// Warten bis das letzte Bit gesendet wurde
	while(!MaP_CLK);
	
	// Beenden und Zurücksetzen des gesamten USI
	USICR = 0x00;
}

int8_t check_parity(uint16_t x)
{
	int8_t parity = 0;
/*
	while(x)
	{
		par = !par;
		x = x & (x - 1);
	}*/
	return parity;
}


