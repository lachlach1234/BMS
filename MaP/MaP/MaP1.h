/*
*	Header für die Implementierung des MaP
*	
*/

#ifndef MaP_TASK_H
#define MaP_TASK_H

#include <avr/interrupt.h>
#include <avr/io.h>

/*
Definition der Slaveanzahl
*/
#define number_Slaves 16	//Maximale Anzahl der Slaves

/**
*Definition of Input and Output of the Master PF6 and PF7
* PF6
**/
//#define S_OUT_H (PORTF = PORTF & ~(1<<PORTF6))	//output '1' on S_OUT
//#define S_OUT_L (PORTF = PORTF | (1<<PORTF6))	//output '0' on S_OUT
#define S_OUT_H (PORTF = PORTF | (1<<PORTF6))
#define S_OUT_L (PORTF = PORTF & ~(1<<PORTF6))


#define S_IN (PINF & (1<<PINF7))			//Input of S_IN
/** **/

//#define MaP_BITRATE 208		///< default value for bitrate of 10kHz
#define MaP_BITRATE 104		///< default value for chiprate of 20kHz
#define MaP_SYNCRATE 104	///< default value for syncrate = bitrate/2

#define ACDF 0x00	// Maske für ACDF - Adressed Command Frame
#define GCDF 0x01	// Maske für GCDF - Global Command Frame
//#define SNCF 0x02	// Maske für SNCF - Synchronisier Frame
#define RQTF 0x04	// Maske für RQTF - Request Frame

#define TYPE1 7	// MSB of CDRQ byte
#define TYPE0 6 // LSB of CDRQ byte

//#define SNC 0x80	// Synchronisations Wert für das CDRQ byte
#define VRQ 0x88	// Voltage Request
#define TRQ 0x82	// Temperature Request
#define SRQ 0x8A	// Status Request

#define SBA 0x08	// Start Balancing
#define EBA 0x02	// End Balancing

#define AQT_T_VAL (number_Slaves*MaP_BITRATE) ///< AQT0 = aquisiton delay for slaves(see LiMP's spec)

/*
*	MaP_Busy - beinhaltet ob im TIMSK3 Register ein Bit gesetzt ist(Ein Interrupt enable ist)
*	ICIE3 - Bit 5 - Input Capture Interrupt is enable
*	OCIE3A - Bit 1 - Output Compare A Match Interrupt enable
*	OCIE3B - Bit 2 - Output Compare B Match Interrupt enable
*	OCIE3C - Bit 3 - Output Compare C Match Interrupt enable
*	TOIE3 - Bit 0 - Timer Counter Overflow Interrupt enable
*/
#define MaP_Busy (TIMSK3 & ((1<<ICIE3) | (1<<OCIE3A) | (1<<OCIE3B) | (1<<OCIE3C) | (1<<TOIE3)))





/*
*	Funktionen mit denen die Kommunikation für den Master
*	in beide M->S und S->M Richtungen möglich ist
*/

/*
*	Konfigurations Funktion für das Senden eines 
*	GCDF Global Command Frame zu den Slaves
*/
void sendGCD(uint8_t cdrq);

/*
*	Konfigurations Funktion für das Senden eines
*	RQTF Request Frame zu den Slaves
*/
int8_t sendRQT(uint8_t cdrq);

/*
*	Konfigurations Funktion für das Senden eines
*	ACDF Adressed Command Frame zu den Slaves
*/
void sendACD(uint8_t cdrq, uint8_t address);

/*
*	Initialisierungs Funktion für die Kommunikation
*	mit den Slaves
*	Initialisierung des MaP Management Protokol
*/
uint8_t* init_MaP();

/*
*	Funktion für das Senden der Daten
*	vom Master zu den Slaves
*/
void send_mts();




/*
*	Funktion für die Erstellung der
*	gesendeten Daten mit Manchester Codierung
*	8 Bit zu 16 Bit
*/
void MC_conversion_8bit(uint8_t cdrq);


/*
*	Funktion für die Erstellung der
*	gesendeten Daten mit Manchester Codierung
*	8 Bit zu 16 Bit
*/
uint32_t MC_conversion_16bit(uint16_t cdrq);


#endif //MaP_TASK_H