/*
 * MaP_S.c
 *
 * Created: 17.10.2018 13:36:36
 * Author : Lukas Frank
 *
 * Header für die Implementierung des MaP am Slave
 */

#ifndef MaP_H
#define MaP_H

//_______ Includes__________________________
#include <avr/io.h>
#include <stdint.h>

//_______ Definition________________________

#ifndef F_CPU
#define F_CPU 8000000UL
#endif


#define M_DDR_Input (DDRB = DDRB & ~(1<<DDB1))
#define M_DDR_Output (DDRB = DDRB | (1<<DDB1))

#define S_DDR_Input (DDRB = DDRB & ~(1<<DDB0))
#define S_DDR_Output (DDB = DDRB | (1<<DDB0))


#define M_IN (PINB & (1<<PINB1))

#define M_OUT_L (PORTB = PORTB & ~(1<<PORTB1))
#define M_OUT_H (PORTB = PORTB | (1<<PORTB1))

#define S_OUT_L (PORTB = PORTB & ~(1<<PORTB0))
#define S_OUT_H (PORTB = PORTB | (1<<PORTB0))

#define MaP_CLK (TIFR & (1<<OCF0A))

#define MaP_BITRATE 104
#define MaP_SYNCRATE 52
#define MaP_DEADTIME 26



#define TYPE1 7	// MSB of CDRQ byte
#define TYPE0 6 // LSB of CDRQ byte

//#define SNC 0x80	// Synchronisations Wert für das CDRQ byte
#define VRQ 0x88	// Voltage Request
#define TRQ 0x82	// Temperature Request
#define SRQ 0x8A	// Status Request

#define SBA 0x08	// Start Balancing
#define EBA 0x02	// End Balancing

//_____________________________________________________//

#define RS0_TIMEOUT 250
#define RSx_TIMEOUT 10


#define CLK_PSC (1<<CS01) ///<defines ref CLKPSC value(prescaler = 8)


#endif //MaP_H