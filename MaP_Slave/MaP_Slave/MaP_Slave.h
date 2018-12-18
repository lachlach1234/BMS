/*
 * MaP_Slave.h
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

//_______ Makros__________________________
#define CLOCK_RATE OCR0A ///<reference clock rate
#define RESET_CLOCK() (TCNT0 = 0x00) ///<resets ref CLK's counter val to 0x00
#define STOP_CLOCK() (TCCR0B = 0x00) ///<stops ref CLK
#define START_CLOCK() (TCCR0B = CLK_PSC) ///<starts clock with given prescaler CLK_PSC
#define WAIT_CLOCK(V) while(TCNT0 < (V)) ///<waits until CLK passes given TIME V
#define CLEAR_CLOCK() (TIFR |= (1<<OCF0A)) ///<clears interrupt flag

#define SYS_CLOCK(PSC) CLKPR = 0x80; CLKPR = (PSC) ///< sets system-CLKPSC


// Makros für das 3-Wire-Interface
#define USI_WI3_CONF ((1<<USIWM0) | (1<<USICS0)) ///<configures USI's Three-Wire-Mode
#define USI_COUNTER_VALUE (USISR & 0x0F) ///<USI counter value makro
#define USI_RESET_COUNTER() (USISR = 0x00) ///<resets USI counter(counter = 0x00)
#define USI_SET_COUNTER(V) (USISR = (V & 0xF)) ///<gets current USI counter value
#define USI_DETACH_CLK() (USICR &= ~(1<<USICS0)) ///<detaches connection of TIMER0 and USI's shift register
#define USI_DISABLE() (USICR = 0x00) ///<disables whole USI



//_______ Definition________________________

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#define TRUE 1
#define FALSE 0

#define M_DDR_Input (DDRB = DDRB & ~(1<<DDB1))
#define M_DDR_Output (DDRB = DDRB | (1<<DDB1))

#define S_DDR_Input (DDRB = DDRB & ~(1<<DDB0))
#define S_DDR_Output (DDRB = DDRB | (1<<DDB0))


#define M_IN (PINB & (1<<PINB1))

#define M_OUT_L (PORTB = PORTB & ~(1<<PORTB1))
#define M_OUT_H (PORTB = PORTB | (1<<PORTB1))

#define S_OUT_L (PORTB = PORTB & ~(1<<PORTB0))
#define S_OUT_H (PORTB = PORTB | (1<<PORTB0))

#define MaP_CLK (TIFR & (1<<OCF0A))

#define MaP_BITRATE 52
#define MaP_SYNCRATE 52
#define MaP_DEADTIME 26


#define number_Slaves 16	//Maximale Anzahl der Slaves


#define TYPE1 7	// MSB of CDRQ byte
#define TYPE0 6 // LSB of CDRQ byte

#define ACDF 0x00	// Maske für ACDF - Adressed Command Frame
#define GCDF 0x01	// Maske für GCDF - Global Command Frame
#define RQTF 0x04	// Maske für RQTF - Request Frame

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






void init_PCINT1(void);

void enable_Timer1(void);

void disable_Timer1(void);

void enable_clock(void);

void disable_clock(void);

void init_MaP(void);

void sleep_until_start(void);

uint8_t start_detect(void);

uint8_t mts_trf(void);

void stm_trf(uint16_t data);

uint8_t wait_for_RS(uint8_t rs_timeout);

uint8_t wi3_transfer_CSD(uint8_t data);

void wi3_transfer_CSD_LOW(uint8_t data);

int8_t check_parity(uint16_t x);


#endif //MaP_H