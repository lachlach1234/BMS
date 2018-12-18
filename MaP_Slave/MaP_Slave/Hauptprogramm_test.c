/**
 * @file main.c
 * @author Lukas Frank
 * @date 01 12 2018
 * @brief test system for the mangement protocol
 *
 *
 */

//_____ I N C L U D E S ____________________________________________________

#include "MaP_Slave.h"
#include "MaP_Slave.c"

//_____ M A C R O S ________________________________________________________

//_____ D E C L A R A T I O N ______________________________________________

/**
 * @brief initialization routine for the slave system
 *
 * calls all module's init functions
 * @param void
 * @return void
 */
void init_slave(void);

//_____ D E F I N I T I O N ________________________________________________



int main(void)
{
    init_slave();
    uint8_t command;
    uint16_t measured;

    while(1)
    {
            command = mts_trf();
            switch(command)
            {
                case VRQ:
                    measured = measureVolt();	// Klemens
                    stm_trf(measured);
                    break;

                case TRQ:
                    measured = measureTemp();	// Klemens
                    stm_trf(measured);
                    break;

                case SRQ:
                    stm_trf(VERSION);
                    break;

                case SBA:
                    start_balance();
                    break;

                case EBA:
                    end_balance();
                    break;
            }
    }
}

void init_slave(void)
{
   // SYS_CLOCK(NOM_FRQ);

    init_statusLED();
    init_MaP();
	enable_clock();
	init_PCINT1();
	M_DDR_Input;
	M_OUT_H;
	enable_Timer1();
	
   // init_ADC();		// Klemens
    //init_balancing();	// Klemens
}
