/*
 *******************************************************************************
 *                                                                       
 * COPYRIGHT    : (c) 2009 Gary Aylward
 *
 * MODULE NAME	: keyer.c
 *                                                                
 * DESCRIPTION	: This module contains the main() function.
 *
 * Revision History:
 * P1   Original                                                        18/3/09
  *******************************************************************************
 */
/*******************************************************************************
 * Morse beacon keyer project
 *
 * This program runs on an ATtiny45 processor.
 * It provides all the control for a morse code beacon keyer.
 * It also provides for download of keyer messages and setting timing
 * via UART (RS-232 or USB).
 *
 *******************************************************************************
 */

#define MAIN_C

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <types.h>
#include <sys.h>
#include <menu.h>
#include <fsm.h>
#include <uart.h>

/* Configuration fuse definitions */

FUSES =
{
	.low = (FUSE_CKDIV8 & FUSE_SUT0 & FUSE_CKSEL3 & FUSE_CKSEL2 & FUSE_CKSEL0),
	.high = (FUSE_SPIEN & FUSE_EESAVE & FUSE_BODLEVEL1 & FUSE_BODLEVEL0),
	.extended = EFUSE_DEFAULT
};


/* definitions for this module */

/* global variables for this module */

/* function prototypes */
int main(void);
uint8 fGetMode (void);

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : main
 *                                                                       
 * DESCRIPTION   : This is the main function for the system. 
 *                 It calls the system initialisation then enables interrupts
 *                 then runs background tasks. The main functions are run
 *                 by event or timer interrupts.
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
int main (void)
{
	static volatile uint8 vMode = 0;
	static uint16 vCounter = 0;
	static uint16 vStart = 0;
	static uint16 vStop = 0;
	static uint16 vReset = 0;
	uint8 vRxMessage[8];
	
    sys_fInit();      /* Initialise system */
	vRxMessage[0] = '\0';
	
    sei();   /* enable global interrupts */

    for (;;)    /* Run main loop continuously */
    {
		if((PINB & sys_io_dBEACON) == sys_io_dBEACON)
		{
			/* Beacon Mode */
			if (vMode != 1)
			{
				fsm_fInit();
				vStart = (uint16)(eeprom_read_byte(&vIndex) - 1)
						* (uint16)(eeprom_read_byte(&vTime)) * 10;
				vStop = (uint16)(eeprom_read_byte(&vIndex))
						* (uint16)(eeprom_read_byte(&vTime)) * 10;
				vReset = (uint16)(eeprom_read_byte(&vTotal))
						* (uint16)(eeprom_read_byte(&vTime)) * 10;
				vMode = 1;
			}
			else
			{
				if (sys_t1_fPoll()) /* if timer has expired, do stuff ... */
				{
					if (vCounter >= vStart)
					{
						if (vCounter < vStop)
						{
							fsm_fExecute(); /* run state machine */
							vCounter++;
						}
						else
						{
							/* Force keyer output OFF */
							PORTB &= ~(sys_io_dKEY);
							if (vCounter >= vReset)
							{
								vCounter = 0;
							}
							else
							{
								vCounter++;
							}
						}
					}
					else
					{
						/* Force keyer output OFF */
						PORTB &= ~(sys_io_dKEY);
						vCounter++;
					}
				}
			}
		}
		else
		{
			/* Program Mode */
			/* Force keyer output OFF */
			PORTB &= ~(sys_io_dKEY);
			vMode = 2;
			if (uart_fGetMessage(vRxMessage))
			{
				menu_fExecute(vRxMessage);
			}
		}
	}
}

