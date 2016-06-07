/*
 *******************************************************************************
 *                                                                       
 * COPYRIGHT    : (c) 2009 Gary Aylward
 *
 * MODULE NAME	: fsm.c
 *                                                                
 * DESCRIPTION	: This module defines the finite-state-machine 
 *                for a morse code beacon keyer.
 *
 *******************************************************************************
 */
#define FSM_C

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <types.h>
#include <sys.h>
#include <fsm.h>

/* definitions for this module */

/* global variables for this module */

fsm_tState vCurrentState, vRestoreState;
uint16 vCycleCount, vStopCount;
uint16 vTimeoutTime, vTimeoutCount;
static uint8 vCharIndex = 0;
static uint8 vCodedChar;
uint8 EEMEM vMsg[] = "... DE G0XAN/P ";	/* Message to transmit */
uint8 EEMEM vTime = 30;					/* Time to transmit for */
uint8 EEMEM vIndex = 1;					/* Time-slot to transmit in (starts at #1) */
uint8 EEMEM vTotal = 1;					/* Total number of time slots (transmitters) */
uint8 EEMEM vCode[64] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
						0xFF, 0xFF, 0xFF, 0xFF, 0x6A, 0x29, 0x3F, 0x3E, 0x3C, 0x38,
						0x30, 0x20, 0x21, 0x23, 0x27, 0x2F, 0xFF, 0xFF, 0xFF, 0x31,
						0xFF, 0x4C, 0xFF, 0x06, 0x11, 0x15, 0x09, 0x02, 0x14, 0x0B,
						0x10, 0x04, 0x1E, 0x0D, 0x12, 0x07, 0x05, 0x0F, 0x16, 0x1B,
						0x0A, 0x08, 0x03, 0x0C, 0x18, 0x0E, 0x19, 0x1D, 0x13, 0xFF,
						0xFF, 0xFF, 0xFF, 0xFF};

/* function prototypes */
void fReset (void);
void fGetMsgChar (void);
void fGetMsgElement (void);
void fDash1 (void);
void fDash2 (void);
void fDash3 (void);
void fSpace (void);
void fWaitPaddle (void);
void fDash1A (void);
void fDash2A (void);
void fDash3A (void);
void fsm_fInit (void);
void fsm_fExecute (void);

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fReset
 *                                                                       
 * DESCRIPTION   : FSM handler. Reset state, get start of message, determine mode
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */

void fReset ()
{
	PORTB &= ~(sys_io_dKEY);
	/* Point to first char in message */
	vCharIndex = 0;
	vCurrentState = eGETCHAR;
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fGetMsgChar
 *                                                                       
 * DESCRIPTION   : FSM handler. Get next character from message string
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */

void fGetMsgChar ()
{
	uint8 vChar;
	
	PORTB &= ~(sys_io_dKEY);

	vChar = eeprom_read_byte(&vMsg[vCharIndex++]);
	
	if (vChar == '\0')
	{
		vCurrentState = eRESET;
	}
	else
	{
		if ((vChar < 0x20) || (vChar > 0x5F))
		{
			vCodedChar = 0xFF;
		}
		else
		{
			vCodedChar = eeprom_read_byte(&(vCode[vChar - 0x20]));
		}
		vCurrentState = eGETELEMENT;
	}

}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fGetMsgElement
 *                                                                       
 * DESCRIPTION   : FSM handler. Get element of coded character 
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */

void fGetMsgElement ()
{
	uint8 vElement;
	
	PORTB &= ~(sys_io_dKEY);
	
	vElement = vCodedChar & 0x01;
	
	if ((vCodedChar == 0x01) || (vCodedChar == 0xFF))
	{
		vCurrentState = eGETCHAR;
	}
	else
	{
		vCodedChar = vCodedChar >> 1;
		if (vElement == 0x01)
		{
			/* Output dash */
			vCurrentState = eDASH1;
		}
		else
		{
			/* Output dot */
			vCurrentState = eDASH3;
		}
	}
}
/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fDash1
 *                                                                       
 * DESCRIPTION   : FSM handler.  Output first period of a dash
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */

void fDash1 ()
{
	PORTB |= sys_io_dKEY;
	vCurrentState = eDASH2;
}
/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fDash2
 *                                                                       
 * DESCRIPTION   : FSM handler.  Output second period of a dash
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */

void fDash2 ()
{
	PORTB |= sys_io_dKEY;
	vCurrentState = eDASH3;
}
/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fDash3
 *                                                                       
 * DESCRIPTION   : FSM handler.  Output third period of a dash (or a dot)
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */

void fDash3 ()
{
	PORTB |= sys_io_dKEY;
	vCurrentState = eGETELEMENT;
}
/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fSpace
 *                                                                       
 * DESCRIPTION   : FSM handler.  Padding state to make spaces the right length
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */

void fSpace ()
{
	PORTB &= ~(sys_io_dKEY);
	vCurrentState = eGETELEMENT;
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fsm_fInit
 *                                                                       
 * DESCRIPTION   : Set up the initial state of the FSM.
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */

void fsm_fInit ()
{
	vCurrentState = eRESET;
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fsm_fExecute
 *                                                                       
 * DESCRIPTION   : This routine is called, once per cycle, to execute the
 *		           current state.
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
 
void fsm_fExecute ()
{
	switch (vCurrentState)
    {
		case eRESET :
			fReset ();
			break;
		case eGETCHAR :
			fGetMsgChar ();
			break;
		case eGETELEMENT :
			fGetMsgElement ();
			break;
		case eDASH1 :
			fDash1 ();
			break;
		case eDASH2 :
			fDash2 ();
			break;
		case eDASH3 :
			fDash3 ();
			break;
		case eSPACE :
			fSpace ();
			break;
		default:
			fsm_fInit();
			break;
    }
}
