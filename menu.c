/*
 *******************************************************************************
 *                                                                       
 * COPYRIGHT    : (c) 2009 Gary Aylward
 *
 * MODULE NAME	: menu.c
 *                                                                
 * DESCRIPTION	: This module contains the menu system and user interface functions
 *
  *******************************************************************************
 */
#define MENU_C

#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <types.h>
#include <uart.h>
#include <sys.h>
#include <menu.h>
#include <fsm.h>

/* definitions for this module */

/* global variables for this module */
uint8 EEMEM vFifteen[] = "15, ";
uint8 EEMEM vThirty[] = "30, ";
uint8 EEMEM vFortyFive[] = "45, ";
uint8 EEMEM vSixty[] = "60, ";

/* function prototypes */
void menu_fExecute(uint8 *);
void menu_fDisplayCurrent (void);
void fTiming (uint8 *);
void fSetMsg (uint8 *);

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : menu_fExecute
 *                                                                       
 * DESCRIPTION   : This routine is called, when a message is received, to execute the
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

void menu_fExecute (uint8 *vMessage)
{
	switch (vMessage[0])
    {
		case 'M' :
		case 'm' :
			fSetMsg (vMessage);
			break;
		case 'T' :
		case 't' :
			fTiming (vMessage);
			break;
		default:
			menu_fDisplayCurrent ();
			break;
    }
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : menu_fDisplayCurrent
 *                                                                       
 * DESCRIPTION   : Display current message and timing settings
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
void menu_fDisplayCurrent (void)
{

	/* Display current message */
	uart_fSendEepromMsg (vMsg);
	uart_fSendMsg("\r");
	/* Display current settings */
	switch (eeprom_read_byte(&vTime))
	{
		case 15:
			uart_fSendEepromMsg(vFifteen);
			break;
		case 30:
			uart_fSendEepromMsg(vThirty);
			break;
		case 45:
			uart_fSendEepromMsg(vFortyFive);
			break;
		case 60:
			uart_fSendEepromMsg(vSixty);
			break;
		default:
			uart_fSendMsg("??,");
			break;
	}
	uart_fTransmitByte(eeprom_read_byte(&vIndex) + 0x30);
	uart_fTransmitByte(',');
	uart_fTransmitByte(eeprom_read_byte(&vTotal) + 0x30);
	uart_fSendMsg("\r ");
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fTiming
 *                                                                       
 * DESCRIPTION   : Menu handler. Setting timing parameters
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
void fTiming (uint8 *vMessage)
{
	bool vValid = true;
	uint8 vTimeTemp = 0;
	uint8 vIndexTemp = 0;
	uint8 vTotalTemp = 0;
	
	if (strlen((char *)vMessage) != 7)
	{
		vValid = false;
	}
	else
	{
		if ((vMessage[1] == '1') && (vMessage[2] == '5'))
		{
			vTimeTemp = 15;
		}
		else if ((vMessage[1] == '3') && (vMessage[2] == '0'))
		{
			vTimeTemp = 30;
		}
		else if ((vMessage[1] == '4') && (vMessage[2] == '5'))
		{
			vTimeTemp = 45;
		}
		else if ((vMessage[1] == '6') && (vMessage[2] == '0'))
		{
			vTimeTemp = 60;
		}
		else
		{
			vValid = false;
		}
		vIndexTemp = vMessage[4] - 0x30;
		vTotalTemp = vMessage[6] - 0x30;
		if ((vTotalTemp > 0) && (vTotalTemp < 10))
		{
			if ((vIndexTemp < 0) || (vIndexTemp > vTotalTemp))
			{
				vValid = false;
			}
		}
		else
		{
			vValid = false;
		}
	}
	if (vValid == true)
	{
		eeprom_write_byte(&vTime, vTimeTemp);
		eeprom_write_byte(&vIndex, vIndexTemp);
		eeprom_write_byte(&vTotal, vTotalTemp);
	}
	else
	{
		uart_fSendMsg("Format: Txx,y,z\r");
	}
	menu_fDisplayCurrent();
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fSetMsg
 *                                                                       
 * DESCRIPTION   : FSM handler. Set new message
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
void fSetMsg (uint8 *vMessage)
{
	uint8 vStrIndex = 0;
	
	/* If message has been entered */
	if ((strlen((char *)vMessage) > 1) && (strlen((char *)vMessage) <= 16))
	{
		vStrIndex = 1;
		do
		{
			/* Copy message into EEPROM, including null terminator */
			eeprom_write_byte(&vMsg[vStrIndex - 1], vMessage[vStrIndex]);
		}
		while (vMessage[vStrIndex++] != '\0');
	}
	else
	{
		uart_fSendMsg("\rMessage must be 1-15 characters\r");
	}
	menu_fDisplayCurrent();
}

