/*
 *******************************************************************************
 *                                                                       
 * COPYRIGHT    : (c) 2009 Gary Aylward
 *
 * MODULE NAME	: sys.c
 *                                                                
 * DESCRIPTION	: This module provides the hardware drivers.
 *
 *******************************************************************************
 */
#define SYS_C

#include <avr/io.h>
#include <avr/interrupt.h>
#include <types.h>
#include <uart.h>
#include <menu.h>
#include <sys.h>
#include <fsm.h>

/* definitions for this module */

/* Definitions for timer 1 (timebase) */
#define sys_t1_dMAX_COUNT       (0x61)
#define sys_t1_dPWM_TCCR1       (0x8E)
#define sys_t1_dPWM_GTCCR       (0x60)
#define sys_t1_dTOGGLE_COUNT    (0x30)
#define sys_t1_dTOV1			(0x04)

/* For output ports, DDRxn = 1. For input ports, DDRxn = 0 */
/* For input ports, PORTxn = 1 enables pull-up, PORTxn = 0 disables pull-up */

/* *** THESE VALUES ARE FOR ATtiny45 ONLY *** */
/* PB5 = /RESET */
/* PB4 = I/O (Mode Switch) */
/* PB3 = I/O (Key output) */
/* PB2 = I/O (Spare) */
/* PB1 = I/O (TxD) */
/* PB0 = I/O (RxD) */
#define sys_io_dPORTB_DIR  (0x0A)
#define sys_io_dPORTB_INIT (0x37)

/* global variables for this module */

uint8 sys_t0_vCount = 0;


/* function prototypes */

void sys_fInit (void);
void sys_io_fInit(void);
bool sys_t1_fPoll(void);
void sys_t1_fInit(void);

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : sys_fInit
 *                                                                       
 * DESCRIPTION   : This function will initialise all the objects required 
 *                 for the system.
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
void sys_fInit (void)
{
    sys_io_fInit();   /* Initialise I/O port data direction and values */
    sys_t1_fInit();   /* Initialise timer1 object */
    fsm_fInit();      /* Initialise the state machine */
	uart_fFlushBuffers();	/* Initialise UART */
    uart_fInitReceiver();	/* Initialise UART */
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : sys_io_fInit
 *                                                                       
 * DESCRIPTION   : This function will initialise all the I/O ports on the chip.
 *                 Note that some settings will be overridden by alternate 
 *                 functions, e.g. UART. This function just ensures that 
 *                 everything starts up in a defined manner.
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
void sys_io_fInit (void)
{
    PORTB = sys_io_dPORTB_INIT;
    DDRB = sys_io_dPORTB_DIR;

}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : sys_t1_fInit
 *                                                                       
 * DESCRIPTION   : This function will initialise timer1 to be used as the 
 *                 CW timebase.
 *                 fCLK = 8.0MHz
 *                 prescaler = 8192
 *                 top value = 0x61 (97 decimal)
 *                 timebase = (8192 * (97 + 1))/8000000 = 100ms
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
void sys_t1_fInit(void)
{
    TCNT1 = 0x00;                   /* set initial value */
    TCCR1 = sys_t1_dPWM_TCCR1;      /* set prescaler, disable OC1A */
    GTCCR = sys_t1_dPWM_GTCCR;      /* enable PWM from OC1B */
    OCR1B = sys_t1_dTOGGLE_COUNT;   /* Set duty cycle to 50% */
	OCR1C = sys_t1_dMAX_COUNT;		/* set max count */
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : sys_t1_fPoll
 *                                                                       
 * DESCRIPTION   : This function is the timer1 "interrupt service routine". 
 *                 It actually polls the timer to avoid interrupt clashes
 *                 with the UART receiver.
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : TRUE if timer has expired
 * 
 *------------------------------------------------------------------------------
 */
bool sys_t1_fPoll(void)
{
   
    if(TIFR & sys_t1_dTOV1)         /* if timer has expired */
    {
        TIFR |= sys_t1_dTOV1;       /* clear overflow flag */
        return (true);              /* return with flag set */
    }
    else
    {
        return (false);             /* return with flag cleared */
    }
}


