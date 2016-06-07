/*
 *******************************************************************************
 *                                                                       
 * COPYRIGHT    : (c) 2009 Gary Aylward
 *
 * MODULE NAME	: fsm.h
 *                                                                
 * DESCRIPTION	: This module provides the finite state machine.
 *
 *******************************************************************************
 */
#ifndef FSM_H
#define FSM_H

#include <types.h>

typedef enum 
{
    eRESET = 1,
	eGETCHAR = 2,
	eGETELEMENT = 3,
	eDASH1 = 4,
	eDASH2 = 5,
	eDASH3 = 6,
	eSPACE = 7
} fsm_tState;

/* Variables visible to other modules */
extern fsm_tState vCurrentState;
extern uint8 vMsg[];
extern uint8 vTime;
extern uint8 vIndex;
extern uint8 vTotal;

/* Functions visible to other modules */
extern void fsm_fInit (void);
extern void fsm_fExecute (void);

#endif
