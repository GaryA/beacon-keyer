/*
 *******************************************************************************
 *                                                                       
 * COPYRIGHT    : (c) 2006 Gary Aylward
 *
 * MODULE NAME	: sys.h
 *                                                                
 * DESCRIPTION	: This module provides the hardware-related stuff.
 *
 *******************************************************************************
 */
#ifndef SYS_H
#define SYS_H

#include <avr/io.h>
#include <types.h>

/* Port B bit definitions */
#define sys_io_dMODE  (0x10)  /* Mode switch */
#define sys_io_dKEY   (0x08)  /* Keyer output bit */
#define sys_io_dTXD   (0x02)  /* TxD output bit */
#define sys_io_dRXD   (0x01)  /* RxD input bit */

#define sys_io_dBEACON  (0x10)	/* Beacon mode */
#define sys_io_dPROGRAM (0x00)	/* Program mode */

/* Functions visible to other modules */
extern void sys_fInit (void);
extern bool sys_t1_fPoll(void);
extern void sys_t1_fInit(void);

#endif
