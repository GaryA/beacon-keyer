/*****************************************************************************
*
* Copyright (C) 2003 Atmel Corporation
*
* File              : USI_UART.c
* Compiler          : IAR EWAAVR 2.28a
* Created           : 18.07.2002 by JLL
* Modified          : 02-10-2003 by LTA
*
* Support mail      : avr@atmel.com
*
* Supported devices : ATtiny26
*
* Application Note  : AVR307 - Half duplex UART using the USI Interface
*
* Description       : Functions for USI_UART_receiver and USI_UART_transmitter.
*                     Uses Pin Change Interrupt to detect incomming signals.
*
* Modified for AVR-GCC compiler and ATtiny45, GRA, 2009
*
****************************************************************************/
    
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <types.h>
#include <uart.h>
#include <sys.h>


//********** USI UART Defines **********//

#define DATA_BITS                 8
#define START_BIT                 1
#define STOP_BIT                  1
#define HALF_FRAME                5

#define USI_COUNTER_MAX_COUNT     16
#define USI_COUNTER_SEED_TRANSMIT (USI_COUNTER_MAX_COUNT - HALF_FRAME) // = 11
#define INTERRUPT_STARTUP_DELAY   (0x11 / TIMER_PRESCALER) // = 0x02
#define TIMER0_SEED               (256 - ( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ))
					// = 104
#if ( (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) > (256 - INTERRUPT_STARTUP_DELAY) )
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 1/2) )
    #define USI_COUNTER_SEED_RECEIVE  ( USI_COUNTER_MAX_COUNT - (START_BIT + DATA_BITS) )
#else
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) )
								// = 28
    #define USI_COUNTER_SEED_RECEIVE  (USI_COUNTER_MAX_COUNT - DATA_BITS)
									// = 8
#endif

#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 ) // = 3
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
    #error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 ) // = 3
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
    #error TX buffer size is not a power of 2
#endif

/* General defines */
#define TRUE                      1
#define FALSE                     0

//********** Static Variables **********//

//register uint8 uart_vTxData asm("r15");   // Tells the compiler to store the byte to be transmitted in registry.
static volatile uint8 uart_vTxData; // Make TxData volatile so it is always read correctly
static uint8 uart_vRxBuffer[UART_RX_BUFFER_SIZE];  // UART buffers. Size is definable in the header file.
static volatile uint8 uart_vRxHead;
static volatile uint8 uart_vRxTail;
static uint8 uart_vTxBuffer[UART_TX_BUFFER_SIZE];
static volatile uint8 uart_vTxHead;
static volatile uint8 uart_vTxTail;
uint8 uart_vRxIndex = 0;

static volatile union USI_UART_status                           // Status byte holding flags.
{
    uint8 status;
    struct
    {
        uint8 ongoing_Transmission_From_Buffer:1;
        uint8 ongoing_Transmission_Of_Package:1;
        uint8 ongoing_Reception_Of_Package:1;
        uint8 reception_Buffer_Overflow:1;
        uint8 flag4:1;
        uint8 flag5:1;
        uint8 flag6:1;
        uint8 flag7:1;
    };
} USI_UART_status = {0};

/* Function prototypes */
uint8 fBitReverse( uint8 );
void uart_fFlushBuffers( void );
void uart_fInitReceiver( void );
void uart_fInitTransmitter( void );
void uart_fTransmitByte( uint8 );
uint8 uart_fReceiveByte( void );
uint8 uart_fDataInRxBuffer( void );
bool uart_fGetMessage(uint8 *);
void uart_fSendMsg(char *);
void uart_fSendFlashMsg(char *);
void uart_fSendEepromMsg(uint8 *);

//********** USI_UART functions **********//

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : fBitReverse
 *                                                                       
 * DESCRIPTION   : Reverses the order of bits in a byte, so that USI sends
 *                 data LSB-first like a real UART.
 *
 * INPUTS        : x, byte to be reversed
 *
 * OUTPUTS       : None
 *
 * RETURNS       : x, reversed byte
 *                                                                       
 *------------------------------------------------------------------------------
 */
uint8 fBitReverse( uint8 x )
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;    
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fFlushBuffers
 *                                                                       
 * DESCRIPTION   : "Flushes" the Rx and Tx buffers by initialising the array indexes
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : None
 *                                                                       
 *------------------------------------------------------------------------------
 */
void uart_fFlushBuffers( void )  
{  
    uart_vRxTail = 0;
    uart_vRxHead = 0;
    uart_vTxTail = 0;
    uart_vTxHead = 0;
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fInitTransmitter
 *                                                                       
 * DESCRIPTION   : Initialise USI for UART transmission
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : None
 *                                                                       
 *------------------------------------------------------------------------------
 */
void uart_fInitTransmitter( void )                              
{
    cli();
    TCNT0  = 0x00;
	GTCCR |= (1<<PSR0);  // Reset the prescaler
	TCCR0A = 0x00;      // Normal counter mode
	OCR0A = 0xFF;       // Compare with FF to keep most code the same
    TCCR0B  = (0<<CS02)|(1<<CS01)|(0<<CS00);         // Start Timer0.
    TIFR   = (1<<OCF0A);                                       // Clear Timer0 COMPA interrupt flag.
    TIMSK |= (1<<OCIE0A);                                      // Enable Timer0 COMPA interrupt.
                                                                
    USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
             (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
             (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 COMP as USI Clock source.
             (0<<USITC);                                           
    USIBR  = 0xFF;                                            // Make sure MSB is '1' before enabling USI_DO.


    USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
             0x0F;                                            // Preload the USI counter to generate interrupt at first USI clock.
    DDRB  |= (1<<PB1);                                        // Configure USI_DO as output.
                  
    USI_UART_status.ongoing_Transmission_From_Buffer = TRUE;
                  
    sei();
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fInitReceiver
 *                                                                       
 * DESCRIPTION   : Enable the pinchange interrupt to detect a start bit.
 *                 Note: The USI is configured the receive the data byte in 
 *                       the pinchange ISR
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : None
 *                                                                       
 *------------------------------------------------------------------------------
 */
void uart_fInitReceiver( void )        
{  
    PCMSK = 0x01;    // Pin change interupt on PB0 only
	OCR0A = 0xFF;    // Compare with FF to keep most code the same
	PORTB |=   ((1<<PB2)|(1<<PB1)|(1<<PB0));  // Enable pull up on USI DO, DI and SCK pins.    
    DDRB  &= ~((1<<PB2)|(1<<PB1)|(1<<PB0));   // Set USI DI, DO and SCK pins as inputs.
												// Set DO as input to prevent echoing characters  
    USICR  =  0;                                // Disable USI.
    GIFR  =  (1<<PCIF);                         // Clear pin change interrupt flag.
    GIMSK |=  (1<<PCIE);                        // Enable pin change interrupt for PB3:0.
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fTransmitByte
 *                                                                       
 * DESCRIPTION   : This function does not actually transmit the byte!
 *                 Rather, it reverses the bits and puts the byte into the Tx FIFO 
 *                 It also initiates the Tx routines if necessary
 *
 * INPUTS        : data, character to be transmitted
 *
 * OUTPUTS       : None
 *
 * RETURNS       : None
 *                                                                       
 *------------------------------------------------------------------------------
 */
void uart_fTransmitByte( uint8 data )          
{
    uint8 tmphead;

    tmphead = ( uart_vTxHead + 1 ) & UART_TX_BUFFER_MASK;        // Calculate buffer index.
    while ( tmphead == uart_vTxTail );                           // Wait for free space in buffer.
    uart_vTxBuffer[tmphead] = fBitReverse(data);                    // Reverse the order of the bits in the data byte and store data in buffer.
    uart_vTxHead = tmphead;                                      // Store new index.
    
    if ( !USI_UART_status.ongoing_Transmission_From_Buffer )    // Start transmission from buffer (if not already started).
    {
        while ( USI_UART_status.ongoing_Reception_Of_Package ); // Wait for USI to finsh reading incoming data.
        uart_fInitTransmitter();              
    }
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fGetMessage
 *                                                                       
 * DESCRIPTION   : This function gets a message from the terminal.
 *
 * INPUTS        : Pointer to message buffer
 *
 * OUTPUTS       : None
 *
 * RETURNS       : True if buffer contains a complete message
 *                                                                       
 *------------------------------------------------------------------------------
 */
bool uart_fGetMessage(uint8 *pArray)
{
	bool vMsgComplete = false;
	uint8 vChar;
	
	if (uart_fDataInRxBuffer() ) /* If there is data in the Rx buffer, add it to the message */
	{
		vChar = uart_fReceiveByte();
		pArray[uart_vRxIndex] = vChar;
		if (vChar == '\r') /* \r indicates end of message */
		{
			vMsgComplete = true;
			pArray[uart_vRxIndex] = '\0'; /* Replace \r with null to terminate string correctly */
			uart_vRxIndex = 0;
		}
		else
		{
			if (uart_vRxIndex < 50)
			{
				uart_vRxIndex++; /* Increment buffer pointer, but stop at end of buffer! */
			}
		}
	}
	return (vMsgComplete);
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fReceiveByte
 *                                                                       
 * DESCRIPTION   : This function gets a byte from the Rx FIFO and bit-reverses it.
 *                 If the FIFO is empty the function waits until data is available
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : character received
 *                                                                       
 *------------------------------------------------------------------------------
 */

uint8 uart_fReceiveByte( void )                
{
    uint8 tmptail;
        
    while ( uart_vRxHead == uart_vRxTail );                 // Wait for incoming data 
    tmptail = ( uart_vRxTail + 1 ) & UART_RX_BUFFER_MASK;  // Calculate buffer index 
    uart_vRxTail = tmptail;                                // Store new index 
    return fBitReverse(uart_vRxBuffer[tmptail]);              // Reverse the order of the bits in the data byte before it returns data from the buffer.
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fDataInRxBuffer
 *                                                                       
 * DESCRIPTION   : Checks to see if there is data in the Rx FIFO
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : true if data available
 *                                                                       
 *------------------------------------------------------------------------------
 */
uint8 uart_fDataInRxBuffer( void )        
{
    return ( uart_vRxHead != uart_vRxTail );                // Return 0 (FALSE) if the receive buffer is empty.
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fSendMsg
 *                                                                       
 * DESCRIPTION   : This function sends a message to the terminal via the UART
 *                 The message must be a null-terminated string
 *
 * INPUTS        : pvTxBuffer - pointer to message buffer
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
void uart_fSendMsg(char *pMessage)
{
    char vTxByte;
    
    while (*pMessage != '\0')
    {
        vTxByte = *pMessage++;
        uart_fTransmitByte(vTxByte);
    } 
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fSendFlashMsg
 *                                                                       
 * DESCRIPTION   : This function sends a message to the terminal via the UART
 *                 The message must be a null-terminated string stored in flash
 *
 * INPUTS        : pMessage - pointer to message in flash
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
void uart_fSendFlashMsg(char *pMessage)
{
    char vTxByte;
    
	vTxByte = pgm_read_byte(pMessage);
    while (vTxByte != '\0')
    {
        uart_fTransmitByte(vTxByte);
		vTxByte = pgm_read_byte(++pMessage);
    } 
}

/*
 *------------------------------------------------------------------------------
 *
 * FUNCTION NAME : uart_fSendEepromMsg
 *                                                                       
 * DESCRIPTION   : This function sends a message to the terminal via the UART
 *                 The message must be a null-terminated string stored in eeprom
 *
 * INPUTS        : pMessage - pointer to message in flash
 *
 * OUTPUTS       : None
 *
 * RETURNS       : Nothing
 *                                                                       
 *------------------------------------------------------------------------------
 */
void uart_fSendEepromMsg(uint8 *pMessage)
{
    char vTxByte;
    
	vTxByte = (char)eeprom_read_byte(pMessage);
    while (vTxByte != '\0')
    {
        uart_fTransmitByte(vTxByte);
		vTxByte = eeprom_read_byte(++pMessage);
    } 
}

// ********** Interrupt Handlers ********** //

/*
 *------------------------------------------------------------------------------
 *
 * ISR NAME      : PCINT0_vect
 *                                                                       
 * DESCRIPTION   : Pin change interrupt vector
 *                 The pin change is used to detect the start bit of a 
 *                 received frame. In this function the USI is configured to
 *                 sample the incoming data bits.
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : None
 *                                                                       
 *------------------------------------------------------------------------------
 */
ISR(PCINT0_vect)                                      
{                                                                    
    if (!( PINB & (1<<PB0) ))                                     // If the USI DI pin is low, then it is likely that it
    {                                                             //  was this pin that generated the pin change interrupt.
        TCNT0  = INTERRUPT_STARTUP_DELAY + INITIAL_TIMER0_SEED;   // Plant TIMER0 seed to match baudrate (incl interrupt start up time.).
        GTCCR |= (1<<PSR0);                                 // Reset the prescaler
		TCCR0B  = (0<<CS02)|(1<<CS01)|(0<<CS00);         // re-start Timer0.
        TIFR   = (1<<OCF0A);                                       // Clear Timer0 COMPA interrupt flag.
        TIMSK |= (1<<OCIE0A);                                      // Enable Timer0 COMPA interrupt.
                                                                    
        USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
                 (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
                 (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 COMPA as USI Clock source.
                 (0<<USITC);                                           
                                                                  // Note that enabling the USI will also disable the pin change interrupt.
        USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
                 USI_COUNTER_SEED_RECEIVE;                        // Preload the USI counter to generate interrupt.
                                                                  
        GIMSK &=  ~(1<<PCIE);                                    // Disable pin change interrupt for PB3:0. 
        
        USI_UART_status.ongoing_Reception_Of_Package = TRUE;             
    }
}

/*
 *------------------------------------------------------------------------------
 *
 * ISR NAME      : USI_OVF_vect
 *                                                                       
 * DESCRIPTION   : USI counter overflow interrupt vector
 *                 The counter overflow is used to transfer data between the 
 *                 USI data register and the FIFOs. 
 *                 This interrupt is used for both tranmission and reception
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : None
 *                                                                       
 *------------------------------------------------------------------------------
 */
ISR(USI_OVF_vect)
{
    uint8 tmphead,tmptail;
    
    // Check if we are running in Transmit mode.
    if( USI_UART_status.ongoing_Transmission_From_Buffer )      
    {
        // If ongoing transmission, then send second half of transmit data.
        if( USI_UART_status.ongoing_Transmission_Of_Package )   
        {                                   
            USI_UART_status.ongoing_Transmission_Of_Package = FALSE;    // Clear on-going package transmission flag.
            
            USISR = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);                 // Load USI Counter seed and clear all USI flags.
            USIBR = (uart_vTxData << 3) | 0x07;                      // Reload the USIDR with the rest of the data and a stop-bit.
        }
        // Else start sending more data or leave transmit mode.
        else
        {
            // If there is data in the transmit buffer, then send first half of data.
            if ( uart_vTxHead != uart_vTxTail )                           
            {
                USI_UART_status.ongoing_Transmission_Of_Package = TRUE; // Set on-going package transmission flag.
                
                tmptail = ( uart_vTxTail + 1 ) & UART_TX_BUFFER_MASK;    // Calculate buffer index.
                uart_vTxTail = tmptail;                                  // Store new index.            
                uart_vTxData = uart_vTxBuffer[tmptail];                  // Read out the data that is to be sent. Note that the data must be bit reversed before sent.
                                                                        // The bit reversing is moved to the application section to save time within the interrupt.
                USISR  = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);            // Load USI Counter seed and clear all USI flags.
                USIBR  = (uart_vTxData >> 2) | 0x80;                 	// Copy (initial high state,) start-bit and 6 LSB of original data (6 MSB
            }
            // Else enter receive mode.
            else
            {
                USI_UART_status.ongoing_Transmission_From_Buffer = FALSE; 
                
                TCCR0B  = (0<<CS02)|(0<<CS01)|(0<<CS00);                 // Stop Timer0.
                PORTB |=   (1<<PB2)|(1<<PB1)|(1<<PB0);         // Enable pull up on USI DO, DI and SCK pins.  
                DDRB  &= ~((1<<PB2)|(1<<PB1)|(1<<PB0));        // Set USI DI, DO and SCK pins as inputs.
																// Disable DO output to prevent echoing characters 
                USICR  =  0;                                            // Disable USI.
                GIFR   =  (1<<PCIF);                                    // Clear pin change interrupt flag.
                GIMSK |=  (1<<PCIE);                                   // Enable pin change interrupt for PB3:0.
            }
        }
    }
    
    // Else running in receive mode.
    else                                                                
    {              
        USI_UART_status.ongoing_Reception_Of_Package = FALSE;           

        tmphead     = ( uart_vRxHead + 1 ) & UART_RX_BUFFER_MASK;        // Calculate buffer index.
        
        if ( tmphead == uart_vRxTail )                                   // If buffer is full trash data and set buffer full flag.
        {
            USI_UART_status.reception_Buffer_Overflow = TRUE;           // Store status to take actions elsewhere in the application code
        }
        else                                                            // If there is space in the buffer then store the data.
        {
            uart_vRxHead = tmphead;                                      // Store new index.
            uart_vRxBuffer[tmphead] = USIBR;                                // Store received data in buffer. Note that the data must be bit reversed before used. 
        }                                                               // The bit reversing is moved to the application section to save time within the interrupt.
                                                                
        TCCR0B  = (0<<CS02)|(0<<CS01)|(0<<CS00);                 // Stop Timer0.
        PORTB |=   (1<<PB2)|(1<<PB1)|(1<<PB0);         // Enable pull up on USI DO, DI and SCK pins.    
        DDRB  &= ~((1<<PB2)|(0<<PB1)|(1<<PB0));        // Set USI DI and SCK pins as inputs.  
        USICR  =  0;                                            // Disable USI.
        GIFR   =  (1<<PCIF);                                    // Clear pin change interrupt flag.
        GIMSK |=  (1<<PCIE);                                   // Enable pin change interrupt for PB3:0.
    }
    
}

/*
 *------------------------------------------------------------------------------
 *
 * ISR NAME      : TIM0_COMPA_vect
 *                                                                       
 * DESCRIPTION   : Timer0 counter compare interrupt vector
 *                 The counter compare is used to clock the USI. 
 *                 In Rx mode it samples the incoming data bits,
 *                 in Tx mode it shifts the data bits out of the DO pin.
 *
 * INPUTS        : None
 *
 * OUTPUTS       : None
 *
 * RETURNS       : None
 *                                                                       
 *------------------------------------------------------------------------------
 */// Timer0 Compare A interrupt is used to trigger the sampling of signals on the USI ports.
ISR(TIM0_COMPA_vect)
{
    TCNT0 += TIMER0_SEED;                   // Reload the timer,
                                            // current count is added for timing correction.

}

