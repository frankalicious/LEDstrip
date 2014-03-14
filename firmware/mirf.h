/*
	Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
	
	Permission is hereby granted, free of charge, to any person 
	obtaining a copy of this software and associated documentation 
	files (the "Software"), to deal in the Software without 
	restriction, including without limitation the rights to use, copy, 
	modify, merge, publish, distribute, sublicense, and/or sell copies 
	of the Software, and to permit persons to whom the Software is 
	furnished to do so, subject to the following conditions:
	
	The above copyright notice and this permission notice shall be 
	included in all copies or substantial portions of the Software.
	
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
	MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
	DEALINGS IN THE SOFTWARE.
	
	$Id$
*/

#ifndef _MIRF_H_
#define _MIRF_H_

#include <avr/io.h>

/* choose RECEIVE_MODE or TRANSMIT_MODE*/
/* #define RECEIVE_MODE */
#define TRANSMIT_MODE

// Mirf settings
#define RXMODE 1
#define TXMODE 0
#define mirf_CH			0
#define mirf_PAYLOAD		15
#define mirf_CONFIG		( (1<<EN_CRC) | (0<<CRCO) )
#if 0
#define RADDR				(uint8_t *)"clnt2"
#define TADDR				(uint8_t *)"clnt1"
#endif
// Pin definitions for chip select and chip enabled of the MiRF module
#define CE  PD7
#define CSN PB0
#define IRQ PC0
// Definitions for selecting and enabling MiRF module
#define mirf_CSN_hi	PORTB |=  (1<<CSN);
#define mirf_CSN_lo	PORTB &= ~(1<<CSN);
#define mirf_CE_hi	PORTD |=  (1<<CE);
#define mirf_CE_lo	PORTD &= ~(1<<CE);

// Public standard functions
extern void mirf_init(void);
extern void mirf_config(void);
extern void mirf_flush_rx_tx(void);
extern uint8_t mirf_status(void);
extern void mirf_send(uint8_t * value, uint8_t len);
extern uint8_t mirf_data_ready(void);
extern uint8_t mirf_max_rt_reached(void);
extern void mirf_get_data(uint8_t * data);

// Public extended functions
extern void mirf_config_register(uint8_t reg, uint8_t value);
extern void mirf_read_register(uint8_t reg, uint8_t * value, uint8_t len);
extern void mirf_write_register(uint8_t reg, uint8_t * value, uint8_t len);

#endif /* _MIRF_H_ */
