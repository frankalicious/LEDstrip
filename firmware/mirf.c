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
  
  -----
  
  Modified by Alex from Inside Gadgets (http://www.insidegadgets.com)
  Last Modified: 22/08/2012
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "mirf.h"
#include "nRF24L01.h"
#include "spi.h"

// Defines for setting the MiRF registers for transmitting or receiving mode
#define TX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP mirf_config_register(CONFIG, mirf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )

// Flag which denotes transmitting or receiving mode
volatile uint8_t PMODE;
uint8_t addr[]={ 0x0, 0x1, 0x2, 0x3, 0x4};
/* uint8_t rx_addr[]={ 0x0, 0x1, 0x2, 0x3, 0x4}; */
/* uint8_t tx_addr[]={ 0x5, 0x6, 0x7, 0x8, 0x9}; */
/* uint8_t tx_addr[]={ 0xDE, 0xAD, 0xBE, 0xEF, 0x01}; */
/* uint8_t rx_addr[]={ 0xDE, 0xAD, 0xBE, 0xEF, 0x01}; */
/* uint8_t rx_addr[]={ 0x01, 0xEF, 0xBE, 0xAD, 0xDE}; */
// Initializes pins ans interrupt to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
void mirf_init(void) {
  // Define CSN and CE as Output and set them to default
  DDRB |= (1<<CSN);
  DDRD |= (1<<CE);
  mirf_CE_lo;
  mirf_CSN_hi;
  DDRC &= ~(1<<IRQ);/* irq pin as input*/
  /* PORTC |= (1<<IRQ); /\* enable pullup on irq *\/ */
  PORTC &= ~(1<<IRQ); /* disable pullup on irq */
}

// Sets the important registers in the MiRF module and powers the module
// in receiving mode
void mirf_config(void) {
  // Set RF channel
  mirf_config_register(RF_CH, mirf_CH);

  // air data rate: 1Mbps
  mirf_config_register(RF_SETUP, 0x6);

  // Set length of incoming payload 
  mirf_config_register(RX_PW_P0, mirf_PAYLOAD);
  mirf_config_register(RX_PW_P1, mirf_PAYLOAD);  
  // Set RADDR and TADDR(autoack)
  /* mirf_write_register(RX_ADDR_P0, &tx_addr[0], 5); */
  /* mirf_write_register(TX_ADDR, &tx_addr[0], 5); */
  /* mirf_write_register(RX_ADDR_P1, &rx_addr[0], 5); */
#ifdef TRANSMIT_MODE
  mirf_write_register(RX_ADDR_P0, &addr[0], 5);
  mirf_write_register(TX_ADDR, &addr[0], 5);
#endif
#ifdef RECEIVE_MODE
  mirf_CE_hi
  mirf_write_register(RX_ADDR_P1, &addr[0], 5);
#endif
  mirf_config_register(EN_AA, 0x03);

  // Enable RX_ADDR_P0 address matching since we also enable auto acknowledgement
  /* mirf_config_register(EN_RXADDR, 1<<ERX_P1); */
  mirf_config_register(EN_RXADDR, 1<<ERX_P0 | 1<<ERX_P1);
//from enrf24
  //mirf_config_register(FEATURE, 0x04);  // enable dynamic payloads length
  //mirf_config_register(DYNPD, 0x03); // enable dynamic payload length data pipe 0 and data pipe 1

  /* mirf_config_register(SETUP_RETR, 0x7F); */
  /* PMODE = TXMODE; // Start in transmitting mode */
  /* TX_POWERUP;     // Power up in transmitting mode */
  //frankalicious: testing
  /* PMODE = RXMODE; // Start in transmitting mode */
  /* RX_POWERUP; */
#ifdef TRANSMIT_MODE
  PMODE = TXMODE; // Start in transmitting mode
  TX_POWERUP;     // Power up in transmitting mode
#endif
#ifdef RECEIVE_MODE
  PMODE = RXMODE; // Start in transmitting mode
  RX_POWERUP;     // Power up in transmitting mode
#endif

}

// Flush RX and TX FIFO
void mirf_flush_rx_tx(void) {
  mirf_CSN_lo; // Pull down chip select
  spi_fast_shift(FLUSH_RX); // Flush RX
  mirf_CSN_hi; // Pull up chip select

  mirf_CSN_lo; // Pull down chip select
  spi_fast_shift(FLUSH_TX);  // Write cmd to flush tx fifo
  mirf_CSN_hi; // Pull up chip select
}

// Read the status register
uint8_t mirf_status(void) {
  mirf_CSN_lo; // Pull down chip select
  spi_fast_shift(R_REGISTER | (REGISTER_MASK & STATUS));
  uint8_t status = spi_fast_shift(NOP); // Read status register
  mirf_CSN_hi; // Pull up chip select
  return status;
}

// Checks if data is available for reading
uint8_t mirf_data_ready(void) {
  mirf_CSN_lo; // Pull down chip select
  spi_fast_shift(R_REGISTER | (REGISTER_MASK & STATUS));
  uint8_t status = spi_fast_shift(NOP); // Read status register
  mirf_CSN_hi; // Pull up chip select
  return status & (1<<RX_DR);
}

// Checks if MAX_RT has been reached
uint8_t mirf_max_rt_reached(void) {
  mirf_CSN_lo; // Pull down chip select
  spi_fast_shift(R_REGISTER | (REGISTER_MASK & STATUS));
  uint8_t status = spi_fast_shift(NOP); // Read status register
  mirf_CSN_hi; // Pull up chip select
  return status & (1<<MAX_RT);
}

// Reads mirf_PAYLOAD bytes into data array
void mirf_get_data(uint8_t *data) {
  mirf_CSN_lo; // Pull down chip select
  spi_fast_shift(R_RX_PAYLOAD); // Send cmd to read rx payload
  spi_read_data(data, mirf_PAYLOAD); // Read payload
  mirf_CSN_hi; // Pull up chip select
  mirf_config_register(STATUS,(1<<RX_DR)); // Reset status register
}

// Write one byte into the MiRF register
void mirf_config_register(uint8_t reg, uint8_t value) {
  mirf_CSN_lo;
  spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
  spi_fast_shift(value);
  mirf_CSN_hi;
}

// Reads an array of bytes from the MiRF registers.
void mirf_read_register(uint8_t reg, uint8_t *value, uint8_t len) {
  mirf_CSN_lo;
  spi_fast_shift(R_REGISTER | (REGISTER_MASK & reg));
  spi_read_data(value, len);
  mirf_CSN_hi;
}

// Writes an array of bytes into the MiRF register
void mirf_write_register(uint8_t reg, uint8_t *value, uint8_t len) {
  mirf_CSN_lo;
  spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
  spi_transmit_sync(value, len);
  mirf_CSN_hi;
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void mirf_send(uint8_t *value, uint8_t len) {
  PMODE = TXMODE; // Set to transmitter mode
  TX_POWERUP; // Power up

  mirf_CSN_lo; // Pull down chip select
  spi_fast_shift(FLUSH_TX);  // Write cmd to flush tx fifo
  mirf_CSN_hi; // Pull up chip select

  mirf_CSN_lo;  // Pull down chip select
  spi_fast_shift(W_TX_PAYLOAD); // Write cmd to write payload
  spi_transmit_sync(value, len); // Write payload
  mirf_CSN_hi; // Pull up chip select

  mirf_CE_hi; // Start transmission
  _delay_us(15);
  mirf_CE_lo;
}
