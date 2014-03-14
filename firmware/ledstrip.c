#include <ctype.h>
#include <inttypes.h>

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include <avr/wdt.h> 
#include <avr/interrupt.h>
#include <avr/eeprom.h> 
#include <avr/pgmspace.h>
#include <avr/eeprom.h> 

#include "mstdio.h"
#include "i2cmaster.h"
#include "pca9685.h"
#include "nRF24L01.h" //for FIFO_STATUS
#include "mirf.h"
#include "spi.h"

/*  
    OC2B: PWM ~OE

    OC1A: G
    OC1B: R
    OC0A: B
    OC0B: W

    PB0: nRF24L01+ CSN
    PD7: nRF24L01+ CE
 
    PD4: ~LED
*/

void led(char on);
void setledpwm(unsigned int channel, unsigned char value);
void register_status(void);
void endless_serial_test (void);
void endless_pwm_test (void);
void endless_radio_test (void);

char* cmd_name[]={
  "CONFIG",
  "EN_AA",
  "EN_RXADDR",
  "SETUP_AW",
  "SETUP_RETR",
  "RF_CH",
  "RF_SETUP",
  "STATUS",
  "OBSERVE_TX",
  "CD",
  "RX_ADDR_P0",
  "RX_ADDR_P1",
  "RX_ADDR_P2",
  "RX_ADDR_P3",
  "RX_ADDR_P4",
  "RX_ADDR_P5",
  "TX_ADDR",
  "RX_PW_P0",
  "RX_PW_P1",
  "RX_PW_P2",
  "RX_PW_P3",
  "RX_PW_P4",
  "RX_PW_P5",
  "FIFO_STATUS",
  "N/A",
  "N/A",
  "N/A",
  "N/A",
  "DYNPD",
  "FEATURE"
};

// We don't really care about unhandled interrupts.
EMPTY_INTERRUPT(__vector_default)

void led(char on) {
  if (on) {
    PORTD &=~ _BV(PD4);   
  } else {
    PORTD |= _BV(PD4);   
  }
}

#define ON_BOARD 0x1000

void setledpwm(unsigned int channel, unsigned char value) {
 
  unsigned char ch = channel & 0xf;
  if ((channel & ON_BOARD) == ON_BOARD) {

    if (ch == 0) {

      if (value == 0) {
        TCCR1A &=~ _BV(COM1A1);
      } else {
        OCR1A = ciel10bit(value); // Green
        TCCR1A |= _BV(COM1A1);
      }

    } else if (ch == 1) {

      if (value == 0) {
        TCCR1A &=~ _BV(COM1B1);
      } else {
        OCR1B = ciel10bit(value); // Red
        TCCR1A |= _BV(COM1B1);
      }      

    } else if (ch == 2) {

      if (value == 0) {
        TCCR0A &=~ _BV(COM0A1);
      } else {
        OCR0A = ciel8bit(value); // Blue
        TCCR0A |= _BV(COM0A1); // OC0A clear on match
      }

    } else if (ch == 3) {

      if (value == 0) {
        TCCR0A &=~ _BV(COM0B1);
      } else {
        OCR0B = ciel8bit(value); // White
        TCCR0A |= _BV(COM0B1); // OC0B clear on match
      }

    } else if (ch == 4) {

      if (value == 0) {
        TCCR2B &=~ _BV(COM2B1);
      } else {
        OCR2B = ciel8bit(value); // Common enable
        TCCR2B |= _BV(COM2B1); // OC0B clear on match
      }
    }

  } else {
    
    unsigned char i2c = 0x80 + ((channel >> 4) << 1);

    pca9685_led_pwm(i2c, channel & 0xf, value);
  }
}

int main(void) {
  wdt_enable(WDTO_4S);
  DDRD  |= _BV(PD4);  // LED output
  led(1);

  DDRD |= _BV(PD5);
  DDRD |= _BV(PD6);
  DDRB |= _BV(PB1);
  DDRB |= _BV(PB2);

//  initADC();
  muartInit();

  mprintf(PSTR("Power up\n"));


// Set up timer 0 for fast PWM mode & the highest frequency available
  TCCR0A =  _BV(WGM00) |  _BV(WGM01); // Fast PWM
  TCCR0B = _BV(CS00); // Fastest clock source

  TCCR0A |= _BV(COM0A1); // OC0A clear on match
  TCCR0A |= _BV(COM0B1); // OC0B clear on match

  DDRD  |= _BV(PD6);  // OC0A
  DDRD  |= _BV(PD5);  // OC0B



// Set up timer 1 for fast PWM mode & the highest frequency available
  TCCR1A = _BV(WGM12) | _BV(WGM11) | _BV(WGM10); // 10 bit fast pwm

  TCCR1A |= _BV(COM1A1); // OC1A clear on match
  TCCR1A |= _BV(COM1B1); // OC1B clear on match

  TCCR1B = _BV(CS10); // Fastest clock

  DDRB  |= _BV(PB1);  // OC1A
  DDRB  |= _BV(PB2);  // OC1B


// Set up timer 2 for fast PWM mode & the highest frequency available
  TCCR2A =  _BV(WGM20) |  _BV(WGM21); // Fast PWM
  TCCR2B = _BV(CS20); // Fastest clock source

  TCCR2A |= _BV(COM2B1) | _BV(COM2B0); // OC0B clear on match

  DDRD  |= _BV(PD3);  // OC2B

  OCR0A = 0; // Blue
  OCR0B = 0; // White
  OCR1A = 0; // Green
  OCR1B = 0; // Red
  OCR2B = 255; // Common output enable

  i2c_init();
  pca9685_init(0x80, PCA9685_FREQUENCY(1200UL)); 

  setledpwm(ON_BOARD|0, 0);
  setledpwm(ON_BOARD|1, 0);
  setledpwm(ON_BOARD|2, 0);
  pca9685_led_pwm(0x80, 0, 0);
  pca9685_led_pwm(0x80, 1, 0);
  pca9685_led_pwm(0x80, 2, 0);

#if 0
  endless_serial_test();
#endif

#if 1
  endless_pwm_test();
#endif

#if 0
  endless_radio_test();
#endif
}


void register_status(void)
{
  uint8_t reg;
  for (reg = 0; reg <=0x1d ; reg++)
  {
    mprintf(PSTR("\"%s\" "),cmd_name[reg]);;
    if (reg == 0xa || reg == 0xb || reg == 0x10)
    {
      uint8_t value[5];
      mirf_read_register(reg, &value[0], 5);
      mprintf(PSTR("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n"),
              value[0], value[1], 
              value[2], value[3], 
              value[4]);

      /* mprintf(PSTR("register:0x%02x value:0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n"), */
      /*         reg,  */
      /*         value[0], value[1],  */
      /*         value[2], value[3],  */
      /*         value[4]); */
    }
    else
    {
      uint8_t value;
      mirf_read_register(reg, &value, 1);
      mprintf(PSTR("0x%02x\n"),
              value);
      /* mprintf(PSTR("register:0x%02x value:0x%02x\n"), */
      /* reg, value); */
    }
  }
}

void endless_serial_test (void)
{
  while (1)
  {
    _delay_ms(1000);
    mprintf(PSTR("serial\n"));
    wdt_reset();
  }
}

void endless_pwm_test (void)
{
  unsigned char frame = 0;
//unsigned char inten = 0;
  /* uint8_t led_nr =0; */
#if 1
  while (1) {
//mprintf(PSTR("F\n"));
    led(frame++ & 15);
    _delay_ms(10);
    wdt_reset();
/*
  pca9685_led_pwm(0x80, 0, frame);
  pca9685_led_pwm(0x80, 1, 128-frame);
  pca9685_led_pwm(0x80, 2, 255-frame);
*/

    setledpwm(0, 255-frame);
    setledpwm(1, frame);
    setledpwm(2, frame >> 4);

    if (frame & 15) {
      setledpwm(ON_BOARD|3, 0);
    } else {
      setledpwm(ON_BOARD|3, 255);
    }

//setledpwm(ON_BOARD|4, frame);
  }
#endif

#if 0
  while (1){
    static uint8_t up=1;
    led(frame & 15);
    wdt_reset();
    _delay_ms(10);
    if (frame++ == 0)
    {
      if (up == 1)
        up = 0;
      else {
        setledpwm(0, 0);
        setledpwm(1, 0);
        setledpwm(2, 0);
        led_nr++;
        up = 1;
      }
    }
    if (led_nr == 3)
      led_nr = 0;
    if (up == 1 )
      setledpwm(led_nr, frame);
    else
      setledpwm(led_nr, 255-frame);
  }
#endif
}

void endless_radio_test (void)
{
  spi_init();
  mirf_init();
// Initialize AVR for use with mirf
  mirf_config();
  mprintf(PSTR("1\n"));
// Wait for mirf to come up
  _delay_ms(50);
  wdt_reset();
  mprintf(PSTR("2\n"));
// Activate interrupts
  sei();
  wdt_reset();
// Configure mirf
  mirf_config();
  mprintf(PSTR("3\n"));
  mprintf(PSTR("finished\n"));

/* nrf24 testing */
  register_status();
#ifdef RECEIVE_MODE
  mprintf(PSTR("Receive mode\n"));
#endif
#ifdef TRANSMIT_MODE
  mprintf(PSTR("Transmit mode\n"));
#endif
#ifdef TRANSMIT_MODE
  uint8_t buffer_tx[mirf_PAYLOAD] = {0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xA,0xB,0xC,0xD,0xE,0xF,};
#endif
  while (1){
#ifdef RECEIVE_MODE
    uint8_t buffer_rx[mirf_PAYLOAD];
#endif

#ifdef RECEIVE_MODE
    uint8_t data_available = mirf_data_ready();
#endif
    uint8_t status = mirf_status();
    static uint8_t last_status = 0xFF;
    uint8_t fifo_status;
    static uint8_t last_fifo_status = 0xFF;
    uint8_t irq_status = PINC & (1<<PC0);
    static uint8_t last_irq_status = 0xFF;
    /* char data[] = "N123;B555;M666"; */
    wdt_reset();
    /* mirf_send((uint8_t*)&data[0], sizeof(data)/sizeof(data[0])); */
    /* mirf_send((uint8_t*)&data[0], 15); */
    mirf_read_register(FIFO_STATUS, &fifo_status, 1);
#ifdef RECEIVE_MODE
    if (data_available)
    {
      uint8_t i;
      mirf_get_data(&buffer_rx[0]);
      PORTD ^= _BV(PD4);   //toggle led
      /* mprintf(PSTR("buffer: 0x%x 0x%x\n"),buffer[0], buffer[1]); */
      mprintf(PSTR("buffer:"));
      for (i=0; i<mirf_PAYLOAD; i++)
      {
        /* mprintf(PSTR(" %d:0x%x"),i, buffer_rx[i]); */
        mprintf(PSTR(" 0x%x"),buffer_rx[i]);
      }
      mprintf(PSTR("\n"));
    }
    /* else */
    /* { */
    /*   mirf_flush_rx_tx(); */
    /* } */
#endif

#ifdef TRANSMIT_MODE
    if (mirf_max_rt_reached())
    {
      mprintf(PSTR("max_rt_reached\n"));
      /* mirf_config_register(STATUS, MAX_RT); */
      mirf_config_register(STATUS,
                           _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
      mirf_flush_rx_tx();
    }
    else
    {
      buffer_tx[0]++;
    }
    mirf_send(&buffer_tx[0],mirf_PAYLOAD);
#endif

    wdt_reset();
    if (status != last_status)
    {
      mprintf(PSTR("Status:0x%x\n"),status);
      last_status = status;
    }
    if (fifo_status != last_fifo_status)
    {
      mprintf(PSTR("FIFO Status:0x%x\n"),fifo_status);
      last_fifo_status = fifo_status;
    }
    if (irq_status != last_irq_status)
    {
      mprintf(PSTR("IRQ:0x%x\n"),irq_status);
      last_irq_status = irq_status;
    }
/* delay for receive should be lesser than for transmit */
#ifdef TRANSMIT_MODE
    /* _delay_ms(10); */
    _delay_ms(1000);
#endif
/* #ifdef RECEIVE_MODE */
    /* _delay_ms(100); */
/* #endif */
  }
}
