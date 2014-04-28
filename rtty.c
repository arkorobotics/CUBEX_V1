
/* Project Swift - High altitude balloon flight software                 */
/*=======================================================================*/
/* Copyright 2010-2012 Philip Heron <phil@sanslogic.co.uk>               */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "rtty.h"
#include "timeout.h"
#include "Arduino.h"

volatile static uint8_t  txpgm = 0;
volatile static uint8_t *txbuf = 0;
volatile static uint16_t txlen = 0;

ISR(TIMER1_COMPA_vect)
{
	// The currently transmitting byte, including framing //
	static uint8_t byte = 0x00;
	static uint8_t bit  = 0x00;
	
	uint8_t b = 0;
	switch(bit++)
	{
	case 0: b = 0; break; // Start bit //
	case 9: b = 1; break; // Stop bit //
	//case 10: b = 1; TCNT0 += OCR0A >> 1; bit = 0; break; // Stop bit 0.5 //
	case 10: b = 1; bit = 0; break; // Stop bit part 2 //
	default: b = byte & 1; byte >>= 1; break;
	}
	
	TXBIT(b);
	
	if(bit == 0 && txlen > 0)
	{
		if(txpgm == 0) byte = *(txbuf++);
		else byte = pgm_read_byte(txbuf++);
		txlen--;
	}
	
	// Timeout tick //
	to_tick(RTTY_BAUD);
}

void TXBIT(int bit)
{
  if (bit)
  {
     digitalWrite(SI446x_GPIO_PIN, HIGH);
  }
  else
  {
     digitalWrite(SI446x_GPIO_PIN, LOW);
  }
}

void rtx_init(void)
{
        // initialize Timer1
        cli();          // disable global interrupts
        TCCR1A = 0;     // set entire TCCR1A register to 0
        TCCR1B = 0;     // same for TCCR1B
        OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
        TCCR1B |= (1 << WGM12);   // turn on CTC mode:
        // Set CS10 and CS12 bits for:
        TCCR1B |= (1 << CS10);
        TCCR1B |= (1 << CS12);
        // enable timer compare interrupt:
        TIMSK1 |= (1 << OCIE1A);
        sei();          // enable global interrupts
        TXBIT(1);
}

void inline rtx_wait(void)
{
	/* Wait for interrupt driven TX to finish */
	while(txlen > 0) while(txlen > 0);
}

void rtx_data(uint8_t *data, size_t length)
{
	rtx_wait();
	txpgm = 0;
	txbuf = data;
	txlen = length;
}

void rtx_data_P(PGM_P data, size_t length)
{
	rtx_wait();
	txpgm = 1;
	txbuf = (uint8_t *) data;
	txlen = length;
}

void rtx_string(char *s)
{
	uint16_t length = strlen(s);
	rtx_data((uint8_t *) s, length);
}

void rtx_string_P(PGM_P s)
{
	uint16_t length = strlen_P(s);
	rtx_data_P(s, length);
}

