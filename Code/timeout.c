/* Copyright 2012 Philip Heron <phil@sanslogic.co.uk>                    */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program. If not, see <http://www.gnu.org/licenses/>.  */

#include "timeout.h"

volatile static to_int _clocka = 0;
volatile static to_int _clockb = 0;

/* Call this function from an interrupt to advance the clock.
 * "frequency" should be the number of times per second the
 * function is called. This is used to correct the number of
 * ticks per-second if it is not the ideal 1000Hz.
 * frequency must be greater than zero */

void to_tick(to_int frequency)
{
	static to_int dv = 0;
	while(dv < 1000)
	{
		_clockb = ++_clocka;
		dv += frequency;
	}
	dv -= 1000;
}

/* Returns the current clock value */

to_int to_clock(void)
{
	to_int r = _clocka;
	/* Wait for both clock values to match. They will vary if a tick
	 * occurs while we are reading them */
	while(r != _clockb) r = _clocka;
	return(r);
}

/* Returns the number of milliseconds since "timestamp".
 * The maximum timespan is 65,535 milliseconds */

to_int to_since(to_int timestamp)
{
	to_int r = to_clock();
	return(r - timestamp);
}

/* Delay for "delay" milliseconds.
 * The maximum delay is 65,535 milliseconds */

void to_delay(to_int delay)
{
	to_int r = to_clock();
	while(to_since(r) < delay);
}

