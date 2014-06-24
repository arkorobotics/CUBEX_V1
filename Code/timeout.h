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

#ifndef __TIMEOUT_H__
#define __TIMEOUT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t to_int;

extern void to_tick(to_int frequency);
extern to_int to_clock(void);
extern to_int to_since(to_int timestamp);
extern void to_delay(to_int delay);

#ifdef __cplusplus
}
#endif

#endif
