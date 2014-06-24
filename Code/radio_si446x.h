//
//
// Code modified by Arko for the Si446x
// Code based on KT5TK's Si446x
//
/* pecanpico2 copyright (C) 2013  KT5TK
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __RADIO_SI446X_H__
#define __RADIO_SI446X_H__

  void startup();
  void ptt_on();
  void ptt_off();
  void set_freq(unsigned long freq);
  int get_powerlevel();
  
  void SendCmdReceiveAnswer(int byteCountTx, int byteCountRx, const char* pData);
  void resetradio(void);
  void setModem(void);
  void start_tx(void);
  void stop_tx(void);
  void tune_tx(void);
  void setFrequency(unsigned long freq); 

  #define SCKpin  13   // SCK
  #define SSpin   10    // SS
  #define MOSIpin 11   // MOSI
  #define MISOpin 12   // MISO

#endif
