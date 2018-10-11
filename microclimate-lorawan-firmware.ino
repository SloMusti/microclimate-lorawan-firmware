/* Arduino based firmware for microclimate.network device
 *
 *  Built for EU868, modify code for other bands if hardware is available
 *    
 *  Currently implemented:
 *  Reading sensors
 *  Sending data periodically
 *  Sleep mode (about 500uA)
 *  
 *  TODO:
 *  Properly implement sensors for low-power operation, currently about 6uA
 *  Note HW modifications are required for power optimization.
 *  Cover corner-case of Join failures etc...
 *  
 *    
 *  This device has the following active I2C devices
 *  0x19  ! LIS2DH12
 *  0x40  ! HDC2080
 *  0x77  ! DPS310
 *  
 * Copyright (C) 2018 Musti - Institute IRNAS - contact@irnas.eu
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Affero General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Affero General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <STM32L0.h>
#include "TimerMillis.h"

TimerMillis wdtTimer; //timer for transmission events

#define debug
#define serial_debug  Serial1

boolean comms_transmit_flag = false;

void ISR_WDT() {
    STM32L0.wdtReset();
}

void setup( void )
{
    //Serial port setup
    #ifdef debug
      serial_debug.begin(115200);
    #endif
    comms_setup();
    sensors_setup();
    wdtTimer.start(ISR_WDT, 0, 15*1000);
    STM32L0.wdtEnable(18000);
}
void loop( void )
{
  STM32L0.wdtReset();
  if (comms_transmit_flag){
    //TODO: requires better handling, stay in sleep with 1s cycle until beign able to send
    comms_transmit_flag = false;
    comms_transmit();
  }
  //flush data before sleep, otherwise not sent correctly
  #ifdef debug
    serial_debug.flush();
  #endif
  STM32L0.stop(); // Enter STOP mode and wait for an interrupt
}
