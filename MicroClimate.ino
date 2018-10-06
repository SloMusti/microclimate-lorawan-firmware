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
 *  Properly implement sensors for low-power operation
 *  Cover corner-case of Join failures etc...
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
#include "LoRaWAN.h"
#include <STM32L0.h>
#include <Wire.h> 
#include <Adafruit_LIS3DH.h> // Install through Manager
#include <Adafruit_Sensor.h> // Install through Manager
#include <Dps310.h>// Install through Manager
#include "HDC2080.h"

#define debug
#define serial_debug            Serial1

// Configure the keys here, node the DevEUI is acquired from the module, but you can manually override
const char *appEui  = "70B3D57ED0010F91";
const char *appKey  = "50BC4179C8259B9D9B9C05FCBD80A7FD";
//const char *devEui  = "enter here"; //uncomment if manual
char devEui[32]; // uncomment if not manual

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
#define LIS_INT2  6 //PB2

// Dps310 object
Dps310 Dps310PressureSensor = Dps310();

// HDC2080 object
HDC2080 hdc2080 = HDC2080();

// battery voltage monitor definitions
uint32_t UID[3] = {0, 0, 0}; 

void ISR_LIS() {

    STM32L0.wakeup();

    #ifdef debug
        serial_debug.println("LIS (ISR_LIS) - interrupt on accel");
    #endif

}

void setup( void )
{
    //Serial port setup
    #ifdef debug
      serial_debug.begin(115200);
    #endif

    pinMode(LIS_INT2, INPUT);
    attachInterrupt(digitalPinToInterrupt(LIS_INT2), ISR_LIS, RISING);

    //Get the device ID and print
    STM32L0.getUID(UID);
    LoRaWAN.getDevEui(devEui, 18); //comment if manual override
    
    #ifdef debug
      serial_debug.print("STM32L0 MCU UID = 0x"); 
      serial_debug.print(UID[0], HEX); 
      serial_debug.print(UID[1], HEX); 
      serial_debug.println(UID[2], HEX); 
      serial_debug.print("STM32L0 Device EUI = "); 
      serial_debug.println(devEui); 
    #endif

    //Configure lora parameters
    LoRaWAN.begin(EU868);
    LoRaWAN.addChannel(1, 868300000, 0, 6);
    
    LoRaWAN.setDutyCycle(true); // must be true except for development in confined environments
    LoRaWAN.setAntennaGain(0.0); // mot be equal to the installed antenna
    
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    LoRaWAN.setADR(true);
    //LoRaWAN.setDataRate(1);
    serial_debug.println("JOIN( )");

    // Each sensor should have a check to determine if present on boot
    // Code in reading should handle if sensor becomes unavailable
    // All sensors need to be placed in the low power mode

    //LIS2DH sensor
    lis.begin(0x19);
    lis.setRange(LIS3DH_RANGE_2_G);
    lis.setClick(2, 40);
    

    //DSO130 sensor
    Dps310PressureSensor.begin(Wire);

    //hdc2080
    hdc2080.begin();

    
}
void loop( void )
{
  //evaluate if time to wake up and senda data or return to sleep
  
  //check connectivity
  if (!LoRaWAN.linkGateways())
  {
    LoRaWAN.rejoinOTAA();
  }

  float stm32l0_vdd = STM32L0.getVDDA();
  float stm32l0_temp = STM32L0.getTemperature();

  //LIS
  lis.read();      // get X Y and Z data at once
  float lis_movement=lis.x+lis.y+lis.z;

  //DPOS130 read
  int32_t dsp310_temp;
  int32_t dsp310_pres;
  
  int ret = Dps310PressureSensor.measureTempOnce(dsp310_temp, 7);
  ret += Dps310PressureSensor.measurePressureOnce(dsp310_pres, 7);
  dsp310_pres=dsp310_pres/10;

  hdc2080.read();
  float hdc2080_temp = hdc2080.getTemp();
  float hdc2080_hum = hdc2080.getHum();
  

  // Build the lorawan message
//  var I1 = bytes[0];
//  var T1 = bytes[1];
//  var T01 = bytes[2];
//  var H1 = bytes[3];
//  var H01 = bytes[4];
//  var AP1 = (bytes[5] << 8) | bytes[6];
//  var ACC1 = bytes[7];

    #ifdef debug
      serial_debug.println(""); 
      serial_debug.print("stm32l0_vdd: "); 
      serial_debug.println(stm32l0_vdd); 
      serial_debug.print("stm32l0_temp: "); 
      serial_debug.println(stm32l0_temp);
      serial_debug.print("lis_movement: "); 
      serial_debug.println(lis_movement);  
      serial_debug.print("dsp310_temp: "); 
      serial_debug.println(dsp310_temp);
      serial_debug.print("dsp310_pres: "); 
      serial_debug.println(dsp310_pres); 
      serial_debug.print("hdc2080_temp: "); 
      serial_debug.println(hdc2080_temp);
      serial_debug.print("hdc2080_hum: "); 
      serial_debug.println(hdc2080_hum);  
    #endif
    
    if (LoRaWAN.joined() && !LoRaWAN.busy())
    {
      #ifdef debug
        serial_debug.print("TRANSMIT( ");
        serial_debug.print("TimeOnAir: ");
        serial_debug.print(LoRaWAN.getTimeOnAir());
        serial_debug.print(", NextTxTime: ");
        serial_debug.print(LoRaWAN.getNextTxTime());
        serial_debug.print(", MaxPayloadSize: ");
        serial_debug.print(LoRaWAN.getMaxPayloadSize());
        serial_debug.print(", DR: ");
        serial_debug.print(LoRaWAN.getDataRate());
        serial_debug.print(", TxPower: ");
        serial_debug.print(LoRaWAN.getTxPower(), 1);
        serial_debug.print("dbm, UpLinkCounter: ");
        serial_debug.print(LoRaWAN.getUpLinkCounter());
        serial_debug.print(", DownLinkCounter: ");
        serial_debug.print(LoRaWAN.getDownLinkCounter());
        serial_debug.println(" )");
      #endif


        LoRaWAN.beginPacket();
        /* I1 - STATUS HEADER BYTE
        bit7 - error
        bit6 - alarm/interrupt
        bit5 - reserved 
        bit4 - reserved
        bit3 - ADCs
        bit2 - preassure
        bit1 - temperature+humidity
        bit0 - accelerometer
        */
        LoRaWAN.write(0x01);//I1

        //BASIC DATA
        int8_t t1 = (int8_t)hdc2080_temp;
        uint8_t t01 = (hdc2080_temp-t1)*100;
        LoRaWAN.write(t1);//T1
        LoRaWAN.write(t01);//T01
        int8_t h1 = (int8_t)hdc2080_hum;
        uint8_t h01 = (hdc2080_hum-h1)*100;
        LoRaWAN.write(h1);//H1
        LoRaWAN.write(h01);//H01
        uint16_t ap = (uint16_t)dsp310_pres;
        LoRaWAN.write(highByte(ap));//AP1 high byte
        LoRaWAN.write(lowByte(ap));//AP2 low byte
        LoRaWAN.write((int8_t)lis_movement/1000);//ACC
        
        //EXTENDED DATA
        uint16_t vdd = stm32l0_vdd*100;
        LoRaWAN.write(highByte(vdd));//VBAT
        LoRaWAN.write(lowByte(vdd));//VBAT
        int8_t tc1 = (int8_t)stm32l0_temp;
        uint8_t tc01 = (stm32l0_temp-tc1)*100;
        LoRaWAN.write(tc1);//TCPU1
        LoRaWAN.write(tc01);//TCPU01
        
        LoRaWAN.endPacket();
    }
    //delay(10000);

    
  //serial_debug.end();
  //USBDevice.detach();
  serial_debug.println("sleep for 60s");
  STM32L0.stop(60000);
}
