// Handle sensors
#include <STM32L0.h>
#include <Wire.h> 
#include <Adafruit_Sensor.h> // Install through Manager
#include <Dps310.h>// Install through Manager
#include "HDC2080.h"
#include "LIS2DH12.h"

// I2C
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();
LIS2DH12 lis = LIS2DH12();

#define LIS_INT2  6 //PB2

// Dps310 object
Dps310 Dps310PressureSensor = Dps310();

// HDC2080 object
HDC2080 hdc2080 = HDC2080();

void ISR_LIS() {

    STM32L0.wakeup();

    #ifdef debug
        serial_debug.println("LIS (ISR_LIS) - interrupt on accel");
    #endif

}

void sensors_setup( void )
{
    pinMode(LIS_INT2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LIS_INT2), ISR_LIS, FALLING);

    // Each sensor should have a check to determine if present on boot
    // Code in reading should handle if sensor becomes unavailable
    // All sensors need to be placed in the low power mode

    //LIS2DH sensor
    lis.begin();
    //lis.setRange(LIS3DH_RANGE_2_G);
    //lis.setClick(2, 40);
    
    //DSO130 sensor
    Dps310PressureSensor.begin(Wire);

    //hdc2080
    hdc2080.begin();
     #ifdef debug
        serial_debug.println("sensors_setup()");
    #endif  
}

void read_sensors( void )
{
    float stm32l0_vdd = STM32L0.getVDDA();
    float stm32l0_temp = STM32L0.getTemperature();
  
    //LIS
    lis.read();      // get X Y and Z data at once
    float lis_movement=lis.acc_x_value+lis.acc_y_value+lis.acc_z_value;
  
    //DPOS130 read
    int32_t dsp310_temp;
    int32_t dsp310_pres;
    
    int ret = Dps310PressureSensor.measureTempOnce(dsp310_temp, 7);
    ret += Dps310PressureSensor.measurePressureOnce(dsp310_pres, 7);
    dsp310_pres=dsp310_pres/10;
  
    hdc2080.read();
    float hdc2080_temp = hdc2080.getTemp();
    float hdc2080_hum = hdc2080.getHum();

    packet.sensor.stat=   0x01;
    packet.sensor.t1  =   (int8_t)hdc2080_temp;
    packet.sensor.t01 =   (uint8_t)((hdc2080_temp-packet.sensor.t1)*100);
    packet.sensor.h1 =    (int8_t)hdc2080_hum;
    packet.sensor.h01 =   (uint8_t)((hdc2080_hum-packet.sensor.h1)*100);
    packet.sensor.ap =    (uint16_t)dsp310_pres;
    packet.sensor.acc=    (int8_t)(lis_movement/10);//ACC
    packet.sensor.vdd =   (uint16_t)(stm32l0_vdd*100);
    packet.sensor.tc1 =   (int8_t)stm32l0_temp;
    packet.sensor.tc01 =  (uint8_t)((stm32l0_temp-packet.sensor.tc1)*100);
    
    #ifdef debug
      /*serial_debug.println(""); 

      serial_debug.print("hdc2080_temp: "); serial_debug.print(hdc2080_temp);
      serial_debug.print(" 0x"); serial_debug.print(packet.sensor.t1,HEX); serial_debug.print(" 0x"); serial_debug.println((uint8_t)((hdc2080_temp-packet.sensor.t1)*100),HEX);
       
      serial_debug.print("hdc2080_hum: "); serial_debug.print(hdc2080_hum);
      serial_debug.print(" 0x"); serial_debug.print((int8_t)hdc2080_hum,HEX); serial_debug.print(" 0x"); serial_debug.println((uint8_t)((hdc2080_hum-packet.sensor.h1)*100),HEX);

      serial_debug.print("dsp310_pres: "); serial_debug.print(dsp310_pres);
      serial_debug.print(" 0x"); serial_debug.println(dsp310_pres,HEX);

      serial_debug.print("lis_movement: "); serial_debug.print(lis_movement);
      serial_debug.print(" 0x"); serial_debug.println((int8_t)(lis_movement/10),HEX);
      
      serial_debug.print("stm32l0_vdd: "); serial_debug.print(stm32l0_vdd);
      serial_debug.print(" 0x"); serial_debug.println((uint16_t)(stm32l0_vdd*100),HEX);
        
      serial_debug.print("stm32l0_temp: "); serial_debug.print(stm32l0_temp);
      serial_debug.print(" 0x"); serial_debug.print((int8_t)stm32l0_temp,HEX);serial_debug.print(" 0x"); serial_debug.println((uint8_t)((stm32l0_temp-packet.sensor.tc1)*100),HEX);
               
      serial_debug.print("dsp310_temp: "); serial_debug.println(dsp310_temp);
      
      serial_debug.print("dsp310_pres: "); serial_debug.println(dsp310_pres); 

      serial_debug.print("packet.bytes[");
      serial_debug.print(sizeof(sensorData_t));
      serial_debug.print("] ");
      for(int i = 0; i < sizeof(sensorData_t); i++){
        serial_debug.print(" 0x");
        serial_debug.print(packet.bytes[i],HEX);
      }
      serial_debug.println(""); */
    #endif  
}
