// Handle sensors
#include <STM32L0.h>
#include <Wire.h> 
#include <Adafruit_LIS3DH.h> // Install through Manager
#include <Adafruit_Sensor.h> // Install through Manager
#include <Dps310.h>// Install through Manager
#include "HDC2080.h"

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
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
    pinMode(LIS_INT2, INPUT);
    attachInterrupt(digitalPinToInterrupt(LIS_INT2), ISR_LIS, RISING);

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

void read_sensors( void )
{
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
    
    #ifdef debug
      /*serial_debug.println(""); 
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
      serial_debug.println(hdc2080_hum);  */
    #endif  
    
    packet.sensor.stat=0x01;
    packet.sensor.t1 = (int8_t)hdc2080_temp;
    packet.sensor.t01 = (hdc2080_temp-packet.sensor.t1)*100;
    packet.sensor.h1 = (int8_t)hdc2080_hum;
    packet.sensor.h01 = (hdc2080_hum-packet.sensor.h1)*100;
    packet.sensor.ap = (uint16_t)dsp310_pres;
    packet.sensor.acc=(int8_t)lis_movement/1000;//ACC
    packet.sensor.vdd = stm32l0_vdd*100;
    packet.sensor.tc1 = (int8_t)stm32l0_temp;
    packet.sensor.tc01 = (stm32l0_temp-packet.sensor.tc1)*100;
}
