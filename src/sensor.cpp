#include "sensor.h"

//#define debug
//#define serial_debug  Serial

TimerMillis sensor_timer;

sensorPacket_t sensor_packet;
boolean sensor_send_flag = false;

boolean gps_periodic = false;
boolean gps_triggered = false;
boolean gps_hot_fix = false;
boolean accelerometer_enabled = false;
boolean light_enabled = false;
boolean temperature_enabled = false;
boolean humidity_enabled = false;
boolean pressure_enabled = false;

// Sensor objects
LIS2DH12 lis = LIS2DH12();

#define LIS_INT2  6 //PB2

// Dps310 object
Dps310 Dps310PressureSensor = Dps310();

// HDC2080 object
HDC2080 hdc2080 = HDC2080();

/**
 * @brief called with a timer to perform sensor readign periodically
 * 
 */
void sensor_timer_callback(void)
{
  #ifdef debug
    serial_debug.println("sensor_timer_callback()");
  #endif
  sensor_send_flag = true;
}

/**
 * @brief load system functions form settings to variables
 * 
 */
void sensor_system_functions_load(void){
  // Initialize sensors to low-power mode
  /* system_functions - enable/disable certain features
  *    bit 0 - gps periodic enabled
  *    bit 1 - gps triggered enabled
  *    bit 2 - gps cold fix = 0 / hot fix = 1
  *    bit 3 - accelerometer enabled
  *    bit 4 - light sensor enabled
  *    bit 5 - temperature sensor enabled
  *    bit 6 - humidity sensor enabled
  *    bit 7 - pressure sensor enabled
  */
  uint8_t system_functions = settings_packet.data.system_functions;
  gps_periodic=bitRead(system_functions,0);
  gps_triggered=bitRead(system_functions,1);
  gps_hot_fix=bitRead(system_functions,2);
  accelerometer_enabled=bitRead(system_functions,3);
  light_enabled=bitRead(system_functions,4);
  temperature_enabled=bitRead(system_functions,5);
  humidity_enabled=bitRead(system_functions,6);
  pressure_enabled=bitRead(system_functions,7);
}

void sensor_accelerometer_init(){
  // Accelerometer
  if(accelerometer_enabled==true){
    if(lis.begin()==false){
      //sensor not found, report error
      bitSet(status_packet.data.system_functions_errors,3);
      //self-disable
      accelerometer_enabled=false;
    }
  }
  else{
    // put into low power - via i2c only if initialized
  }
}

void sensor_temperature_init(){
  // Temperature
  if(temperature_enabled==true){
    if(hdc2080.begin()==false){
      //sensor not found, report error
      bitSet(status_packet.data.system_functions_errors,5);
      //self-disable
      temperature_enabled=false;
    }
  }
  else{
    // put into low power - via i2c only if initialized
  }
}

void sensor_pressure_init(){
  // Pressure
  if(pressure_enabled==true){
    //check if present
    Wire.beginTransmission(0x77);
    Wire.write(0x0d);
    Wire.endTransmission();
    Wire.requestFrom(0x77, (uint8_t)1);
    uint8_t dummy = Wire.read(); 

    if(dummy!=0x10){
        #ifdef debug
          serial_debug.print("Dps310 not found: 0x");
          serial_debug.println(dummy,HEX);
        #endif
      //sensor not found, report error
      bitSet(status_packet.data.system_functions_errors,7);
      //self-disable
      pressure_enabled=false;
      return;
    }
  }
  Dps310PressureSensor.begin(Wire);
}

void sensor_gps_init(){
  // GPS
  // Check if GPS is enabled, then set it up accordingly
  if((gps_periodic==true)|(gps_triggered==true)){
    //initialize gps
    if(true){
      // GPS periodic error
      bitSet(status_packet.data.system_functions_errors,0);
      // GPS triggered error
      bitSet(status_packet.data.system_functions_errors,1);
      return;
    }
    if(gps_hot_fix==true){
      // enable backup power pin
    }
  }
  else{
    // disable GPS completely
    // disable power
    // disable backup
  }
}
/**
 * @brief initialize sensors upon boot or new settings
 * 
 * @details Make sure each sensors is firstlu properly reset and put into sleep and then if enabled in settings, initialize it
 * 
 */
void sensor_init(void){
  sensor_timer.stop();
  sensor_timer.start(sensor_timer_callback, 0, settings_packet.data.sensor_interval*60*1000);

  #ifdef debug
    serial_debug.print("sensor_init - sensor_timer_callback( ");
    serial_debug.print("interval: ");
    serial_debug.print(settings_packet.data.sensor_interval);
    serial_debug.println(" )");
  #endif

  sensor_system_functions_load();

  sensor_accelerometer_init();
  sensor_temperature_init();
  sensor_pressure_init();

  // If enabled then initialize them to operation mode
    
}

void sensor_read( void )
{
  float lis_movement=0;
  int32_t dsp310_temp=0;
  int32_t dsp310_pres=0;
  float hdc2080_temp=0;
  float hdc2080_hum=0;

  if(accelerometer_enabled==true){
    //LIS
    lis.read();      // get X Y and Z data at once
    lis_movement=lis.acc_x_value+lis.acc_y_value+lis.acc_z_value;
  }

  if(pressure_enabled==true){
    int ret = Dps310PressureSensor.measureTempOnce(dsp310_temp, 7);
    ret += Dps310PressureSensor.measurePressureOnce(dsp310_pres, 7);
    dsp310_pres=dsp310_pres/10;
  }

  if(temperature_enabled==true){
    hdc2080.read();
    hdc2080_temp = hdc2080.getTemp();
    hdc2080_hum = hdc2080.getHum();
  }

  sensor_packet.data.status=0x00;
  sensor_packet.data.temperature=(uint16_t)get_bits(hdc2080_temp,-20,80,16);
  sensor_packet.data.humidity=(uint8_t)get_bits(hdc2080_hum,0,100,8);
  sensor_packet.data.pressure=(uint16_t)get_bits(dsp310_pres,8000,12000,16);
  sensor_packet.data.acclereometer=(uint8_t)get_bits(lis_movement,-100,100,8);
    
  #ifdef debug
    serial_debug.print(" hdc2080_temp: "); serial_debug.print(hdc2080_temp);
    serial_debug.print(" hdc2080_hum: "); serial_debug.print(hdc2080_hum);
    serial_debug.print(" dsp310_pres: "); serial_debug.print(dsp310_pres);
    serial_debug.print(" lis_movement: "); serial_debug.print(lis_movement);     
    serial_debug.print(" dsp310_temp: "); serial_debug.println(dsp310_temp);
    serial_debug.print(" dsp310_pres: "); serial_debug.println(dsp310_pres); 
    serial_debug.println(""); 
  #endif  
}

/**
 * @brief acquire and send sensor data. 
 * 
 * @details Make sure to do this only for features enabled in the settings
 * 
 */
void sensor_send(void){
    //assemble information
    sensor_read();
    lorawan_send(sensor_packet_port, &sensor_packet.bytes[0], sizeof(sensorData_t));
}