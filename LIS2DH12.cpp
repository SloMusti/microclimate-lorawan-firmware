#include "LIS2DH12.h"

LIS2DH12::LIS2DH12(){
}

boolean LIS2DH12::begin(){

  Wire.begin();
  #ifdef debug
    serial_debug.println("LIS2DH12::begin()");
  #endif

  uint8_t error_code; 
  uint8_t send_data[2];

  //check if present
  Wire.beginTransmission(ADDR);
  Wire.write(LIS2DH12_DUMMY_REG);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, (uint8_t)1);
  uint8_t dummy = Wire.read(); 

  if(dummy!=0x33){
      #ifdef debug
        serial_debug.print("LIS not found: 0x");
        serial_debug.println(dummy,HEX);
      #endif
    return false;
  }
  
  #ifdef debug
      serial_debug.print("LIS present: 0x");
      serial_debug.println(dummy,HEX);
  #endif

  Wire.beginTransmission(ADDR);
  Wire.write(LIS2DH12_CTRL_REG1);
  Wire.write(LIS2DH12_ODR_25HZ | LIS2DH12_LP_EN | LIS2DH12_Z_EN | LIS2DH12_Y_EN | LIS2DH12_X_EN);
  Wire.endTransmission();

  Wire.beginTransmission(ADDR);
  Wire.write(LIS2DH12_CTRL_REG4);
  Wire.write(LIS2DH12_BDU_EN | LIS2DH12_2G | LIS2DH12_HR_LP);
  Wire.endTransmission();

  Wire.beginTransmission(ADDR);
  Wire.write(LIS2DH12_CTRL_REG6);
  Wire.write(LIS2DH12_I2IA1_EN | LIS2DH12_INT2_ACT_EN | LIS2DH12_INT_POL_ACT_HIGH);
  Wire.endTransmission();

  Wire.beginTransmission(ADDR);
  Wire.write(LIS2DH12_ACT_THS);
  Wire.write(0x10); // 256 mg
  Wire.endTransmission();

  Wire.beginTransmission(ADDR);
  Wire.write(LIS2DH12_ACT_DUR);
  Wire.write(0x08);
  Wire.endTransmission();

}

void LIS2DH12::read(){
  // i2c
  Wire.beginTransmission(ADDR);
  Wire.write(LIS2DH12_OUT_X_LSB);
  Wire.endTransmission();
  
  Wire.requestFrom(ADDR, (uint8_t)6);
  acc_x_value = Wire.read(); 
  acc_x_value |= ((uint16_t)Wire.read()) << 8;
  acc_y_value = Wire.read(); 
  acc_y_value |= ((uint16_t)Wire.read()) << 8;
  acc_z_value = Wire.read();
  acc_z_value |= ((uint16_t)Wire.read()) << 8;

  #ifdef debug
    serial_debug.print("LIS x: ");
    serial_debug.println(acc_x_value);
    serial_debug.print("LIS y: ");
    serial_debug.println(acc_y_value);
    serial_debug.print("LIS z: ");
    serial_debug.println(acc_z_value);
  #endif
}
