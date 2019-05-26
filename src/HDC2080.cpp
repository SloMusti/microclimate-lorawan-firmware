#include "HDC2080.h"

//#define debug
//#define serial_debug  Serial

HDC2080::HDC2080(){
}

boolean HDC2080::begin(){

  #ifdef debug
    serial_debug.println("HDC2080::begin()");
  #endif

  //check if present The DeviceID for theHDC2080is 0x07D0
  Wire.beginTransmission(HDC2080_ADDR);
  Wire.write(0xfe);
  Wire.endTransmission();
  Wire.requestFrom(HDC2080_ADDR, (uint8_t)2);
  uint16_t dummy  = Wire.read();
  dummy = dummy | (unsigned int)Wire.read() << 8;

  if(dummy!=0x07d0){
      #ifdef debug
        serial_debug.print("HDC2080 not found: 0x");
        serial_debug.println(dummy,HEX);
      #endif
    return false;
  }

	temperatureRaw=0;
	humidityRaw=0;
	//config the temp sensor to read temp then humidity in one transaction
	//config the resolution to 14 bits for temp & humidity
  Wire.beginTransmission(HDC2080_ADDR);
  Wire.write(0x0e);
  Wire.write(0x80);
  Wire.endTransmission();
  
  Wire.beginTransmission(HDC2080_ADDR);
  Wire.write(0x0f);
  Wire.write(0x01);
  Wire.endTransmission();
  return true;
}

void HDC2080::read(){

	//enable the measurement
	Wire.beginTransmission(HDC2080_ADDR);
	Wire.write(0x0f);
	Wire.write(0x01);
	Wire.endTransmission();
 
	delay(200);//wait for conversion
 
	Wire.beginTransmission(HDC2080_ADDR);
	Wire.write(0x00);
	Wire.endTransmission();
 
	delay(1);
 
	Wire.requestFrom(HDC2080_ADDR, (uint8_t)4);
	temperatureRaw = Wire.read();
	temperatureRaw = temperatureRaw | (unsigned int)Wire.read() << 8 ;
	humidityRaw = Wire.read();
	humidityRaw = humidityRaw | (unsigned int)Wire.read() << 8;
}

//returns temp in celcius
float HDC2080::getTemp(){

	// (rawTemp/2^16)*165 - 40
	return ( (float)temperatureRaw )*165/65536 - 40;
}

float HDC2080::getHum(){

	//(rawHumidity/2^16)*100
	return ( (float)humidityRaw )*100/65536;
}
