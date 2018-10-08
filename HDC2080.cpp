#include "HDC2080.h"

HDC2080::HDC2080(){
}

void HDC2080::begin(){

  #ifdef debug
    serial_debug.println("HDC2080::begin()");
  #endif

	temperatureRaw=0;
	humidityRaw=0;
	//config the temp sensor to read temp then humidity in one transaction
	//config the resolution to 14 bits for temp & humidity
  Wire.beginTransmission(ADDR);
  Wire.write(0x0e);
  Wire.write(0x80);
  Wire.endTransmission();
  
  Wire.beginTransmission(ADDR);
  Wire.write(0x0f);
  Wire.write(0x01);
  Wire.endTransmission();
}

void HDC2080::read(){

	//enable the measurement
	Wire.beginTransmission(ADDR);
	Wire.write(0x0f);
	Wire.write(0x01);
	Wire.endTransmission();
 
	delay(200);//wait for conversion
 
	Wire.beginTransmission(ADDR);
	Wire.write(0x00);
	Wire.endTransmission();
 
	delay(1);
 
	Wire.requestFrom(ADDR, (uint8_t)4);
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
