#ifndef HDC2080_H_
#define HDC2080_H_

#include <Arduino.h>
#include "Wire.h"

////#define debug
//#define serial_debug  Serial

#define HDC2080_ADDR 0x40

class HDC2080{
	public:
		HDC2080();
		boolean begin();
		void read();
		float getTemp();
		float getHum();

	private:
		uint16_t temperatureRaw;
		uint16_t humidityRaw;
};

#endif