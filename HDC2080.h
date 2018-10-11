#include <Arduino.h>
#include "Wire.h"

//#define debug
#define serial_debug  Serial1

#define ADDR 0x40

class HDC2080{
	public:
		HDC2080();
		void begin();
		void read();
		float getTemp();
		float getHum();

	private:
		uint16_t temperatureRaw;
		uint16_t humidityRaw;
};
