#ifndef SENSOR_H_
#define SENSOR_H_

#include "Arduino.h"
#include "TimerMillis.h"
#include <STM32L0.h>
#include "lorawan.h"
#include "settings.h"
#include "status.h"
#include <Wire.h> 
#include <Adafruit_Sensor.h> // Install through Manager
#include <Dps310.h>// Install through Manager
#include "HDC2080.h"
#include "LIS2DH12.h"

extern boolean sensor_send_flag;

/**
 * @brief LoraWAN sensor packet setup - port 1
 * 
 * @details Important! 8bit and 16biut values must be in even numbers and aligned, packing works only if two 8 bit values are aligned with 16bit border
 */
struct sensorData_t{
  uint16_t temperature;
  uint16_t pressure;
  uint8_t  status;
  uint8_t humidity;
  uint8_t acclereometer;
  uint8_t dummy;
}__attribute__((packed));

union sensorPacket_t{
  sensorData_t data;
  byte bytes[sizeof(sensorData_t)];
};

static const uint8_t sensor_packet_port = 1;
extern sensorPacket_t sensor_packet;

void sensor_timer_callback(void);
void sensor_init(void);
void sensor_send(void);
void sensor_read(void);

#endif