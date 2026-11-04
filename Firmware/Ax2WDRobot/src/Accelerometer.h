/*
  Accelerometer.h - LIS2DW12 3-axis accelerometer (I2C, same bus state as the display)
*/

#ifndef __ACCELEROMETER_H
#define __ACCELEROMETER_H

#include <Arduino.h>
#include <Wire.h>

#define I2C_ADDR_ACCEL 0x19
#define ACCEL_INTERVAL_MILLIS 250

class Accelerometer
{
public:
  void Init(TwoWire *twi);
  void Process();
  bool Read(float &x, float &y, float &z);
  bool IsAvailable() { return available; }

private:
  TwoWire *wire = nullptr;
  bool available = false;
  unsigned long lastReadMillis = 0;
  bool WriteReg(uint8_t reg, uint8_t value);
  bool ReadRegs(uint8_t reg, uint8_t *data, uint8_t length);
};

extern Accelerometer AccelerometerInstance;

#endif
