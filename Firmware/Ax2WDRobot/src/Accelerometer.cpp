#include "Accelerometer.h"
#include "Trace.h"
#include "AxDisplay.h"
#include "CoreDefinitions.h"

#define LIS2DW12_WHO_AM_I   0x0F
#define LIS2DW12_ID         0x44
#define LIS2DW12_CTRL1      0x20
#define LIS2DW12_CTRL6      0x25
#define LIS2DW12_OUT_X_L    0x28

Accelerometer AccelerometerInstance;

bool Accelerometer::WriteReg(uint8_t reg, uint8_t value)
{
  wire->beginTransmission(I2C_ADDR_ACCEL);
  wire->write(reg);
  wire->write(value);
  return wire->endTransmission() == 0;
}

bool Accelerometer::ReadRegs(uint8_t reg, uint8_t *data, uint8_t length)
{
  wire->beginTransmission(I2C_ADDR_ACCEL);
  wire->write(reg);
  if(wire->endTransmission(false) != 0)
  {
    return false;
  }
  if(wire->requestFrom((uint8_t)I2C_ADDR_ACCEL, length) != length)
  {
    return false;
  }
  for(uint8_t i = 0; i < length; i++)
  {
    data[i] = wire->read();
  }
  return true;
}

void Accelerometer::Init(TwoWire *twi)
{
  wire = twi;
  uint8_t id = 0;

  xSemaphoreTake(TwiMutex, portMAX_DELAY);
  digitalWrite(TWI_SEL_PIN, HIGH);
  if(ReadRegs(LIS2DW12_WHO_AM_I, &id, 1) && id == LIS2DW12_ID)
  {
    // 50 Hz, high performance mode, +-2g, low noise
    available = WriteReg(LIS2DW12_CTRL1, 0x44) && WriteReg(LIS2DW12_CTRL6, 0x04);
  }
  digitalWrite(TWI_SEL_PIN, LOW);
  xSemaphoreGive(TwiMutex);

  TRACEPRINTF("Accelerometer: %s\n", available ? "LIS2DW12 ready" : "not found");
}

bool Accelerometer::Read(float &x, float &y, float &z)
{
  if(!available)
  {
    return false;
  }

  uint8_t raw[6];
  xSemaphoreTake(TwiMutex, portMAX_DELAY);
  digitalWrite(TWI_SEL_PIN, HIGH);
  bool ok = ReadRegs(LIS2DW12_OUT_X_L, raw, sizeof(raw));
  digitalWrite(TWI_SEL_PIN, LOW);
  xSemaphoreGive(TwiMutex);

  if(ok)
  {
    // 14 bit left-justified in 16 bit, 0.244 mg/LSB at +-2g in high performance mode
    const float gPerLsb = 0.000244f;
    x = ((int16_t)(raw[0] | (raw[1] << 8)) >> 2) * gPerLsb;
    y = ((int16_t)(raw[2] | (raw[3] << 8)) >> 2) * gPerLsb;
    z = ((int16_t)(raw[4] | (raw[5] << 8)) >> 2) * gPerLsb;
  }
  return ok;
}

void Accelerometer::Process()
{
  if(!available || millis() - lastReadMillis < ACCEL_INTERVAL_MILLIS)
  {
    return;
  }
  lastReadMillis = millis();

  float x, y, z;
  if(Read(x, y, z))
  {
    AxDisplayInstance.UpdateAcceleration(x, y, z);
  }
}
