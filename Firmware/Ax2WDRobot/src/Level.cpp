#include "Level.h"
#include "AxDisplay.h"
#include "Accelerometer.h"

Level LevelInstance;

static const int CX = 64;
static const int CY = 40;
static const int RADIUS = 22;
static const float PIXELS_PER_G = 45.0f;
static const float LEVEL_DEG = 1.5f;

void Level::Frame(float dt)
{
  (void)dt;
  float x, y, z;
  if(AccelerometerInstance.Read(x, y, z))
  {
    if(!hasData)
    {
      filtered[0] = x; filtered[1] = y; filtered[2] = z;
      hasData = true;
    }
    filtered[0] += 0.4f * (x - filtered[0]);
    filtered[1] += 0.4f * (y - filtered[1]);
    filtered[2] += 0.4f * (z - filtered[2]);
  }

  // Tilt angles in degrees: pitch around the y axis (x end up = positive), roll around the x axis (y end up = positive)
  float ax = constrain(filtered[0], -1.0f, 1.0f);
  float ay = constrain(filtered[1], -1.0f, 1.0f);
  float pitch = asinf(ax) * 57.2958f;
  float roll = asinf(ay) * 57.2958f;

  // Like a real bubble, it moves to the raised side: +x = up, +y = left
  float bx = -ay * PIXELS_PER_G;
  float by = -ax * PIXELS_PER_G;
  float len = sqrtf(bx * bx + by * by);
  float maxLen = RADIUS - 4;
  if(len > maxLen)
  {
    bx *= maxLen / len;
    by *= maxLen / len;
  }
  bool isLevel = fabsf(pitch) < LEVEL_DEG && fabsf(roll) < LEVEL_DEG;

  Adafruit_SSD1306* d = AxDisplayInstance.BeginDraw();
  d->setCursor(0, 4);
  d->printf("X%+.1f", pitch);
  d->setCursor(74, 4);
  d->printf("Y%+.1f", roll);

  d->drawCircle(CX, CY, RADIUS, SSD1306_WHITE);
  d->drawCircle(CX, CY, 5, SSD1306_WHITE);
  d->drawFastHLine(CX - RADIUS - 6, CY, 12, SSD1306_WHITE);
  d->drawFastHLine(CX + RADIUS - 6, CY, 12, SSD1306_WHITE);
  d->drawFastVLine(CX, CY - RADIUS - 2, 6, SSD1306_WHITE);
  d->drawFastVLine(CX, CY + RADIUS - 4, 6, SSD1306_WHITE);

  int px = CX + (int)roundf(bx);
  int py = CY + (int)roundf(by);
  if(isLevel)
  {
    d->fillCircle(px, py, 3, SSD1306_WHITE);
  }
  else
  {
    d->drawCircle(px, py, 3, SSD1306_WHITE);
  }
  AxDisplayInstance.EndDraw();
}
