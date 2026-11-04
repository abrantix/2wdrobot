#include <Wire.h>
#include "Trace.h"
#include "AxDisplay.h"
#include "CoreDefinitions.h"
#include "SplashLogo.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

AxDisplay AxDisplayInstance;

AxDisplay::AxDisplay()
{
  isInitialized = false;
  infoUntil = 0;
}

void AxDisplay::Init(TwoWire *twi)
{
  //We're reducing the display I2C clock from 400000 to 100000 - it seems some displays might not work reliable at 400000
  xSemaphoreTake(TwiMutex, portMAX_DELAY);
  digitalWrite(TWI_SEL_PIN, HIGH);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, twi, -1, 100000UL, 100000UL);
	display->begin(SSD1306_SWITCHCAPVCC, I2C_ADDR_DISPLAY);
  display->setRotation(2);
  digitalWrite(TWI_SEL_PIN, LOW);
  xSemaphoreGive(TwiMutex);
  isInitialized = true;
	RefreshDisplay();
}

void AxDisplay::Process()
{
  if(gameMode)
  {
    lastShowmillis = millis(); // no screen saver while playing
    return;
  }

  if(infoUntil)
  {
    if((long)(millis() - infoUntil) < 0)
    {
      return;
    }
    infoUntil = 0;
    RefreshDisplay();
  }

	bool refresh = false;
  bool newNetworkConnectionStatus = AxNetworkInstance.IsEthConnected();
  if(currentNetworkConnectionStatus != newNetworkConnectionStatus)
  {
    TRACELN("display: network connection status changed.");
	  refresh = true;
	  currentNetworkConnectionStatus = newNetworkConnectionStatus;
  }

  if(AxNetworkInstance.IsEthConnected())
  {
	  IPAddress localAddress = AxNetworkInstance.GetLocalIP();
  	if(0 != memcmp(&localAddress, &currentDisplayIp, sizeof(IPAddress)))
  	{
  		//IP changed, update screen;
  		currentDisplayIp = localAddress;
  		refresh = true;
  	}
  }
  
  if(refresh)
  {
	  RefreshDisplay();
  }
  else
  {
    if(!screenSave && millis() - lastShowmillis >= SCREEN_SAVE_MILLIS)
    {
      TRACELN("Display: Screensave");
      screenSave = true;
      xSemaphoreTake(TwiMutex, portMAX_DELAY);
      digitalWrite(TWI_SEL_PIN, HIGH);
      display->clearDisplay();
      display->display();
      digitalWrite(TWI_SEL_PIN, LOW);
      xSemaphoreGive(TwiMutex);
    }
  }
}


void AxDisplay::SetGameMode(bool active)
{
  gameMode = active;
  if(active)
  {
    infoUntil = 0;
    screenSave = false;
  }
  else
  {
    RefreshDisplay();
  }
}

Adafruit_SSD1306* AxDisplay::BeginDraw()
{
  xSemaphoreTake(TwiMutex, portMAX_DELAY);
  digitalWrite(TWI_SEL_PIN, HIGH);
  display->clearDisplay();
  display->setFont();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);
  display->setTextWrap(false);
  return display;
}

void AxDisplay::EndDraw()
{
  display->display();
  digitalWrite(TWI_SEL_PIN, LOW);
  xSemaphoreGive(TwiMutex);
}

void AxDisplay::DisableScreenSaver()
{
  RefreshDisplay();
}

void AxDisplay::DisplayCustom(const char* message)
{
  strcpy(customMessage, message);
}

void AxDisplay::Display(const char* message)
{
  TRACELN("Display: custom message");
  xSemaphoreTake(TwiMutex, portMAX_DELAY);
  digitalWrite(TWI_SEL_PIN, HIGH);
  display->clearDisplay();
  display->setFont();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);
  currentYPosition = 4;
  display->setCursor(0, currentYPosition); 
  display->cp437(true);
  display->write(message);
  display->display();
  digitalWrite(TWI_SEL_PIN, LOW);
  xSemaphoreGive(TwiMutex);
}

void AxDisplay::NextLine()
{
  currentYPosition += 16;
  display->setCursor(0,currentYPosition);
}

// Approximate Li-ion discharge curve (mV -> %), calibrated so that a full cell reads ~4.04V on this board
static int BatteryPercent(uint16_t mv)
{
  static const uint16_t curveMv[] = {3300, 3500, 3600, 3700, 3800, 3900, 3980, 4040};
  static const uint8_t curvePct[] = {0, 5, 15, 30, 50, 70, 90, 100};
  const int n = sizeof(curveMv) / sizeof(curveMv[0]);
  if(mv <= curveMv[0]) return 0;
  if(mv >= curveMv[n - 1]) return 100;
  for(int i = 1; i < n; i++)
  {
    if(mv <= curveMv[i])
    {
      return curvePct[i - 1] + (curvePct[i] - curvePct[i - 1]) * (mv - curveMv[i - 1]) / (curveMv[i] - curveMv[i - 1]);
    }
  }
  return 100;
}

void AxDisplay::RefreshDisplay()
{
  // Don't overwrite the connection info while it is shown
  if(gameMode || (infoUntil && (long)(millis() - infoUntil) < 0))
  {
    return;
  }
  if(isInitialized)
  {
    //TRACELN("Display: refresh");
    xSemaphoreTake(TwiMutex, portMAX_DELAY);
    digitalWrite(TWI_SEL_PIN, HIGH);
    display->clearDisplay();
    display->setFont();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setTextWrap(false);
    // The top rows of the panel are cut off, so the first line starts at y=4
    currentYPosition = 4;
    display->setCursor(0, currentYPosition);

    display->print(AxNetworkInstance.GetHostname());
    NextLine();
    if(currentNetworkConnectionStatus)
    {
        display->print(currentDisplayIp);
    }
    else
    {
      display->print(AxNetworkInstance.GetNetworkTypeString());
      display->print(" - No network");
    }
    NextLine();
    if(customMessage[0] == 0 && accelValid)
    {
      display->printf("x%+.2f y%+.2f z%+.2f", accel[0], accel[1], accel[2]);
    }
    else
    {
      display->print(customMessage);
    }
    NextLine();
    if(distance > 300 || distance <= 0)
    {
      display->print("n/a");   // echo error / out of range
    }
    else
    {
      display->printf("%gcm", distance);
    }

    // Battery: mini icon (fill = charge) at the right edge, percentage left of it
    int percent = BatteryPercent(currentBatteryVoltage);
    const int iconW = 12, iconH = 7;
    int iconX = 127 - iconW - 2;
    display->drawRect(iconX, currentYPosition, iconW, iconH, SSD1306_WHITE);
    display->fillRect(iconX + iconW, currentYPosition + 2, 2, iconH - 4, SSD1306_WHITE);
    display->fillRect(iconX + 2, currentYPosition + 2, (iconW - 4) * percent / 100, iconH - 4, SSD1306_WHITE);
    char pct[8];
    snprintf(pct, sizeof(pct), "%d%%", percent);
    display->setCursor(iconX - 4 - 6 * strlen(pct), currentYPosition);
    display->print(pct);
    display->display();
    
    lastShowmillis = millis();
    screenSave = false;
    digitalWrite(TWI_SEL_PIN, LOW);
    xSemaphoreGive(TwiMutex);
  }
}


void AxDisplay::ShowConnectInfo(uint32_t durationMillis)
{
  if(!isInitialized)
  {
    return;
  }
  IPAddress ip = WiFi.softAPIP();
  String ipString = (ip == IPAddress(0, 0, 0, 0)) ? String("192.168.4.1") : ip.toString();

  xSemaphoreTake(TwiMutex, portMAX_DELAY);
  digitalWrite(TWI_SEL_PIN, HIGH);
  display->clearDisplay();
  display->setFont();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);
  display->setTextWrap(false);
  display->setCursor(0, 4);
  display->print("1. Connect WiFi to:");
  display->setCursor(0, 15);
  display->print(AxNetworkInstance.GetHostname());
  display->setCursor(0, 30);
  display->print("2. Open in browser:");
  display->setCursor(0, 41);
  display->print("http://");
  display->print(ipString);
  display->display();
  digitalWrite(TWI_SEL_PIN, LOW);
  xSemaphoreGive(TwiMutex);

  infoUntil = millis() + durationMillis;
}

void AxDisplay::ShowSplash()
{
  if(!isInitialized)
  {
    return;
  }
  char version[16];
  snprintf(version, sizeof(version), "V%02d.%02d", MAJOR_VERSION, MINOR_VERSION);

  xSemaphoreTake(TwiMutex, portMAX_DELAY);
  digitalWrite(TWI_SEL_PIN, HIGH);
  display->clearDisplay();
  display->drawBitmap(0, 0, SplashLogo, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display->setFont();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);
  display->setTextWrap(false);
  int16_t x1, y1;
  uint16_t w, h;
  display->getTextBounds(version, 0, 0, &x1, &y1, &w, &h);
  display->setCursor((SCREEN_WIDTH - w) / 2, SCREEN_HEIGHT - h - 4);
  display->print(version);
  display->display();
  digitalWrite(TWI_SEL_PIN, LOW);
  xSemaphoreGive(TwiMutex);
}

 void AxDisplay::DisplayBitmap(const unsigned char* bitmap, uint16_t length)
 {
  digitalWrite(TWI_SEL_PIN, HIGH);
  display->clearDisplay();
  display->drawBitmap(0, 0, (const uint8_t*)bitmap, 128, 64, BLACK, WHITE);
  display->display();
  digitalWrite(TWI_SEL_PIN, LOW);
 }

void AxDisplay::UpdateAcceleration(float x, float y, float z)
{
  // Redrawing blocks the loop (slow I2C), so only refresh on a clear change
  const float threshold = 0.04f;
  bool changed = !accelValid || fabsf(x - accel[0]) > threshold || fabsf(y - accel[1]) > threshold || fabsf(z - accel[2]) > threshold;
  if(changed)
  {
    accel[0] = x;
    accel[1] = y;
    accel[2] = z;
    accelValid = true;
    RefreshDisplay();
  }
}

void AxDisplay::UpdateBatteryVoltage(uint16_t batteryVoltage)
{
  if(currentBatteryVoltage != batteryVoltage)
  {
    currentBatteryVoltage = batteryVoltage;
    RefreshDisplay();
  }
}

void AxDisplay::UpdateDistance(float distance)
{
  if(this->distance != distance)
  {
    this->distance = distance;
    RefreshDisplay();
  }
}
