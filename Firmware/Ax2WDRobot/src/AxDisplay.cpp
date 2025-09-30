#include <Wire.h>
#include "Trace.h"
#include "AxDisplay.h"
#include "CoreDefinitions.h"
#include "FreeSansBold7pt7b.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

AxDisplay AxDisplayInstance;

AxDisplay::AxDisplay()
{
  isInitialized = false;
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
  display->setFont(&FreeSansBold7pt7b);
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);
  currentYPosition = 10;
  display->setCursor(0, currentYPosition); 
  display->cp437(true);
  display->write(message);
  display->display();
  digitalWrite(TWI_SEL_PIN, LOW);
  xSemaphoreGive(TwiMutex);
}

void AxDisplay::NextLine()
{
  currentYPosition += 17;
  display->setCursor(0,currentYPosition);
}

void AxDisplay::RefreshDisplay()
{
  if(isInitialized)
  {
    //TRACELN("Display: refresh");
    xSemaphoreTake(TwiMutex, portMAX_DELAY);
    digitalWrite(TWI_SEL_PIN, HIGH);
    const char* mac = AxNetworkInstance.GetBaseMacAddress();
    display->clearDisplay();
    display->setFont(&FreeSansBold7pt7b);
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setTextWrap(false);
    currentYPosition = 10;
    display->setCursor(0, currentYPosition); 
  
    display->printf("%02X-%02X-%02X-%02X-%02X-%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    NextLine();
  
    if(currentNetworkConnectionStatus)
    {
        switch (AxNetworkInstance.GetNetworkMode())
        {
          case USB:
           display->print("USB - ready");
           break;
          case WIRELESS_STA:
          case WIRELESS_AP:
          case WIRED_DHCP:
          default:
            display->print(currentDisplayIp);
            break;
        }
    }
    else
    {
      display->print(AxNetworkInstance.GetNetworkTypeString());
      display->print(" - No network");
    }
    NextLine();
    display->print(customMessage);
    NextLine();
    display->printf("%4gcm          %.2fV", distance, currentBatteryVoltage/1000.0);
    display->display();
    
    lastShowmillis = millis();
    screenSave = false;
    digitalWrite(TWI_SEL_PIN, LOW);
    xSemaphoreGive(TwiMutex);
  }
}


 void AxDisplay::DisplayBitmap(const unsigned char* bitmap, uint16_t length)
 {
  digitalWrite(TWI_SEL_PIN, HIGH);
  display->clearDisplay();
  display->drawBitmap(0, 0, (const uint8_t*)bitmap, 128, 64, BLACK, WHITE);
  display->display();
  digitalWrite(TWI_SEL_PIN, LOW);
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
