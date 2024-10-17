/*
  AxDisplay.h - I2C Display interface for PayTestHub devices Frontend processor
  (c) Abrantix AG
*/

#ifndef __AXDISPLAY_H
#define __AXDISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AxNetwork.h"

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_SAVE_MILLIS  120000 //Number of milliseconds after the display will be blanked to protect against burn-in

class AxDisplay 
{
public:
  AxDisplay();
  void Process();
  void RefreshDisplay();
  void UpdateBatteryVoltage(uint16_t batteryVoltage);
  void UpdateDistance(float distance);
  void Init(TwoWire *twi);
  void DisableScreenSaver();
  void Display(const char* message);
  void DisplayCustom(const char* message);
  void DisplayBitmap(const unsigned char* bitmap, uint16_t length);


private:
  char customMessage[64];
  float distance;
  uint16_t currentBatteryVoltage;
  IPAddress currentDisplayIp;
  bool currentNetworkConnectionStatus:1;
  bool screenSave:1;
  bool isInitialized:1;
  Adafruit_SSD1306* display;
  unsigned long lastShowmillis;
  uint8_t currentYPosition;
  void NextLine();
};

extern AxDisplay AxDisplayInstance;

#endif
