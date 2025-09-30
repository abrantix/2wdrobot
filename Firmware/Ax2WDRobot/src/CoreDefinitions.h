#ifndef __CORE_DEFINITIONS
#define __CORE_DEFINITIONS

#include <Arduino.h>

#define MAJOR_VERSION 0x01
#define MINOR_VERSION 0x08

//Store Version info into Progmem with leading magic bytes "AXTTVERSION" to allow to parse firmware version information from binary flash file
const char VERSION_PROGMEM[] = {0x41, 0x58, 0x54, 0x54, 0x56, 0x45, 0x52, 0x53, 0x49, 0x4F, 0x4E, MAJOR_VERSION, MINOR_VERSION};

#define I2C_ADDR_DISPLAY    0x3C

// default media access control (ethernet hardware) address for the shield if no mac address has been defined in EEPROM
const byte MAC_DEFAULT[] = { 0x14, 0x07, 0xEF, 0xFF, 0xFF, 0xFF };

#define EEPROM_MAGIC "\x59\x41\x4D\x02"

#define PRODUCT_NAME "AX2WDROBOT"
#define HOSTNAME_PREFIX "AX2WDROBOT"
#include <Arduino.h>
#include <SPI.h>
#include <esp_wifi.h>
#include "Trace.h"
#include "Pinout.h"
#include "AxNetwork.h"
#include "AxWebServer.h"
#include "Ports.h"
#include "AxNetwork.h"
#include "AxDisplay.h"
#include "AxButton.h"
#include "AxEEPROM.h"
#include "soc/efuse_reg.h"
#include "esp_efuse.h"

extern SemaphoreHandle_t TwiMutex;

#endif
