/*
  AxNetwork.cpp - Networking interface for Ax2WDRobot (AP mode only)
  (c) Abrantix AG
*/

#include "Trace.h"
#include "AxNetwork.h"
#include "esp_mac.h"
#include <DNSServer.h>

AxNetwork AxNetworkInstance;
volatile bool apUp = false;
IPAddress LocalIP;
uint8_t baseMacAddress[6];
PIPCONFIG ipConfig;
char hostname[30];
bool networkChanged;
DNSServer dnsServer;

void AxNetwork::PeristNewIpConfig(PIPCONFIG ipCfg)
{
  AxEEPROM axEEPROM;
  TRACELN(F("AxNetwork: persisting new config, restarting"));
  memcpy(&EEPROMData.IPConfiguration, ipCfg, sizeof EEPROMData.IPConfiguration);
  axEEPROM.SaveEEPROM();
  delay(1000);
  ESP.restart();
}

AxNetwork::AxNetwork() : isBound(false)
{
}

const char* AxNetwork::GetHostname()
{
  return hostname;
}

const char* AxNetwork::GetNetworkTypeString()
{
  return "WIFI AP";
}

void AxNetwork::Setup(PIPCONFIG ipCfg)
{
  esp_efuse_mac_get_default(baseMacAddress);
  snprintf(hostname, sizeof(hostname), "AXTT-%02X%02X%02X%02X%02X%02X",
    baseMacAddress[0], baseMacAddress[1], baseMacAddress[2],
    baseMacAddress[3], baseMacAddress[4], baseMacAddress[5]);

  TRACELN("AxNetwork::Setup (AP mode)");
  ipConfig = ipCfg;

  const int wifiChannels[] = {1, 6, 11};
  WiFi.onEvent(AxNetwork::WiFiEvent);

  // Pick the least congested of the non-overlapping channels
  int channel = wifiChannels[random(0, 3)];
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int found = WiFi.scanNetworks();
  if (found > 0)
  {
    long load[3] = {0, 0, 0};
    for (int i = 0; i < found; i++)
    {
      int ch = WiFi.channel(i);
      long weight = 120 + WiFi.RSSI(i); // stronger neighbours weigh more
      if (weight < 1) weight = 1;
      // Channels 1/6/11 each cover roughly +/-2 channels
      for (int c = 0; c < 3; c++)
      {
        if (abs(ch - wifiChannels[c]) <= 2) load[c] += weight;
      }
    }
    int best = 0;
    for (int c = 1; c < 3; c++)
    {
      if (load[c] < load[best]) best = c;
    }
    channel = wifiChannels[best];
  }
  WiFi.scanDelete();
  TRACE("AxNetwork: AP channel: ");
  TRACELN(channel);

  WiFi.mode(WIFI_AP);
  esp_wifi_set_max_tx_power(78);
  WiFi.setSleep(false);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  WiFi.softAP(hostname, emptyString, channel);

  TRACE("AxNetwork: AP SSID: ");
  TRACELN(hostname);
}
void AxNetwork::Process()
{
  if (apUp)
  {
    if (!isBound)
    {
      isBound = true;
      networkChanged = true;
    }

    if (networkChanged)
    {
      if (ConnectionChanged)
      {
        MDNS.begin(hostname);
        dnsServer.start(53, "*", LocalIP);
        ConnectionChanged();
        networkChanged = false;
      }
    }

    dnsServer.processNextRequest();
  }
  else
  {
    isBound = false;
  }
}

bool AxNetwork::IsEthConnected()
{
  return apUp;
}

IPAddress AxNetwork::GetLocalIP()
{
  return LocalIP;
}

const char* AxNetwork::GetBaseMacAddress()
{
  return (const char*)baseMacAddress;
}

void AxNetwork::WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
    case ARDUINO_EVENT_WIFI_AP_START:
      TRACE("AP started, IP: ");
      LocalIP = WiFi.softAPIP();
      TRACELN(LocalIP);
      apUp = true;
      networkChanged = true;
      break;

    case ARDUINO_EVENT_WIFI_AP_STOP:
      TRACELN("AP stopped");
      apUp = false;
      networkChanged = true;
      break;

    default:
      TRACE("Unhandled WiFi event ");
      TRACELN(event);
      break;
  }
}
