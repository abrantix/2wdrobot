/*
  AxNetworkInstance.cpp - Networking interface for PayTestHub devices Frontend processor
  (c) Abrantix AG
*/

#include "Trace.h"
#include "AxNetwork.h"
#include <DNSServer.h>

AxNetwork AxNetworkInstance;
volatile bool EthConnected = false;
volatile bool Reconnect = false;
IPAddress LocalIP;
uint8_t baseMacAddress[6];
wifi_network_t networkMode;
PIPCONFIG ipConfig;
char hostname[30];
bool networkChanged;
bool stopWifiRequested = false;
DNSServer dnsServer;


void AxNetwork::PeristNewIpConfig(PIPCONFIG ipCfg)
{
  AxEEPROM axEEPROM;
  TRACE(F("Apply new network configuration mode "));
  TRACELNF(ipCfg->Mode, DEC);
  memcpy(&EEPROMData.IPConfiguration, ipCfg, sizeof EEPROMData.IPConfiguration);
  axEEPROM.SaveEEPROM();            
  delay(1000);
  //Restarts program from beginning        
  ESP.restart();
}

IPAddress AxNetwork::ArrayToIPAddress(char *bytes)
{
  return IPAddress(bytes[0], bytes[1], bytes[2], bytes[3]);
}

AxNetwork::AxNetwork()
{
  networkMode = NETWORK_UNDEFINED;
}

wifi_network_t AxNetwork::GetNetworkMode()
{
  return networkMode;
}

const char* AxNetwork::GetNetworkTypeString()
{
  if (0 > networkMode)
  {
    return "n/a";
  }
  switch (networkMode)
  {
    case 4:
      return "USB";
    case 3:
      return "WIFI AP";
    case 2:
      return "WIFI";
    case 1:
      return "STATIC";
    
    case 0:
    default:
      return "DHCP";
  }
}

const PIPCONFIG AxNetwork::GetIpConfig()
{
  return ipConfig;  
}

void AxNetwork::Setup(PIPCONFIG ipCfg)
{
  esp_efuse_mac_get_default(baseMacAddress);
  snprintf(hostname, sizeof(hostname), "AXTT-%02X%02X%02X%02X%02X%02X", baseMacAddress[0], baseMacAddress[1], baseMacAddress[2], baseMacAddress[3], baseMacAddress[4], baseMacAddress[5]);

  TRACELN(("AxNetwork::Setup"));
  EthConnected = false;
  WiFi.onEvent(AxNetwork::WiFiEvent);
  WiFi.disconnect(true);

  ipConfig = ipCfg;

  networkMode = (wifi_network_t)ipCfg->Mode;
  SetupNetwork(ipCfg->Mode);
}

void AxNetwork::SetupNetwork(char mode){
  const int wifiChannels[] = {1, 6, 11};

  switch (mode)
  {
    case WIRED_NETWORK_STATIC_IP:
      //Wired network Static IP
      TRACELN("AxNetwork: Configuring Static IP mode");
      ETH.config(ArrayToIPAddress(ipConfig->IP), ArrayToIPAddress(ipConfig->Gateway), ArrayToIPAddress(ipConfig->Subnet), ArrayToIPAddress(ipConfig->DNS), ArrayToIPAddress(ipConfig->DNS));
      break;

    case WIRELESS_STA:
      TRACEPRINTF("AxNetwork: Configuring WIFI AP mode for SSID '%s' PW '%s'", ipConfig->SSID, ipConfig->WifiPassword);
      WiFi.mode(WIFI_STA);
      WiFi.begin(ipConfig->SSID, ipConfig->WifiPassword);
      break;

    case WIRELESS_AP:
      TRACE("AxNetwork: Configuring WIFI SOFTAP mode for SSID ");
      TRACELN(hostname);
      WiFi.mode(WIFI_AP);
      //Choose an arbitrary WIFI channel 1, 6 or 11
      WiFi.softAP(hostname, emptyString, wifiChannels[random(0,2)]);
      break;

    case USB:
      TRACE("AxNetwork: USBSerialMode - nothing to do");
      break;

    case WIRED_DHCP:
    default:
      //Wired network DHCP mode
      TRACELN("AxNetwork: Configuring DHCP mode");
      break;
  }
}
void AxNetwork::Process()
{
  if(IsEthConnected())
  {
    if(!isBound)
    {
      isBound = true;
      networkChanged = true;
    }
    
    if(networkChanged) {
      if(ConnectionChanged) {
        MDNS.begin(hostname);
        // if DNSServer is started with "*" for domain name, it will reply with
        // provided IP to all DNS request
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

  if(Reconnect)
  {
    TRACELN("WIFI reconnect");
    WiFi.reconnect();
    Reconnect = false;
  }
  
  if(stopWifiRequested)
  {
    stopWifiRequested = false;
    StopWifi();
  }
}

bool AxNetwork::IsEthConnected()
{ 
  return EthConnected && ipConfig->Mode !=4;
}

IPAddress AxNetwork::GetLocalIP()
{
  return LocalIP;
}

const char* AxNetwork::GetBaseMacAddress()
{
  return (const char*)baseMacAddress;
}

void AxNetwork::StopWifi(){
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  TRACELN("Stopping Wifi");
}

void AxNetwork::WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      TRACELN("ETH Started");
      //set eth hostname here
      ETH.setHostname(hostname);
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      TRACELN("ETH Connected");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      TRACE("ETH MAC: ");
      TRACE(ETH.macAddress());
      TRACE(", IPv4: ");
      LocalIP = ETH.localIP();
      TRACE(LocalIP);
      if (ETH.fullDuplex()) {
        TRACE(", FULL_DUPLEX");
      }
      TRACE(", ");
      TRACE(ETH.linkSpeed());
      TRACELN("Mbps");
      EthConnected = true;
      networkMode = WIRED_DHCP;
      stopWifiRequested = true;
      networkChanged = true;
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      TRACELN("ETH Disconnected");
      EthConnected = false;
      networkMode = (wifi_network_t)ipConfig->Mode;
      SetupNetwork(ipConfig->Mode);
      networkChanged = true;
      break;

    case ARDUINO_EVENT_WIFI_AP_STOP:
      TRACELN("AP Stopped");
      if(networkMode != WIRED_DHCP) {
       EthConnected = false;
      }
      networkChanged = true;
      break;

    case ARDUINO_EVENT_ETH_STOP:
      TRACELN("ETH Stopped");
      EthConnected = false;
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      TRACELN("WIFI connected to STA");
      WiFi.setHostname(hostname);
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      TRACELN("WIFI disconnected from STA");
      if (WIRELESS_STA == networkMode)
      {
        EthConnected = false;
        Reconnect = true;
      }
      break;

    case ARDUINO_EVENT_WIFI_AP_START:
      if(networkMode != WIRED_DHCP) {
        TRACE("WIFI SoftAP: Obtained IP address: ");
        LocalIP = WiFi.softAPIP();
        TRACELN(LocalIP);
        EthConnected = true;
      }
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      TRACE("WIFI STA: Obtained IP address: ");
      LocalIP = WiFi.localIP();
      TRACELN(LocalIP);
      EthConnected = true;
      break;
    
    default:
      TRACE("Unhandled WIFI event ");
      TRACELN(event);
      break;
  }
}
