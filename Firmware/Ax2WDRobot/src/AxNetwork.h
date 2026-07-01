/*
  AxNetwork.h - Networking interface for Ax2WDRobot (AP mode only)
  (c) Abrantix AG
*/

#ifndef __AXNETWORK_H
#define __AXNETWORK_H

#include <WiFi.h>
#include <ESPmDNS.h>
#include "AxEEPROM.h"
#include "IpConfig.h"

typedef void (*CONNECTION_CHANGED_FCT)(void);

class AxNetwork 
{
public:
  AxNetwork();
  void Setup(PIPCONFIG ipCfg);
  void Process();
  bool IsEthConnected();
  IPAddress GetLocalIP();
  const char* GetBaseMacAddress();
  const char* GetHostname();
  const char* GetNetworkTypeString();
  void PeristNewIpConfig(PIPCONFIG ipCfg);
  CONNECTION_CHANGED_FCT ConnectionChanged;

private:
  static void WiFiEvent(WiFiEvent_t event);
  bool isBound:1;
};

extern AxNetwork AxNetworkInstance;

#endif
