/*
  AxWebServer.cpp - Web interface for Ax ESP32 products
  (c) Abrantix AG
*/


#include <Update.h>
#include <SPIFFS.h>
#include "AxWebServer.h"
#include "Trace.h"
#include "CoreDefinitions.h"
#include "esp_camera.h"
#include "sensor.h"
#include "WiFi.h"
#include "Font.h"
#include "WebPages.h"
#include "AxEEPROM.h"
#include "CameraHandler.h"
#include "AsyncStreamResponse.h"
#include "MotorDriver.hpp"
#include "ObstacleAvoidance.hpp"
#include "Style.h"
#include "Image.h"
#include "Icon.h"



/*************** Camera specific section - grabbed/rewritten from esp32-cam app_httpd.cpp example *************/

static bool captureRunning = false;

typedef struct
{
    AsyncWebServerResponse *response;
    size_t len;
} jpg_chunking_t;


static void capture_handler(AsyncWebServerRequest *request)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();
    char ts[32];
    uint8_t * buf;              /*!< Pointer to the pixel data */
    size_t len;                 /*!< Length of the buffer in bytes */
    bool error;

    AsyncWebServerResponse *response;

    if(captureRunning)
    {
        TRACELN("Capture already running: try later");
        response = request->beginResponse(429, "text/plain", "Try later");
        error = true;
    }
    else
    {
      //TRACELN("Capture: get FB");
      fb = esp_camera_fb_get();
      snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);

      if (!fb)
      {
          TRACELN("Camera capture failed");
          response = request->beginResponse(500, "text/plain", "ERROR: fb");
          error = true;
      }
      else
      {
        if (fb->format == PIXFORMAT_JPEG)
        {
            buf = fb->buf;
            len = fb->len;
            //TRACELN("Capture: prepare chunked response");
            response = request->beginChunkedResponse("image/jpg", [len, buf](uint8_t* buffer, size_t maxLen, size_t alreadySent) -> size_t {
            if (len - alreadySent >= maxLen) {
              memcpy(buffer, buf + alreadySent, maxLen);
              return maxLen;
            } else {  // last chunk and then 0
              memcpy(buffer, buf + alreadySent, len - alreadySent);
              return len - alreadySent;
            }
          });
        }
        else
        {
            TRACELN("ERROR: Only JPEG supported");
            response = request->beginResponse(500, "text/plain", "ERROR: NOJPG");
            error = true;
        }
        esp_camera_fb_return(fb);
      }
    }
    if(!error)
    {
      response->addHeader("X-Timestamp", (const char *)ts);
      response->addHeader("Content-Disposition", "inline; filename=capture.jpg");
      response->addHeader("Access-Control-Allow-Origin", "*");
    }
    response->addHeader("Connection", "close");
    request->send(response);

    int64_t fr_end = esp_timer_get_time();
    TRACEPRINTF("JPG: %uB %ums\n", (uint32_t)(len), (uint32_t)((fr_end - fr_start) / 1000));
}

// Track single active JPEG stream (close previous when new one connects)
static AsyncClient* activeStreamClient = nullptr;
static AsyncJpegStreamResponse* activeStreamResponse = nullptr;
static void streamJpg(AsyncWebServerRequest *request){
  // Gracefully terminate previous stream by ending its client without forced close
  if(activeStreamClient){
    // initiate half-close if connected; AsyncWebServer will cleanup request & response
    if(activeStreamClient->connected()) {
      activeStreamClient->close(); // normal close (no "true" forced abort)
    }
    activeStreamClient = nullptr;
    activeStreamResponse = nullptr;
  }
  AsyncJpegStreamResponse *response = new AsyncJpegStreamResponse(66); // ~15 FPS pacing
  if(!response){
    request->send(501);
    return;
  }
  response->addHeader("Access-Control-Allow-Origin", "*");
  response->addHeader("X-Framerate", "15");
  // Capture client and response; ensure pointers cleared on disconnect
  request->onDisconnect([](){ activeStreamClient = nullptr; activeStreamResponse = nullptr; });
  activeStreamClient = request->client();
  activeStreamResponse = response;
  request->send(response);
}

/*********************************************************************************************/

static bool AxWebServer_GetIpAddressFromParam(AsyncWebServerRequest *request, std::array<const char*, 4> arguments, char* ip)
{
  bool success = true;
  for(int i=0; i<4; i++)
  {
    if(request->hasParam(arguments[i]))
    {
      byte addressByte = (byte)request->getParam(arguments[i])->value().toInt();
      ip[i] = addressByte;
    }
    else
    {
      success = false;
      break;
    }
  }

  return success;
}

static bool AxWebServer_ProcessRobotControlPage(AsyncWebServerRequest *request)
{
  bool withParam = false;
  if(request->hasParam("go"))
  {
    withParam = true;
    String goString = request->getParam("go")->value();
    if(goString.equals("forward"))
    {
      TRACELN("forward");
      MotorDriverInstance.Forward();
    }
    else if(goString.equals("backward"))
    {
      TRACELN("backward");
      MotorDriverInstance.Backward();
    }
    else if(goString.equals("left"))
    {
      TRACELN("left");
      MotorDriverInstance.Left();
    }
    else if(goString.equals("right"))
    {
      TRACELN("right");
      MotorDriverInstance.Right();
    }
    else if(goString.equals("stop"))
    {
      TRACELN("stop");
      MotorDriverInstance.Stop();
    }
    else
    {
      withParam = false;
      TRACEPRINTF("Unknown go-string %s\n", goString.c_str());
    }
  }
  else if(request->hasParam("x") && request->hasParam("y"))
  {
    withParam = true;
    String xString = request->getParam("x")->value();
    String yString = request->getParam("y")->value();
    TRACEPRINTF("go x:%s y:%s\n", xString, yString);
    MotorDriverInstance.JoyStickControl(xString.toFloat(), yString.toFloat());
  }
  return withParam;
}

static String TemplateProcessor(const String& var)
{
  if(var == "WEB_TCP_PORT")
    return String(EEPROMData.WebIfPort);
  if(var == "NETWORKTYPE_CHECKED_2")
    return EEPROMData.IPConfiguration.Mode == 2 ? String("checked") : String();
  if(var == "NETWORKTYPE_CHECKED_3")
    return EEPROMData.IPConfiguration.Mode == 3 ? String("checked") : String();
  if(var == "SSID")
    return EEPROMData.IPConfiguration.Mode == 2 ? String(EEPROMData.IPConfiguration.SSID) : String();
  if(var == "FIRMWARE_VERSION")
    {
      char buffer[30];
      snprintf(buffer, sizeof(buffer), "V%02d.%02d", MAJOR_VERSION, MINOR_VERSION);
      return String(buffer);
    }
  if(var == "MAC_ADDRESS")
    {
      char buffer[30];
      const char* mac = AxNetworkInstance.GetBaseMacAddress();
      snprintf(buffer, sizeof(buffer), "%02X-%02X-%02X-%02X-%02X-%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      return String(buffer);
    }
  return String();
}

static bool AxWebServer_ProcessNetworkConfigPage(AsyncWebServerRequest *request)
{
  AsyncWebServerResponse *response;
  IPCONFIG newIpConfig;

  bool success = true;
  if(request->hasParam("networktype"))
  {
    if(request->hasParam("Port") && request->hasParam("networktype"))
    {
      EEPROMData.WebIfPort = request->getParam("Port")->value().toInt();
      AxEEPROMInstance.SaveEEPROM();
      int networkType = request->getParam("networktype")->value().toInt();
      newIpConfig.Mode = (char)networkType;
      
      switch(networkType)
      {
        case 2:
        //Wifi STA
        if(request->hasParam("SSID") && request->hasParam("Password"))
        {
          request->getParam("SSID")->value().toCharArray(newIpConfig.SSID, 63);
          request->getParam("Password")->value().toCharArray(newIpConfig.WifiPassword, 63);
          if(strlen(newIpConfig.SSID) == 0 || strlen(newIpConfig.WifiPassword) == 0)
          {
            TRACELN("Error: SSID or password is empty");
            success = false;
          }
        }
        else
        {
          success = false;
        }
        break;
        
        case 3:
        //Wifi softAP
        break;

        default:
        success = false;
        break;
      }
    }
    else
    {
      success = false;
    }

    if(success)
    {
      response = request->beginResponse(200, "text/plain", "Network Configuration changed");
      response->addHeader("Connection", "close");
      request->send(response);
      AxNetworkInstance.PeristNewIpConfig(&newIpConfig);
    }
    else
    {
      response = request->beginResponse(400, "text/plain", "ERROR: Invalid arguments");
      response->addHeader("Connection", "close");
      request->send(response);
    }
  }
  else
  {
    request->send(SPIFFS, "/networkconfig.html", "text/html", false, TemplateProcessor);
  }

  return success;
}

static AsyncWebServer *webServer;
static AsyncWebServer *redirectWebServer;
static int uploadStatusCode;
const char* uploadStatusMessage;
static String fileName;
static bool fpUploadRunning = false;
static unsigned long lastFileUploadTime;

static void HandleUploadResponse(AsyncWebServerRequest *request)
{
    TRACELN("HandleUploadResponse");
    AsyncWebServerResponse *response = request->beginResponse(uploadStatusCode, "text/plain", uploadStatusMessage);
    response->addHeader("Connection", "close");
    request->send(response);
}

 
AxWebServer::AxWebServer()
{
}


static void handleFPUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  if (!index){
    TRACEPRINTF("Update: %s\n", filename.c_str());
    fpUploadRunning = true;
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      Update.printError(Serial);
      fpUploadRunning = false;
    }
  }

  //due to multi-threading, millis() might return a slightly lower value in main thread (!). 
  //So we will assure that this value is _always_ in the past 
  lastFileUploadTime = millis() - 10;
  if (Update.write(data, len) != len) {
    Update.printError(Serial);
    fpUploadRunning = false;
  }

  if (final) {
    fpUploadRunning = false;
    if (Update.end(true))
    {
      TRACELN("Update Success: Rebooting...");
    }
    else
    {
      Update.printError(Serial);
    }
  }  
}


void AxWebServer::Init()
{
  SPIFFS.begin(true);
  TRACEPRINTF("Setting up WebServer on Port %u\n", EEPROMData.WebIfPort);
  webServer = new AsyncWebServer(EEPROMData.WebIfPort);

  if(EEPROMData.WebIfPort != TCP_PORT_WEBIF_DEFAULT)
  {
      TRACEPRINTF("Create redirector on default port %u\n", TCP_PORT_WEBIF_DEFAULT);
      redirectWebServer = new AsyncWebServer(TCP_PORT_WEBIF_DEFAULT);
      redirectWebServer->onNotFound([](AsyncWebServerRequest *request){
        char targetUri[1024];
        snprintf(targetUri, sizeof(targetUri), "http://%s:%u", request->host().c_str(), EEPROMData.WebIfPort);
        TRACEPRINTF("Redirecting to %s", targetUri);
        request->redirect(targetUri);
      });
  }

  webServer->on("/capture", HTTP_GET, [](AsyncWebServerRequest *request) {
      capture_handler(request);
  });

  webServer->on("/stream", HTTP_GET, [](AsyncWebServerRequest *request) {
      streamJpg(request);
  });

  webServer->on("/networkconfig.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    TRACELN("GET /networkconfig.html");
    AxWebServer_ProcessNetworkConfigPage(request);
  });

/*
  webServer->on("/esp32update", HTTP_GET, [](AsyncWebServerRequest *request) {
    TRACELN("GET /esp32update");
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", AxOTAWebServerPage);
    response->addHeader("Connection", "close");
    request->send(response);
  });*/

  webServer->on("/robot-control", HTTP_GET, [](AsyncWebServerRequest *request) {
    TRACELN("GET /robot-control");
    AsyncWebServerResponse *response;
    bool withParam = AxWebServer_ProcessRobotControlPage(request);
    response = request->beginResponse(200, "text/html", "OK");
    response->addHeader("Connection", "close");
    request->send(response);
  });

/*
*/

  webServer->on("/about.html", HTTP_GET, [](AsyncWebServerRequest *request) {

    if(request->hasParam("option"))
    {
      int option = request->getParam("option")->value().toInt();
      switch(option)
      {
        case 1:
        AxDisplayInstance.DisplayCustom("Option 1");
        ObstacleAvoidanceInstance.enabled = false;
        MotorDriverInstance.EnqueueMovement(MovementType_RotateClockwise, SPEED_DEFAULT, 1500);
        break;

        case 2:
        AxDisplayInstance.DisplayCustom("Option 2");
        ObstacleAvoidanceInstance.enabled = true;
        break;

        case 3:
        AxDisplayInstance.DisplayCustom("Option 3");
        MotorDriverInstance.EnqueueMovement(MovementType_Stop, SPEED_DEFAULT, 0);
        ObstacleAvoidanceInstance.enabled = false;
        break;
      }
    }
    request->send(SPIFFS, "/about.html", "text/html", false, TemplateProcessor);
  });

  webServer->serveStatic("/about.html", SPIFFS, "/about.html")
    .setTemplateProcessor(TemplateProcessor);

  webServer->serveStatic("/", SPIFFS, "/")
    .setDefaultFile("home.html");

/*  //handling uploading ESP32 firmware file
  webServer->on("/esp32update", HTTP_POST, [](AsyncWebServerRequest *request) {
    if(Update.hasError())
    {
      TRACELN("Frontend Processor Firmware update failed.");
      uploadStatusCode = 400;
      uploadStatusMessage = "ERROR";
    }
    else
    {
      TRACELN("Frontend Processor Firmware update success.");
      uploadStatusCode = 200;
      uploadStatusMessage = "OK";
    }
    HandleUploadResponse(request);
    if(!Update.hasError())
    {
      delay(2000);
      ESP.restart();
    }
  }, handleFPUpload);

  */
}

void AxWebServer::Process()
{
  if(AxNetworkInstance.IsEthConnected())
  {
    if(!isBound)
    {
      TRACELN("Starting AxWebServer listener");
	    webServer->begin();
      if(redirectWebServer)
      {
        redirectWebServer->begin();
      }
      isBound = true;
    }
  }
  else
  {
    if(isBound)
    {
      //ETH/WIFI disconnect
      webServer->end();
      if(redirectWebServer)
      {
        redirectWebServer->end();
      }
    }
    isBound = false;
  }

  if(isBound)
  {
    if(fpUploadRunning && millis() - lastFileUploadTime >= FILE_UPLOAD_CHUNK_TIMEOUT_MILLIS)
    {
      TRACELN(F("FP Firmware File upload timeout.")); 
      fpUploadRunning = false;
      Update.abort();
    }
  }
}

void AxWebServer::OnNetworkChanged() {
  if(AxNetworkInstance.IsEthConnected()) {
    webServer->end();
    webServer->begin();
    if(redirectWebServer)
    {
      redirectWebServer->end();
      redirectWebServer->begin();
    }
  }
}

