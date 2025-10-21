/*  
    CameraHandler.cpp
*/

#include "CameraHandler.h"
#include "AxNetwork.h"
#include "driver/ledc.h"

CameraHandler CameraHandlerInstance;

CameraHandler::CameraHandler()
{
    afStatus = AF_STATUS_UNKNOWN;
    ledValue = 0;
}

void CameraHandler::Init()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  // Start with a conservative XCLK; adjust for problematic OV3660 modules.
  config.xclk_freq_hz = 12000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // Use LATEST to avoid blocking waits when buffers fill; improves responsiveness.
  config.grab_mode = CAMERA_GRAB_LATEST;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  // Native QVGA target for reliability (avoid large initial buffer allocations + downscale).
  // Quality set moderately to reduce encode time on flaky modules.
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12; // moderate quality
    config.fb_count = 2; // keep double buffering if PSRAM present
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 15; // slightly lower quality for speed
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  xSemaphoreTake(TwiMutex, portMAX_DELAY);
  // camera init
  sensor = NULL;
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    TRACEPRINTF("FATAL: Camera init failed with error 0x%x\n", err);
    AxDisplayInstance.Display("Error:\n  Cam Init\n");
    delay(2000);
    return;
  }

  sensor = esp_camera_sensor_get();
  switch(sensor->id.PID)
  {
    case OV2640_PID:
    sensor->set_vflip(sensor, 0); 
    sensor->set_hmirror(sensor, 0); 
    break;
    case OV3660_PID:
      sensor->set_vflip(sensor, 1); // flip it back
      sensor->set_brightness(sensor, 1); // up the brightness just a bit
      sensor->set_saturation(sensor, -2); // lower the saturation
      break;

    case OV5640_PID:
    sensor->set_vflip(sensor, 1); // flip it back
    break;

    default:
    break;
  }
  sensor->set_framesize(sensor, FRAMESIZE_VGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  sensor->set_vflip(sensor, 1);
  sensor->set_hmirror(sensor, 1);
#endif

  if(sensor->id.PID == OV5640_PID)
  {
    //Bypass internal voltage regulator
    sensor->set_reg(sensor, OV5640_AF_SC_POWER_CONTROL,  0xFF, 0x08);
  }
  xSemaphoreGive(TwiMutex);  
}

void CameraHandler::Process()
{
}

bool CameraHandler::WaitForRegisterValue(int reg, int mask, int expectedValue, uint16_t timeoutMillis)
{
  if(!sensor)
  {
    return false;
  }

  bool success = false;
  unsigned long startTime = millis();
  while(millis() - startTime < timeoutMillis)
  {
    if(expectedValue == sensor->get_reg(sensor, reg, mask))
    {
        success = true;
        break;
    }
    delay(10);
  }

  return success;
}

