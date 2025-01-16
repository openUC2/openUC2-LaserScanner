#define IS_XIAO 1
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <arpa/inet.h>
#include <vector>
#include "SPIRenderer.h"

static const char *TAG = "main";

extern "C"
{
  void app_main(void);
}

void app_main()
{
  esp_log_level_set("gpio", ESP_LOG_WARN);
  ESP_LOGD(TAG, "Starting up...");
  
  int X_MIN = 0;
  int X_MAX = 30000;
  int Y_MIN = 0;
  int Y_MAX = 30000;
  int STEP = 1000; // Adjust based on your desired resolution
  int tPixelDwelltime = 1;
  int nFrames = 10;
  SPIRenderer *renderer = new SPIRenderer(X_MIN, X_MAX, Y_MIN, Y_MAX, STEP, tPixelDwelltime, nFrames);
  while (1){
    tPixelDwelltime +=1;
    renderer->setParameters(X_MIN, X_MAX, Y_MIN, Y_MAX, STEP, tPixelDwelltime, nFrames);
    renderer->start();
  }

}

