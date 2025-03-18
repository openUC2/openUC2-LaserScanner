#define IS_XIAO 1
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <arpa/inet.h>
#include <vector>
#include "SPIRenderer.h"
#include "esp_task_wdt.h"

static const char *TAG = "main";

extern "C"
{
  void app_main(void);
}

void app_main()
{
  esp_log_level_set("gpio", ESP_LOG_WARN);
  ESP_LOGD(TAG, "Starting up...");
  
  // Disable the task watchdog for the main task
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));

  int X_MIN = 0;
  int X_MAX = 100;
  int Y_MIN = 0;
  int Y_MAX = 100;
  int STEP = 1; // Adjust based on your desired resolution
  int tPixelDwelltime = 5;
  int nFrames = 100;
  SPIRenderer *renderer = new SPIRenderer(X_MIN, X_MAX, Y_MIN, Y_MAX, STEP, tPixelDwelltime, nFrames);
  while (1){
    tPixelDwelltime +=1;
    renderer->setParameters(X_MIN, X_MAX, Y_MIN, Y_MAX, STEP, tPixelDwelltime, nFrames);
    renderer->start();

    // Give other tasks a chance to run
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 milliseconds

  }

}

