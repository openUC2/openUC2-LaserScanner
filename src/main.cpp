#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include "driver/gpio.h"

extern "C" {
  void app_main(void);
}

#include "SPIRenderer.h"

static const char *TAG = "main";
static bool needUpdate = false;
static bool stopRendering = false;
static SPIRenderer *renderer = nullptr;

// Example defaults
// xmin;xmax;ymin;ymax;step
// 0;100;0;100;1;

int scalingFactor = 4;
static int X_MIN = 0;
static int X_MAX = 256*scalingFactor;
static int Y_MIN = 0;
static int Y_MAX = 256*scalingFactor;
static int STEP = scalingFactor;
static int tPixelDwelltime =100;
static int nFrames = 1;

static bool parseParams(const char *line,
                        int &xmin, int &xmax,
                        int &ymin, int &ymax,
                        int &step)
{
    // Adjust this if you need more parameters
    int count = sscanf(line, "%d;%d;%d;%d;%d",
                       &xmin, &xmax, &ymin, &ymax, &step);
    ESP_LOGI(TAG, "Parsed %d params", count);
    return (count == 5);
}

static void serialTask(void *arg)
{
    char rxLine[128];
    while (true) {
        memset(rxLine, 0, sizeof(rxLine));

        int len = 0;
        while (len < sizeof(rxLine) - 1) {
            int c = fgetc(stdin);
            if (c == EOF) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            if (c == '\r' || c == '\n') {
              ESP_LOGI(TAG, "Received: %s", rxLine);
              break;}
            rxLine[len++] = (char)c;
        }

        if (len > 0) {
            int newXmin, newXmax, newYmin, newYmax, newStep;
            if (parseParams(rxLine, newXmin, newXmax, newYmin, newYmax, newStep)) {
                X_MIN = newXmin;
                X_MAX = newXmax;
                Y_MIN = newYmin;
                Y_MAX = newYmax;
                STEP  = newStep;
                stopRendering = true;
                needUpdate = true;
                ESP_LOGI(TAG, "New params -> X=[%d..%d], Y=[%d..%d], STEP=%d",
                      X_MIN, X_MAX, Y_MIN, Y_MAX, STEP);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void renderTask(void *arg)
{
    while (true) {
        if (needUpdate && renderer) {
            renderer->setParameters(X_MIN, X_MAX,
                                    Y_MIN, Y_MAX,
                                    STEP,
                                    tPixelDwelltime, nFrames);
            needUpdate = false;
        }

        if (renderer) {
            stopRendering = false;
            renderer->start(&stopRendering);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));

    // Setup UART for console input
    setvbuf(stdin, NULL, _IONBF, 0);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);

    renderer = new SPIRenderer(X_MIN, X_MAX,
                               Y_MIN, Y_MAX,
                               STEP,
                               tPixelDwelltime, nFrames);

    xTaskCreate(serialTask, "serialTask", 4096, NULL, 1, NULL);
    xTaskCreate(renderTask, "renderTask", 4096, NULL, 1, NULL);
}
