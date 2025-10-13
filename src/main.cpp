#define IS_XIAO 1
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <arpa/inet.h>
#include <vector>
#include "SPIRenderer.h"
#include "esp_task_wdt.h"
#include <ArduinoJson.h>

static const char *TAG = "main";

// Simple serial buffer
String incomingData;

// Renderer pointer
SPIRenderer *renderer = nullptr;

// Default parameters
int X_MIN = 0;
int X_MAX = 6000;
int Y_MIN = 0;
int Y_MAX = 6000;
int STEP = 20;
int tPixelDwelltime = 10;
int nFrames = 100;

extern "C"
{
  void app_main(void);
}

// -------------------------------------------------------------------
// HELPER: Handle JSON commands
// -------------------------------------------------------------------
void handleJSON(const String &jsonString) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  if (error) {
    Serial.println("{\"status\":\"error\",\"info\":\"JSON parse failed\"}");
    return;
  }

  const char* task = doc["task"];
  if (!task) {
    Serial.println("{\"status\":\"error\",\"info\":\"Missing task\"}");
    return;
  }

  // Handle /state_get command
  if (strcmp(task, "/state_get") == 0) {
    int qid = doc["qid"] | 0;
    Serial.print("++\n{\"identifier_name\":\"UC2_GalvoScanner\",");
    Serial.print("\"identifier_id\":\"V1.0\",");
    Serial.print("\"identifier_date\":\"");
    Serial.print(__DATE__); 
    Serial.print(" ");
    Serial.print(__TIME__);
    Serial.print("\",");
    Serial.print("\"identifier_author\":\"UC2\",");
    Serial.print("\"IDENTIFIER_NAME\":\"uc2-esp\",");
    Serial.print("\"configIsSet\":0,");
    Serial.print("\"pindef\":\"UC2\",");
    Serial.print("\"success\":1");
    if (qid != 0) {
      Serial.print(",\"qid\":");
      Serial.print(qid);
    }
    Serial.println("}\n--");
    return;
  }

  // Handle /galvo_act command
  if (strcmp(task, "/galvo_act") == 0) {
    int qid = doc["qid"] | 0;
    
    // Get parameters from JSON, use current values as defaults
    int newXMin = doc["X_MIN"] | X_MIN;
    int newXMax = doc["X_MAX"] | X_MAX;
    int newYMin = doc["Y_MIN"] | Y_MIN;
    int newYMax = doc["Y_MAX"] | Y_MAX;
    int newStep = doc["STEP"] | STEP;
    int newDwell = doc["tPixelDwelltime"] | tPixelDwelltime;
    int newFrames = doc["nFrames"] | nFrames;

    // Update global parameters
    X_MIN = newXMin;
    X_MAX = newXMax;
    Y_MIN = newYMin;
    Y_MAX = newYMax;
    STEP = newStep;
    tPixelDwelltime = newDwell;
    nFrames = newFrames;

    // Update renderer if it exists
    if (renderer != nullptr) {
      renderer->setParameters(X_MIN, X_MAX, Y_MIN, Y_MAX, STEP, tPixelDwelltime, nFrames);
    }

    // Report success
    Serial.print("++\n{\"task\":\"/galvo_act\",\"status\":\"success\"");
    if (qid != 0) {
      Serial.print(",\"qid\":");
      Serial.print(qid);
    }
    Serial.println("}\n--");
    return;
  }

  // Unknown task
  Serial.println("{\"status\":\"error\",\"info\":\"Unknown task\"}");
}

// -------------------------------------------------------------------
// HELPER: Process serial input
// -------------------------------------------------------------------
void processSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      // Process one line of JSON
      if (incomingData.length() > 0) {
        handleJSON(incomingData);
        incomingData = "";
      }
    } else if (c != '\r') {
      incomingData += c;
    }
  }
}


void app_main()
{
  esp_log_level_set("gpio", ESP_LOG_WARN);
  ESP_LOGD(TAG, "Starting up...");
  
  // Initialize Serial
  Serial.begin(115200);
  
  // Disable the task watchdog for the main task
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));

  // Create renderer with default parameters
  renderer = new SPIRenderer(X_MIN, X_MAX, Y_MIN, Y_MAX, STEP, tPixelDwelltime, nFrames);
  
  while (1) {
    // Process any incoming serial commands
    processSerial();
    
    // Run the renderer with current parameters
    renderer->start();

    // Give other tasks a chance to run
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 milliseconds
  }
}

