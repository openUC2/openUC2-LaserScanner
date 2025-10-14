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
#include <Preferences.h>

static const char *TAG = "main";

// Preferences object for persistent storage
Preferences preferences;

// Simple serial buffer
String incomingData;

// Renderer pointer
SPIRenderer *renderer = nullptr;

// Default parameters
int X_MIN = 0;
int X_MAX = 6000;
int Y_MIN = 0;
int Y_MAX = 6000;
int X_OFFSET = 0;  // X-axis offset
int Y_OFFSET = 0;  // Y-axis offset
int STEP_X = 20;   // Step size for X axis
int STEP_Y = 20;   // Step size for Y axis
int tPixelDwelltime = 10;
int nFrames = 100;
bool SNAKE = false; // Snake scanning pattern (alternate line direction)
bool ENABLE_TRIG_FRAME = true;  // Enable frame trigger
bool ENABLE_TRIG_LINE = true;   // Enable line trigger
bool ENABLE_TRIG_PIXEL = true;  // Enable pixel trigger

extern "C"
{
  void app_main(void);
}

// -------------------------------------------------------------------
// HELPER: Save parameters to preferences
// -------------------------------------------------------------------
void saveParameters() {
  preferences.begin("galvo", false);
  preferences.putInt("X_MIN", X_MIN);
  preferences.putInt("X_MAX", X_MAX);
  preferences.putInt("Y_MIN", Y_MIN);
  preferences.putInt("Y_MAX", Y_MAX);
  preferences.putInt("X_OFFSET", X_OFFSET);
  preferences.putInt("Y_OFFSET", Y_OFFSET);
  preferences.putInt("STEP_X", STEP_X);
  preferences.putInt("STEP_Y", STEP_Y);
  preferences.putInt("tPixelDwell", tPixelDwelltime);
  preferences.putInt("nFrames", nFrames);
  preferences.putBool("SNAKE", SNAKE);
  preferences.putBool("TRIG_FRAME", ENABLE_TRIG_FRAME);
  preferences.putBool("TRIG_LINE", ENABLE_TRIG_LINE);
  preferences.putBool("TRIG_PIXEL", ENABLE_TRIG_PIXEL);
  preferences.end();
  ESP_LOGI(TAG, "Parameters saved to preferences");
}

// -------------------------------------------------------------------
// HELPER: Load parameters from preferences
// -------------------------------------------------------------------
void loadParameters() {
  preferences.begin("galvo", true); // read-only mode
  X_MIN = preferences.getInt("X_MIN", 0);
  X_MAX = preferences.getInt("X_MAX", 6000);
  Y_MIN = preferences.getInt("Y_MIN", 0);
  Y_MAX = preferences.getInt("Y_MAX", 6000);
  X_OFFSET = preferences.getInt("X_OFFSET", 0);
  Y_OFFSET = preferences.getInt("Y_OFFSET", 0);
  STEP_X = preferences.getInt("STEP_X", 20);
  STEP_Y = preferences.getInt("STEP_Y", 20);
  tPixelDwelltime = preferences.getInt("tPixelDwell", 10);
  nFrames = preferences.getInt("nFrames", 100);
  SNAKE = preferences.getBool("SNAKE", false);
  ENABLE_TRIG_FRAME = preferences.getBool("TRIG_FRAME", true);
  ENABLE_TRIG_LINE = preferences.getBool("TRIG_LINE", true);
  ENABLE_TRIG_PIXEL = preferences.getBool("TRIG_PIXEL", true);
  preferences.end();
  ESP_LOGI(TAG, "Parameters loaded from preferences: X_MIN=%d X_MAX=%d Y_MIN=%d Y_MAX=%d X_OFF=%d Y_OFF=%d STEP_X=%d STEP_Y=%d tPixelDwell=%d nFrames=%d SNAKE=%d TRIG_F=%d TRIG_L=%d TRIG_P=%d", 
           X_MIN, X_MAX, Y_MIN, Y_MAX, X_OFFSET, Y_OFFSET, STEP_X, STEP_Y, tPixelDwelltime, nFrames, SNAKE, ENABLE_TRIG_FRAME, ENABLE_TRIG_LINE, ENABLE_TRIG_PIXEL);
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
    int newXOffset = doc["X_OFFSET"] | X_OFFSET;
    int newYOffset = doc["Y_OFFSET"] | Y_OFFSET;
    
    // Handle both STEP (backward compatibility) and STEP_X/STEP_Y
    int newStepX, newStepY;
    if (doc.containsKey("STEP")) {
      int stepValue = doc["STEP"];
      newStepX = stepValue;
      newStepY = stepValue;
    } else {
      newStepX = doc["STEP_X"] | STEP_X;
      newStepY = doc["STEP_Y"] | STEP_Y;
    }
    
    int newDwell = doc["tPixelDwelltime"] | tPixelDwelltime;
    int newFrames = doc["nFrames"] | nFrames;
    bool newSnake = doc["SNAKE"] | SNAKE;
    bool newTrigFrame = doc["ENABLE_TRIG_FRAME"] | ENABLE_TRIG_FRAME;
    bool newTrigLine = doc["ENABLE_TRIG_LINE"] | ENABLE_TRIG_LINE;
    bool newTrigPixel = doc["ENABLE_TRIG_PIXEL"] | ENABLE_TRIG_PIXEL;

    // Update global parameters
    X_MIN = newXMin;
    X_MAX = newXMax;
    Y_MIN = newYMin;
    Y_MAX = newYMax;
    X_OFFSET = newXOffset;
    Y_OFFSET = newYOffset;
    STEP_X = newStepX;
    STEP_Y = newStepY;
    tPixelDwelltime = newDwell;
    nFrames = newFrames;
    SNAKE = newSnake;
    ENABLE_TRIG_FRAME = newTrigFrame;
    ENABLE_TRIG_LINE = newTrigLine;
    ENABLE_TRIG_PIXEL = newTrigPixel;

    // Save parameters to preferences
    saveParameters();

    // Update renderer if it exists
    if (renderer != nullptr) {
      renderer->setParameters(X_MIN, X_MAX, Y_MIN, Y_MAX, X_OFFSET, Y_OFFSET, STEP_X, STEP_Y, 
                              tPixelDwelltime, nFrames, SNAKE, 
                              ENABLE_TRIG_FRAME, ENABLE_TRIG_LINE, ENABLE_TRIG_PIXEL);
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

  // Handle /galvo_get command
  if (strcmp(task, "/galvo_get") == 0) {
    int qid = doc["qid"] | 0;
    Serial.print("++\n{\"task\":\"/galvo_get\",");
    Serial.print("\"X_MIN\":");
    Serial.print(X_MIN);
    Serial.print(",\"X_MAX\":");
    Serial.print(X_MAX);
    Serial.print(",\"Y_MIN\":");
    Serial.print(Y_MIN);
    Serial.print(",\"Y_MAX\":");
    Serial.print(Y_MAX);
    Serial.print(",\"X_OFFSET\":");
    Serial.print(X_OFFSET);
    Serial.print(",\"Y_OFFSET\":");
    Serial.print(Y_OFFSET);
    Serial.print(",\"STEP_X\":");
    Serial.print(STEP_X);
    Serial.print(",\"STEP_Y\":");
    Serial.print(STEP_Y);
    Serial.print(",\"tPixelDwelltime\":");
    Serial.print(tPixelDwelltime);
    Serial.print(",\"nFrames\":");
    Serial.print(nFrames);
    Serial.print(",\"SNAKE\":");
    Serial.print(SNAKE ? "true" : "false");
    Serial.print(",\"ENABLE_TRIG_FRAME\":");
    Serial.print(ENABLE_TRIG_FRAME ? "true" : "false");
    Serial.print(",\"ENABLE_TRIG_LINE\":");
    Serial.print(ENABLE_TRIG_LINE ? "true" : "false");
    Serial.print(",\"ENABLE_TRIG_PIXEL\":");
    Serial.print(ENABLE_TRIG_PIXEL ? "true" : "false");
    Serial.print(",\"success\":1");
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
  
  // Load parameters from preferences
  loadParameters();
  
  // Disable the task watchdog for the main task
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));

  // Create renderer with loaded parameters
  renderer = new SPIRenderer(X_MIN, X_MAX, Y_MIN, Y_MAX, X_OFFSET, Y_OFFSET, STEP_X, STEP_Y, 
                             tPixelDwelltime, nFrames, SNAKE, 
                             ENABLE_TRIG_FRAME, ENABLE_TRIG_LINE, ENABLE_TRIG_PIXEL);
  
  while (1) {
    // Process any incoming serial commands
    processSerial();
    
    // Render one frame (allows serial processing between frames)
    renderer->start();

    // Give other tasks a chance to run
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 milliseconds
  }
}

