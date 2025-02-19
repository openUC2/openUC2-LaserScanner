/*
   Example Arduino-style sketch for Seeed XIAO ESP32S3
   demonstrating a simple SPI-based renderer with serial
   commands for updating rendering parameters in real time.

   CONNECTIONS / PIN DEFINITIONS (XIAO):
   D0 -> pin 1
   D1 -> pin 2
   D2 -> pin 3
   D3 -> pin 4
   D4 -> pin 5
   D5 -> pin 6
   D6 -> pin 43
   D7 -> pin 44
   D8 -> pin 7
   D9 -> pin 8
   D10 -> pin 9

   Here we map:
   - PIN_NUM_MISO      -> -1      (not used)
   - PIN_NUM_SCK       -> GPIO 8  (D9)
   - PIN_NUM_SDI       -> GPIO 7  (D8)
   - PIN_NUM_CS        -> GPIO 9  (D10)
   - PIN_NUM_LDAC      -> GPIO 6  (D5)
   - PIN_NUM_LASER     -> GPIO 43 (D6)
   - PIN_NUM_TRIG_PIXEL-> GPIO 2  (D1)
   - PIN_NUM_TRIG_LINE -> GPIO 3  (D2)
   - PIN_NUM_TRIG_FRAME-> GPIO 4  (D3)

   In this single .ino file, we define:
     - A SPIRenderer class that handles SPI transactions and scanning
     - setup() to initialize Serial, SPI, and the renderer
     - loop() to:
         1) read serial input of the form "xmin;xmax;ymin;ymax;stepx;stepy"
         2) update the renderer parameters
         3) run continuous rendering frames (blocking for demonstration)

   NOTE: This is a minimal demonstration. If you want asynchronous or
   background rendering, you could move the scanning code to another
   task. This example simply does it in the main loop for clarity.
*/

#define IS_XIAO 1

#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/uart.h>
#include <esp_rom_sys.h>

// =========================== PIN DEFINITIONS ===========================
#ifdef IS_XIAO
/*
  D0 -> pin 1
  D1 -> pin 2
  D2 -> pin 3
  D3 -> pin 4
  D4 -> pin 5
  D5 -> pin 6
  D6 -> pin 43
  D7 -> pin 44
  D8 -> pin 7
  D9 -> pin 8
  D10 -> pin 9
*/
static const int PIN_NUM_MISO       = -1;
static const int PIN_NUM_SCK        = 8;  // D9
static const int PIN_NUM_SDI        = 7;  // D8
static const int PIN_NUM_CS         = 9;  // D10
static const int PIN_NUM_LDAC       = 6;  // D5
static const int PIN_NUM_LASER      = 43; // D6
static const int PIN_NUM_TRIG_PIXEL = 2;  // D1
static const int PIN_NUM_TRIG_LINE  = 3;  // D2
static const int PIN_NUM_TRIG_FRAME = 4;  // D3
#else
// Example fallback pin definitions for a generic ESP32-S3 dev board
static const int PIN_NUM_MISO       = -1;
static const int PIN_NUM_SCK        = 14;
static const int PIN_NUM_SDI        = 12;
static const int PIN_NUM_CS         = 13;
static const int PIN_NUM_LDAC       = 27;
static const int PIN_NUM_LASER      = 26;
static const int PIN_NUM_TRIG_PIXEL = 23;
static const int PIN_NUM_TRIG_LINE  = 22;
static const int PIN_NUM_TRIG_FRAME = 21;
#endif

// =========================== SPIRenderer CLASS ===========================
class SPIRenderer
{
public:
  SPIRenderer()
    : _spi(NULL),
      X_MIN(0), X_MAX(100), Y_MIN(0), Y_MAX(100),
      STEPX(1), STEPY(1), pixelDwell(5), nFrames(1)
  {
  }

  bool begin(int xmin, int xmax,
             int ymin, int ymax,
             int stepx, int stepy,
             int tPixelDwell, int frames)
  {
    X_MIN = xmin;
    X_MAX = xmax;
    Y_MIN = ymin;
    Y_MAX = ymax;
    STEPX = stepx;
    STEPY = stepy;
    pixelDwell = tPixelDwell;
    nFrames = frames;

    Serial.printf("[SPIRenderer] begin() -> X=[%d..%d], Y=[%d..%d], STEPX=%d, STEPY=%d, dwell=%d, frames=%d\n",
                  X_MIN, X_MAX, Y_MIN, Y_MAX, STEPX, STEPY, pixelDwell, nFrames);

    // Setup pins
    pinMode(PIN_NUM_LASER, OUTPUT);
    pinMode(PIN_NUM_TRIG_PIXEL, OUTPUT);
    pinMode(PIN_NUM_TRIG_LINE, OUTPUT);
    pinMode(PIN_NUM_TRIG_FRAME, OUTPUT);
    pinMode(PIN_NUM_LDAC, OUTPUT);

    // SPI bus config
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_SDI;
    buscfg.sclk_io_num = PIN_NUM_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;

    // SPI device config
    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 20 * 1000000; // 20 MHz
    devcfg.spics_io_num = PIN_NUM_CS;
    devcfg.queue_size = 2;

    // Initialize SPI bus + device
    esp_err_t ret;
#ifdef ARDUINO_ESP32S3_DEV
    // If you are using the standard IDF naming, might be SPI3_HOST
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
      Serial.println("[SPIRenderer] spi_bus_initialize() failed!");
      return false;
    }
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &_spi);
    if (ret != ESP_OK)
    {
      Serial.println("[SPIRenderer] spi_bus_add_device() failed!");
      return false;
    }
#else
    // For standard S3 boards in Arduino: normally you do HSPI_HOST or FSPI_HOST
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK)
    {
      Serial.println("[SPIRenderer] spi_bus_initialize() failed!");
      return false;
    }
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &_spi);
    if (ret != ESP_OK)
    {
      Serial.println("[SPIRenderer] spi_bus_add_device() failed!");
      return false;
    }
#endif
    return true;
  }

  void setParameters(int xmin, int xmax,
                     int ymin, int ymax,
                     int stepx, int stepy,
                     int tPixelDwell, int frames)
  {
    X_MIN = xmin;
    X_MAX = xmax;
    Y_MIN = ymin;
    Y_MAX = ymax;
    STEPX = stepx;
    STEPY = stepy;
    pixelDwell = tPixelDwell;
    nFrames = frames;

    Serial.printf("[SPIRenderer] setParameters() -> X=[%d..%d], Y=[%d..%d], STEPX=%d, STEPY=%d, dwell=%d, frames=%d\n",
                  X_MIN, X_MAX, Y_MIN, Y_MAX, STEPX, STEPY, pixelDwell, nFrames);
  }

  void renderFramesBlocking()
  {
    for (int iFrame = 0; iFrame < nFrames; iFrame++)
    {
      Serial.printf("=== Frame %d of %d ===\n", iFrame + 1, nFrames);

      // Optionally set a frame trigger high
      digitalWrite(PIN_NUM_TRIG_FRAME, HIGH);

      // X loop
      for (int xVal = X_MIN; xVal <= X_MAX; xVal += STEPX)
      {
        // Optionally set a line trigger
        digitalWrite(PIN_NUM_TRIG_LINE, HIGH);

        // Y loop
        for (int yVal = Y_MIN; yVal <= Y_MAX; yVal += STEPY)
        {
          // Pixel trigger
          digitalWrite(PIN_NUM_TRIG_PIXEL, HIGH);

          // Write DAC X
          spi_transaction_t txX = {};
          txX.length = 16; // bits
          txX.flags = SPI_TRANS_USE_TXDATA;
          txX.tx_data[0] = 0b00110000 | ((xVal >> 8) & 0x0F); 
          txX.tx_data[1] = (uint8_t)(xVal & 0xFF);

          // Write DAC Y
          spi_transaction_t txY = {};
          txY.length = 16; 
          txY.flags = SPI_TRANS_USE_TXDATA;
          txY.tx_data[0] = 0b10110000 | ((yVal >> 8) & 0x0F); 
          txY.tx_data[1] = (uint8_t)(yVal & 0xFF);

          // Pulse LDAC low during transmit
          digitalWrite(PIN_NUM_LDAC, LOW);
          spi_device_polling_transmit(_spi, &txX);
          spi_device_polling_transmit(_spi, &txY);
          digitalWrite(PIN_NUM_LDAC, HIGH);

          // Trigger camera for pixelDwell microseconds
          delayMicroseconds(pixelDwell);

          // Clear pixel trigger
          digitalWrite(PIN_NUM_TRIG_PIXEL, LOW);
        }

        // Clear line trigger
        digitalWrite(PIN_NUM_TRIG_LINE, LOW);
      }

      // Clear frame trigger
      digitalWrite(PIN_NUM_TRIG_FRAME, LOW);
    }
  }

private:
  spi_device_handle_t _spi;

  int X_MIN;
  int X_MAX;
  int Y_MIN;
  int Y_MAX;
  int STEPX;
  int STEPY;
  int pixelDwell;
  int nFrames;
};

// ======================== GLOBALS & FUNCTIONS =========================
SPIRenderer renderer;
static int Xmin    = 0;
static int Xmax    = 100;
static int Ymin    = 0;
static int Ymax    = 100;
static int Stepx   = 1;
static int Stepy   = 1;
static int dwellUS = 5;
static int frames  = 1;

// Simple parse function to get six integers from a line
bool parseParams(const char *line,
                 int &xMin, int &xMax,
                 int &yMin, int &yMax,
                 int &stx, int &sty)
{
  int count = sscanf(line, "%d;%d;%d;%d;%d;%d",
                     &xMin, &xMax, &yMin, &yMax, &stx, &sty);
  return (count == 6);
}

// read a line from Serial into buffer (blocking)
bool readSerialLine(char *buffer, size_t maxLen)
{
  size_t idx = 0;
  while (true)
  {
    if (Serial.available() > 0)
    {
      int c = Serial.read();
      if (c == '\r' || c == '\n')
      {
        if (idx > 0) {
          buffer[idx] = '\0';
          return true;
        }
      }
      else if (idx < (maxLen - 1))
      {
        buffer[idx++] = (char)c;
      }
    }
  }
  return false;
}

// =============================== SETUP ================================
void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n[Setup] Starting...");

  // Initialize the renderer with default params
  if (!renderer.begin(Xmin, Xmax,
                      Ymin, Ymax,
                      Stepx, Stepy,
                      dwellUS, frames))
  {
    Serial.println("[Setup] Renderer init failed!");
    while (1) { delay(1000); }
  }

  Serial.println("[Setup] Renderer init OK. Enter commands of the form:");
  Serial.println("  xmin;xmax;ymin;ymax;stepx;stepy");
  Serial.println("Example: 0;200;0;200;2;2");
}

// =============================== LOOP ================================
void loop()
{
  // 1) If there's a line from serial, parse it:
  if (Serial.available() > 0)
  {
    char line[64];
    if (readSerialLine(line, sizeof(line)))
    {
      int newXmin, newXmax, newYmin, newYmax, newStepx, newStepy;
      if (parseParams(line, newXmin, newXmax, newYmin, newYmax, newStepx, newStepy))
      {
        // For demonstration, re-use dwellUS & frames or let user change them as well
        // If you also want dwellUS & frames, you'd parse them too or set them globally
        Xmin  = newXmin;
        Xmax  = newXmax;
        Ymin  = newYmin;
        Ymax  = newYmax;
        Stepx = newStepx;
        Stepy = newStepy;

        renderer.setParameters(Xmin, Xmax,
                               Ymin, Ymax,
                               Stepx, Stepy,
                               dwellUS, frames);
      }
      else
      {
        Serial.printf("[Loop] Invalid input: '%s'\n", line);
      }
    }
  }

  // 2) Render frames (blocking). If you prefer partial updates, you can break earlier.
  renderer.renderFramesBlocking();
  
  // 3) Or just delay to avoid spamming frames:
  //    (But with a real approach, you might want to render only once or do partial)
  delay(5000);
}
