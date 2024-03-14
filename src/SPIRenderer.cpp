#include <cstring>
#include "freertos/FreeRTOS.h"
#include "SPIRenderer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "Renderer";
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 25
#define PIN_NUM_CLK 26
#define PIN_NUM_CS 27
#define PIN_NUM_LDAC GPIO_NUM_33
#define PIN_NUM_LASER GPIO_NUM_32
#define PIN_NUM_CAMERA_TRIGGER GPIO_NUM_23

// Function to trigger the camera
void trigger_camera(int tPixelDwelltime)
{
  gpio_set_level(PIN_NUM_CAMERA_TRIGGER, 1); // Set high
  ets_delay_us(tPixelDwelltime);             // Delay for 10us (adjust based on your camera's requirements)
  gpio_set_level(PIN_NUM_CAMERA_TRIGGER, 0); // Set low
}

// void IRAM_ATTR SPIRenderer::draw() {
void SPIRenderer::draw()
{
  // Set the initial position
  for (int iFrame = 0; iFrame <= nFrames; iFrame++)
  {
    printf("Drawing %d\n out of %d", iFrame, nFrames);
    printf("X_MIN %d\n, X_MAX %d\n, Y_MIN %d\n, Y_MAX %d\n, STEP %d\n", X_MIN, X_MAX, Y_MIN, Y_MAX, STEP);
    for (int x = X_MIN; x <= X_MAX; x += STEP)
    {
      for (int y = Y_MIN; y <= Y_MAX; y += STEP)
      {
        // Perform the scanning by setting x and y positions

        // Convert x, y to DAC values as needed
        int dacX = 2048 + (x * 1024) / 32768;
        int dacY = 2048 + (y * 1024) / 32768;

        // SPI transaction for channel A (X-axis)
        spi_transaction_t t1 = {};
        t1.length = 16;
        t1.flags = SPI_TRANS_USE_TXDATA;
        t1.tx_data[0] = 0b11010000 | ((dacX >> 8) & 0xF);
        t1.tx_data[1] = dacX & 255;

        spi_device_polling_transmit(spi, &t1);

        // SPI transaction for channel B (Y-axis)
        spi_transaction_t t2 = {};
        t2.length = 16;
        t2.flags = SPI_TRANS_USE_TXDATA;
        t2.tx_data[0] = 0b01010000 | ((dacY >> 8) & 0xF);
        t2.tx_data[1] = dacY & 255;
        // printf("x/y %d %d and X/Y to draw %d %d\n", x, y, dacX, dacY);
        spi_device_polling_transmit(spi, &t2);

        // Load the DAC
        gpio_set_level(PIN_NUM_LDAC, 0);
        gpio_set_level(PIN_NUM_LDAC, 1);

        // Trigger the camera for each position
        int tTriggerTime = 1; // 10us trigger time
        trigger_camera(tTriggerTime);
        // Add a small delay if needed to stabilize the position before capturing
      }
    }
  }
}

SPIRenderer::SPIRenderer(int xmin, int xmax, int ymin, int ymax, int step, int tPixelDwelltime, int nFramesI)
{
  nX = (xmax - xmin) / step;
  nY = (ymax - ymin) / step;
  tPixelDwelltime = tPixelDwelltime;
  X_MIN = xmin;
  X_MAX = xmax;
  Y_MIN = ymin;
  Y_MAX = ymax;
  STEP = step;
  nFrames = nFramesI;
  printf("Setting up renderer with parameters: %d %d %d %d %d %d %d\n", xmin, xmax, ymin, ymax, step, tPixelDwelltime, nFrames);

  // setup the laser
  gpio_set_direction(PIN_NUM_LASER, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_CAMERA_TRIGGER, GPIO_MODE_OUTPUT);

  // setup the LDAC output
  gpio_set_direction(PIN_NUM_LDAC, GPIO_MODE_OUTPUT);

  // setup SPI output
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = PIN_NUM_MOSI,
      .miso_io_num = PIN_NUM_MISO,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0};
  spi_device_interface_config_t devcfg = {
      .command_bits = 0,
      .address_bits = 0,
      .dummy_bits = 0,
      .mode = 0,
      .clock_speed_hz = 80000000,
      .spics_io_num = PIN_NUM_CS, // CS pin
      .flags = SPI_DEVICE_NO_DUMMY,
      .queue_size = 2,
  };
  // Initialize the SPI bus
  ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
  assert(ret == ESP_OK);
  // Attach the SPI device
  ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  printf("Error message %s\n", esp_err_to_name(ret));
  // printf("Ret code is %d\n", ret);
  // assert(ret == ESP_OK);
}

void SPIRenderer::setParameters(int xmin, int xmax, int ymin, int ymax, int step, int tPixelDwelltime, int nFramesI)
{
  nX = (xmax - xmin) / step;
  nY = (ymax - ymin) / step;
  tPixelDwelltime = tPixelDwelltime;
  X_MIN = xmin;
  X_MAX = xmax;
  Y_MIN = ymin;
  Y_MAX = ymax;
  STEP = step;
  nFrames = nFramesI;
  printf("Setting up renderer with parameters: %d %d %d %d %d %d %d\n", xmin, xmax, ymin, ymax, step, tPixelDwelltime, nFrames);
}

void SPIRenderer::start()
{
  // start the SPI renderer
  printf("Starting to draw %d\n", 1);
  draw();
  printf("Done with drawing %d", 1);
}
