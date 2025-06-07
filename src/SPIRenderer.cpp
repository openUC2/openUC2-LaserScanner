#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "SPIRenderer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_err.h"
#include "esp_log.h"

// For fast GPIO toggling on ESP32-S3:
#include "soc/gpio_struct.h" // Gives you GPIO.out_w1ts, etc.
#include "hal/gpio_types.h"
#include "esp_rom_sys.h" // For esp_rom_delay_us()

static const char *TAG = "SPIRenderer";

// Comment/uncomment if you want to compile for XIAO or not
// #define IS_XIAO

////////////////////////////////////////////////////////////////
// Helper: Fast toggling of GPIO pins
////////////////////////////////////////////////////////////////
#ifndef IS_XIAO
static inline void fast_gpio_set(int pin)
{
  // For GPIO pin < 32 on ESP32-S3:
  GPIO.out_w1ts = (1U << pin);
}

static inline void fast_gpio_clear(int pin)
{
  // For GPIO pin < 32 on ESP32-S3:
  GPIO.out_w1tc = (1U << pin);
}
#endif

////////////////////////////////////////////////////////////////
// Set pixel/line/frame trigger pins
////////////////////////////////////////////////////////////////

void set_gpio_pins(int pixelTrigVal, int lineTrigVal, int frameTrigVal)
{
  uint32_t gpio_mask = ((1ULL << PIN_NUM_TRIG_PIXEL) |
                        (1ULL << PIN_NUM_TRIG_LINE) |
                        (1ULL << PIN_NUM_TRIG_FRAME));

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = gpio_mask;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

#ifdef IS_XIAO
  if (pixelTrigVal)
  {
    gpio_set_level(PIN_NUM_TRIG_PIXEL, 1);
  }
  else
  {
    gpio_set_level(PIN_NUM_TRIG_PIXEL, 0);
  }

  if (lineTrigVal)
  {
    gpio_set_level(PIN_NUM_TRIG_LINE, 1);
  }
  else
  {
    gpio_set_level(PIN_NUM_TRIG_LINE, 0);
  }

  if (frameTrigVal)
  {
    gpio_set_level(PIN_NUM_TRIG_FRAME, 1);
  }
  else
  {
    gpio_set_level(PIN_NUM_TRIG_FRAME, 0);
  }
#else
  if (pixelTrigVal)
    GPIO.out_w1ts = (1ULL << PIN_NUM_TRIG_PIXEL);
  else
    GPIO.out_w1tc = (1ULL << PIN_NUM_TRIG_PIXEL);

  if (lineTrigVal)
    GPIO.out_w1ts = (1ULL << PIN_NUM_TRIG_LINE);
  else
    GPIO.out_w1tc = (1ULL << PIN_NUM_TRIG_LINE);

  if (frameTrigVal)
    GPIO.out_w1ts = (1ULL << PIN_NUM_TRIG_FRAME);
  else
    GPIO.out_w1tc = (1ULL << PIN_NUM_TRIG_FRAME);
#endif
}

////////////////////////////////////////////////////////////////
// Trigger camera for tPixelDwelltime microseconds
////////////////////////////////////////////////////////////////
void trigger_camera(int tPixelDwelltime, int triggerPin = PIN_NUM_TRIG_PIXEL)
{
#ifdef IS_XIAO
  gpio_set_level((gpio_num_t)triggerPin, 1);
  esp_rom_delay_us(tPixelDwelltime);
  gpio_set_level((gpio_num_t)triggerPin, 0);
#else
  // On ESP32-S3, optionally do direct register toggling
  GPIO.out_w1ts = (1U << triggerPin); // set bit
  esp_rom_delay_us(tPixelDwelltime);
  GPIO.out_w1tc = (1U << triggerPin); // clear bit
#endif
}

////////////////////////////////////////////////////////////////
// The SPIRenderer class
////////////////////////////////////////////////////////////////
SPIRenderer::SPIRenderer(int xmin, int xmax, int ymin, int ymax,
                         int step, int tPixelDwelltime, int nFramesI)
{
  // Avoid variable shadowing:
  this->tPixelDwelltime = tPixelDwelltime;

  // The number of steps in x and y
  nX = (xmax - xmin) / step;
  nY = (ymax - ymin) / step;

  X_MIN = xmin;
  X_MAX = xmax;
  Y_MIN = ymin;
  Y_MAX = ymax;
  STEP = step;
  nFrames = nFramesI;

  printf("Setting up renderer with parameters: %d %d %d %d %d %d %d\n",
         xmin, xmax, ymin, ymax, step, tPixelDwelltime, nFrames);

  // Set up the laser pin
  gpio_set_direction((gpio_num_t)PIN_NUM_LASER, GPIO_MODE_OUTPUT);
  // Set up trigger pins
  gpio_set_direction((gpio_num_t)PIN_NUM_TRIG_PIXEL, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)PIN_NUM_TRIG_FRAME, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)PIN_NUM_TRIG_LINE, GPIO_MODE_OUTPUT);

  // Set up the LDAC output
  gpio_set_direction((gpio_num_t)PIN_NUM_LDAC, GPIO_MODE_OUTPUT);

  // Quick test of the LDAC pin
  for (int i = 0; i < 2; i++)
  {
    gpio_set_level((gpio_num_t)PIN_NUM_LDAC, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level((gpio_num_t)PIN_NUM_LDAC, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // SPI bus configuration
  // MCP4822-E/SN
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = PIN_NUM_SDI,
      .miso_io_num = PIN_NUM_MISO,
      .sclk_io_num = PIN_NUM_SCK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4096};

  spi_device_interface_config_t devcfg = {
      .command_bits = 0,
      .address_bits = 0,
      .dummy_bits = 0,
      .mode = 0,
      .clock_speed_hz = 20 * 1000 * 1000, // 20 MHz is max
      .spics_io_num = PIN_NUM_CS,
      .flags = SPI_DEVICE_NO_DUMMY,
      .queue_size = 2,
  };

#ifdef IS_XIAO
  ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);
#else
  ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
  ESP_ERROR_CHECK(ret);

  ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);
#endif

  // If you want chip-select forced low all the time (not typically recommended),
  // you can do:
  gpio_set_level((gpio_num_t)PIN_NUM_CS, 0);

  printf("SPI Bus init ret = %s\n", esp_err_to_name(ret));
}

void SPIRenderer::setParameters(int xmin, int xmax, int ymin, int ymax,
                                int step, int tPixelDwelltime, int nFramesI)
{
  // Again, fix shadowing
  this->tPixelDwelltime = tPixelDwelltime;

  nX = (xmax - xmin) / step;
  nY = (ymax - ymin) / step;
  X_MIN = xmin;
  X_MAX = xmax;
  Y_MIN = ymin;
  Y_MAX = ymax;
  STEP = step;
  nFrames = nFramesI;

  printf("Setting up renderer with parameters: %d %d %d %d %d %d %d\n",
         xmin, xmax, ymin, ymax, step, tPixelDwelltime, nFrames);
}


void SPIRenderer::draw()
{
    for (int iFrame = 0; iFrame < nFrames; iFrame++)
    {
        printf("Drawing frame %d of %d\n", iFrame + 1, nFrames);

        // Directly set all triggers high to mark frame start
        GPIO.out_w1ts = (1U << PIN_NUM_TRIG_PIXEL) |
                        (1U << PIN_NUM_TRIG_LINE) |
                        (1U << PIN_NUM_TRIG_FRAME);

        // add a small delay to ensure the frame start is registered
        esp_rom_delay_us(1);
        // Loop over X
        for(int dacX = X_MIN; dacX <= X_MAX; dacX += STEP)
        {
            // Loop over Y
            for(int dacY = Y_MIN; dacY <= Y_MAX; dacY += STEP)
            {
                // Clear triggers in one go
                GPIO.out_w1tc = (1U << PIN_NUM_TRIG_PIXEL) |
                                (1U << PIN_NUM_TRIG_LINE) |
                                (1U << PIN_NUM_TRIG_FRAME);
                esp_rom_delay_us(1);
                // Prepare SPI transactions for X and Y
                spi_transaction_t t1 = {};
                t1.length = 16;
                t1.flags = SPI_TRANS_USE_TXDATA;
                t1.tx_data[0] = (0b00110000 | ((dacX >> 8) & 0x0F));
                t1.tx_data[1] = (dacX & 0xFF);

                spi_transaction_t t2 = {};
                t2.length = 16;
                t2.flags = SPI_TRANS_USE_TXDATA;
                t2.tx_data[0] = (0b10110000 | ((dacY >> 8) & 0x0F));
                t2.tx_data[1] = (dacY & 0xFF);

                // Fewer LDAC toggles: latch once per pixel
                GPIO.out_w1tc = (1U << PIN_NUM_LDAC);  // hold LDAC low
                spi_device_polling_transmit(spi, &t1); // send X
                spi_device_polling_transmit(spi, &t2); // send Y
                GPIO.out_w1ts = (1U << PIN_NUM_LDAC);  // latch both channels

                // Optionally set a trigger directly for the pixel
                GPIO.out_w1ts = (1U << PIN_NUM_TRIG_PIXEL);
                // Delay if needed: 
                esp_rom_delay_us(tPixelDwelltime);

                // Clear pixel trigger again
                GPIO.out_w1tc = (1U << PIN_NUM_TRIG_PIXEL);
            }
            // Optionally set line trigger here
            GPIO.out_w1ts = (1U << PIN_NUM_TRIG_LINE);
            // Possibly delay
            // Clear line trigger
            GPIO.out_w1tc = (1U << PIN_NUM_TRIG_LINE);
        }
        // End of frame: clear triggers
        GPIO.out_w1tc = (1U << PIN_NUM_TRIG_PIXEL) |
                        (1U << PIN_NUM_TRIG_LINE) |
                        (1U << PIN_NUM_TRIG_FRAME);
    }
}

/*
void SPIRenderer::draw()
{
  // Outer loop: frames
  for (int iFrame = 0; iFrame < nFrames; iFrame++)
  {
    printf("Drawing frame %d of %d\n", iFrame + 1, nFrames);
    printf("X_MIN %d, X_MAX %d, Y_MIN %d, Y_MAX %d, STEP %d\n",
           X_MIN, X_MAX, Y_MIN, Y_MAX, STEP);

    // Example: set all triggers high at the start of a frame
    // pixelTrigVal, lineTrigVal, frameTrigVal
    set_gpio_pins(1, 1, 1);

    // Loop over X
    for (int dacX = X_MIN; dacX <= X_MAX; dacX += STEP)
    {

      // Loop over Y
      for (int dacY = Y_MIN; dacY <= Y_MAX; dacY += STEP)
      {
        set_gpio_pins(0, 0, 0);

        //ESP_LOGI(TAG, "Drawing pixel at %d %d", dacX, dacY);

        // SPI transaction for channel A (X-axis)
        spi_transaction_t t1 = {};
        t1.length = 16; // 16 bits
        t1.flags = SPI_TRANS_USE_TXDATA;
        t1.tx_data[0] = (0b00110000 | ((dacX >> 8) & 0x0F)); // Bit 5 = 1 (Gain = 1)
        t1.tx_data[1] = (dacX & 0xFF);

        // SPI transaction for channel B (Y-axis)
        spi_transaction_t t2 = {};
        t2.length = 16;
        t2.flags = SPI_TRANS_USE_TXDATA;
        t2.tx_data[0] = (0b10110000 | ((dacY >> 8) & 0x0F)); // Bit 5 = 1 (Gain = 1)
        t2.tx_data[1] = (dacY & 0xFF);

        // Latch the DAC
        gpio_set_level((gpio_num_t)PIN_NUM_LDAC, 0);
        gpio_set_level((gpio_num_t)PIN_NUM_LDAC, 1);

        gpio_set_level((gpio_num_t)PIN_NUM_LDAC, 0); // Hold LDAC low
        spi_device_polling_transmit(spi, &t1);       // Send X value
        spi_device_polling_transmit(spi, &t2);       // Send Y value
        gpio_set_level((gpio_num_t)PIN_NUM_LDAC, 1); // Latch both channels

        // Trigger the camera for each pixel
        // trigger_camera(this->tPixelDwelltime, PIN_NUM_TRIG_PIXEL);

        // Possibly clear certain triggers
        set_gpio_pins(1, 0, 0);
      }
      // Possibly clear certain triggers
      set_gpio_pins(1, 1, 0);
    }
    // End of frame
    set_gpio_pins(0, 0, 0);
  }
}
  */

////////////////////////////////////////////////////////////////
// Start rendering
////////////////////////////////////////////////////////////////
void SPIRenderer::start()
{
  printf("Starting to draw...\n");
  draw();
  printf("Done with drawing.\n");
}
