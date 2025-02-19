#include "SPIRenderer.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// For direct GPIO register toggles on some ESP chips:
#include "soc/gpio_struct.h"

static const char *TAG = "SPIRenderer";

#ifdef IS_XIAO
static inline void fast_gpio_set(int pin)
{
    gpio_set_level((gpio_num_t)pin, 1);
}

static inline void fast_gpio_clear(int pin)
{
    gpio_set_level((gpio_num_t)pin, 0);
}
#else
static inline void fast_gpio_set(int pin)
{
    GPIO.out_w1ts.val = (1U << pin);
}

static inline void fast_gpio_clear(int pin)
{
    GPIO.out_w1tc.val = (1U << pin);
}
#endif

static void set_gpio_pins(int pixelTrigVal, int lineTrigVal, int frameTrigVal)
{
    gpio_set_level((gpio_num_t)PIN_NUM_TRIG_PIXEL, pixelTrigVal);
    gpio_set_level((gpio_num_t)PIN_NUM_TRIG_LINE,  lineTrigVal);
    gpio_set_level((gpio_num_t)PIN_NUM_TRIG_FRAME, frameTrigVal);
}

static void trigger_camera(int tPixelDwelltime, int triggerPin = PIN_NUM_TRIG_PIXEL)
{
    fast_gpio_set(triggerPin);
    esp_rom_delay_us(tPixelDwelltime);
    fast_gpio_clear(triggerPin);
}

SPIRenderer::SPIRenderer(int xmin, int xmax,
                         int ymin, int ymax,
                         int step,
                         int tPixelDwelltime,
                         int nFramesI)
{
    X_MIN = xmin;
    X_MAX = xmax;
    Y_MIN = ymin;
    Y_MAX = ymax;
    STEP = step;
    this->tPixelDwelltime = tPixelDwelltime;
    nFrames = nFramesI;

    nX = (X_MAX - X_MIN) / STEP;
    nY = (Y_MAX - Y_MIN) / STEP;

    gpio_set_direction((gpio_num_t)PIN_NUM_LASER, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)PIN_NUM_TRIG_PIXEL, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)PIN_NUM_TRIG_LINE, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)PIN_NUM_TRIG_FRAME, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)PIN_NUM_LDAC, GPIO_MODE_OUTPUT);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_NUM_SDI;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.sclk_io_num = PIN_NUM_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;

    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 20000000; // 20 MHz
    devcfg.spics_io_num = PIN_NUM_CS;
    devcfg.queue_size = 2;

#ifdef IS_XIAO
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed");
    }
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed");
    }
#else
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed");
    }
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed");
    }
#endif
}

void SPIRenderer::setParameters(int xmin, int xmax,
                                int ymin, int ymax,
                                int step,
                                int tPixelDwelltime,
                                int nFramesI)
{
    X_MIN = xmin;
    X_MAX = xmax;
    Y_MIN = ymin;
    Y_MAX = ymax;
    STEP = step;
    this->tPixelDwelltime = tPixelDwelltime;
    nFrames = nFramesI;

    nX = (X_MAX - X_MIN) / STEP;
    nY = (Y_MAX - Y_MIN) / STEP;
    
    ESP_LOGI(TAG, "New params -> X=[%d..%d], Y=[%d..%d], STEP=%d, dwell=%d, frames=%d",
             X_MIN, X_MAX, Y_MIN, Y_MAX, STEP, this->tPixelDwelltime, nFrames);
}

void SPIRenderer::start(bool *stopFlag)
{
    draw(stopFlag);
}

void SPIRenderer::draw(bool *stopFlag)
{
    for (int iFrame = 0; iFrame < nFrames; iFrame++) {
        if (stopFlag && *stopFlag) return;

        // ESP_LOGI(TAG, "Frame %d of %d", iFrame+1, nFrames);

        // Optionally set frame trigger
        set_gpio_pins(0, 0, 1);

        for (int dacX = X_MIN; dacX <= X_MAX; dacX += STEP) {
            if (stopFlag && *stopFlag) return;
            // Optionally set line trigger
            set_gpio_pins(0, 1, 1);

            for (int dacY = Y_MIN; dacY <= Y_MAX; dacY += STEP) {
                if (stopFlag && *stopFlag) return;
                set_gpio_pins(1, 1, 1);

                // Prepare SPI transactions
                spi_transaction_t tX = {};
                tX.length = 16;
                tX.flags = SPI_TRANS_USE_TXDATA;
                tX.tx_data[0] = 0b00110000 | ((dacX >> 8) & 0x0F);
                tX.tx_data[1] = (uint8_t)(dacX & 0xFF);

                spi_transaction_t tY = {};
                tY.length = 16;
                tY.flags = SPI_TRANS_USE_TXDATA;
                tY.tx_data[0] = 0b10110000 | ((dacY >> 8) & 0x0F);
                tY.tx_data[1] = (uint8_t)(dacY & 0xFF);

                // Drive LDAC low, transmit X, transmit Y, then latch
                gpio_set_level((gpio_num_t)PIN_NUM_LDAC, 0);
                spi_device_polling_transmit(spi, &tX);
                spi_device_polling_transmit(spi, &tY);
                gpio_set_level((gpio_num_t)PIN_NUM_LDAC, 1);

                // Pixel trigger
                trigger_camera(tPixelDwelltime, PIN_NUM_TRIG_PIXEL);
                set_gpio_pins(0, 1, 1);
            }
            set_gpio_pins(0, 0, 1);
            //ESP_LOGI("SPIRenderer", "XMin %d, XMax %d, xVal: %d", X_MIN, X_MAX, dacX);
        }
        set_gpio_pins(0, 0, 0);
    }
}
