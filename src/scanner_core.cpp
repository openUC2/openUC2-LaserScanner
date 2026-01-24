#include "scanner_core.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "esp_rom_sys.h"
#include <cmath>
#include <cstring>

static const char* TAG = "ScannerCore";

ScannerCore::ScannerCore()
{
    config_mutex_ = xSemaphoreCreateMutex();
    
    // Set default configuration
    config_.nx = 512;
    config_.ny = 512;
    config_.x_min = 1500;
    config_.x_max = 4000;
    config_.y_min = 1500;
    config_.y_max = 4000;
    config_.pre_samples = 4;
    config_.fly_samples = 16;
    config_.sample_period_us = 0;
    config_.trig_delay_us = 0;
    config_.trig_width_us = 0;
    config_.line_settle_samples = 0;
    config_.enable_trigger = 1;
    config_.apply_x_lut = 0;
    config_.frame_count = 0;
}

ScannerCore::~ScannerCore()
{
    stop();
    if (config_mutex_) {
        vSemaphoreDelete(config_mutex_);
    }
}

bool ScannerCore::init(MCP4822* dac, int trigger_pin_pixel, int trigger_pin_line, int trigger_pin_frame)
{
    if (!dac) {
        ESP_LOGE(TAG, "DAC pointer is null");
        return false;
    }

    dac_ = dac;
    trigger_pin_pixel_ = trigger_pin_pixel;
    trigger_pin_line_ = trigger_pin_line;
    trigger_pin_frame_ = trigger_pin_frame;

    // Configure all three trigger pins
    uint32_t pin_mask = 0;
    if (trigger_pin_pixel_ >= 0) pin_mask |= (1ULL << trigger_pin_pixel_);
    if (trigger_pin_line_ >= 0) pin_mask |= (1ULL << trigger_pin_line_);
    if (trigger_pin_frame_ >= 0) pin_mask |= (1ULL << trigger_pin_frame_);
    
    if (pin_mask != 0) {
        gpio_config_t io = {};
        io.intr_type = GPIO_INTR_DISABLE;
        io.mode = GPIO_MODE_OUTPUT;
        io.pin_bit_mask = pin_mask;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io);
        
        // Initialize all triggers low
        if (trigger_pin_pixel_ >= 0) gpio_set_level((gpio_num_t)trigger_pin_pixel_, 0);
        if (trigger_pin_line_ >= 0) gpio_set_level((gpio_num_t)trigger_pin_line_, 0);
        if (trigger_pin_frame_ >= 0) gpio_set_level((gpio_num_t)trigger_pin_frame_, 0);
        
        ESP_LOGI(TAG, "Trigger pins configured: Pixel=GPIO%d, Line=GPIO%d, Frame=GPIO%d", 
                 trigger_pin_pixel_, trigger_pin_line_, trigger_pin_frame_);
    }

    // Build initial line profile
    buildLineProfile();

    // Park mirrors at start position
    dac_->setX(config_.x_min);
    dac_->setY(config_.y_min);

    ESP_LOGI(TAG, "Initialized");
    return true;
}

bool ScannerCore::setConfig(const ScanConfig& config)
{
    // Validate configuration
    uint32_t line_total = (uint32_t)config.pre_samples + config.nx + 
                          config.fly_samples + config.line_settle_samples;
    
    if (config.nx == 0 || config.ny == 0) {
        ESP_LOGE(TAG, "Invalid nx or ny");
        return false;
    }
    
    if (line_total == 0 || line_total > MAX_LINE_SAMPLES) {
        ESP_LOGE(TAG, "Line total out of range: %d (max %d)", line_total, MAX_LINE_SAMPLES);
        return false;
    }
    
    if (config.sample_period_us == 0) {
        ESP_LOGE(TAG, "Sample period must be > 0");
        return false;
    }
    
    if ((uint32_t)config.trig_delay_us + (uint32_t)config.trig_width_us >= (uint32_t)config.sample_period_us) {
        ESP_LOGE(TAG, "Trigger timing must fit within sample period");
        return false;
    }

    // Apply configuration
    xSemaphoreTake(config_mutex_, portMAX_DELAY);
    config_ = config;
    buildLineProfile();
    xSemaphoreGive(config_mutex_);

    ESP_LOGI(TAG, "Config updated: %dx%d, %d us/sample", config.nx, config.ny, config.sample_period_us);
    return true;
}

void ScannerCore::setXLUT(const uint16_t* lut256)
{
    if (!lut256) return;

    // Expand 256-entry LUT to 4096 entries for O(1) lookup
    for (int x = 0; x < 4096; ++x) {
        int idx = (x * (X_LUT_N - 1) + 2047) / 4095; // Rounded interpolation
        if (idx < 0) idx = 0;
        if (idx >= X_LUT_N) idx = X_LUT_N - 1;
        x_map_[x] = lut256[idx] & 0x0FFF;
    }
    x_map_valid_ = true;

    ESP_LOGI(TAG, "X LUT updated");
}

void ScannerCore::start()
{
    frame_idx_ = 0;
    line_idx_ = 0;
    overruns_ = 0;
    running_ = true;
    ESP_LOGI(TAG, "Started");
}

void ScannerCore::stop()
{
    running_ = false;
    ESP_LOGI(TAG, "Stopped");
}

ScannerStatus ScannerCore::getStatus() const
{
    return {
        .running = running_,
        .current_line = line_idx_,
        .current_frame = frame_idx_,
        .timing_overruns = overruns_
    };
}

void ScannerCore::buildLineProfile()
{
    // Line structure:
    // [pre blanking] [imaging ramp] [flyback cosine] [settle]
    
    uint32_t total = (uint32_t)config_.pre_samples + config_.nx + 
                     config_.fly_samples + config_.line_settle_samples;
    
    if (total == 0 || total > MAX_LINE_SAMPLES) {
        line_len_ = 0;
        return;
    }

    uint32_t k = 0;

    // Pre-blanking region (hold at x_min)
    for (uint32_t i = 0; i < config_.pre_samples; ++i) {
        line_x_[k++] = config_.x_min;
    }

    // Imaging region (linear ramp from x_min to x_max)
    if (config_.nx <= 1) {
        line_x_[k++] = config_.x_min;
    } else {
        int32_t dx = (int32_t)config_.x_max - (int32_t)config_.x_min;
        for (uint32_t i = 0; i < config_.nx; ++i) {
            int32_t x = (int32_t)config_.x_min + (dx * (int32_t)i) / (int32_t)(config_.nx - 1);
            line_x_[k++] = clamp12(x);
        }
    }

    // Flyback region (cosine ease from x_max to x_min)
    if (config_.fly_samples == 0) {
        // No flyback
    } else if (config_.fly_samples == 1) {
        line_x_[k++] = config_.x_min;
    } else {
        float x0 = (float)config_.x_max;
        float x1 = (float)config_.x_min;
        for (uint32_t i = 0; i < config_.fly_samples; ++i) {
            float t = (float)i / (float)(config_.fly_samples - 1); // 0 to 1
            // Cosine ease: s = 0.5 - 0.5*cos(pi*t)
            float s = 0.5f - 0.5f * cosf((float)M_PI * t);
            float xf = x0 + (x1 - x0) * s;
            line_x_[k++] = clamp12((int)(xf + 0.5f));
        }
    }

    // Settle region (hold at x_min)
    for (uint32_t i = 0; i < config_.line_settle_samples; ++i) {
        line_x_[k++] = config_.x_min;
    }

    line_len_ = (uint16_t)k;
}

uint16_t ScannerCore::computeY(uint16_t line) const
{
    if (config_.ny <= 1) return config_.y_min;
    int32_t dy = (int32_t)config_.y_max - (int32_t)config_.y_min;
    int32_t y = (int32_t)config_.y_min + (dy * (int32_t)line) / (int32_t)(config_.ny - 1);
    return clamp12(y);
}

uint16_t ScannerCore::applyXMap(uint16_t x) const
{
    if (!x_map_valid_) return x;
    return x_map_[x & 0x0FFF];
}

void ScannerCore::triggerPulsePixel(uint16_t dwell_us)
{
    if (trigger_pin_pixel_ < 0) return;
    
    // Fast GPIO register access for minimal jitter
    GPIO.out_w1ts = (1U << trigger_pin_pixel_);
    if (dwell_us > 0) {
        esp_rom_delay_us(dwell_us);
    }
    GPIO.out_w1tc = (1U << trigger_pin_pixel_);
}

void ScannerCore::triggerPulseLine()
{
    if (trigger_pin_line_ < 0) return;
    
    // 5µs pulse like SPIRenderer
    GPIO.out_w1ts = (1U << trigger_pin_line_);
    esp_rom_delay_us(5);
    GPIO.out_w1tc = (1U << trigger_pin_line_);
}

void ScannerCore::triggerPulseFrame()
{
    if (trigger_pin_frame_ < 0) return;
    
    // 5µs pulse like SPIRenderer
    GPIO.out_w1ts = (1U << trigger_pin_frame_);
    esp_rom_delay_us(5);
    GPIO.out_w1tc = (1U << trigger_pin_frame_);
}

void ScannerCore::scannerTask()
{
    ESP_LOGI(TAG, "Scanner task started on core %d", xPortGetCoreID());
    
    // CRITICAL: Unsubscribe this task from task watchdog
    // The tight timing loop cannot yield, so we must disable watchdog monitoring
    esp_task_wdt_delete(NULL);
    ESP_LOGI(TAG, "Task watchdog disabled for scanner task");

    while (true) {
        if (!running_) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Get current configuration (snapshot for this frame)
        ScanConfig cfg;
        xSemaphoreTake(config_mutex_, portMAX_DELAY);
        cfg = config_;
        uint16_t line_len = line_len_;
        xSemaphoreGive(config_mutex_);

        if (line_len == 0 || cfg.nx == 0 || cfg.ny == 0) {
            running_ = false;
            continue;
        }

        // Imaging region boundaries in line buffer
        const uint32_t img_start = cfg.pre_samples;
        const uint32_t img_end = cfg.pre_samples + cfg.nx;

        // ----------------------------------------------------------------------
        // FRAME TRIGGER - Once at start of frame (like SPIRenderer)
        // ----------------------------------------------------------------------
        if (cfg.enable_trigger) {
            triggerPulseFrame();
        }

        int lineNumber = 0;

        // ----------------------------------------------------------------------
        // X-loop: Scan all lines (Y steps)
        // ----------------------------------------------------------------------
        for (uint16_t ly = 0; ly < cfg.ny && running_; ++ly) {
            line_idx_ = ly;

            // LINE TRIGGER - Once per line (like SPIRenderer)
            if (cfg.enable_trigger) {
                triggerPulseLine();
            }

            // Compute Y position for this line
            uint16_t y12;
            xSemaphoreTake(config_mutex_, portMAX_DELAY);
            y12 = computeY(ly);
            bool do_lut = (config_.apply_x_lut != 0) && x_map_valid_;
            bool do_trig = (config_.enable_trigger != 0);
            uint16_t sp_us = config_.sample_period_us;
            xSemaphoreGive(config_mutex_);

            // Set Y position once per line
            dac_->setY(y12);
            dac_->ldacPulse();

            int64_t next_t = esp_timer_get_time();

            // ------------------------------------------------------------------
            // Scan one line (all X samples including pre/imaging/flyback/settle)
            // ------------------------------------------------------------------
            for (uint32_t i = 0; i < line_len && running_; ++i) {
                next_t += sp_us;

                // Get X position and apply LUT if enabled
                uint16_t x12 = line_x_[i];
                if (do_lut) x12 = applyXMap(x12);

                // Update X position via DAC
                dac_->setX(x12);
                dac_->ldacPulse();

                // PIXEL TRIGGER - Only during imaging region (like SPIRenderer)
                // Use sp_us as dwell time for pixel trigger
                if (do_trig && (i >= img_start) && (i < img_end)) {
                    triggerPulsePixel(sp_us);
                }

                // Timing control - busy wait for precise timing
                // This is why we unsubscribed from the watchdog
                while (true) {
                    int64_t now = esp_timer_get_time();
                    if (now >= next_t) {
                        if (now - next_t > (int64_t)sp_us) {
                            overruns_++;
                        }
                        break;
                    }
                }
            }

            lineNumber++;
        }

        frame_idx_++;

        // Stop if frame count limit reached
        xSemaphoreTake(config_mutex_, portMAX_DELAY);
        uint16_t fc = config_.frame_count;
        xSemaphoreGive(config_mutex_);
        
        if (fc != 0 && frame_idx_ >= fc) {
            running_ = false;
        }
    }
}
