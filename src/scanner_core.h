#ifndef SCANNER_CORE_H
#define SCANNER_CORE_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "dac_mcp4822.h"

// Maximum line samples (pre + imaging + flyback + settle)
constexpr int MAX_LINE_SAMPLES = 4096;

// X LUT size for upload
constexpr int X_LUT_N = 256;

/**
 * @brief Scan configuration structure
 * Defines all parameters for a raster scan pattern
 */
#pragma pack(push, 1)
struct ScanConfig {
    uint16_t nx;                    // Number of X samples per line
    uint16_t ny;                    // Number of lines (Y steps)
    uint16_t x_min, x_max;          // X range (0-4095)
    uint16_t y_min, y_max;          // Y range (0-4095)
    uint16_t pre_samples;           // Blanking samples before imaging
    uint16_t fly_samples;           // Flyback samples (cosine ease)
    uint16_t sample_period_us;      // Microseconds per sample
    uint16_t trig_delay_us;         // Trigger delay after position update
    uint16_t trig_width_us;         // Trigger pulse width
    uint16_t line_settle_samples;   // Settling samples after flyback
    uint8_t  enable_trigger;        // Enable pixel trigger output
    uint8_t  apply_x_lut;           // Apply X lookup table
    uint16_t frame_count;           // Number of frames (0 = continuous)
    bool    bidir;               // Bidirectional scanning enabled
};
#pragma pack(pop)

/**
 * @brief Scanner status information
 */
struct ScannerStatus {
    bool running;
    uint16_t current_line;
    uint32_t current_frame;
    int32_t timing_overruns;
};

/**
 * @brief Galvo scanner core engine
 * 
 * Implements high-speed raster scanning with:
 * - Pre-blanking, imaging, flyback (cosine ease), and settling regions
 * - Configurable pixel trigger output
 * - Optional X-axis correction LUT
 * - Real-time timing monitoring
 */
class ScannerCore {
public:
    ScannerCore();
    ~ScannerCore();

    /**
     * @brief Initialize scanner with DAC and trigger pins
     * @param dac Initialized MCP4822 DAC instance
     * @param trigger_pin_pixel GPIO pin for pixel trigger
     * @param trigger_pin_line GPIO pin for line trigger  
     * @param trigger_pin_frame GPIO pin for frame trigger
     * @return true if successful
     */
    bool init(MCP4822* dac, int trigger_pin_pixel, int trigger_pin_line, int trigger_pin_frame);

    /**
     * @brief Set scan configuration
     * @param config Scan configuration structure
     * @return true if configuration is valid and applied
     */
    bool setConfig(const ScanConfig& config);

    /**
     * @brief Get current scan configuration
     */
    const ScanConfig& getConfig() const { return config_; }

    /**
     * @brief Upload X-axis correction LUT (256 entries)
     * @param lut256 Array of 256 12-bit values (0-4095)
     */
    void setXLUT(const uint16_t* lut256);

    /**
     * @brief Start scanning
     */
    void start();

    /**
     * @brief Stop scanning
     */
    void stop();

    /**
     * @brief Get scanner status
     */
    ScannerStatus getStatus() const;

    /**
     * @brief Scanner task function (called by FreeRTOS)
     */
    void scannerTask();

private:
    // DAC interface
    MCP4822* dac_ = nullptr;
    int trigger_pin_pixel_ = -1;
    int trigger_pin_line_ = -1;
    int trigger_pin_frame_ = -1;

    // Configuration
    ScanConfig config_;
    SemaphoreHandle_t config_mutex_;

    // Runtime state
    volatile bool running_ = false;
    volatile uint32_t frame_idx_ = 0;
    volatile uint16_t line_idx_ = 0;
    volatile int32_t overruns_ = 0;

    // Pre-computed line profile
    uint16_t line_x_[MAX_LINE_SAMPLES];
    uint16_t line_len_ = 0;

    // X correction LUT (4096 entries for O(1) lookup)
    uint16_t x_map_[4096];
    bool x_map_valid_ = false;

    // Helper functions
    void buildLineProfile();
    uint16_t computeY(uint16_t line) const;
    uint16_t applyXMap(uint16_t x) const;
    void triggerPulsePixel();
    void triggerPulseLine();
    void triggerPulseFrame();
    
    static inline uint16_t clamp12(int v) {
        if (v < 0) return 0;
        if (v > 4095) return 4095;
        return (uint16_t)v;
    }
};

#endif // SCANNER_CORE_H
