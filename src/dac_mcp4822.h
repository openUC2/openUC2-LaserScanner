#ifndef DAC_MCP4822_H
#define DAC_MCP4822_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

/**
 * @brief MCP4822 Dual 12-bit DAC driver
 * 
 * This driver provides control for the MCP4822 dual-channel DAC via SPI.
 * Channel A is used for X-axis galvo control.
 * Channel B is used for Y-axis galvo control.
 */
class MCP4822 {
public:
    /**
     * @brief Initialize the MCP4822 DAC
     * @param spi_host SPI host (e.g., SPI2_HOST)
     * @param pin_mosi MOSI pin number
     * @param pin_sclk SCLK pin number
     * @param pin_cs CS pin number
     * @param pin_ldac LDAC pin number (-1 if not used)
     * @param clock_speed_hz SPI clock speed in Hz (max 20 MHz for MCP4822)
     * @return true if initialization successful
     */
    bool init(spi_host_device_t spi_host, int pin_mosi, int pin_sclk, 
              int pin_cs, int pin_ldac, int clock_speed_hz = 20000000);

    /**
     * @brief Write a 12-bit value to a DAC channel
     * @param channel_b false for channel A (X), true for channel B (Y)
     * @param value 12-bit value (0-4095)
     */
    void write(bool channel_b, uint16_t value);

    /**
     * @brief Set X-axis position (channel A)
     * @param value 12-bit value (0-4095)
     */
    inline void setX(uint16_t value) { write(false, value); }

    /**
     * @brief Set Y-axis position (channel B)
     * @param value 12-bit value (0-4095)
     */
    inline void setY(uint16_t value) { write(true, value); }

    /**
     * @brief Pulse the LDAC pin to latch all channels
     * Only has effect if LDAC pin was configured during init
     */
    void ldacPulse();

private:
    spi_device_handle_t spi_ = nullptr;
    int pin_ldac_ = -1;
};

#endif // DAC_MCP4822_H
