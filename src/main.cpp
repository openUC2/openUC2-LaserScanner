// main.cpp (ESP-IDF)
// High-speed galvo scanner for laser projection
// Hardware: ESP32-S3 + MCP4822 dual 12-bit DAC

#include "dac_mcp4822.h"
#include "scanner_core.h"
#include "uart_protocol.h"
#include "esp_log.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
// Pin configuration for XIAO ESP32-S3 with UC2 Galvo Board
constexpr spi_host_device_t SPI_HOST = SPI2_HOST;
constexpr int PIN_SPI_MOSI = 9;   // D8 (GPIO9)
constexpr int PIN_SPI_SCLK = 7;   // D9 (GPIO7)
constexpr int PIN_SPI_CS   = 8;   // D10 (GPIO8)
constexpr int PIN_DAC_LDAC = 6;   // D7 (GPIO6)

// 3-Trigger system for camera synchronization
constexpr int PIN_TRIGGER_PIXEL = 2;  // D1 (GPIO2)
constexpr int PIN_TRIGGER_LINE  = 3;  // D2 (GPIO3)
constexpr int PIN_TRIGGER_FRAME = 4;  // D3 (GPIO4)

constexpr uart_port_t UART_PORT = UART_NUM_0;
constexpr int UART_BAUD = 921600;

static const char* TAG = "main";

// Global instances
static MCP4822 g_dac;
static ScannerCore g_scanner;
static UARTProtocol g_protocol;

// FreeRTOS task wrappers
static void scanner_task_wrapper(void* /*arg*/)
{
    g_scanner.scannerTask();
}

static void protocol_task_wrapper(void* /*arg*/)
{
    g_protocol.protocolTask();
}

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGI(TAG, "ESP32-S3 Galvo Scanner starting...");

    // Initialize DAC
    if (!g_dac.init(SPI_HOST, PIN_SPI_MOSI, PIN_SPI_SCLK, PIN_SPI_CS, PIN_DAC_LDAC)) {
        ESP_LOGE(TAG, "DAC initialization failed");
        return;
    }

    // Initialize scanner with 3-trigger system
    if (!g_scanner.init(&g_dac, PIN_TRIGGER_PIXEL, PIN_TRIGGER_LINE, PIN_TRIGGER_FRAME)) {
        ESP_LOGE(TAG, "Scanner initialization failed");
        return;
    }

    // Initialize UART protocol
    if (!g_protocol.init(UART_PORT, UART_BAUD, &g_scanner)) {
        ESP_LOGE(TAG, "Protocol initialization failed");
        return;
    }

    // Start scanner immediately with default parameters
    g_scanner.start();

    // Create scanner task (high priority, pinned to core 1)
    xTaskCreatePinnedToCore(
        scanner_task_wrapper,
        "scanner",
        4096,
        nullptr,
        configMAX_PRIORITIES - 1,  // Highest priority
        nullptr,
        1  // Core 1
    );

    // Create protocol task (lower priority, core 0)
    xTaskCreatePinnedToCore(
        protocol_task_wrapper,
        "proto",
        4096,
        nullptr,
        5,  // Medium priority
        nullptr,
        0  // Core 0
    );

    const ScanConfig& cfg = g_scanner.getConfig();
    ESP_LOGI(TAG, "Scanner started: %dx%d scan, %d us/sample", 
             cfg.nx, cfg.ny, cfg.sample_period_us);
    ESP_LOGI(TAG, "Ready for UART commands");
}
