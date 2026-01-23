#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>
#include "driver/uart.h"
#include "scanner_core.h"

// Protocol magic bytes
constexpr uint8_t MAGIC0 = 0xA5;
constexpr uint8_t MAGIC1 = 0x5A;

// Command codes
enum ProtocolCommand : uint8_t {
    CMD_SET_CONFIG = 0x01,  // Set scan configuration
    CMD_START      = 0x02,  // Start scanning
    CMD_STOP       = 0x03,  // Stop scanning
    CMD_SET_X_LUT  = 0x04,  // Upload X correction LUT
    CMD_GET_STATUS = 0x05   // Get current status
};

// Error codes
enum ProtocolError : uint8_t {
    ERR_OK              = 0x00,
    ERR_LENGTH          = 0x01,
    ERR_CHECKSUM        = 0x02,
    ERR_INVALID_CONFIG  = 0x03,
    ERR_PARAM_RANGE     = 0x04,
    ERR_TRIGGER_TIMING  = 0x05,
    ERR_LUT_LENGTH      = 0x06,
    ERR_LUT_SIZE        = 0x07,
    ERR_UNKNOWN_CMD     = 0xFF
};

// Status reply structure (matches protocol)
#pragma pack(push, 1)
struct StatusReply {
    uint8_t running;
    uint16_t line;
    uint32_t frame;
    int32_t overruns;
};
#pragma pack(pop)

/**
 * @brief Binary UART protocol handler
 * 
 * Implements a binary protocol with:
 * - Magic byte sync (0xA5 0x5A)
 * - Command byte
 * - Length field (16-bit little-endian)
 * - Payload
 * - Checksum (16-bit simple sum)
 * 
 * All replies have bit 7 of command set (0x80 | cmd)
 */
class UARTProtocol {
public:
    UARTProtocol();

    /**
     * @brief Initialize UART protocol
     * @param uart_port UART port number (UART_NUM_0, UART_NUM_1, etc.)
     * @param baud_rate Baud rate
     * @param scanner Scanner core instance to control
     * @return true if successful
     */
    bool init(uart_port_t uart_port, int baud_rate, ScannerCore* scanner);

    /**
     * @brief Protocol task function (called by FreeRTOS)
     */
    void protocolTask();

private:
    uart_port_t uart_port_;
    ScannerCore* scanner_ = nullptr;

    // Protocol state machine
    enum State {
        S_SYNC0,
        S_SYNC1,
        S_CMD,
        S_LEN0,
        S_LEN1,
        S_PAYLOAD,
        S_CK0,
        S_CK1
    };

    // Helper functions
    void sendReply(uint8_t cmd, const uint8_t* payload, uint16_t len);
    uint16_t checksum16(const uint8_t* data, size_t len);
    
    // Command handlers
    void handleSetConfig(const uint8_t* payload, uint16_t len);
    void handleStart();
    void handleStop();
    void handleSetXLUT(const uint8_t* payload, uint16_t len);
    void handleGetStatus();
};

#endif // UART_PROTOCOL_H
