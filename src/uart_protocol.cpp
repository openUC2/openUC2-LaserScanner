#include "uart_protocol.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "UARTProtocol";

UARTProtocol::UARTProtocol()
{
}

bool UARTProtocol::init(uart_port_t uart_port, int baud_rate, ScannerCore* scanner)
{
    if (!scanner) {
        ESP_LOGE(TAG, "Scanner pointer is null");
        return false;
    }

    uart_port_ = uart_port;
    scanner_ = scanner;

    // Configure UART
    uart_config_t uart_config = {};
    uart_config.baud_rate = baud_rate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    esp_err_t ret = uart_param_config(uart_port_, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %d", ret);
        return false;
    }

    ret = uart_driver_install(uart_port_, 4096, 0, 0, nullptr, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %d", ret);
        return false;
    }

    ESP_LOGI(TAG, "Initialized (UART%d, %d baud)", uart_port_, baud_rate);
    return true;
}

uint16_t UARTProtocol::checksum16(const uint8_t* data, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFF);
}

void UARTProtocol::sendReply(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
    // Header: MAGIC0 MAGIC1 (CMD|0x80) LEN_LOW LEN_HIGH
    uint8_t hdr[5];
    hdr[0] = MAGIC0;
    hdr[1] = MAGIC1;
    hdr[2] = cmd | 0x80; // Set reply bit
    hdr[3] = (uint8_t)(len & 0xFF);
    hdr[4] = (uint8_t)(len >> 8);

    // Compute checksum over cmd + len + payload
    uint16_t cks = 0;
    cks += checksum16(&hdr[2], 3); // cmd + len(2)
    if (payload && len) {
        cks += checksum16(payload, len);
    }

    // Tail: CKS_LOW CKS_HIGH
    uint8_t tail[2] = { (uint8_t)(cks & 0xFF), (uint8_t)(cks >> 8) };

    // Send header, payload, tail
    uart_write_bytes(uart_port_, (const char*)hdr, sizeof(hdr));
    if (payload && len) {
        uart_write_bytes(uart_port_, (const char*)payload, len);
    }
    uart_write_bytes(uart_port_, (const char*)tail, sizeof(tail));
}

void UARTProtocol::handleSetConfig(const uint8_t* payload, uint16_t len)
{
    if (len != sizeof(ScanConfig)) {
        uint8_t err = ERR_INVALID_CONFIG;
        sendReply(CMD_SET_CONFIG, &err, 1);
        return;
    }

    ScanConfig config;
    memcpy(&config, payload, sizeof(config));

    if (scanner_->setConfig(config)) {
        uint8_t ok = ERR_OK;
        sendReply(CMD_SET_CONFIG, &ok, 1);
    } else {
        uint8_t err = ERR_PARAM_RANGE;
        sendReply(CMD_SET_CONFIG, &err, 1);
    }
}

void UARTProtocol::handleStart()
{
    scanner_->start();
    uint8_t ok = ERR_OK;
    sendReply(CMD_START, &ok, 1);
}

void UARTProtocol::handleStop()
{
    scanner_->stop();
    uint8_t ok = ERR_OK;
    sendReply(CMD_STOP, &ok, 1);
}

void UARTProtocol::handleSetXLUT(const uint8_t* payload, uint16_t len)
{
    // Expect: 2 bytes (n_entries) + n_entries * 2 bytes (values)
    if (len < 2) {
        uint8_t err = ERR_LUT_LENGTH;
        sendReply(CMD_SET_X_LUT, &err, 1);
        return;
    }

    uint16_t n_entries = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    
    if (n_entries != X_LUT_N) {
        uint8_t err = ERR_LUT_SIZE;
        sendReply(CMD_SET_X_LUT, &err, 1);
        return;
    }

    if (len != (2 + n_entries * 2)) {
        uint8_t err = ERR_LUT_LENGTH;
        sendReply(CMD_SET_X_LUT, &err, 1);
        return;
    }

    // Parse LUT values (16-bit little-endian, 12-bit values)
    uint16_t lut[X_LUT_N];
    for (int i = 0; i < X_LUT_N; ++i) {
        int offset = 2 + i * 2;
        lut[i] = (uint16_t)payload[offset] | ((uint16_t)payload[offset + 1] << 8);
        lut[i] &= 0x0FFF; // Ensure 12-bit
    }

    scanner_->setXLUT(lut);

    uint8_t ok = ERR_OK;
    sendReply(CMD_SET_X_LUT, &ok, 1);
}

void UARTProtocol::handleGetStatus()
{
    ScannerStatus status = scanner_->getStatus();

    // Pack status into binary format
    uint8_t reply[1 + sizeof(StatusReply)];
    reply[0] = ERR_OK;

    StatusReply s;
    s.running = status.running ? 1 : 0;
    s.line = status.current_line;
    s.frame = status.current_frame;
    s.overruns = status.timing_overruns;

    memcpy(&reply[1], &s, sizeof(s));
    sendReply(CMD_GET_STATUS, reply, sizeof(reply));
}

void UARTProtocol::protocolTask()
{
    ESP_LOGI(TAG, "Protocol task started on core %d", xPortGetCoreID());

    static uint8_t rxbuf[2048];
    State state = S_SYNC0;
    uint8_t cmd = 0;
    uint16_t len = 0;
    uint16_t got = 0;
    uint16_t rx_ck = 0;

    uint8_t pkt_hdr[3];      // cmd + len(2)
    uint8_t payload[1536];

    while (true) {
        int n = uart_read_bytes(uart_port_, rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(20));
        if (n <= 0) continue;

        for (int i = 0; i < n; ++i) {
            uint8_t b = rxbuf[i];

            switch (state) {
                case S_SYNC0:
                    if (b == MAGIC0) state = S_SYNC1;
                    break;

                case S_SYNC1:
                    state = (b == MAGIC1) ? S_CMD : S_SYNC0;
                    break;

                case S_CMD:
                    cmd = b;
                    pkt_hdr[0] = b;
                    state = S_LEN0;
                    break;

                case S_LEN0:
                    len = b;
                    pkt_hdr[1] = b;
                    state = S_LEN1;
                    break;

                case S_LEN1:
                    len |= ((uint16_t)b << 8);
                    pkt_hdr[2] = b;
                    got = 0;
                    
                    if (len > sizeof(payload)) {
                        // Payload too large
                        uint8_t err = ERR_LENGTH;
                        sendReply(cmd, &err, 1);
                        state = S_SYNC0;
                    } else {
                        state = (len == 0) ? S_CK0 : S_PAYLOAD;
                    }
                    break;

                case S_PAYLOAD:
                    payload[got++] = b;
                    if (got >= len) state = S_CK0;
                    break;

                case S_CK0:
                    rx_ck = b;
                    state = S_CK1;
                    break;

                case S_CK1: {
                    rx_ck |= ((uint16_t)b << 8);

                    // Verify checksum
                    uint16_t calc_ck = checksum16(pkt_hdr, 3);
                    if (len > 0) {
                        calc_ck += checksum16(payload, len);
                    }

                    if (calc_ck != rx_ck) {
                        uint8_t err = ERR_CHECKSUM;
                        sendReply(cmd, &err, 1);
                        state = S_SYNC0;
                        break;
                    }

                    // Dispatch command
                    switch (cmd) {
                        case CMD_SET_CONFIG:
                            handleSetConfig(payload, len);
                            break;

                        case CMD_START:
                            handleStart();
                            break;

                        case CMD_STOP:
                            handleStop();
                            break;

                        case CMD_SET_X_LUT:
                            handleSetXLUT(payload, len);
                            break;

                        case CMD_GET_STATUS:
                            handleGetStatus();
                            break;

                        default: {
                            uint8_t err = ERR_UNKNOWN_CMD;
                            sendReply(cmd, &err, 1);
                            break;
                        }
                    }

                    state = S_SYNC0;
                } break;

                default:
                    state = S_SYNC0;
                    break;
            }
        }
    }
}
