// main.cpp (ESP-IDF)
// Minimal raster galvo scan engine for MCP4822 DAC via SPI
// - Fast axis: ramp (nx samples)
// - Slow axis: step per line (ny lines)
// - Pre blank + imaging + flyback (cosine ease) + optional line settle
// - Optional 256-entry X LUT mapped to 4096-entry table for O(1) per sample
// - Binary protocol over UART (no JSON)

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

// -------------------- Pins (edit to match your wiring) --------------------
// Pin definitions for XIAO ESP32-S3 with UC2 Galvo Board
static constexpr spi_host_device_t SPI_HOST = SPI2_HOST;
static constexpr int PIN_SPI_MOSI = 9;   // D8 (GPIO9) - MOSI/SDI
static constexpr int PIN_SPI_SCLK = 7;   // D9 (GPIO7) - SCLK
static constexpr int PIN_SPI_CS   = 8;   // D10 (GPIO8) - CS

// Optional LDAC pin (if you have it wired). If not used, set to -1.
static constexpr int PIN_DAC_LDAC = 6;   // D7 (GPIO6) - LDAC

// Trigger output pin (pixel strobe)
static constexpr int PIN_TRIGGER  = 2;   // D1 (GPIO2) - Pixel trigger

// UART for protocol (UART0 shares with logs; UART1/2 recommended if you have pins)
static constexpr uart_port_t UART_PORT = UART_NUM_0;
static constexpr int UART_BAUD = 921600;

// -------------------- Limits --------------------
static constexpr int MAX_LINE_SAMPLES = 4096; // pre + nx + fly + settle must fit
static constexpr int X_LUT_N = 256;           // upload size

// -------------------- DAC (MCP4822) --------------------
static spi_device_handle_t g_spi = nullptr;

// MCP4822 command bits:
// D15: DAC select (0=A, 1=B)
// D14: BUF (0=unbuffered, 1=buffered Vref)
// D13: GA (1=1x, 0=2x)
// D12: SHDN (1=active, 0=shutdown)
// D11..D0: data
static inline void mcp4822_write(bool chan_b, uint16_t v12)
{
  v12 &= 0x0FFF;
  uint16_t cmd = 0;
  cmd |= (chan_b ? 1 : 0) << 15;
  cmd |= (0u) << 14;   // BUF=0
  cmd |= (1u) << 13;   // GA=1 (1x)
  cmd |= (1u) << 12;   // active
  cmd |= v12;

  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.flags = SPI_TRANS_USE_TXDATA;
  t.length = 16;
  t.tx_data[0] = (uint8_t)(cmd >> 8);
  t.tx_data[1] = (uint8_t)(cmd & 0xFF);

  // Polling transmit: deterministic, no queue jitter
  spi_device_polling_transmit(g_spi, &t);
}

static inline void ldac_pulse_if_used()
{
  if (PIN_DAC_LDAC < 0) return;
  gpio_set_level((gpio_num_t)PIN_DAC_LDAC, 0);
  // short pulse
  __asm__ __volatile__("nop; nop; nop; nop;");
  gpio_set_level((gpio_num_t)PIN_DAC_LDAC, 1);
}

// -------------------- Protocol --------------------
static constexpr uint8_t MAGIC0 = 0xA5;
static constexpr uint8_t MAGIC1 = 0x5A;

static uint16_t checksum16(const uint8_t* p, size_t n)
{
  uint32_t s = 0;
  for (size_t i = 0; i < n; ++i) s += p[i];
  return (uint16_t)(s & 0xFFFF);
}

#pragma pack(push, 1)
struct scan_config_v1
{
  uint16_t nx, ny;
  uint16_t x_min, x_max, y_min, y_max;
  uint16_t pre_samples;
  uint16_t fly_samples;
  uint16_t sample_period_us;
  uint16_t trig_delay_us;
  uint16_t trig_width_us;
  uint16_t line_settle_samples;
  uint8_t  enable_trigger;
  uint8_t  apply_x_lut;
  uint16_t frame_count; // 0 = continuous
};
#pragma pack(pop)

struct status_v1
{
  uint8_t running;
  uint16_t line;
  uint32_t frame;
  int32_t overruns;
};

// -------------------- Scan state --------------------
static const char* TAG = "galvo_scan";

static scan_config_v1 g_cfg = {
  .nx = 256, .ny = 256,
  .x_min = 500, .x_max = 3500,
  .y_min = 500, .y_max = 3500,
  .pre_samples = 16,
  .fly_samples = 64,
  .sample_period_us = 20,
  .trig_delay_us = 3,
  .trig_width_us = 2,
  .line_settle_samples = 0,
  .enable_trigger = 1,
  .apply_x_lut = 0,
  .frame_count = 0
};

static SemaphoreHandle_t g_cfg_mutex;
static volatile bool g_running = false;
static volatile uint32_t g_frame_idx = 0;
static volatile uint16_t g_line_idx = 0;
static volatile int32_t g_overruns = 0;

// Precomputed X command sequence for a full line
static uint16_t g_line_x[MAX_LINE_SAMPLES];
static uint16_t g_line_len = 0;

// X LUT mapping (4096 entry) for O(1) apply
static uint16_t g_x_map_4096[4096];
static bool g_x_map_valid = false;

static inline uint16_t clamp12(int v)
{
  if (v < 0) return 0;
  if (v > 4095) return 4095;
  return (uint16_t)v;
}

static void rebuild_x_map_from_256(const uint16_t* lut256)
{
  // lut256 entries are 0..4095
  for (int x = 0; x < 4096; ++x) {
    int idx = (x * (X_LUT_N - 1) + 2047) / 4095; // rounded
    if (idx < 0) idx = 0;
    if (idx >= X_LUT_N) idx = X_LUT_N - 1;
    g_x_map_4096[x] = lut256[idx] & 0x0FFF;
  }
  g_x_map_valid = true;
}

static inline uint16_t apply_x_map(uint16_t x12)
{
  if (!g_x_map_valid) return x12;
  return g_x_map_4096[x12 & 0x0FFF];
}

static void build_line_profile_locked(const scan_config_v1& c)
{
  // Line layout:
  // pre (blank, x=x_min)
  // imaging ramp (nx samples, linear)
  // flyback (blank, cosine ease from x_max to x_min)
  // optional settle (blank, x=x_min)
  uint32_t total = (uint32_t)c.pre_samples + c.nx + c.fly_samples + c.line_settle_samples;
  if (total == 0 || total > MAX_LINE_SAMPLES) {
    g_line_len = 0;
    return;
  }

  uint32_t k = 0;

  // pre
  for (uint32_t i = 0; i < c.pre_samples; ++i) {
    g_line_x[k++] = c.x_min;
  }

  // imaging ramp
  if (c.nx <= 1) {
    g_line_x[k++] = c.x_min;
  } else {
    int32_t dx = (int32_t)c.x_max - (int32_t)c.x_min;
    for (uint32_t i = 0; i < c.nx; ++i) {
      // inclusive ramp: i=0 -> x_min, i=nx-1 -> x_max
      int32_t x = (int32_t)c.x_min + (dx * (int32_t)i) / (int32_t)(c.nx - 1);
      g_line_x[k++] = clamp12(x);
    }
  }

  // flyback: cosine ease (zero slope at ends)
  if (c.fly_samples == 0) {
    // no flyback region
  } else if (c.fly_samples == 1) {
    g_line_x[k++] = c.x_min;
  } else {
    float x0 = (float)c.x_max;
    float x1 = (float)c.x_min;
    for (uint32_t i = 0; i < c.fly_samples; ++i) {
      float t = (float)i / (float)(c.fly_samples - 1); // 0..1
      // ease: s = 0.5 - 0.5*cos(pi*t)
      float s = 0.5f - 0.5f * cosf((float)M_PI * t);
      float xf = x0 + (x1 - x0) * s;
      g_line_x[k++] = clamp12((int)(xf + 0.5f));
    }
  }

  // settle
  for (uint32_t i = 0; i < c.line_settle_samples; ++i) {
    g_line_x[k++] = c.x_min;
  }

  g_line_len = (uint16_t)k;
}

static inline uint16_t y_for_line_locked(const scan_config_v1& c, uint16_t line)
{
  if (c.ny <= 1) return c.y_min;
  int32_t dy = (int32_t)c.y_max - (int32_t)c.y_min;
  int32_t y = (int32_t)c.y_min + (dy * (int32_t)line) / (int32_t)(c.ny - 1);
  return clamp12(y);
}

// -------------------- Trigger pulse --------------------
static inline void trigger_pulse_us(uint16_t delay_us, uint16_t width_us)
{
  if (delay_us) ets_delay_us(delay_us);
  gpio_set_level((gpio_num_t)PIN_TRIGGER, 1);
  if (width_us) ets_delay_us(width_us);
  gpio_set_level((gpio_num_t)PIN_TRIGGER, 0);
}

// -------------------- Scanner task --------------------
static void scanner_task(void* /*arg*/)
{
  ESP_LOGI(TAG, "Scanner task started");
  // Pin task to a core for stability (create pinned task)
  while (true) {
    if (!g_running) {
      vTaskDelay(1);
      continue;
    }

    scan_config_v1 c;
    xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
    c = g_cfg;
    uint16_t line_len = g_line_len;
    xSemaphoreGive(g_cfg_mutex);

    if (line_len == 0 || c.nx == 0 || c.ny == 0) {
      g_running = false;
      continue;
    }

    // For speed: decide where imaging region starts/ends in the line buffer
    const uint32_t img_start = c.pre_samples;
    const uint32_t img_end   = c.pre_samples + c.nx; // exclusive

    for (uint16_t ly = 0; ly < c.ny && g_running; ++ly) {
      g_line_idx = ly;

      // Compute and set Y once per line
      uint16_t y12;
      xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
      y12 = y_for_line_locked(g_cfg, ly);
      bool do_lut = (g_cfg.apply_x_lut != 0) && g_x_map_valid;
      bool do_trig = (g_cfg.enable_trigger != 0);
      uint16_t sp_us = g_cfg.sample_period_us;
      uint16_t tdelay = g_cfg.trig_delay_us;
      uint16_t twidth = g_cfg.trig_width_us;
      xSemaphoreGive(g_cfg_mutex);

      ESP_LOGI("Writing Y", "Line %d/%d: Y=%d", ly + 1, c.ny, y12);
      mcp4822_write(true, y12); // channel B = Y
      // optional LDAC latch if you use it
      ldac_pulse_if_used(); 

      int64_t next_t = esp_timer_get_time();

      for (uint32_t i = 0; i < line_len && g_running; ++i) {
        next_t += sp_us;

        uint16_t x12 = g_line_x[i];
        if (do_lut) x12 = apply_x_map(x12);

        // Update X (channel A)
        mcp4822_write(false, x12);
        ldac_pulse_if_used();

        // Trigger only during imaging region
        if (do_trig && (i >= img_start) && (i < img_end)) {
          // Keep this short: ensure (tdelay + twidth) < sp_us
          trigger_pulse_us(tdelay, twidth);
        }

        // Wait to keep constant sample period
        while (true) {
          int64_t now = esp_timer_get_time();
          if (now >= next_t) {
            if (now - next_t > (int64_t)sp_us) {
              // late by more than one period
              g_overruns++;
            }
            break;
          }
        }
      }
    }

    g_frame_idx++;

    // Stop after N frames if requested
    xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
    uint16_t fc = g_cfg.frame_count;
    xSemaphoreGive(g_cfg_mutex);
    if (fc != 0 && g_frame_idx >= fc) {
      g_running = false;
    }
  }
}

// -------------------- UART protocol task --------------------
static void send_reply(uint8_t cmd, const uint8_t* payload, uint16_t len)
{
  uint8_t hdr[5];
  hdr[0] = MAGIC0;
  hdr[1] = MAGIC1;
  hdr[2] = (uint8_t)(cmd | 0x80);
  hdr[3] = (uint8_t)(len & 0xFF);
  hdr[4] = (uint8_t)(len >> 8);

  uint16_t cks = 0;
  cks += checksum16(&hdr[2], 3); // cmd + len(2)
  if (payload && len) cks += checksum16(payload, len);

  uint8_t tail[2] = { (uint8_t)(cks & 0xFF), (uint8_t)(cks >> 8) };

  uart_write_bytes(UART_PORT, (const char*)hdr, sizeof(hdr));
  if (payload && len) uart_write_bytes(UART_PORT, (const char*)payload, len);
  uart_write_bytes(UART_PORT, (const char*)tail, sizeof(tail));
}

static void proto_task(void* /*arg*/)
{
  static uint8_t rxbuf[2048];

  enum { S_SYNC0, S_SYNC1, S_CMD, S_LEN0, S_LEN1, S_PAYLOAD, S_CK0, S_CK1 } st = S_SYNC0;
  uint8_t cmd = 0;
  uint16_t len = 0;
  uint16_t got = 0;
  uint16_t rx_ck = 0;

  uint8_t pkt_hdr_part[3]; // cmd + len0 + len1 (for checksum)
  uint8_t payload[1536];

  while (true) {
    int n = uart_read_bytes(UART_PORT, rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(20));
    if (n <= 0) continue;

    for (int i = 0; i < n; ++i) {
      uint8_t b = rxbuf[i];

      switch (st) {
        case S_SYNC0:
          if (b == MAGIC0) st = S_SYNC1;
          break;

        case S_SYNC1:
          st = (b == MAGIC1) ? S_CMD : S_SYNC0;
          break;

        case S_CMD:
          cmd = b;
          pkt_hdr_part[0] = b;
          st = S_LEN0;
          break;

        case S_LEN0:
          len = b;
          pkt_hdr_part[1] = b;
          st = S_LEN1;
          break;

        case S_LEN1:
          len |= ((uint16_t)b << 8);
          pkt_hdr_part[2] = b;
          got = 0;
          if (len > sizeof(payload)) {
            st = S_SYNC0;
            // error reply
            uint8_t p[1] = { 1 };
            send_reply(cmd, p, 1);
          } else {
            st = (len == 0) ? S_CK0 : S_PAYLOAD;
          }
          break;

        case S_PAYLOAD:
          payload[got++] = b;
          if (got >= len) st = S_CK0;
          break;

        case S_CK0:
          rx_ck = b;
          st = S_CK1;
          break;

        case S_CK1: {
          rx_ck |= ((uint16_t)b << 8);

          uint16_t ck = 0;
          ck += checksum16(pkt_hdr_part, 3);
          if (len) ck += checksum16(payload, len);

          if (ck != rx_ck) {
            uint8_t p[1] = { 2 }; // checksum error
            send_reply(cmd, p, 1);
            st = S_SYNC0;
            break;
          }

          // Handle command
          if (cmd == 0x01) { // SET_CONFIG
            if (len != sizeof(scan_config_v1)) {
              uint8_t p[1] = { 3 };
              send_reply(cmd, p, 1);
            } else {
              scan_config_v1 c;
              memcpy(&c, payload, sizeof(c));

              // Basic sanity
              uint32_t line_total = (uint32_t)c.pre_samples + c.nx + c.fly_samples + c.line_settle_samples;
              if (c.nx == 0 || c.ny == 0 || line_total == 0 || line_total > MAX_LINE_SAMPLES || c.sample_period_us == 0) {
                uint8_t p[1] = { 4 };
                send_reply(cmd, p, 1);
              } else if ((uint32_t)c.trig_delay_us + (uint32_t)c.trig_width_us >= (uint32_t)c.sample_period_us) {
                // Must fit inside one sample period
                uint8_t p[1] = { 5 };
                send_reply(cmd, p, 1);
              } else {
                xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
                g_cfg = c;
                build_line_profile_locked(g_cfg);
                xSemaphoreGive(g_cfg_mutex);

                uint8_t p[1] = { 0 };
                send_reply(cmd, p, 1);
              }
            }
          }
          else if (cmd == 0x02) { // START
            g_frame_idx = 0;
            g_line_idx = 0;
            g_overruns = 0;
            g_running = true;
            uint8_t p[1] = { 0 };
            send_reply(cmd, p, 1);
          }
          else if (cmd == 0x03) { // STOP
            g_running = false;
            uint8_t p[1] = { 0 };
            send_reply(cmd, p, 1);
          }
          else if (cmd == 0x04) { // SET_X_LUT (expects 256 entries)
            if (len != (2 + X_LUT_N * 2)) {
              uint8_t p[1] = { 6 };
              send_reply(cmd, p, 1);
            } else {
              uint16_t n_lut = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
              if (n_lut != X_LUT_N) {
                uint8_t p[1] = { 7 };
                send_reply(cmd, p, 1);
              } else {
                uint16_t lut256[X_LUT_N];
                for (int j = 0; j < X_LUT_N; ++j) {
                  int o = 2 + j * 2;
                  lut256[j] = (uint16_t)payload[o] | ((uint16_t)payload[o + 1] << 8);
                  lut256[j] &= 0x0FFF;
                }
                rebuild_x_map_from_256(lut256);
                uint8_t p[1] = { 0 };
                send_reply(cmd, p, 1);
              }
            }
          }
          else if (cmd == 0x05) { // GET_STATUS
            uint8_t out[1 + sizeof(status_v1)];
            out[0] = 0;
            status_v1 s;
            s.running = g_running ? 1 : 0;
            s.line = g_line_idx;
            s.frame = g_frame_idx;
            s.overruns = g_overruns;
            memcpy(&out[1], &s, sizeof(s));
            send_reply(cmd, out, (uint16_t)sizeof(out));
          }
          else {
            uint8_t p[1] = { 0xFF };
            send_reply(cmd, p, 1);
          }

          st = S_SYNC0;
        } break;

        default:
          st = S_SYNC0;
          break;
      }
    }
  }
}

// -------------------- Init --------------------
extern "C" void app_main(void)
{
  // SPI bus
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = PIN_SPI_MOSI;
  buscfg.miso_io_num = -1;
  buscfg.sclk_io_num = PIN_SPI_SCLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

  ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 20 * 1000 * 1000; // 20 MHz
  devcfg.mode = 0;
  devcfg.spics_io_num = PIN_SPI_CS;
  devcfg.queue_size = 1;

  ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &g_spi));

  // Trigger pin
  gpio_config_t io = {};
  io.intr_type = GPIO_INTR_DISABLE;
  io.mode = GPIO_MODE_OUTPUT;
  io.pin_bit_mask = (1ULL << PIN_TRIGGER);
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io));
  gpio_set_level((gpio_num_t)PIN_TRIGGER, 0);

  if (PIN_DAC_LDAC >= 0) {
    gpio_config_t ld = io;
    ld.pin_bit_mask = (1ULL << PIN_DAC_LDAC);
    ESP_ERROR_CHECK(gpio_config(&ld));
    gpio_set_level((gpio_num_t)PIN_DAC_LDAC, 1);
  }

  // UART protocol
  uart_config_t ucfg = {};
  ucfg.baud_rate = UART_BAUD;
  ucfg.data_bits = UART_DATA_8_BITS;
  ucfg.parity = UART_PARITY_DISABLE;
  ucfg.stop_bits = UART_STOP_BITS_1;
  ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &ucfg));
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 4096, 0, 0, nullptr, 0));

  // Build initial line profile
  g_cfg_mutex = xSemaphoreCreateMutex();
  xSemaphoreTake(g_cfg_mutex, portMAX_DELAY);
  build_line_profile_locked(g_cfg);
  xSemaphoreGive(g_cfg_mutex);

  // Park mirrors at start
  mcp4822_write(false, g_cfg.x_min);
  mcp4822_write(true,  g_cfg.y_min);

  // Start scanning immediately with default parameters
  g_running = true;
  g_frame_idx = 0;
  g_line_idx = 0;
  g_overruns = 0;

  // Tasks: scanner high prio pinned, proto lower prio
  xTaskCreatePinnedToCore(scanner_task, "scanner", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr, 1);
  xTaskCreatePinnedToCore(proto_task,   "proto",   4096, nullptr, 5,                         nullptr, 0);

  ESP_LOGI(TAG, "Scanner started with default parameters (nx=%d, ny=%d, %d us/sample).", 
           g_cfg.nx, g_cfg.ny, g_cfg.sample_period_us);
}
