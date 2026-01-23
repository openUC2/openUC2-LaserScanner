# ESP32-S3 Galvo Scanner Architecture

High-performance galvo mirror scanner for laser projection systems using ESP32-S3 and MCP4822 dual 12-bit DAC.

## Overview

This firmware implements a high-speed raster scanning system with:
- **Hardware**: ESP32-S3 XIAO + MCP4822 dual DAC (SPI)
- **Performance**: Microsecond-precision timing with deterministic scanning
- **Control**: Binary UART protocol for configuration and real-time control
- **Features**: X-axis correction LUT, configurable trigger outputs, multi-region line structure

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         ESP32-S3                             │
│  ┌────────────┐  ┌──────────────┐  ┌──────────────────┐    │
│  │ UART       │  │ Scanner Core │  │ DAC Driver       │    │
│  │ Protocol   ├─→│              ├─→│ (MCP4822)        │─→ SPI
│  │ (Core 0)   │  │ (Core 1)     │  │                  │    │
│  └────────────┘  └──────────────┘  └──────────────────┘    │
│                         │                                    │
│                         ↓                                    │
│                  GPIO Trigger Out                            │
└─────────────────────────────────────────────────────────────┘
```

### Core Components

#### 1. DAC Driver (`dac_mcp4822.h/cpp`)
- **Purpose**: Low-level SPI communication with MCP4822 DAC
- **Features**:
  - Polling-based SPI transfers (deterministic timing, no queue jitter)
  - Channel A: X-axis galvo control
  - Channel B: Y-axis galvo control
  - Optional LDAC pulse for synchronous latch
  - 12-bit resolution (0-4095)

#### 2. Scanner Core (`scanner_core.h/cpp`)
- **Purpose**: High-speed raster scan pattern generation
- **Key Features**:
  - **Line Structure**: Pre-blanking → Imaging → Flyback (cosine ease) → Settle
  - **Timing**: Microsecond-precision busy-wait loop (unsubscribed from watchdog)
  - **X-LUT**: Optional 256-entry correction table (expanded to 4096 for O(1) lookup)
  - **Trigger Control**: Configurable pixel trigger during imaging region
  - **Thread Safety**: Mutex-protected configuration updates

#### 3. UART Protocol (`uart_protocol.h/cpp`)
- **Purpose**: Binary command/response protocol for control
- **Protocol Format**:
  ```
  [MAGIC0][MAGIC1][CMD][LEN_LOW][LEN_HIGH][PAYLOAD...][CKS_LOW][CKS_HIGH]
  ```
- **Commands**:
  - `0x01`: SET_CONFIG - Upload scan configuration
  - `0x02`: START - Begin scanning
  - `0x03`: STOP - Stop scanning
  - `0x04`: SET_X_LUT - Upload X-axis correction table
  - `0x05`: GET_STATUS - Query current status

#### 4. Main (`main.cpp`)
- **Purpose**: System initialization and task creation
- **Responsibilities**:
  - Hardware initialization
  - Task creation (Scanner on Core 1, Protocol on Core 0)
  - Auto-start with default parameters

## Pin Configuration (XIAO ESP32-S3)

| Function    | GPIO | XIAO Pin | Description              |
|-------------|------|----------|--------------------------|
| SPI MOSI    | 9    | D8       | DAC data input           |
| SPI SCLK    | 7    | D9       | DAC clock                |
| SPI CS      | 8    | D10      | DAC chip select          |
| LDAC        | 6    | D7       | DAC latch (optional)     |
| TRIGGER     | 2    | D1       | Pixel trigger output     |

## Scan Pattern Details

### Line Structure
Each scan line consists of four regions:

```
Time →
┌──────┬─────────────────────┬──────────────────┬──────┐
│ Pre  │   Imaging Region    │    Flyback       │Settle│
│Blank │   (nx samples)      │  (cosine ease)   │      │
└──────┴─────────────────────┴──────────────────┴──────┘
  ↑                              ↑
 x_min                         x_max → x_min
```

1. **Pre-blanking**: Hold at `x_min` before imaging starts
2. **Imaging**: Linear ramp from `x_min` to `x_max` (pixel data region)
3. **Flyback**: Smooth cosine ease back to `x_min` (zero velocity at endpoints)
4. **Settle**: Hold at `x_min` after flyback (optional)

### Frame Structure
- **Y-axis**: Steps linearly from `y_min` to `y_max` over `ny` lines
- **Continuous**: Repeats indefinitely (or until `frame_count` reached)

## Timing and Performance

### Critical Timing Loop
The scanner task uses a **busy-wait loop** for precise timing:
```cpp
while (now < next_t) {
    now = esp_timer_get_time();
}
```

**Why busy-wait?**
- `vTaskDelay()` has ~1ms granularity (too coarse for µs timing)
- Busy-wait provides microsecond precision
- Scanner task **unsubscribes from watchdog** to prevent timeout

### Performance Characteristics
- **Sample Rate**: Configurable via `sample_period_us` (default 20 µs = 50 kHz)
- **Typical Scan**: 256×256 @ 20 µs/sample ≈ 1.3 seconds/frame
- **Max Speed**: Limited by:
  - SPI transfer time (~1 µs @ 20 MHz)
  - DAC settling time
  - Galvo mechanical response

### Overrun Detection
If timing deadline is missed:
```cpp
if (now - next_t > sample_period_us) {
    overruns++;  // Logged for diagnostics
}
```

## Configuration Parameters

### ScanConfig Structure
```cpp
struct ScanConfig {
    uint16_t nx, ny;              // Grid size
    uint16_t x_min, x_max;        // X range (0-4095)
    uint16_t y_min, y_max;        // Y range (0-4095)
    uint16_t pre_samples;         // Pre-blanking samples
    uint16_t fly_samples;         // Flyback samples
    uint16_t sample_period_us;    // µs per sample
    uint16_t trig_delay_us;       // Trigger delay
    uint16_t trig_width_us;       // Trigger pulse width
    uint16_t line_settle_samples; // Post-flyback settle
    uint8_t  enable_trigger;      // Enable pixel trigger
    uint8_t  apply_x_lut;         // Apply X correction
    uint16_t frame_count;         // Frames to scan (0=∞)
};
```

### Default Parameters
```cpp
nx = 256, ny = 256
x_min = 500, x_max = 3500
y_min = 500, y_max = 3500
pre_samples = 16
fly_samples = 64
sample_period_us = 20
frame_count = 0 (continuous)
```

## X-Axis Correction LUT

### Purpose
Compensate for:
- Galvo mirror non-linearity
- Optical distortion
- System calibration offsets

### Implementation
1. **Upload**: 256 entries via UART (CMD 0x04)
2. **Expansion**: Interpolated to 4096 entries for O(1) lookup
3. **Application**: `x_corrected = x_lut[x_input]` per sample

### Usage
```
Input:  0 ────────────────→ 4095 (linear)
           ↓ LUT
Output: corrected values (non-linear)
```

## Task Architecture

### Scanner Task (Core 1, High Priority)
- **Priority**: `configMAX_PRIORITIES - 1` (highest)
- **Core**: 1 (dedicated for real-time performance)
- **Watchdog**: **Disabled** (busy-wait timing loop)
- **Responsibilities**:
  - Generate scan pattern
  - Update DAC via SPI
  - Trigger pulse generation
  - Timing control

### Protocol Task (Core 0, Medium Priority)
- **Priority**: 5 (medium)
- **Core**: 0 (shares with system tasks)
- **Watchdog**: Enabled (yielding I/O)
- **Responsibilities**:
  - Parse UART commands
  - Update scanner configuration
  - Send status responses

## Error Handling

### Protocol Errors
- `0x01`: Payload length mismatch
- `0x02`: Checksum error
- `0x03`: Invalid configuration
- `0x04`: Parameter out of range
- `0x05`: Trigger timing conflict
- `0x06/0x07`: LUT size mismatch
- `0xFF`: Unknown command

### Runtime Diagnostics
- **Overruns**: Counts timing deadline misses
- **Status Query**: Real-time frame/line position
- **Validation**: Config sanity checks before apply

## Building and Flashing

### PlatformIO
```bash
# Build
pio run -e UC2_3_Xiao

# Flash
pio run -e UC2_3_Xiao -t upload

# Monitor
pio device monitor
```

### ESP-IDF
```bash
idf.py build
idf.py flash monitor
```

## Usage Example

### Python Control Script
```python
import serial
import struct

ser = serial.Serial('/dev/ttyUSB0', 921600)

def send_command(cmd, payload=b''):
    # Build packet
    magic = b'\xA5\x5A'
    length = struct.pack('<H', len(payload))
    header = bytes([cmd]) + length
    
    # Checksum
    cks = sum(header + payload) & 0xFFFF
    cks_bytes = struct.pack('<H', cks)
    
    # Send
    ser.write(magic + header + payload + cks_bytes)

# Configure scan: 128x128, 50µs/sample
config = struct.pack('<HHHHHHHHHHHBBH',
    128, 128,           # nx, ny
    500, 3500,          # x_min, x_max
    500, 3500,          # y_min, y_max
    8,                  # pre_samples
    32,                 # fly_samples
    50,                 # sample_period_us
    3, 2,               # trig_delay_us, trig_width_us
    0,                  # line_settle_samples
    1, 0,               # enable_trigger, apply_x_lut
    0                   # frame_count (continuous)
)
send_command(0x01, config)

# Start scanning
send_command(0x02)
```

## Troubleshooting

### Watchdog Timeout
**Symptom**: `task_wdt: scanner` error
**Cause**: Scanner task not unsubscribed from watchdog
**Solution**: Verify `esp_task_wdt_delete(NULL)` in scanner task

### Timing Overruns
**Symptom**: High `overruns` count in status
**Cause**: `sample_period_us` too short for processing overhead
**Solution**: Increase `sample_period_us` or reduce `nx`

### SPI Initialization Failed
**Symptom**: Boot error "mosi not valid"
**Cause**: Incorrect GPIO pin for ESP32-S3
**Solution**: Verify pin definitions match XIAO pinout

### No Trigger Output
**Symptom**: No pulse on trigger pin
**Cause**: 
- `enable_trigger = 0`
- `trig_delay + trig_width >= sample_period_us`
**Solution**: Enable trigger and adjust timing parameters

## Performance Optimization

### Already Implemented
✅ Polling SPI (no queue overhead)
✅ Pre-computed line profile
✅ O(1) LUT lookup
✅ Core pinning
✅ High priority task
✅ Busy-wait timing

### Future Enhancements
- DMA SPI transfers (batch updates)
- Dual-buffered line profiles
- Hardware timer triggers
- Bidirectional scanning

## License

See LICENSE file in repository root.

## Authors

- Original implementation: atomic14
- Modular refactor: Copilot (2026)
