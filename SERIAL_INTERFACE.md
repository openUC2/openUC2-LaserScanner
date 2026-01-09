# Serial JSON Interface Documentation

This document describes the serial interface for controlling the galvo scanner parameters.

## Setup

- **Baud Rate**: 115200
- **Format**: JSON commands terminated by newline (`\n`)
- **Response Format**: JSON with `++` prefix and `--` suffix

## Supported Commands

### 1. Get Device State - `/state_get`

Retrieves device identification and status information.

**Request:**
```json
{"task":"/state_get","qid":1}
```

**Response:**
```json
++
{
  "identifier_name":"UC2_GalvoScanner",
  "identifier_id":"V1.0",
  "identifier_date":"Jan 01 2024 12:00:00",
  "identifier_author":"UC2",
  "IDENTIFIER_NAME":"uc2-esp",
  "configIsSet":0,
  "pindef":"UC2",
  "success":1,
  "qid":1
}
--
```

### 2. Set Galvo Parameters - `/galvo_act`

Updates the galvo scanner parameters for X/Y scanning and saves them to persistent storage.

**BEWARE: Y is pixelclock**
**Request:**
```json
{"task":"/galvo_act","qid":1,"X_MIN":0,"X_MAX":0,"Y_MIN":0,"Y_MAX":4094,"STEP_X":1,"STEP_Y":10,"tPixelDwelltime":0,"nFrames":1,"SNAKE":false}

{"task":"/galvo_act","qid":1,"X_MIN":0,"X_MAX":512,"Y_MIN":0,"Y_MAX":512,"STEP_X":2,"STEP_Y":2,"tPixelDwelltime":10,"nFrames":1,"SNAKE":false}

{"task":"/galvo_act","X_MIN":0,"X_MAX":2048,"Y_MIN":0,"Y_MAX":2048,"X_OFFSET":0,"Y_OFFSET":0,"STEP_X":10,"STEP_Y":10,"tPixelDwelltime":0,"nFrames":10,"SNAKE":false,"SIM":false,"SINGLE":false,"X_POS":2048,"Y_POS":2048,"ENABLE_TRIG_FRAME":true,"ENABLE_TRIG_LINE":true,"ENABLE_TRIG_PIXEL":true,"success":1,"qid":1}

{"task":"/galvo_act","qid":1,"X_MIN":0,"X_MAX":1024,"Y_MIN":0,"Y_MAX":1024,"STEP_X":4,"STEP_Y":4,"tPixelDwelltime":0,"nFrames":10,"SNAKE":false, "SIM":false, "SINGLE":false}

{"task":"/galvo_act","qid":1,"X_MIN":0,"X_MAX":1024,"Y_MIN":0,"Y_MAX":1024,"STEP_X":4,"STEP_Y":4,"tPixelDwelltime":0,"nFrames":10,"SNAKE":false, "SIM":false, "SINGLE":false}

{"task":"/galvo_act","qid":1,"SINGLE":true,"X_POS":2048,"Y_POS":2048}

{"task":"/galvo_act","qid":1,"LIGHTSHEET":true,"LS_AMPLITUDE":1000,"LS_FREQUENCY":100.0,"LS_OFFSET":1000,"LS_DELAY":0}

```

**Parameters:**
- `X_MIN` (int): Minimum X coordinate (default: 0)
- `X_MAX` (int): Maximum X coordinate (default: 6000)
- `Y_MIN` (int): Minimum Y coordinate (default: 0)
- `Y_MAX` (int): Maximum Y coordinate (default: 6000)
- `X_OFFSET` (int): X-axis offset applied to all positions (default: 0)
- `Y_OFFSET` (int): Y-axis offset applied to all positions (default: 0)
- `STEP` (int): Step size for scanning (default: 20) - backward compatibility
- `STEP_X` (int): Step size for X-axis scanning (default: 20)
- `STEP_Y` (int): Step size for Y-axis scanning (default: 20)
- `tPixelDwelltime` (int): Pixel dwell time in microseconds (default: 10)
- `nFrames` (int): Number of frames to scan (default: 100)
- `SNAKE` (bool): Enable snake scanning pattern - even lines scan left-to-right, odd lines scan right-to-left (default: false)
- `SIM` (bool): Enable structured illumination mode - shifts pattern by STEP_Y/nFrames per frame (default: false)
- `SINGLE` (bool): Enable single point positioning mode - galvos remain at fixed position (default: false)
- `X_POS` (int): X position for single point mode (0-4095, default: 2048)
- `Y_POS` (int): Y position for single point mode (0-4095, default: 2048)
- `LIGHTSHEET` (bool): Enable light-sheet mode - sinusoidal Y-axis scanning only (default: false)
- `LS_AMPLITUDE` (int): Amplitude of sinusoidal pattern for light-sheet mode (0-4095, default: 2048)
- `LS_FREQUENCY` (float): Frequency of sinusoidal pattern in Hz (default: 1.0)
- `LS_OFFSET` (int): Y-axis offset (center position) for light-sheet mode (0-4095, default: 2048)
- `LS_DELAY` (int): Delay between points in microseconds for light-sheet mode (default: 1000)
- `ENABLE_TRIG_FRAME` (bool): Enable frame trigger signal (default: true)
- `ENABLE_TRIG_LINE` (bool): Enable line trigger signal (default: true)
- `ENABLE_TRIG_PIXEL` (bool): Enable pixel trigger signal (default: true)
- `qid` (int, optional): Query ID for tracking requests

**Response:**
```json
++
{"task":"/galvo_act","status":"success","qid":1}
--
```

**Note:** Parameters are automatically saved to non-volatile storage and will be restored on device reboot.

### 3. Get Galvo Parameters - `/galvo_get`

Retrieves the current galvo scanner parameters.

**Request:**
```json
{"task":"/galvo_get","qid":1}
```

**Response:**
```json
++
{
  "task":"/galvo_get",
  "X_MIN":0,
  "X_MAX":30000,
  "Y_MIN":0,
  "Y_MAX":30000,
  "STEP":1000,
  "tPixelDwelltime":1,
  "nFrames":1,
  "SNAKE":true,
  "success":1,
  "qid":1
}
--
```

## Error Responses

**JSON Parse Error:**
```json
{"status":"error","info":"JSON parse failed"}
```

**Missing Task:**
```json
{"status":"error","info":"Missing task"}
```

**Unknown Task:**
```json
{"status":"error","info":"Unknown task"}
```

## Scanning Patterns

### Normal Raster Scanning Pattern

The default scanning pattern uses a raster scan where the galvo mirrors scan the field of view in a grid pattern.

### Snake Scanning Pattern

The `SNAKE` parameter enables an optimized scanning pattern that reduces the time needed to reposition the galvo mirrors between lines:

**Normal Scanning (SNAKE=false):**
```
Line 0: Y_MIN → Y_MAX (left to right)
Line 1: Y_MIN → Y_MAX (left to right)
Line 2: Y_MIN → Y_MAX (left to right)
...
```
After each line, the scanner must return from Y_MAX back to Y_MIN before starting the next line.

**Snake Scanning (SNAKE=true):**
```
Line 0 (even): Y_MIN → Y_MAX (left to right)
Line 1 (odd):  Y_MAX → Y_MIN (right to left)
Line 2 (even): Y_MIN → Y_MAX (left to right)
Line 3 (odd):  Y_MAX → Y_MIN (right to left)
...
```
The scanner alternates direction, eliminating the need to return to the start position between lines, which can significantly improve scanning speed and reduce mechanical wear.

**Benefits of Snake Scanning:**
- Faster scanning (no flyback time between lines)
- Reduced mechanical stress on galvo mirrors
- More continuous motion
- Better for high-speed applications

**Usage Example:**
```json
{"task":"/galvo_act","qid":1,"X_MIN":0,"X_MAX":10000,"Y_MIN":0,"Y_MAX":10000,"STEP":100,"SNAKE":true}
```

### Single Point Positioning Mode

The `SINGLE` mode allows you to position the galvos at a fixed, stationary position. This is useful for calibration, alignment, or focusing the laser at a specific point.

**Parameters:**
- `SINGLE` (bool): Enable single point mode
- `X_POS` (int): X position (0-4095, where 2048 is center)
- `Y_POS` (int): Y position (0-4095, where 2048 is center)

**Usage Example:**
```json
{"task":"/galvo_act","qid":1,"SINGLE":true,"X_POS":2048,"Y_POS":2048}
```

To exit single point mode and return to scanning, send a command with `SINGLE` set to false or send a scanning configuration without the `SINGLE` parameter.

### Light-Sheet Mode

The `LIGHTSHEET` mode enables a specialized scanning pattern for light-sheet microscopy or similar applications. In this mode:

- The X-axis remains fixed at center position (2048)
- The Y-axis scans continuously through a pre-computed sinusoidal pattern
- The number of sine points is determined by (Y_MAX - Y_MIN) / STEP_Y
- The delay between points controls the scanning speed (and effectively the frequency)
- No trigger signals are generated in this mode

**Parameters:**
- `LIGHTSHEET` (bool): Enable light-sheet mode
- `Y_MIN` (int): Minimum Y value for sine calculation (default: 0)
- `Y_MAX` (int): Maximum Y value for sine calculation (default: 4095)
- `STEP_Y` (int): Step size that determines number of points in sine table (default: 20)
- `LS_AMPLITUDE` (int): Amplitude of the sine wave (0-4095, defines scan range)
- `LS_FREQUENCY` (float): Frequency in Hz (max 100 Hz, for reference/documentation)
- `LS_OFFSET` (int): Center position on Y-axis (0-4095, default: 2048)
- `LS_DELAY` (int): Delay between points in microseconds (controls actual scanning speed)

**How it works:**
1. A sine table is pre-computed with (Y_MAX - Y_MIN) / STEP_Y points
2. Each point represents one position in a complete sine cycle
3. The scanner iterates through the table with LS_DELAY between each point
4. Actual frequency = 1 / (table_size × LS_DELAY × 10^-6) Hz

**Usage Example:**
```json
{"task":"/galvo_act","qid":1,"LIGHTSHEET":true,"Y_MIN":0,"Y_MAX":4095,"STEP_Y":20,"LS_AMPLITUDE":2000,"LS_FREQUENCY":1.0,"LS_OFFSET":2000,"LS_DELAY":100}
```

**Benefits of Light-Sheet Mode:**
- Pre-computed sine table eliminates time-aliasing issues
- Predictable, deterministic scanning pattern
- Number of points controlled by Y range and step size
- Delay parameter allows precise control of scanning speed
- No triggering overhead for faster operation
- Maximum frequency limited to 100 Hz to prevent timing issues

To exit light-sheet mode and return to normal scanning, send a command with `LIGHTSHEET` set to false or send a scanning configuration without the `LIGHTSHEET` parameter.

## Usage Examples

### Using Python

```python
import serial
import json
import time

# Open serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for device to initialize

# Get device state
cmd = {"task": "/state_get", "qid": 1}
ser.write((json.dumps(cmd) + '\n').encode())
response = ser.readline().decode()
print(response)

# Set galvo parameters with snake scanning
cmd = {
    "task": "/galvo_act",
    "qid": 2,
    "X_MIN": 0,
    "X_MAX": 10000,
    "Y_MIN": 0,
    "Y_MAX": 10000,
    "STEP": 100,
    "tPixelDwelltime": 5,
    "nFrames": 10,
    "SNAKE": True
}
ser.write((json.dumps(cmd) + '\n').encode())
response = ser.readline().decode()
print(response)

# Get current galvo parameters
cmd = {"task": "/galvo_get", "qid": 3}
ser.write((json.dumps(cmd) + '\n').encode())
response = ser.readline().decode()
print(response)

ser.close()
```

### Using Arduino Serial Monitor

1. Open the Serial Monitor at 115200 baud
2. Set line ending to "Newline" or "Both NL & CR"
3. Type the JSON command and press Enter:
   ```
   {"task":"/state_get","qid":1}
   ```

### Using screen/minicom

```bash
# Using screen
screen /dev/ttyUSB0 115200

# Type JSON commands followed by Enter
{"task":"/state_get","qid":1}
{"task":"/galvo_get","qid":2}
{"task":"/galvo_act","qid":3,"X_MIN":0,"X_MAX":5000,"Y_MIN":0,"Y_MAX":5000,"STEP":50,"tPixelDwelltime":10,"nFrames":1,"SNAKE":true}
```

## Notes

- All parameters are optional in the `/galvo_act` command. Omitted parameters will retain their current values.
- The device continuously scans using the current parameters between command processing.
- Parameters take effect immediately after the command is processed.
- The `qid` (query ID) parameter is optional and is echoed back in the response for request tracking.
- **Persistent Storage**: Parameters set via `/galvo_act` are automatically saved to non-volatile storage (NVS) and will be restored on device reboot.
- Use `/galvo_get` to query the current parameters at any time.
