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

Updates the galvo scanner parameters for X/Y scanning.

**Request:**
```json
{"task":"/galvo_act","qid":1,"X_MIN":0,"X_MAX":30000,"Y_MIN":0,"Y_MAX":30000,"STEP":1000,"tPixelDwelltime":1,"nFrames":1}
```

**Parameters:**
- `X_MIN` (int): Minimum X coordinate (default: 0)
- `X_MAX` (int): Maximum X coordinate (default: 6000)
- `Y_MIN` (int): Minimum Y coordinate (default: 0)
- `Y_MAX` (int): Maximum Y coordinate (default: 6000)
- `STEP` (int): Step size for scanning (default: 20)
- `tPixelDwelltime` (int): Pixel dwell time in microseconds (default: 10)
- `nFrames` (int): Number of frames to scan (default: 100)
- `qid` (int, optional): Query ID for tracking requests

**Response:**
```json
++
{"task":"/galvo_act","status":"success","qid":1}
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

# Set galvo parameters
cmd = {
    "task": "/galvo_act",
    "qid": 2,
    "X_MIN": 0,
    "X_MAX": 10000,
    "Y_MIN": 0,
    "Y_MAX": 10000,
    "STEP": 100,
    "tPixelDwelltime": 5,
    "nFrames": 10
}
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
{"task":"/galvo_act","qid":1,"X_MIN":0,"X_MAX":5000,"Y_MIN":0,"Y_MAX":5000,"STEP":50,"tPixelDwelltime":10,"nFrames":1}
```

## Notes

- All parameters are optional in the `/galvo_act` command. Omitted parameters will retain their current values.
- The device continuously scans using the current parameters between command processing.
- Parameters take effect immediately after the command is processed.
- The `qid` (query ID) parameter is optional and is echoed back in the response for request tracking.
