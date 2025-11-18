# Point Cloud Scanning Mode

The galvo scanner supports point cloud scanning mode, where you can upload a list of arbitrary (X, Y) coordinates and have the galvo scan through those points in sequence.

## Overview

**Point cloud mode** allows you to:
- Upload a list of up to 10,000 arbitrary (X, Y) coordinates
- Scan through the points in sequence
- Use binary transfer protocol for memory-efficient uploads
- Create custom scanning patterns not possible with raster scanning

## Binary Transfer Protocol

The binary protocol is designed to be memory-efficient and fast:

1. **Initiate Transfer** (JSON):
   ```json
   {"task":"/pointcloud_start", "numPoints":1000, "qid":1}
   ```

2. **Device Response** (JSON):
   ```json
   ++
   {"task":"/pointcloud_start","status":"ready","numPoints":1000,"qid":1}
   --
   ```

3. **Send Binary Data**:
   - Each point is 4 bytes: `[X_low, X_high, Y_low, Y_high]`
   - Coordinates are 16-bit unsigned integers (0-4095)
   - Little-endian byte order
   - Total bytes = numPoints × 4

4. **Completion Response** (JSON):
   ```json
   ++
   {"task":"/pointcloud_start","status":"complete","receivedPoints":1000}
   --
   ```

## Supported Commands

### `/pointcloud_start` - Initiate Binary Transfer

Starts binary transfer mode and prepares to receive point cloud data.

**Request:**
```json
{"task":"/pointcloud_start", "numPoints":1000, "qid":1}
```

**Parameters:**
- `numPoints` (required): Number of points to upload (1-10000)
- `qid` (optional): Query ID for tracking

**Response:**
```json
++
{"task":"/pointcloud_start","status":"ready","numPoints":1000,"qid":1}
--
```

After this response, immediately send binary data (no JSON).

### `/pointcloud_clear` - Clear Point Cloud

Clears all points from the point cloud buffer.

**Request:**
```json
{"task":"/pointcloud_clear", "qid":1}
```

**Response:**
```json
++
{"task":"/pointcloud_clear","status":"success","qid":1}
--
```

### `/pointcloud_render` - Render Point Cloud

Scans through all points in the uploaded point cloud.

**Request:**
```json
{"task":"/pointcloud_render", "qid":1}
```

**Response:**
```json
++
{"task":"/pointcloud_render","status":"success","qid":1}
--
```

## Binary Data Format

Each point consists of 4 bytes in little-endian format:

```
[X_low_byte, X_high_byte, Y_low_byte, Y_high_byte]
```

**Example:** Point at (1000, 2048)
- X = 1000 = 0x03E8 = [0xE8, 0x03]
- Y = 2048 = 0x0800 = [0x00, 0x08]
- Binary: `[0xE8, 0x03, 0x00, 0x08]`

**Python example:**
```python
import struct

x = 1000
y = 2048
binary_data = struct.pack('<HH', x, y)  # Little-endian, 2 unsigned shorts
```

## Python Uploader Script

The repository includes `point_cloud_uploader.py` for easy point cloud uploads.

### Installation

```bash
pip install pyserial
```

### Usage

**Upload a circle pattern with 100 points:**
```bash
python point_cloud_uploader.py /dev/ttyUSB0 --pattern circle --points 100 --render
```

**Upload a grid pattern:**
```bash
python point_cloud_uploader.py /dev/ttyUSB0 --pattern grid --points 100
```

**Upload a spiral pattern:**
```bash
python point_cloud_uploader.py /dev/ttyUSB0 --pattern spiral --points 500 --render
```

**Available patterns:**
- `grid`: Regular grid pattern
- `circle`: Points arranged in a circle
- `spiral`: Logarithmic spiral
- `random`: Random points

### Python API

```python
from point_cloud_uploader import PointCloudUploader

# Connect to device
uploader = PointCloudUploader('/dev/ttyUSB0')

# Define points (X, Y coordinates 0-4095)
points = [
    (1000, 1000),
    (2000, 1500),
    (3000, 2000),
    (2000, 2500),
    (1000, 3000)
]

# Upload points
uploader.clear_point_cloud()
success = uploader.upload_point_cloud(points)

if success:
    # Render the point cloud
    uploader.render_point_cloud()

uploader.close()
```

## Memory Efficiency

The binary protocol is highly memory-efficient:

**Comparison:**
- **JSON format**: `{"x":1000,"y":2048}` = ~20 bytes per point
- **Binary format**: `[0xE8, 0x03, 0x00, 0x08]` = 4 bytes per point
- **Savings**: 80% reduction in transfer size

**Example:**
- 1000 points in JSON: ~20 KB
- 1000 points in binary: 4 KB
- 10000 points in binary: 40 KB (max supported)

## Coordinate System

- **Range**: 0 - 4095 (12-bit DAC resolution)
- **Center**: (2048, 2048)
- **Offsets**: X_OFFSET and Y_OFFSET are applied to point cloud coordinates
- **Clamping**: Coordinates outside 0-4095 are automatically clamped

## Triggering

Point cloud rendering respects the trigger enable settings:

- `ENABLE_TRIG_FRAME`: Triggers at start/end of point cloud scan
- `ENABLE_TRIG_PIXEL`: Triggers at each point
- `ENABLE_TRIG_LINE`: Not used in point cloud mode

The `tPixelDwelltime` parameter controls how long to dwell at each point.

## Use Cases

### 1. Custom Patterns
Create arbitrary patterns not possible with raster scanning:
- Logos and text
- Lissajous figures
- Custom ROIs (Regions of Interest)
- Adaptive sampling patterns

### 2. Optimized Scanning
Scan only relevant points:
- Feature-based scanning
- Sparse sampling
- ROI-only scanning
- Path optimization

### 3. Image Tracing
Convert images to point clouds:
- Edge detection → points on edges
- Stippling algorithms
- Halftone patterns

### 4. Calibration
Precise calibration patterns:
- Grid calibration points
- Known reference patterns
- Distortion measurement

## Example: Manual Binary Transfer (Python)

```python
import serial
import struct
import json
import time

# Connect
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
time.sleep(0.5)

# Define points
points = [
    (1000, 1000),
    (2000, 2000),
    (3000, 3000)
]

# Step 1: Send start command
start_cmd = json.dumps({
    "task": "/pointcloud_start",
    "numPoints": len(points),
    "qid": 1
}) + '\n'
ser.write(start_cmd.encode())
ser.flush()

# Wait for ready response
while True:
    line = ser.readline().decode().strip()
    if '"status":"ready"' in line:
        break

# Step 2: Send binary data
for x, y in points:
    data = struct.pack('<HH', x, y)
    ser.write(data)
    ser.flush()
    time.sleep(0.01)

# Step 3: Wait for completion
while True:
    line = ser.readline().decode().strip()
    if '"status":"complete"' in line:
        print("Upload complete!")
        break

# Step 4: Render
render_cmd = json.dumps({
    "task": "/pointcloud_render",
    "qid": 2
}) + '\n'
ser.write(render_cmd.encode())

ser.close()
```

## Limitations

- **Maximum points**: 10,000 points per upload
- **Memory**: Limited by ESP32 RAM (~40 KB for 10K points)
- **No streaming**: Must upload complete point cloud before rendering
- **Single cloud**: Only one point cloud can be stored at a time

## Best Practices

1. **Clear before upload**: Always call `/pointcloud_clear` before uploading new data
2. **Validate coordinates**: Ensure all coordinates are in range (0-4095)
3. **Chunked transfer**: Send binary data in chunks (e.g., 256 bytes) to avoid buffer overflow
4. **Error handling**: Check completion response to verify all points received
5. **Optimize count**: Use only as many points as needed for your pattern

## Troubleshooting

**Upload fails or times out:**
- Check serial connection and baud rate
- Reduce number of points
- Add delays between chunks
- Verify binary format (little-endian)

**Missing points:**
- Check completion response for `receivedPoints`
- Increase delays between chunks
- Reduce chunk size

**Incorrect rendering:**
- Verify coordinate range (0-4095)
- Check X_OFFSET and Y_OFFSET settings
- Verify binary byte order (little-endian)
