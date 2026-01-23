#!/usr/bin/env python3
"""
ESP32-S3 Galvo Scanner Serial Interface
Binary protocol for controlling the galvo scanner via UART
"""

import serial
import struct
import time
from dataclasses import dataclass
from typing import Optional, Tuple
from enum import IntEnum


class Command(IntEnum):
    """Protocol command codes"""
    SET_CONFIG = 0x01
    START = 0x02
    STOP = 0x03
    SET_X_LUT = 0x04
    GET_STATUS = 0x05


class ErrorCode(IntEnum):
    """Protocol error codes"""
    OK = 0x00
    LENGTH = 0x01
    CHECKSUM = 0x02
    INVALID_CONFIG = 0x03
    PARAM_RANGE = 0x04
    TRIGGER_TIMING = 0x05
    LUT_LENGTH = 0x06
    LUT_SIZE = 0x07
    UNKNOWN_CMD = 0xFF


@dataclass
class ScanConfig:
    """Scan configuration structure"""
    nx: int = 256               # Number of X samples
    ny: int = 256               # Number of Y lines
    x_min: int = 500            # X minimum (0-4095)
    x_max: int = 3500           # X maximum (0-4095)
    y_min: int = 500            # Y minimum (0-4095)
    y_max: int = 3500           # Y maximum (0-4095)
    pre_samples: int = 16       # Pre-blanking samples
    fly_samples: int = 64       # Flyback samples
    sample_period_us: int = 20  # Microseconds per sample
    trig_delay_us: int = 3      # Trigger delay
    trig_width_us: int = 2      # Trigger pulse width
    line_settle_samples: int = 0  # Post-flyback settle
    enable_trigger: bool = True   # Enable pixel trigger
    apply_x_lut: bool = False     # Apply X correction LUT
    frame_count: int = 0          # Number of frames (0=continuous)

    def pack(self) -> bytes:
        """Pack configuration into binary format"""
        return struct.pack(
            '<HHHHHHHHHHHBBH',
            self.nx, self.ny,
            self.x_min, self.x_max,
            self.y_min, self.y_max,
            self.pre_samples,
            self.fly_samples,
            self.sample_period_us,
            self.trig_delay_us,
            self.trig_width_us,
            self.line_settle_samples,
            1 if self.enable_trigger else 0,
            1 if self.apply_x_lut else 0,
            self.frame_count
        )


@dataclass
class ScannerStatus:
    """Scanner status information"""
    running: bool
    line: int
    frame: int
    overruns: int


class GalvoScanner:
    """Serial interface to ESP32-S3 Galvo Scanner"""
    
    MAGIC = b'\xA5\x5A'
    TIMEOUT = 2.0  # seconds
    
    def __init__(self, port: str, baudrate: int = 921600):
        """
        Initialize scanner connection
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Baud rate (default 921600)
        """
        self.ser = serial.Serial(port, baudrate, timeout=self.TIMEOUT)
        time.sleep(0.1)  # Wait for ESP32 to be ready
        
    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
    
    def _checksum(self, data: bytes) -> int:
        """Calculate 16-bit checksum"""
        return sum(data) & 0xFFFF
    
    def _send_command(self, cmd: Command, payload: bytes = b'') -> bytes:
        """
        Send command and receive reply
        
        Args:
            cmd: Command code
            payload: Command payload
            
        Returns:
            Reply payload (without error code byte)
            
        Raises:
            RuntimeError: On protocol or command error
        """
        # Build packet
        length = len(payload)
        header = bytes([cmd]) + struct.pack('<H', length)
        
        # Calculate checksum
        cks = self._checksum(header + payload)
        cks_bytes = struct.pack('<H', cks)
        
        # Send packet
        packet = self.MAGIC + header + payload + cks_bytes
        self.ser.write(packet)
        self.ser.flush()
        
        # Read reply
        # Format: MAGIC0 MAGIC1 (CMD|0x80) LEN_LOW LEN_HIGH [PAYLOAD...] CKS_LOW CKS_HIGH
        
        # Read magic bytes
        magic = self.ser.read(2)
        if magic != self.MAGIC:
            raise RuntimeError(f"Invalid magic bytes: {magic.hex()}")
        
        # Read command (should have bit 7 set)
        reply_cmd = self.ser.read(1)[0]
        if reply_cmd != (cmd | 0x80):
            raise RuntimeError(f"Command mismatch: expected {cmd | 0x80:02x}, got {reply_cmd:02x}")
        
        # Read length
        len_bytes = self.ser.read(2)
        reply_len = struct.unpack('<H', len_bytes)[0]
        
        # Read payload
        reply_payload = b''
        if reply_len > 0:
            reply_payload = self.ser.read(reply_len)
            if len(reply_payload) != reply_len:
                raise RuntimeError(f"Incomplete payload: expected {reply_len}, got {len(reply_payload)}")
        
        # Read checksum
        reply_cks_bytes = self.ser.read(2)
        reply_cks = struct.unpack('<H', reply_cks_bytes)[0]
        
        # Verify checksum
        calc_cks = self._checksum(bytes([reply_cmd]) + len_bytes + reply_payload)
        if calc_cks != reply_cks:
            raise RuntimeError(f"Checksum mismatch: expected {reply_cks:04x}, got {calc_cks:04x}")
        
        # Check error code (first byte of payload)
        if reply_len > 0:
            error_code = reply_payload[0]
            if error_code != ErrorCode.OK:
                error_name = ErrorCode(error_code).name if error_code in ErrorCode._value2member_map_ else f"0x{error_code:02x}"
                raise RuntimeError(f"Command failed with error: {error_name}")
            return reply_payload[1:]  # Return payload without error code
        
        return b''
    
    def set_config(self, config: ScanConfig):
        """
        Set scan configuration
        
        Args:
            config: Scan configuration
        """
        payload = config.pack()
        self._send_command(Command.SET_CONFIG, payload)
    
    def start(self):
        """Start scanning"""
        self._send_command(Command.START)
    
    def stop(self):
        """Stop scanning"""
        self._send_command(Command.STOP)
    
    def set_x_lut(self, lut: list[int]):
        """
        Upload X-axis correction lookup table
        
        Args:
            lut: List of 256 values (0-4095)
            
        Raises:
            ValueError: If LUT size is not 256
        """
        if len(lut) != 256:
            raise ValueError(f"LUT must have 256 entries, got {len(lut)}")
        
        # Pack: n_entries (uint16) + 256 x uint16 values
        payload = struct.pack('<H', 256)
        for val in lut:
            payload += struct.pack('<H', val & 0x0FFF)  # Ensure 12-bit
        
        self._send_command(Command.SET_X_LUT, payload)
    
    def get_status(self) -> ScannerStatus:
        """
        Get current scanner status
        
        Returns:
            Scanner status
        """
        reply = self._send_command(Command.GET_STATUS)
        
        # Unpack: uint8 running, uint16 line, uint32 frame, int32 overruns
        running, line, frame, overruns = struct.unpack('<BHIi', reply)
        
        return ScannerStatus(
            running=bool(running),
            line=line,
            frame=frame,
            overruns=overruns
        )


# Example usage and test functions
def example_basic_scan():
    """Example: Basic raster scan"""
    with GalvoScanner('/dev/ttyUSB0') as scanner:
        # Configure 128x128 scan
        config = ScanConfig(
            nx=128,
            ny=128,
            x_min=500,
            x_max=3500,
            y_min=500,
            y_max=3500,
            sample_period_us=50,
            enable_trigger=True
        )
        scanner.set_config(config)
        
        # Start scanning
        scanner.start()
        print("Scanning started...")
        
        # Monitor status
        time.sleep(2)
        status = scanner.get_status()
        print(f"Status: Running={status.running}, Frame={status.frame}, Line={status.line}, Overruns={status.overruns}")
        
        # Stop after 5 seconds
        time.sleep(3)
        scanner.stop()
        print("Scanning stopped")


def example_x_correction():
    """Example: Apply X-axis correction"""
    with GalvoScanner('/dev/ttyUSB0') as scanner:
        # Generate simple barrel distortion correction LUT
        # (this is just an example - calibrate for your system)
        lut = []
        for i in range(256):
            x_norm = i / 255.0  # 0 to 1
            x_center = x_norm - 0.5  # -0.5 to 0.5
            
            # Barrel distortion correction (quadratic)
            correction = 1.0 + 0.1 * (x_center ** 2)
            x_corrected = 0.5 + x_center * correction
            
            # Map back to 12-bit range
            dac_value = int(x_corrected * 4095)
            lut.append(max(0, min(4095, dac_value)))
        
        # Upload LUT
        scanner.set_x_lut(lut)
        print("X correction LUT uploaded")
        
        # Configure to use LUT
        config = ScanConfig(apply_x_lut=True)
        scanner.set_config(config)
        scanner.start()


def example_high_speed_scan():
    """Example: High-speed scanning"""
    with GalvoScanner('/dev/ttyUSB0') as scanner:
        config = ScanConfig(
            nx=512,
            ny=512,
            sample_period_us=10,  # 100 kHz sample rate
            pre_samples=8,        # Minimal blanking
            fly_samples=32,       # Fast flyback
            enable_trigger=True
        )
        scanner.set_config(config)
        scanner.start()
        
        print("High-speed scan started (512x512 @ 10µs/sample)")
        
        # Monitor for overruns
        for _ in range(10):
            time.sleep(1)
            status = scanner.get_status()
            if status.overruns > 0:
                print(f"Warning: {status.overruns} timing overruns detected!")
            print(f"Frame {status.frame}, Line {status.line}/{config.ny}")


def interactive_console():
    """Interactive console for manual control"""
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python galvo_serial.py <port>")
        print("Example: python galvo_serial.py /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    with GalvoScanner(port) as scanner:
        print(f"Connected to {port}")
        print("\nCommands:")
        print("  start  - Start scanning")
        print("  stop   - Stop scanning")
        print("  status - Get status")
        print("  config <nx> <ny> <period_us> - Set config")
        print("  quit   - Exit")
        
        while True:
            try:
                cmd = input("\n> ").strip().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'quit':
                    break
                elif cmd[0] == 'start':
                    scanner.start()
                    print("Started")
                elif cmd[0] == 'stop':
                    scanner.stop()
                    print("Stopped")
                elif cmd[0] == 'status':
                    status = scanner.get_status()
                    print(f"Running: {status.running}")
                    print(f"Frame: {status.frame}")
                    print(f"Line: {status.line}")
                    print(f"Overruns: {status.overruns}")
                elif cmd[0] == 'config' and len(cmd) == 4:
                    config = ScanConfig(
                        nx=int(cmd[1]),
                        ny=int(cmd[2]),
                        sample_period_us=int(cmd[3])
                    )
                    scanner.set_config(config)
                    print(f"Config set: {cmd[1]}x{cmd[2]} @ {cmd[3]}µs")
                else:
                    print("Unknown command")
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")


if __name__ == '__main__':
    # Run interactive console if called as script
    interactive_console()
