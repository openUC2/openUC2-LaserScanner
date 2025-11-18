#!/usr/bin/env python3
"""
Point Cloud Uploader for UC2 Galvo Scanner
Sends point cloud data to ESP32 via binary protocol for memory-efficient transfer
"""

import serial
import struct
import json
import time
import argparse
from typing import List, Tuple

class PointCloudUploader:
    def __init__(self, port: str, baudrate: int = 115200):
        """
        Initialize the point cloud uploader
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Serial baud rate (default: 115200)
        """
        self.ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(0.5)  # Wait for serial connection to stabilize
        
    def send_json_command(self, command: dict) -> dict:
        """Send a JSON command and wait for response"""
        json_str = json.dumps(command) + '\n'
        self.ser.write(json_str.encode('utf-8'))
        self.ser.flush()
        
        # Read response (between ++ and --)
        response_lines = []
        in_response = False
        
        timeout = time.time() + 5  # 5 second timeout
        while time.time() < timeout:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line == '++':
                    in_response = True
                elif line == '--':
                    break
                elif in_response:
                    response_lines.append(line)
        
        if response_lines:
            try:
                return json.loads('\n'.join(response_lines))
            except json.JSONDecodeError:
                return {'status': 'error', 'info': 'Invalid JSON response'}
        return {'status': 'error', 'info': 'No response received'}
    
    def upload_point_cloud(self, points: List[Tuple[int, int]], qid: int = 1) -> bool:
        """
        Upload a point cloud to the ESP32
        
        Args:
            points: List of (x, y) tuples, each coordinate 0-4095
            qid: Query ID for tracking
            
        Returns:
            True if successful, False otherwise
        """
        num_points = len(points)
        
        if num_points == 0:
            print("Error: Empty point cloud")
            return False
        
        if num_points > 10000:
            print(f"Error: Too many points ({num_points}), maximum is 10000")
            return False
        
        # Validate coordinates
        for i, (x, y) in enumerate(points):
            if x < 0 or x > 4095 or y < 0 or y > 4095:
                print(f"Error: Point {i} ({x}, {y}) out of range (0-4095)")
                return False
        
        print(f"Uploading {num_points} points...")
        
        # Step 1: Send start command
        start_cmd = {
            "task": "/pointcloud_start",
            "numPoints": num_points,
            "qid": qid
        }
        
        response = self.send_json_command(start_cmd)
        print(f"Start response: {response}")
        
        if response.get('status') != 'ready':
            print("Error: Device not ready for binary transfer")
            return False
        
        # Step 2: Send binary data
        print("Sending binary data...")
        binary_data = bytearray()
        
        for x, y in points:
            # Pack as 16-bit little-endian values
            binary_data.extend(struct.pack('<HH', x, y))
        
        # Send in chunks to avoid buffer overflow
        chunk_size = 256
        bytes_sent = 0
        
        for i in range(0, len(binary_data), chunk_size):
            chunk = binary_data[i:i+chunk_size]
            self.ser.write(chunk)
            self.ser.flush()
            bytes_sent += len(chunk)
            
            # Progress indicator
            progress = (bytes_sent / len(binary_data)) * 100
            print(f"Progress: {progress:.1f}% ({bytes_sent}/{len(binary_data)} bytes)", end='\r')
            
            time.sleep(0.01)  # Small delay between chunks
        
        print()  # New line after progress
        
        # Step 3: Wait for completion response
        print("Waiting for completion...")
        timeout = time.time() + 10
        while time.time() < timeout:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line == '++':
                    # Read completion message
                    response_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    self.ser.readline()  # Read '--'
                    try:
                        completion = json.loads(response_line)
                        print(f"Completion response: {completion}")
                        
                        if completion.get('status') == 'complete':
                            received = completion.get('receivedPoints', 0)
                            if received == num_points:
                                print(f"✓ Successfully uploaded {num_points} points")
                                return True
                            else:
                                print(f"⚠ Warning: Sent {num_points} but received {received}")
                                return False
                    except json.JSONDecodeError:
                        pass
        
        print("Error: Timeout waiting for completion")
        return False
    
    def clear_point_cloud(self, qid: int = 1) -> bool:
        """Clear the point cloud on the device"""
        response = self.send_json_command({"task": "/pointcloud_clear", "qid": qid})
        print(f"Clear response: {response}")
        return response.get('status') == 'success'
    
    def render_point_cloud(self, qid: int = 1) -> bool:
        """Render the uploaded point cloud"""
        print("Rendering point cloud...")
        response = self.send_json_command({"task": "/pointcloud_render", "qid": qid})
        print(f"Render response: {response}")
        return response.get('status') == 'success'
    
    def close(self):
        """Close the serial connection"""
        self.ser.close()


def generate_test_pattern(pattern: str = 'grid', size: int = 100) -> List[Tuple[int, int]]:
    """
    Generate test patterns
    
    Args:
        pattern: Pattern type ('grid', 'circle', 'spiral', 'random')
        size: Number of points to generate
        
    Returns:
        List of (x, y) coordinate tuples
    """
    import math
    import random
    
    points = []
    
    if pattern == 'grid':
        # Generate a grid pattern
        side = int(math.sqrt(size))
        for i in range(side):
            for j in range(side):
                x = int((i / side) * 4095)
                y = int((j / side) * 4095)
                points.append((x, y))
                
    elif pattern == 'circle':
        # Generate points in a circle
        for i in range(size):
            angle = (i / size) * 2 * math.pi
            radius = 1500
            x = int(2048 + radius * math.cos(angle))
            y = int(2048 + radius * math.sin(angle))
            points.append((x, y))
            
    elif pattern == 'spiral':
        # Generate a spiral pattern
        for i in range(size):
            t = i / size * 4 * math.pi
            radius = (i / size) * 1500
            x = int(2048 + radius * math.cos(t))
            y = int(2048 + radius * math.sin(t))
            points.append((x, y))
            
    elif pattern == 'random':
        # Generate random points
        for _ in range(size):
            x = random.randint(500, 3595)  # Leave margin
            y = random.randint(500, 3595)
            points.append((x, y))
    
    return points


def main():
    parser = argparse.ArgumentParser(description='Upload point cloud to UC2 Galvo Scanner')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--pattern', choices=['grid', 'circle', 'spiral', 'random'], 
                        default='circle', help='Test pattern to generate')
    parser.add_argument('--points', type=int, default=100, 
                        help='Number of points to generate (1-10000)')
    parser.add_argument('--render', action='store_true', 
                        help='Render the point cloud after upload')
    parser.add_argument('--baudrate', type=int, default=115200, 
                        help='Serial baud rate')
    
    args = parser.parse_args()
    
    # Validate number of points
    if args.points < 1 or args.points > 10000:
        print("Error: Number of points must be between 1 and 10000")
        return
    
    # Generate test pattern
    print(f"Generating {args.pattern} pattern with {args.points} points...")
    points = generate_test_pattern(args.pattern, args.points)
    
    # Upload to device
    try:
        uploader = PointCloudUploader(args.port, args.baudrate)
        
        # Clear existing point cloud
        uploader.clear_point_cloud()
        
        # Upload new point cloud
        success = uploader.upload_point_cloud(points)
        
        if success and args.render:
            # Render the point cloud
            uploader.render_point_cloud()
        
        uploader.close()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user")


if __name__ == '__main__':
    main()
