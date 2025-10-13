#!/usr/bin/env python3
"""
Test script for the UC2 Galvo Scanner serial JSON interface.

This script tests the serial interface by sending various commands
and validating the responses.

Usage:
    python test_serial_interface.py /dev/ttyUSB0
    python test_serial_interface.py COM3

Requirements:
    pip install pyserial
"""

import serial
import json
import time
import sys


def send_command(ser, cmd_dict):
    """Send a JSON command and read the response."""
    cmd_json = json.dumps(cmd_dict)
    print(f"\n>>> Sending: {cmd_json}")
    ser.write((cmd_json + '\n').encode())
    time.sleep(0.2)  # Give device time to respond
    
    # Read all available response lines
    response_lines = []
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        if line:
            response_lines.append(line)
    
    response = '\n'.join(response_lines)
    print(f"<<< Received: {response}")
    return response


def test_state_get(ser):
    """Test the /state_get command."""
    print("\n=== Testing /state_get command ===")
    cmd = {"task": "/state_get", "qid": 1}
    response = send_command(ser, cmd)
    
    # Validate response contains expected fields
    assert "identifier_name" in response, "Missing identifier_name in response"
    assert "UC2_GalvoScanner" in response, "Unexpected identifier_name"
    assert "success" in response, "Missing success field"
    assert "qid" in response, "Missing qid in response"
    print("✓ /state_get test passed")


def test_galvo_act_full(ser):
    """Test the /galvo_act command with all parameters."""
    print("\n=== Testing /galvo_act with all parameters ===")
    cmd = {
        "task": "/galvo_act",
        "qid": 2,
        "X_MIN": 0,
        "X_MAX": 5000,
        "Y_MIN": 0,
        "Y_MAX": 5000,
        "STEP": 100,
        "tPixelDwelltime": 5,
        "nFrames": 1
    }
    response = send_command(ser, cmd)
    
    # Validate response
    assert "success" in response, "Missing success status"
    assert "qid" in response and "2" in response, "QID not echoed correctly"
    print("✓ /galvo_act (full parameters) test passed")


def test_galvo_act_partial(ser):
    """Test the /galvo_act command with partial parameters."""
    print("\n=== Testing /galvo_act with partial parameters ===")
    cmd = {
        "task": "/galvo_act",
        "qid": 3,
        "X_MAX": 10000,
        "STEP": 50
    }
    response = send_command(ser, cmd)
    
    # Validate response
    assert "success" in response, "Missing success status"
    assert "qid" in response and "3" in response, "QID not echoed correctly"
    print("✓ /galvo_act (partial parameters) test passed")


def test_invalid_json(ser):
    """Test handling of invalid JSON."""
    print("\n=== Testing invalid JSON ===")
    ser.write(b'{"invalid json\n')
    time.sleep(0.2)
    
    response_lines = []
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        if line:
            response_lines.append(line)
    
    response = '\n'.join(response_lines)
    print(f"<<< Received: {response}")
    assert "error" in response.lower(), "Expected error response for invalid JSON"
    print("✓ Invalid JSON handling test passed")


def test_missing_task(ser):
    """Test handling of missing task field."""
    print("\n=== Testing missing task field ===")
    cmd = {"qid": 4, "X_MIN": 0}
    response = send_command(ser, cmd)
    
    assert "error" in response.lower(), "Expected error response for missing task"
    assert "missing task" in response.lower(), "Expected 'missing task' error message"
    print("✓ Missing task handling test passed")


def test_unknown_task(ser):
    """Test handling of unknown task."""
    print("\n=== Testing unknown task ===")
    cmd = {"task": "/unknown_command", "qid": 5}
    response = send_command(ser, cmd)
    
    assert "error" in response.lower(), "Expected error response for unknown task"
    assert "unknown task" in response.lower(), "Expected 'unknown task' error message"
    print("✓ Unknown task handling test passed")


def main():
    if len(sys.argv) < 2:
        print("Usage: python test_serial_interface.py <serial_port>")
        print("Example: python test_serial_interface.py /dev/ttyUSB0")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print(f"Connecting to {port} at 115200 baud...")
    try:
        ser = serial.Serial(port, 115200, timeout=2)
        time.sleep(2)  # Wait for device to initialize
        print("✓ Connected successfully")
    except serial.SerialException as e:
        print(f"✗ Failed to connect: {e}")
        sys.exit(1)
    
    try:
        # Run all tests
        test_state_get(ser)
        test_galvo_act_full(ser)
        test_galvo_act_partial(ser)
        test_invalid_json(ser)
        test_missing_task(ser)
        test_unknown_task(ser)
        
        print("\n" + "="*50)
        print("✓ All tests passed successfully!")
        print("="*50)
        
    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        sys.exit(1)
    finally:
        ser.close()
        print("\nSerial port closed")


if __name__ == "__main__":
    main()
