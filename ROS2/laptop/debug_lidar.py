#!/usr/bin/env python3

import serial
import time
import struct
import sys


def test_rplidar_direct(port='/dev/ttyUSB0', baudrate=115200):
    """
    Direct serial communication test with RPLiDAR
    """
    print(f"Testing RPLiDAR on {port} at {baudrate} baud...")

    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"✓ Serial port {port} opened successfully")

        # Wait for device to stabilize
        time.sleep(0.5)

        # Clear any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("✓ Serial buffers cleared")

        # Test 1: Send STOP command (should always work)
        print("\n--- Test 1: STOP Command ---")
        stop_cmd = bytes([0xA5, 0x25])
        ser.write(stop_cmd)
        ser.flush()
        print("✓ STOP command sent")
        time.sleep(0.5)

        # Clear buffer after stop
        ser.reset_input_buffer()

        # Test 2: Send RESET command
        print("\n--- Test 2: RESET Command ---")
        reset_cmd = bytes([0xA5, 0x40])
        ser.write(reset_cmd)
        ser.flush()
        print("✓ RESET command sent")
        time.sleep(2)  # Wait for reset to complete

        # Clear buffer after reset
        ser.reset_input_buffer()

        # Test 3: Get Device Info
        print("\n--- Test 3: GET_INFO Command ---")
        info_cmd = bytes([0xA5, 0x50])
        ser.write(info_cmd)
        ser.flush()
        print("✓ GET_INFO command sent")

        # Wait for response with longer timeout
        print("Waiting for response header...")
        header_data = ser.read(7)

        if len(header_data) == 7:
            print(f"✓ Received {len(header_data)} header bytes: {[hex(b) for b in header_data]}")

            # Parse header
            if header_data[0] == 0xA5 and header_data[1] == 0x5A:
                print("✓ Valid response header sync bytes")

                response_length = (header_data[2] & 0x3F) | \
                                  (header_data[3] << 6) | \
                                  (header_data[4] << 14) | \
                                  (header_data[5] << 22)
                response_type = header_data[6]

                print(f"Response type: 0x{response_type:02X}, length: {response_length}")

                if response_type == 0x04:  # DEVINFO response
                    print("✓ Correct response type for device info")

                    # Read device info data
                    info_data = ser.read(20)
                    if len(info_data) == 20:
                        print(f"✓ Received {len(info_data)} info bytes")

                        model = info_data[0]
                        firmware_minor = info_data[1]
                        firmware_major = info_data[2]
                        hardware = info_data[3]
                        serial_num = info_data[4:20]

                        print(f"✓ Model: {model}")
                        print(f"✓ Firmware: {firmware_major}.{firmware_minor}")
                        print(f"✓ Hardware: {hardware}")
                        print(f"✓ Serial: {serial_num.hex()}")

                        print("\n🎉 RPLiDAR communication test PASSED!")
                        return True
                    else:
                        print(f"✗ Expected 20 info bytes, got {len(info_data)}")
                else:
                    print(f"✗ Wrong response type: 0x{response_type:02X}, expected 0x04")
            else:
                print(f"✗ Invalid sync bytes: 0x{header_data[0]:02X} 0x{header_data[1]:02X}")
        else:
            print(f"✗ Expected 7 header bytes, got {len(header_data)}")
            if len(header_data) > 0:
                print(f"Received: {[hex(b) for b in header_data]}")

    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False
    finally:
        try:
            if 'ser' in locals():
                # Send stop command before closing
                ser.write(bytes([0xA5, 0x25]))
                ser.close()
                print("✓ Serial port closed")
        except:
            pass

    return False


def test_ros_service():
    """Test the ROS service for controlling the lidar"""
    print("\n--- Testing ROS Service ---")

    import subprocess
    import os

    # Check if ROS is sourced
    if 'ROS_DISTRO' not in os.environ:
        print("✗ ROS environment not sourced")
        return False

    try:
        # Check if service exists
        result = subprocess.run(['ros2', 'service', 'list'],
                                capture_output=True, text=True, timeout=5)

        if '/start_stop_lidar' in result.stdout:
            print("✓ start_stop_lidar service found")

            # Test service call
            print("Testing service call...")
            result = subprocess.run([
                'ros2', 'service', 'call', '/start_stop_lidar',
                'std_srvs/srv/SetBool', '{data: false}'
            ], capture_output=True, text=True, timeout=10)

            if result.returncode == 0:
                print("✓ Service call successful")
                print(f"Response: {result.stdout.strip()}")
                return True
            else:
                print(f"✗ Service call failed: {result.stderr}")
                return False
        else:
            print("✗ start_stop_lidar service not found")
            print("Available services:")
            for line in result.stdout.split('\n'):
                if 'lidar' in line.lower() or 'scan' in line.lower():
                    print(f"  {line}")
            return False

    except subprocess.TimeoutExpired:
        print("✗ ROS command timed out")
        return False
    except FileNotFoundError:
        print("✗ ros2 command not found")
        return False


def main():
    print("RPLiDAR Diagnostic Tool")
    print("=" * 50)

    # Check for different possible ports
    possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyAMA0', '/dev/serial0']
    port_found = None

    for port in possible_ports:
        try:
            import os
            if os.path.exists(port):
                print(f"Found port: {port}")
                if port_found is None:
                    port_found = port
        except:
            pass

    if port_found is None:
        print("No serial ports found. Please check RPLiDAR connection.")
        return

    print(f"Using port: {port_found}")

    # Test direct serial communication
    if test_rplidar_direct(port_found):
        print("\n✓ Direct serial communication works!")
    else:
        print("\n✗ Direct serial communication failed!")
        print("Troubleshooting tips:")
        print("1. Check USB connection")
        print("2. Check if user is in dialout group: sudo usermod -a -G dialout $USER")
        print("3. Try different baud rates: 256000, 115200")
        print("4. Check dmesg for USB device detection")
        return

    # Test ROS service if available
    try:
        import rclpy
        rclpy.init()
        test_ros_service()
        rclpy.shutdown()
    except ImportError:
        print("ROS2 Python not available, skipping ROS service test")
    except Exception as e:
        print(f"ROS test error: {e}")


if __name__ == '__main__':
    main()