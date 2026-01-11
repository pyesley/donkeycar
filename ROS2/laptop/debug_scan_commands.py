#!/usr/bin/env python3

import serial
import time
import threading


def test_scan_command(port='/dev/ttyUSB0', baudrate=115200):
    """
    Test the actual scan command to see why motor isn't spinning
    """
    print(f"Testing RPLiDAR scan command on {port}")

    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        print("✓ Serial port opened")

        # Clear buffers and wait
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.5)

        print("\n--- Step 1: Send STOP command ---")
        stop_cmd = bytes([0xA5, 0x25])
        ser.write(stop_cmd)
        ser.flush()
        time.sleep(0.5)
        ser.reset_input_buffer()  # Clear any response
        print("✓ STOP sent")

        print("\n--- Step 2: Send RESET command ---")
        reset_cmd = bytes([0xA5, 0x40])
        ser.write(reset_cmd)
        ser.flush()
        print("✓ RESET sent")
        time.sleep(2)  # Wait for reset
        ser.reset_input_buffer()

        print("\n--- Step 3: Get device info ---")
        info_cmd = bytes([0xA5, 0x50])
        ser.write(info_cmd)
        ser.flush()

        # Read info response quickly
        header = ser.read(7)
        if len(header) == 7 and header[0] == 0xA5 and header[1] == 0x5A:
            info_data = ser.read(20)
            print(f"✓ Device info received (Model: {info_data[0]}, FW: {info_data[2]}.{info_data[1]})")
        else:
            print("✗ Failed to get device info")
            return False

        # Clear any remaining data
        ser.reset_input_buffer()
        time.sleep(0.1)

        print("\n--- Step 4: Send SCAN command ---")
        print("Sending scan command... motor should start spinning now!")

        scan_cmd = bytes([0xA5, 0x20])  # SCAN command
        ser.write(scan_cmd)
        ser.flush()
        print("✓ SCAN command sent")

        print("Waiting for scan response header...")
        header = ser.read(7)

        if len(header) == 7:
            print(f"Response header: {[hex(b) for b in header]}")

            if header[0] == 0xA5 and header[1] == 0x5A:
                response_type = header[6]
                print(f"Response type: 0x{response_type:02X}")

                if response_type == 0x81:  # MEASUREMENT response
                    print("✓ Correct measurement response received")
                    print("Motor should be spinning now!")

                    print("\n--- Reading scan data for 10 seconds ---")
                    start_time = time.time()
                    valid_measurements = 0

                    while time.time() - start_time < 10:
                        try:
                            # Read measurement packet (5 bytes)
                            measurement = ser.read(5)
                            if len(measurement) == 5:
                                # Parse basic measurement
                                sync_quality = measurement[0]
                                check_angle_low = measurement[1]
                                angle_high = measurement[2]
                                distance_low = measurement[3]
                                distance_high = measurement[4]

                                # Check if valid measurement
                                check_bit = (check_angle_low & 0x01) != 0
                                if check_bit:
                                    valid_measurements += 1

                                    if valid_measurements % 100 == 0:
                                        # Parse and display every 100th measurement
                                        angle_raw = ((angle_high << 8) | check_angle_low) >> 1
                                        distance_raw = (distance_high << 8) | distance_low
                                        angle_deg = angle_raw / 64.0
                                        distance_m = distance_raw / 4000.0
                                        quality = sync_quality >> 2

                                        print(
                                            f"Measurement {valid_measurements}: angle={angle_deg:.1f}°, distance={distance_m:.2f}m, quality={quality}")

                        except Exception as e:
                            print(f"Read error: {e}")
                            break

                    print(f"\n✓ Received {valid_measurements} valid measurements in 10 seconds")

                    if valid_measurements > 0:
                        print("🎉 Scan command working! Motor is spinning and data is flowing!")
                    else:
                        print("✗ No valid measurements received")

                else:
                    print(f"✗ Wrong response type: 0x{response_type:02X}, expected 0x81")
            else:
                print(f"✗ Invalid response header: {[hex(b) for b in header]}")
        else:
            print(f"✗ Expected 7 header bytes, got {len(header)}")

        print("\n--- Sending STOP command ---")
        ser.write(bytes([0xA5, 0x25]))
        ser.flush()
        time.sleep(0.2)
        print("✓ Motor should stop now")

    except Exception as e:
        print(f"Error: {e}")
        return False
    finally:
        if 'ser' in locals():
            try:
                ser.write(bytes([0xA5, 0x25]))  # Final stop
                ser.close()
            except:
                pass

    return True


def main():
    print("RPLiDAR Scan Command Debug Tool")
    print("=" * 50)
    print("This will test the scan command directly")
    print("Watch for the motor to start spinning!")
    print("=" * 50)

    test_scan_command()


if __name__ == '__main__':
    main()