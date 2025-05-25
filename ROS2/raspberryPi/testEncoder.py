# test_encoder.py
import serial
import struct
import time
import sys

# --- Import necessary components from your bridge script ---
try:
    # Assuming arduino_ros_bridge.py is in the same directory
    from arduino_ros_bridge import (
        SENSOR_START_BYTE_1,
        SENSOR_START_BYTE_2,
        SENSOR_PACKET_FORMAT,
        SENSOR_PACKET_SIZE,
        calculate_checksum
    )
    # Note: The format string '<ffffffi' indicates 6 floats followed by 1 integer.
    # The integer 'i' at the end is expected to be the encoder count.
    ENCODER_INDEX_IN_PACKET = 7 # The 7th value (index 6 if 0-based) is the integer
    print("Successfully imported constants from arduino_ros_bridge.py")
except ImportError:
    print("ERROR: Could not import from arduino_ros_bridge.py.")
    print("Ensure test_encoder.py is in the same directory as arduino_ros_bridge.py")
    sys.exit(1)
except AttributeError as e:
    print(f"ERROR: Missing expected constant/function in arduino_ros_bridge.py: {e}")
    print("Please ensure your arduino_ros_bridge.py file is complete and correct.")
    sys.exit(1)

# --- Configuration (Match your ArduinoBridgeNode defaults or settings) ---
SERIAL_PORT = '/dev/ttyAMA0' # Default from your script
BAUD_RATE = 115200          # Default from your script
READ_TIMEOUT = 0.5          # How long to wait for data before looping (seconds)
WAIT_FOR_READY = True       # Set to True if your Arduino sends "READY" on startup
READY_TIMEOUT = 5           # Seconds to wait for the "READY" signal

# --- Main Test Logic ---
ser = None
try:
    print(f"Attempting to connect to {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=READ_TIMEOUT)
    print("Serial port opened successfully.")

    # Optional: Wait for Arduino "READY" signal
    if WAIT_FOR_READY:
        print(f"Waiting up to {READY_TIMEOUT} seconds for 'READY' signal from Arduino...")
        ready_line = b""
        got_ready = False
        start_time = time.time()
        while time.time() - start_time < READY_TIMEOUT:
            if ser.in_waiting > 0:
                byte = ser.read(1)
                if byte == b'\n':
                    try:
                        decoded_line = ready_line.decode('utf-8').strip()
                        print(f"Received line: '{decoded_line}'")
                        if "READY" in decoded_line:
                             print("Arduino is READY.")
                             got_ready = True
                             break # Exit ready-wait loop
                        else:
                             ready_line = b"" # Reset for next line
                    except UnicodeDecodeError:
                         print(f"Warning: Received non-text data ended by newline (bytes: {ready_line.hex()}).")
                         ready_line = b""
                         break # Assume binary data started
                elif byte != b'\r': # Ignore carriage returns
                    ready_line += byte
            else:
                time.sleep(0.01) # Small delay to prevent busy-waiting

        if not got_ready:
            print("Warning: Did not receive 'READY' signal (or timed out). Proceeding anyway.")
        # Clear any remaining buffer after waiting for READY
        ser.reset_input_buffer()
        print("Input buffer cleared. Starting packet read loop.")


    # Variables for packet parsing state machine
    packet_state = 0 # 0: Wait Start1, 1: Wait Start2, 2: Read Data, 3: Check Checksum
    serial_buffer = bytearray()
    data_buffer = bytearray()

    print("\n--- Reading Encoder Values (Press Ctrl+C to stop) ---")
    while True:
        # Read available data
        if ser.in_waiting > 0:
            read_bytes = ser.read(ser.in_waiting)
            serial_buffer.extend(read_bytes)

        # Process buffer byte by byte
        while len(serial_buffer) > 0:
            byte = serial_buffer.pop(0)

            if packet_state == 0: # Wait for first start byte
                if byte == SENSOR_START_BYTE_1:
                    packet_state = 1
            elif packet_state == 1: # Wait for second start byte
                if byte == SENSOR_START_BYTE_2:
                    packet_state = 2
                    data_buffer.clear()
                elif byte == SENSOR_START_BYTE_1: # Handle back-to-back start bytes
                    packet_state = 1 # Stay in state 1
                else:
                    packet_state = 0 # Invalid sequence, reset
            elif packet_state == 2: # Read data packet bytes
                data_buffer.append(byte)
                if len(data_buffer) == SENSOR_PACKET_SIZE:
                    packet_state = 3 # Got all data, next byte is checksum
            elif packet_state == 3: # Check checksum
                received_checksum = byte
                calculated_checksum = calculate_checksum(data_buffer)

                if received_checksum == calculated_checksum:
                    try:
                        # Unpack the data
                        unpacked_data = struct.unpack(SENSOR_PACKET_FORMAT, data_buffer)
                        # Extract the encoder value (the last integer 'i')
                        encoder_value = unpacked_data[-1] # Last element is the int
                        print(f"Encoder Ticks: {encoder_value}    ", end='\r') # Print and overwrite line
                    except struct.error as e:
                        print(f"\nError unpacking packet: {e}")
                    except IndexError:
                         print(f"\nError: Packet format '{SENSOR_PACKET_FORMAT}' doesn't seem to contain enough elements for encoder value.")
                else:
                    # Use print instead of logger for simplicity
                    print(f"\nChecksum mismatch! Got: {received_checksum:02X}, Calc: {calculated_checksum:02X}. Packet dropped.")

                # Reset state machine for next packet
                packet_state = 0
                data_buffer.clear()

                # If we just processed a packet, break inner loop to allow reading more data
                break
            # Check added to prevent infinite loop if state machine gets stuck
            if packet_state not in [0,1,2,3]:
                print("\nError: Invalid packet state reached. Resetting.")
                packet_state = 0

        # Small delay if no data was processed in the inner loop
        # to prevent high CPU usage when idle
        if not ser.in_waiting and len(serial_buffer) == 0:
             time.sleep(0.01)


except serial.SerialException as e:
    print(f"\nFATAL: Serial error: {e}")
    print("Check port name, permissions (e.g., 'sudo usermod -a -G dialout $USER'), and connection.")
except KeyboardInterrupt:
    print("\nExiting script.")
except Exception as e:
    print(f"\nAn unexpected error occurred: {e}")
finally:
    if ser and ser.is_open:
        ser.close()
        print("Serial port closed.")