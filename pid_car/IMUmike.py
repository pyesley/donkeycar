import serial
import time
import math

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 57600  # Increased baud rate

# Increased timeout and buffer size
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=BAUD_RATE,
    timeout=1,
    write_timeout=1,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False
)


def flush_input_buffer():
    """Flush any pending input from the serial buffer."""
    ser.reset_input_buffer()


def read_until_complete():
    """Read from serial until we get a complete message."""
    start_time = time.time()
    buffer = ""

    while True:
        if ser.in_waiting:
            char = ser.read().decode('utf-8')
            buffer += char

            # Check if we have a complete message
            if buffer.endswith('\n'):
                return buffer.strip()

        # Timeout after 1 second
        if time.time() - start_time > 1:
            return None


def move_servo(angle):
    """Send a command to move the servo."""
    flush_input_buffer()
    print(f'angle {angle}')
    command = f"STE:{angle}\n"
    ser.write(command.encode('utf-8'))
    response = read_until_complete()
    if response:
        print(f"Servo Response: {response}")
    else:
        print("No servo response received")

def move_throttle(throttle):
    """Send a command to move the servo."""
    flush_input_buffer()
    realThrottle = int(throttle*100)
    command = f"DRV:{realThrottle}\n"
    ser.write(command.encode('utf-8'))
    response = read_until_complete()
    if response:
        print(f"Drive Response: {response}")
    else:
        print("No drive response received")

def request_imu():
    """Request IMU data."""
    flush_input_buffer()
    ser.write(b"IMU\n")
    response = read_until_complete()
    if response:
        print(f"IMU Data: {response}")
    else:
        print("No IMU data received")

def request_ENC():
    """Request IMU data."""
    flush_input_buffer()
    ser.write(b"ENC\n")
    response = read_until_complete()
    if response:
        print(f"ENC Data: {response}")
    else:
        print("No ENC data received")

def main():
    """Main loop."""
    angle = 0

    # Wait for Arduino to initialize
    time.sleep(2)
    flush_input_buffer()

    t = 0
    angleIndex = 0
    angles = [146,363]
    while True:

        try:
            drive = math.sin(t)
            #angle  = int( 120*math.fabs(math.sin(t)) )
            angle = int( angles[angleIndex]*255/1000 )
            angleIndex = ( angleIndex + 1 ) % 2

            move_servo(angle)
            request_imu()
            request_ENC()
            move_throttle(drive)
            time.sleep(1)
            t = t + 2*math.pi/10
            if t > 2*math.pi :
                t = 0
            print( t,drive,angle)

        except serial.SerialException as e:
            print(f"Serial error: {e}")
            time.sleep(1)  # Wait before retrying


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()