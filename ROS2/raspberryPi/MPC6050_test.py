import smbus2
import time

# MPU-6050 Register Addresses
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# MPU-6050 Sensor sensitivity (default, can be configured)
# These are the default sensitivities if you haven't changed the FSR (Full Scale Range)
# 16384.0 LSB/g for accelerometer (+/- 2g)
# 131.0 LSB/deg/s for gyroscope (+/- 250 deg/s)
ACCEL_SCALE_FACTOR = 16384.0
GYRO_SCALE_FACTOR = 131.0

# I2C bus number (usually 1 for Raspberry Pi)
I2C_BUS = 1

# MPU-6050 I2C address (default address is 0x68, but can be 0x69 if AD0 is high)
# You can verify this using 'i2cdetect -y 1' in the terminal
MPU6050_ADDRESS = 0x68

def read_word(bus, adr, reg):
    """Reads a 16-bit word from two 8-bit registers."""
    high = bus.read_byte_data(adr, reg)
    low = bus.read_byte_data(adr, reg + 1)
    value = (high << 8) + low
    return value

def read_word_2c(bus, adr, reg):
    """Reads a 16-bit signed word (2's complement)."""
    val = read_word(bus, adr, reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def init_mpu6050(bus):
    """Initializes the MPU-6050."""
    try:
        # Wake up MPU-6050 (set PWR_MGMT_1 to 0)
        bus.write_byte_data(MPU6050_ADDRESS, PWR_MGMT_1, 0)
        print("MPU-6050 initialized successfully.")
    except Exception as e:
        print(f"Error initializing MPU-6050: {e}")
        print("Please check your wiring and I2C address.")
        exit()

def get_mpu6050_data(bus):
    """Reads accelerometer and gyroscope data from MPU-6050."""
    try:
        # Read accelerometer values
        accel_x = read_word_2c(bus, MPU6050_ADDRESS, ACCEL_XOUT_H) / ACCEL_SCALE_FACTOR
        accel_y = read_word_2c(bus, MPU6050_ADDRESS, ACCEL_XOUT_H + 2) / ACCEL_SCALE_FACTOR
        accel_z = read_word_2c(bus, MPU6050_ADDRESS, ACCEL_XOUT_H + 4) / ACCEL_SCALE_FACTOR

        # Read gyroscope values
        gyro_x = read_word_2c(bus, MPU6050_ADDRESS, GYRO_XOUT_H) / GYRO_SCALE_FACTOR
        gyro_y = read_word_2c(bus, MPU6050_ADDRESS, GYRO_XOUT_H + 2) / GYRO_SCALE_FACTOR
        gyro_z = read_word_2c(bus, MPU6050_ADDRESS, GYRO_XOUT_H + 4) / GYRO_SCALE_FACTOR

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    except Exception as e:
        print(f"Error reading MPU-6050 data: {e}")
        return None, None, None, None, None, None

if __name__ == "__main__":
    try:
        # Open I2C bus
        bus = smbus2.SMBus(I2C_BUS)
        print(f"Opened I2C bus {I2C_BUS}")

        # Initialize MPU-6050
        init_mpu6050(bus)

        print("\nReading MPU-6050 data every 1 second (Ctrl+C to stop)...")
        while True:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = get_mpu6050_data(bus)

            if all(v is not None for v in [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]):
                print(f"Accel (g): X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f} | "
                      f"Gyro (deg/s): X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")
            else:
                print("Failed to read MPU-6050 data.")

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if 'bus' in locals() and bus:
            bus.close()
            print("I2C bus closed.")