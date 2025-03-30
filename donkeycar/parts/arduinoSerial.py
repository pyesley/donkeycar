import serial
import time
import math
import logging

from numpy.core.numeric import rollaxis


class ARDUINONanoSerial:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(ARDUINONanoSerial, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):
            self.SERIAL_PORT = '/dev/ttyAMA0'
            self.BAUD_RATE = 115200

            # Increased timeout and buffer size
            self.serPort = serial.Serial(
                port=self.SERIAL_PORT,
                baudrate=self.BAUD_RATE,
                timeout=1,
                write_timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.initialized = True

    def flush_input_buffer(self):
        """Flush any pending input from the serial buffer."""
        self.serPort.reset_input_buffer()

    def read_until_complete(self):
        """Read from serial until we get a complete message."""
        start_time = time.time()
        buffer = ""

        while True:
            if self.serPort.in_waiting:
                char = self.serPort.read().decode('utf-8')
                buffer += char

                # Check if we have a complete message
                if buffer.endswith('\n'):
                    return buffer.strip()

            # Timeout after 1 second
            if time.time() - start_time > .01:
                return None

    def steering(self, pwm_angle):
        '''
            This function accepts numbers between 0 and 255
            It is assumed you have calibrated which value steers
            left and right
        '''
        """Send a command to move the servo."""
        pwm_angle = int(max(min(pwm_angle, 180), 0))
        self.flush_input_buffer()
        #logging.info(f'angle {pwm_angle}')
        command = f"STE:{pwm_angle}\n"
        self.serPort.write(command.encode('utf-8'))
        #response = self.read_until_complete()
        '''
        if response:
            logging.info(f"Servo Response: {response}")
        else:
            logging.info("No servo response received")
        '''

    def throttle(self, pwm_throttle):
        '''
            This function accepts numbers between -255 and 255
            It is assumed you have calibrated which value is forward and reverse throttle
        '''
        """Send a command to move the servo."""
        pwm_throttle =int(max(min(pwm_throttle, 255), -255))
        self.flush_input_buffer()
        command = f"DRV:{pwm_throttle}\n"
        self.serPort.write(command.encode('utf-8'))
        #response = self.read_until_complete()
        ''''
        if response:
            logging.info(f"Drive Response: {response}")
        else:
            logging.info("No drive response received")
        '''

    def parse_im2_data(self,data_string):
        sections = data_string.split('|')
        roll  = float( sections[0].split(':')[1] )
        pitch = float( sections[1].split(':')[1] )
        yaw = float( sections[2].split(':')[1] )

        return roll,pitch,yaw

    def parse_imu_data(self,data_string):
        sections = data_string.split('|')
        accel_data = sections[0].split(':')[1].split(',')
        gyro_data = sections[1].split(':')[1].split(',')
        mag_data = sections[2].split(':')[1].split(',')

        accel = {
            'x': float(accel_data[0]),
            'y': float(accel_data[1]),
            'z': float(accel_data[2])
        }
        gyro = {
            'x': float(gyro_data[0]),
            'y': float(gyro_data[1]),
            'z': float(gyro_data[2])
        }
        mag = {
            'x': float(mag_data[0]),
            'y': float(mag_data[1]),
            'z': float(mag_data[2])
        }
        return accel, gyro, mag

    def request_im2(self):
        """Request IMU YAW data from fancy Kalman Filter running on audrino."""
        ''' say we return a string like 
             ROLL:{roll}|PITCH:{pitch}|YAW:{yaw}
        '''
        self.flush_input_buffer()
        self.serPort.write(b"IM2\n")
        response = self.read_until_complete()
        if response:
            try :
                logging.info(f"RPY Data: {response}")
                roll, pitch, yaw = self.parse_im2_data(response)
                return roll, pitch, yaw
            except :
                logging.info("bad parse")
                return -999,-999,-999
        else:
            logging.info("No IM2 data received")
            return -999,-999,-999



    def request_imu(self):
        """Request IMU data."""
        ''' say we return a string like 
             f"ACC:{accel['x']},{accel['y']},{accel['z']}|GYO:{gyro['x']},{gyro['y']},{gyro['z']}|MAG:{mag['x']},{mag['y']},{mag['z']}"
        '''
        self.flush_input_buffer()
        self.serPort.write(b"IMU\n")
        response = self.read_until_complete()
        if response:
            try :
                logging.info(f"IMU Data: {response}")
                accel, gyro, mag = self.parse_imu_data(response)
                return accel, gyro, mag
            except :
                logging.info("bad parse")
        else:
            logging.info("No IMU data received")

    def request_ENC(self):
        """Request Encoder distance and velocity data."""
        self.flush_input_buffer()
        self.serPort.write(b"ENC\n")
        response = self.read_until_complete()
        try :
            if response:
                data = response.split(',')
                if len(data) == 2:
                    ticks = int(data[0])
                    velocity = float(data[1])
                    logging.info(f"ENC Data: {ticks}, {velocity}")
                    return ticks, velocity
            else:
                logging.info("No ENC data received")
        except :
            logging.info("bad parse")

    def reset_encoder(self):
        """Reset the encoder."""
        self.flush_input_buffer()
        self.serPort.write(b"ENR\n")

class ArduinoSteering:
    """
    Wrapper over a PWM pulse controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, pwm_left, pwm_right):
        logging.info(f'ArduinoSteering {pwm_left} {pwm_right}')
        self.intercept = (pwm_right + pwm_left) / 2.0
        self.slope = pwm_right - self.intercept
        self.ADR = ARDUINONanoSerial()
        self.running = True
        self.pulse = self.intercept
        logging.info(f'ArduinoSteering intercept {self.intercept} slope {self.slope}')
        logging.info('Arduino Steering created')

    def update(self):
        while self.running:
            self.ADR.steering(self.pulse)

    def run_threaded(self, steeringIn):
        # map absolute angle to angle that vehicle can implement.
        steering = int(self.intercept + self.slope * steeringIn)
        steering = int( max ( min(steering, 120), 0))
        #logging.info(f'************steering {steeringIn} -> {steering}')
        self.pulse = steering
        self.ADR.steering(steering)

    def run(self, steering):
        self.run_threaded(steering)
        self.pulse = steering

    def shutdown(self):
        # set steering straight
        self.ADR.steering(self.intercept)
        time.sleep(0.3)
        self.running = False


class ArduinoThrottle:
    """
    Wrapper over a PWM pulse controller to convert angles to PWM pulses.
    """

    def __init__(self ):
        self.ADR = ARDUINONanoSerial()
        self.running = True
        self.pulse = 0
        self.running = True
        logging.info('Arduino Throttle created')

    def update(self):
        while self.running:
            self.ADR.throttle(self.pulse)

    def run_threaded(self, throttle):
        pwm = int(max(min(throttle * 255, 255), -255))
        self.pulse = pwm
        self.ADR.throttle(pwm)

    def run(self, throttle):
        self.run_threaded(throttle)

    def shutdown(self):
        # set steering straight
        self.ADR.steering(0)
        time.sleep(0.3)
        self.running = False


class ArduinoEncoder:

    def __init__(self, ENCODER_TICKS_PER_METER=1):
        self.ADR = ARDUINONanoSerial()
        self.ADR.reset_encoder()
        self.ticks_per_MM = ENCODER_TICKS_PER_METER
        self.distance = 0
        self.velocity = 0

    def update(self):
        ticks, self.velocity = self.ADR.request_ENC()
        self.distance = ticks / self.ticks_per_MM

    def run_threaded(self ):
        distance, velocity = self.update()
        return distance, velocity

    def run(self, throttle):
        distance, velocity = self.update()
        return distance, velocity

    def shutdown(self):
        # set steering straight
        self.ADR.reset_encoder()


def calibrate_encoder():
    total_ticks = 0
    total_distance_cm = 0
    measurements = 0
    arduino = ARDUINONanoSerial()
    arduino.steering(44)

    while True:
        run_time_seconds = input("Enter the run seconds and press Enter to start the calibration: ")
        run_time_seconds = float(run_time_seconds)

        # Reset encoder
        arduino.reset_encoder()

        # Start the throttle
        arduino.throttle(255)
        start_time = time.time()

        # Run the throttle for the specified time
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < run_time_seconds:
            time.sleep(0.05)
        # Stop the throttle
        arduino.throttle(0)

        logging.info(f"time run {time.perf_counter() - start_time}")

        # Get the encoder data
        ticks, _ = arduino.request_ENC()

        # Ask the user for the distance
        logging.info( f'ticks {ticks}')
        distance_inches = float(input("Enter the distance traveled in inches: "))
        distance_cm = distance_inches * 2.54

        # Update total ticks and distance
        total_ticks += ticks
        total_distance_cm += distance_cm
        measurements += 1

        # Compute the cm/ticks ratio
        cm_per_tick = total_distance_cm / total_ticks
        logging.info(f"Current cm/tick ratio: {cm_per_tick:.4f}")

        # Ask if the user wants to run another calibration
        another_run = input("Do you want to run another calibration? (y/n): ")
        if another_run.lower() != 'y':
            break

    logging.info(f"Final cm/tick ratio after {measurements} measurements: {cm_per_tick:.4f}")
    return cm_per_tick



def mainOld():
    """Main loop."""
    arduino = ARDUINONanoSerial()
    angle = 0

    # Wait for Arduino to initialize
    time.sleep(2)
    arduino.flush_input_buffer()

    t = 0
    angleIndex = 0
    angles = [20, 60]
    isecCount = 0
    while True:
        try:
            drive = 255 * math.sin(t)
            if isecCount % 3 == 0:
                angle = angles[angleIndex]
                angleIndex = (angleIndex + 1) % 2
            arduino.throttle(drive)
            arduino.steering(angle)
            imu_data = arduino.request_imu()
            enc_data = arduino.request_ENC()
            logging.info(f"IMU Data: {imu_data}")
            logging.info(f"Encoder Data: {enc_data}")
            time.sleep(1)
            isecCount += 1
            t = t + 2 * math.pi / 10
            if t > 2 * math.pi:
                t = 0
            logging.info(t, drive, angle)
        except serial.SerialException as e:
            logging.info(f"Serial error: {e}")
            time.sleep(1)  # Wait before retrying


if __name__ == "__main__":
    try:
        #main()
        calibrate_encoder()
    except KeyboardInterrupt:
        logging.info("\nExiting...")
    finally:
        ARDUINONanoSerial().serPort.close()