#include <Arduino.h>
#include <Servo.h>
#include "Arduino_BMI270_BMM150.h" // Use the specific IMU library
#include <cstdint>

// Pin Definitions
const int SERVO_STEERING_PIN = 3;
const int SERVO_THROTTLE_PIN = 5; // PWM Pin for motor driver ESC/H-Bridge
const int DIRECTION_PIN = 7;      // Digital Pin for motor direction
const int ENCODER_PIN_A = 4;      // Encoder channel A (using only one for simplicity now)
// const int ENCODER_PIN_B = 2;   // Optional: Add for quadrature if needed

// Serial Communication (with RPi via TX/RX pins)
HardwareSerial& RPiSerial = Serial1; // Use Serial1 for RPi connection
const unsigned long BAUD_RATE_PI = 115200;

// Timing Constants
const unsigned long ENCODER_DEBOUNCE_TIME_MICROS = 500; // Microseconds debounce for encoder
const unsigned long SENSOR_SEND_INTERVAL_MS = 50;     // Send sensor data every 50ms (20Hz)

// Global Objects
Servo servoSteering;

// Encoder Variables
volatile int32_t encoderCount = 0;
volatile unsigned long lastEncoderInterruptTime = 0;
volatile int8_t currentDirection = 1; // 1 for forward, -1 for reverse based on throttle cmd

// IMU Variables
float accX = 0.0, accY = 0.0, accZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;

// Timing
unsigned long lastSensorSendTime = 0;

// --- Binary Protocol Definition ---
// RPi -> Arduino Command Packet (Size: 2 + 2 + 2 + 1 = 7 bytes)
const uint8_t CMD_START_BYTE_1 = 0xA5;
const uint8_t CMD_START_BYTE_2 = 0x5A;
struct CommandPacket {
    int16_t steering_angle; // e.g., 0-180
    int16_t throttle_speed; // -255 to 255
};

// Arduino -> RPi Sensor Packet (Size: 2 + (6 * 4) + 4 + 1 = 31 bytes)
const uint8_t SENSOR_START_BYTE_1 = 0xB6;
const uint8_t SENSOR_START_BYTE_2 = 0x6B;
struct __attribute__((packed)) SensorPacket { // Use packed to potentially avoid padding issues
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    int32_t encoder_ticks;
};
// --- End Protocol Definition ---

// Function Prototypes
void setupSerial();
void setupIMU();
void setupActuators();
void setupEncoder();
void handleSerialCommands();
void sendSensorData();
void readIMUData();
uint8_t calculateChecksum(const uint8_t* data, size_t len);
void applyCommands(int16_t steering, int16_t throttle);

// --- Interrupt Service Routine ---
void encoderInterrupt() {
    unsigned long now = micros();
    if (now - lastEncoderInterruptTime > ENCODER_DEBOUNCE_TIME_MICROS) {
        // Basic counting - assumes direction is known from throttle command
        // For quadrature, you would read PIN_B here to determine direction
        encoderCount += currentDirection;
        lastEncoderInterruptTime = now;
    }
}

// --- Setup ---
void setup() {
    // Initialize debug serial if needed (optional)
    // Serial.begin(9600);
    // while (!Serial); // Wait for Serial Monitor
    // Serial.println("Arduino Setup Started");

    setupSerial();
    setupIMU();
    setupActuators();
    setupEncoder();

    lastSensorSendTime = millis();
    RPiSerial.println("READY"); // Send simple ready signal on startup
    // Serial.println("Arduino Setup Complete");
}

// --- Main Loop ---
void loop() {
    handleSerialCommands(); // Check for incoming commands from RPi

    unsigned long now = millis();
    if (now - lastSensorSendTime >= SENSOR_SEND_INTERVAL_MS) {
        readIMUData();      // Read the latest IMU values
        sendSensorData();   // Send the sensor packet to the RPi
        lastSensorSendTime = now;
    }

    // Add short delay if needed, but keep loop fast
    // delay(1);
}

// --- Initialization Functions ---
void setupSerial() {
    RPiSerial.begin(BAUD_RATE_PI);
    // Serial.println("RPi Serial Initialized at 115200");
}

void setupIMU() {
    if (!IMU.begin()) {
        // Serial.println("ERROR: IMU initialization failed!");
        while (1); // Halt execution if IMU fails critical component
    }
    // Serial.print("Accelerometer sample rate = ");
    // Serial.print(IMU.accelerationSampleRate());
    // Serial.println(" Hz");
    // Serial.print("Gyroscope sample rate = ");
    // Serial.print(IMU.gyroscopeSampleRate());
    // Serial.println(" Hz");
    // Serial.println("IMU Initialized");
}

void setupActuators() {
    servoSteering.attach(SERVO_STEERING_PIN);
    servoSteering.write(90); // Center steering servo initially

    pinMode(SERVO_THROTTLE_PIN, OUTPUT);
    pinMode(DIRECTION_PIN, OUTPUT);
    digitalWrite(DIRECTION_PIN, HIGH); // Default forward
    analogWrite(SERVO_THROTTLE_PIN, 0); // Start stopped

    // Serial.println("Actuators Initialized");
}

void setupEncoder() {
    pinMode(ENCODER_PIN_A, INPUT); // Use INPUT or INPUT_PULLUP/PULLDOWN depending on encoder hardware
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderInterrupt, RISING);
    // Serial.println("Encoder Initialized");
}

// --- Communication Functions ---

uint8_t calculateChecksum(const uint8_t* data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

void handleSerialCommands() {
    static enum { IDLE, GOT_START1, GOT_START2, READING_DATA } state = IDLE;
    static uint8_t cmdbuffer[sizeof(CommandPacket)];
    static uint8_t bytesRead = 0;

    while (RPiSerial.available() > 0) {
        uint8_t incomingByte = RPiSerial.read();

        switch (state) {
            case IDLE:
                if (incomingByte == CMD_START_BYTE_1) {
                    state = GOT_START1;
                }
                break;
            case GOT_START1:
                if (incomingByte == CMD_START_BYTE_2) {
                    state = GOT_START2;
                    bytesRead = 0;
                } else {
                    state = IDLE; // Incorrect sequence, reset
                }
                break;
            case GOT_START2: // Reading data payload
                if (bytesRead < sizeof(CommandPacket)) {
                    cmdbuffer[bytesRead++] = incomingByte;
                    if (bytesRead == sizeof(CommandPacket)) {
                       state = READING_DATA; // Transition to reading checksum
                    }
                }
                 // If too many bytes arrive somehow before checksum byte, error state? For now just wait for checksum.
                break;
           case READING_DATA: // Reading the checksum byte
                uint8_t receivedChecksum = incomingByte;
                uint8_t calculatedChecksum = calculateChecksum(cmdbuffer, sizeof(CommandPacket));

                if (receivedChecksum == calculatedChecksum) {
                    // Checksum matches, process command
                    CommandPacket cmd;
                    memcpy(&cmd, cmdbuffer, sizeof(CommandPacket));
                    applyCommands(cmd.steering_angle, cmd.throttle_speed);
                    // Serial.print("CMD RX: Steer="); Serial.print(cmd.steering_angle);
                    // Serial.print(" Throttle="); Serial.println(cmd.throttle_speed);
                } else {
                    // Checksum mismatch
                    // Serial.print("WARN: Checksum mismatch. Got: "); Serial.print(receivedChecksum);
                    // Serial.print(" Calculated: "); Serial.println(calculatedChecksum);
                }
                state = IDLE; // Reset state machine
                break;
        }
    }
}

void sendSensorData() {
    SensorPacket packet;
    packet.accel_x = accX;
    packet.accel_y = accY;
    packet.accel_z = accZ;
    packet.gyro_x = gyroX;
    packet.gyro_y = gyroY;
    packet.gyro_z = gyroZ;

    // Atomically read volatile encoder count
    noInterrupts();
    packet.encoder_ticks = encoderCount;
    interrupts();

    uint8_t buffer[sizeof(SensorPacket)];
    memcpy(buffer, &packet, sizeof(SensorPacket));
    uint8_t checksum = calculateChecksum(buffer, sizeof(SensorPacket));

    // Send start bytes, data, and checksum
    uint8_t startBytes[] = {SENSOR_START_BYTE_1, SENSOR_START_BYTE_2};
    RPiSerial.write(startBytes, sizeof(startBytes));
    RPiSerial.write((uint8_t*)&packet, sizeof(SensorPacket)); // Send the packet data
    RPiSerial.write(&checksum, 1);                            // Send the checksum
    // Serial.print("."); // Indicate sending data for debugging
}


// --- Sensor & Actuator Functions ---

void readIMUData() {
    // Only read if data is available to avoid blocking
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ); // Read in m/s^2
    }
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ); // Read in degrees/sec

        // Convert gyro data from degrees/sec to radians/sec for ROS standard
        gyroX = radians(gyroX);
        gyroY = radians(gyroY);
        gyroZ = radians(gyroZ);
    }
    // Magnetometer data is not included in the protocol, but could be read here:
    // if (IMU.magneticFieldAvailable()) {
    //     IMU.readMagneticField(magX, magY, magZ); // Read in micro-Tesla (uT)
    // }
}

void applyCommands(int16_t steering, int16_t throttle) {
    // Steering Control
    // Clamp steering angle to valid servo range (adjust if servo range is different)
    int16_t servoAngle = constrain(steering, 0, 180);
    servoSteering.write(servoAngle);

    // Throttle Control
    int16_t motorSpeed = constrain(throttle, -255, 255);

    // Update global direction based on command for encoder ISR
    // Use a small deadzone around 0 if needed
    if (motorSpeed > 10) {
      currentDirection = 1; // Forward
      digitalWrite(DIRECTION_PIN, HIGH); // Set direction pin HIGH for forward
      analogWrite(SERVO_THROTTLE_PIN, motorSpeed); // Set PWM speed
    } else if (motorSpeed < -10) {
      currentDirection = -1; // Reverse
      digitalWrite(DIRECTION_PIN, LOW); // Set direction pin LOW for reverse
      analogWrite(SERVO_THROTTLE_PIN, abs(motorSpeed)); // Set PWM speed (absolute value)
    } else {
      // Stop
      currentDirection = 1; // Keep direction positive when stopped? Or doesn't matter?
      analogWrite(SERVO_THROTTLE_PIN, 0); // Set PWM speed to 0
    }
}