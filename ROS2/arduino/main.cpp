#include <Arduino.h>
#include <Servo.h>
// #include "Arduino_BMI270_BMM150.h" // IMU Library Removed as per previous modification
#include <cstdint>

// Pin Definitions
const int SERVO_STEERING_PIN = 3;
const int SERVO_THROTTLE_PIN = 5;
const int DIRECTION_PIN = 7;
const int ENCODER_PIN_A = 4;

// Serial Communication (with RPi via TX/RX pins)
HardwareSerial& RPiSerial = Serial1;
const unsigned long BAUD_RATE_PI = 57600;//115200; // Matches your modified arduino_ros_bridge.py

// Timing Constants
const unsigned long ENCODER_DEBOUNCE_TIME_MICROS = 500;
const unsigned long SENSOR_SEND_INTERVAL_MS = 50;     // Send sensor data every 50ms (20Hz)

// Global Objects
Servo servoSteering;

// Encoder Variables
volatile int32_t encoderCount = 0;
volatile unsigned long lastEncoderInterruptTime = 0;
volatile int8_t currentDirection = 1;

// Timing
unsigned long lastSensorSendTime = 0;

// --- Binary Protocol Definition ---
// RPi -> Arduino Command Packet
const uint8_t CMD_START_BYTE_1 = 0xA5;
const uint8_t CMD_START_BYTE_2 = 0x5A;
struct CommandPacket {
    int16_t steering_angle;
    int16_t throttle_speed;
};

// Arduino -> RPi Sensor Packet (Only encoder_ticks)
const uint8_t SENSOR_START_BYTE_1 = 0xB6;
const uint8_t SENSOR_START_BYTE_2 = 0x6B;
struct __attribute__((packed)) SensorPacket {
    int32_t encoder_ticks;
};

// Arduino -> Pi Heartbeat Ping
const uint8_t ARDUINO_TO_PI_HEARTBEAT_PING = 0xC1; // Expected by arduino_ros_bridge.py
const uint8_t PI_TO_ARDUINO_HEARTBEAT_PONG = 0xD1; // Arduino can listen for this if needed

// Function Prototypes
void setupSerial();
void setupActuators();
void setupEncoder();
void handleSerialCommands();
void sendSensorDataAndPing(); // Modified function name
uint8_t calculateChecksum(const uint8_t* data, size_t len);
void applyCommands(int16_t steering, int16_t throttle);

// --- Interrupt Service Routine ---
void encoderInterrupt() {
    unsigned long now = micros();
    if (now - lastEncoderInterruptTime > ENCODER_DEBOUNCE_TIME_MICROS) {
        encoderCount += currentDirection;
        lastEncoderInterruptTime = now;
    }
}

// --- Setup ---
void setup() {
    setupSerial();
    setupActuators();
    setupEncoder();

    lastSensorSendTime = millis();
    // RPiSerial.println("ARDUINO_READY"); // You can keep this, but the 0xC1 ping is crucial for the Python script's current logic
}

// --- Main Loop ---
// --- Main Loop ---
void loop() {
    handleSerialCommands();

    unsigned long now = millis();
    if (now - lastSensorSendTime >= SENSOR_SEND_INTERVAL_MS) {
        sendSensorDataAndPing();   // Send the sensor packet and heartbeat ping to the RPi
        lastSensorSendTime = now;
    }

    // PROBLEM BLOCK REMOVED:
    /*
    // Listen for PONG if you want the Arduino to also verify Pi's connection
    if (RPiSerial.available() > 0) {
        uint8_t incomingByte = RPiSerial.read();
        if (incomingByte == PI_TO_ARDUINO_HEARTBEAT_PONG) {
            // Optional: Acknowledge PONG received, e.g., blink an LED
        }
        // Note: This simple PONG check might interfere if PONG is sent amidst other packet data.
        // For robust bidirectional heartbeat, integrate PONG checking into the command parser or a separate state.
    }
    */
}

// --- Initialization Functions ---
void setupSerial() {
    RPiSerial.begin(BAUD_RATE_PI);
}

void setupActuators() {
    servoSteering.attach(SERVO_STEERING_PIN);
    servoSteering.write(90); // Center steering servo initially

    pinMode(SERVO_THROTTLE_PIN, OUTPUT);
    pinMode(DIRECTION_PIN, OUTPUT);
    digitalWrite(DIRECTION_PIN, HIGH); // Default forward
    analogWrite(SERVO_THROTTLE_PIN, 0); // Start stopped
}

void setupEncoder() {
    pinMode(ENCODER_PIN_A, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderInterrupt, RISING);
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

        // Allow PONG byte to be consumed without disrupting command parsing
        // if (incomingByte == PI_TO_ARDUINO_HEARTBEAT_PONG) {
        //     continue; // Or handle it as mentioned in loop()
        // }

        switch (state) {
            case IDLE:
                if (incomingByte == CMD_START_BYTE_1) state = GOT_START1;
                break;
            case GOT_START1:
                if (incomingByte == CMD_START_BYTE_2) {
                    state = GOT_START2;
                    bytesRead = 0;
                } else if (incomingByte == CMD_START_BYTE_1) {
                    // Stay in GOT_START1 if another start byte 1 is received
                } else {
                    state = IDLE;
                }
                break;
            case GOT_START2:
                if (bytesRead < sizeof(CommandPacket)) {
                    cmdbuffer[bytesRead++] = incomingByte;
                    if (bytesRead == sizeof(CommandPacket)) {
                       state = READING_DATA;
                    }
                }
                break;
           case READING_DATA:
                uint8_t receivedChecksum = incomingByte;

                // --- ADD DEBUGGING HERE ---
                Serial.print("Potential packet data (before checksum): "); // Use USB Serial for this debug
                for(int k=0; k < sizeof(CommandPacket); k++) {
                    Serial.print(cmdbuffer[k], HEX);
                    Serial.print(" ");
                }
                Serial.print(" | Received Checksum: ");
                Serial.println(receivedChecksum, HEX);
                // --- END DEBUGGING ---
            
                uint8_t calculatedChecksum = calculateChecksum(cmdbuffer, sizeof(CommandPacket));
            
                if (receivedChecksum == calculatedChecksum) {
                    CommandPacket cmd;
                    memcpy(&cmd, cmdbuffer, sizeof(CommandPacket));
                    // --- ADD MORE DEBUGGING ---
                    Serial.print("CHECKSUM OK. Applying cmd: Steer=");
                    Serial.print(cmd.steering_angle);
                    Serial.print(", Throttle=");
                    Serial.println(cmd.throttle_speed);
                    // --- END DEBUGGING ---
                    applyCommands(cmd.steering_angle, cmd.throttle_speed);
                } else {
                    // --- ADD DEBUGGING FOR CHECKSUM FAIL ---
                    Serial.print("CHECKSUM FAIL! Calc: ");
                    Serial.print(calculatedChecksum, HEX);
                    Serial.print(", Recv: ");
                    Serial.println(receivedChecksum, HEX);
                    // --- END DEBUGGING ---
                }
                state = IDLE;
                break;
        }
    }
}

void sendSensorDataAndPing() {
    // 1. Send the Heartbeat PING
    RPiSerial.write(ARDUINO_TO_PI_HEARTBEAT_PING);

    // 2. Send the Sensor Packet
    SensorPacket packet;
    noInterrupts();
    packet.encoder_ticks = encoderCount;
    interrupts();

    uint8_t sensor_payload_buffer[sizeof(SensorPacket)];
    memcpy(sensor_payload_buffer, &packet, sizeof(SensorPacket));
    uint8_t checksum = calculateChecksum(sensor_payload_buffer, sizeof(SensorPacket));

    uint8_t startBytes[] = {SENSOR_START_BYTE_1, SENSOR_START_BYTE_2};
    RPiSerial.write(startBytes, sizeof(startBytes));
    RPiSerial.write((uint8_t*)&packet, sizeof(SensorPacket));
    RPiSerial.write(&checksum, 1);
}

void applyCommands(int16_t steering, int16_t throttle) {
    int16_t servoAngle = constrain(steering, 0, 180);
    servoSteering.write(servoAngle);

    int16_t motorSpeed = constrain(throttle, -255, 255);

    if (motorSpeed > 10) {
      currentDirection = 1;
      digitalWrite(DIRECTION_PIN, HIGH);
      analogWrite(SERVO_THROTTLE_PIN, motorSpeed);
    } else if (motorSpeed < -10) {
      currentDirection = -1;
      digitalWrite(DIRECTION_PIN, LOW);
      analogWrite(SERVO_THROTTLE_PIN, abs(motorSpeed));
    } else {
      currentDirection = 1;
      analogWrite(SERVO_THROTTLE_PIN, 0);
    }
}