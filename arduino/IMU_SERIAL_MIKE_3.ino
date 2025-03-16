/**
   March 9, 2025, I believe this is the software we are currenlty using on the Donkey car



 * Arduino Nano 33 Control System
 * Controls servos, reads encoder data, and handles IMU sensor data
 * Communication protocol with Raspberry Pi via Serial1
 */

#include <Arduino.h>
#include <Servo.h>
#include "Arduino_BMI270_BMM150.h"
#include <cstdio>
#include <cstdint>
#include "MEKF.h"

// Pin Definitions
const int SERVO_STEERING_PIN = 3;
const int SERVO_THROTTLE_PIN = 5;
const int DIRECTION_PIN = 7;
const int ENCODER_PIN = 4;

// Serial Communication
const int MAX_COMMAND_LENGTH = 20;
const unsigned long BAUD_RATE_DEBUG = 9600;
const unsigned long BAUD_RATE_PI = 115200;

// Timing Constants
const unsigned long ENCODER_DEBOUNCE_TIME = 200;     // microseconds
const unsigned long VELOCITY_UPDATE_INTERVAL = 250;   // milliseconds
const unsigned long IMU_UPDATE_INTERVAL = 100;       // milliseconds (if needed for periodic updates)

// Global Objects
Servo servoSteering;

// Command Processing
char command[MAX_COMMAND_LENGTH];
int commandIndex = 0;

// Encoder Variables
volatile int32_t encoderCount = 0;  //counts ticks from the encoder can be negative
int32_t lastencoderCount = 0;
volatile unsigned long lastInterruptTime = 0;
volatile int direction = 1;  //direction we are going now
unsigned long lastVelocityUpdate = 0;
float velocity = 0;

// IMU Variables
//unsigned long lastIMUUpdate = 0;

// IMU-related global variables
MEKF filter;
unsigned long lastIMUUpdate = 0;
float euler[3] = {0}; // Roll, pitch, yaw in degrees
bool imuInitialized = false;


// Function Prototypes
void setupSerial();
void setupIMU();
void setupServos();
void setupEncoder();
void processCommand();
void sendIMUData();
void updateVelocity();
void handleSerialCommand();
void updateIMUFilter();


/**
 * Interrupt Service Routine for encoder
 */
void encoderInterrupt() {
    unsigned long currentInterruptTime = micros();
    if (currentInterruptTime - lastInterruptTime > ENCODER_DEBOUNCE_TIME) {
        encoderCount = encoderCount + direction;
        lastInterruptTime = currentInterruptTime;
    }
}

void setup() {
    setupSerial();
    setupIMU2();
    setupServos();
    setupEncoder();
    
    Serial1.println("READY");
    Serial.println("Setup complete");
}

void loop() {
    updateIMUFilter();
    updateVelocity();
    handleSerialCommand();
}

/**
 * Setup Functions
 */
void setupSerial() {
    Serial.begin(BAUD_RATE_DEBUG);
    Serial1.begin(BAUD_RATE_PI);
    Serial.println("Starting setup...");
}

void setupIMU() {
    if (!IMU.begin()) {
        Serial.println("ERROR:IMU_INIT_FAILED");
        while (1);
    }
    Serial.println("IMU initialized");
}

void setupIMU2() {
    // Initialize IMU
    if (IMU.begin()) {
        imuInitialized = true;
        lastIMUUpdate = millis();
        Serial.println("IMU initialized successfully");
    } else {
        Serial.println("Failed to initialize IMU!");
    // Continue without IMU
    }
}

void setupServos() {
    servoSteering.attach(SERVO_STEERING_PIN);
    servoSteering.write(44);  // Default position
    pinMode(SERVO_THROTTLE_PIN,OUTPUT);
}

void setupEncoder() {
    pinMode(ENCODER_PIN, INPUT_PULLDOWN);
    pinMode(DIRECTION_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderInterrupt, RISING);
    lastVelocityUpdate = millis();
}

/**
 * Command Processing Functions
 */
void processCommand() {
    
    if (strncmp(command, "STE:", 4) == 0) {
        processSteeringCommand();
    }
    else if (strcmp(command, "IMU") == 0) {
        sendIMUData();
    }
    else if (strcmp(command, "IM2") == 0) {
        sendIM2Data();
    }
    else if (strncmp(command, "DRV:", 4) == 0) {
        processThrottleCommand();
    }
    else if (strcmp(command, "ENC") == 0) {
        sendEncoderData();
    }
    else if (strcmp(command, "ENR") == 0) {
        resetEncoderData();
    }
    else {
        Serial1.print("ERROR:UNKNOWN_COMMAND:");
        Serial1.println(command);
    }
    
    // Reset command buffer
    memset(command, 0, sizeof(command));
    commandIndex = 0;
}

void processSteeringCommand() {
    int angle = atoi(command + 4);
    if (angle >= 0 && angle <= 120) { //doing 120 because higher angles cause the servo arm to hit the hardware
        servoSteering.write(angle);
        //Serial1.print("OK:SERVO:");
        //Serial1.println(angle);
    } else {
        Serial1.println("ERROR:INVALID_ANGLE");
    }
}

void processThrottleCommand() {
    int pwmValue = atoi(command + 4);
    if (pwmValue >= -255 && pwmValue <= 255) {
        direction = pwmValue >= 0 ? 1 : -1 ;
        digitalWrite(DIRECTION_PIN, pwmValue >= 0 ? HIGH : LOW);
        analogWrite(SERVO_THROTTLE_PIN, abs(pwmValue));
        //Serial1.print("OK:THROTTLE:");
        //Serial1.println(pwmValue);
    } else {
        Serial1.println("ERROR:INVALID_THROTTLE");
    }
}

/**
 * Sensor Data Functions
 */
void sendIMUData() {
    float x, y, z;
    String imuData = "";
    //noInterrupts();
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        imuData += "ACC:" + String(x, 4) + "," + String(y, 4) + "," + String(z, 4);
    }

    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        imuData += "|GYO:" + String(x, 4) + "," + String(y, 4) + "," + String(z, 4);
    }

    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(x, y, z);
        imuData += "|MAG:" + String(x, 4) + "," + String(y, 4) + "," + String(z, 4);
    }
    //interrupts();
    Serial.println(imuData);
    Serial1.println(imuData);
    Serial1.flush();
}

void sendIM2Data() {

    if (!imuInitialized){
        Serial.println("noIMU");
        Serial1.println("noIMU");
        Serial1.flush();
    }
    filter.getEulerAngles(euler);
    String eulerData = "";
    eulerData += "ROLL:" + String(euler[0], 4);
    eulerData += "|PITCH:" + String(euler[1], 4);
    eulerData += "|YAW:" + String(euler[2], 4);
    Serial.println(eulerData);
    Serial1.println(eulerData);
    Serial1.flush();
}

void sendEncoderData() {
    noInterrupts();
      float currentVelocity = velocity;
      int32_t encoderCnt = encoderCount;
    interrupts();
    //get the message read to send to the Raspberry Pi
    char buffer[12];
    snprintf(buffer, sizeof(buffer),"%1d",static_cast<long>(encoderCnt));
    String encData = buffer;
    encData += ","+String(currentVelocity,4);
    Serial1.println(encData);
}

void resetEncoderData() {
    unsigned long currentTime = millis();
    noInterrupts();
      encoderCount = 0;
      lastInterruptTime = 0;
      lastVelocityUpdate = currentTime;
      velocity = 0;
    interrupts();
}



void updateVelocity() {
    unsigned long currentTime = millis();
    if (currentTime - lastVelocityUpdate >= VELOCITY_UPDATE_INTERVAL) {
        noInterrupts();
        int count = encoderCount;
        interrupts();
        
        unsigned long elapsedTime = currentTime - lastVelocityUpdate;
        lastVelocityUpdate = currentTime;
        long countDiff = count - lastencoderCount;
        lastencoderCount = count;

        if (elapsedTime > 0) {
            velocity = (countDiff * 1000.0) / elapsedTime;  // pulses per second
        }
    }
}

void handleSerialCommand() {
    while (Serial1.available() > 0) {
        int incomingByte = Serial1.read();
        if (incomingByte == -1) return;

        char c = (char)incomingByte;

        if (c == '\n' || c == '\r') {
            if (commandIndex > 0) {
                processCommand();
            }
        } else if (commandIndex < MAX_COMMAND_LENGTH - 1) {
            command[commandIndex++] = c;
            command[commandIndex] = '\0';
        } else {
            Serial1.println("ERROR:COMMAND_TOO_LONG");
            memset(command, 0, sizeof(command));
            commandIndex = 0;
        }
    }
}

void updateIMUFilter() {
  if (!imuInitialized) return;

  // Check if new IMU data is available
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az;
    float gx, gy, gz;

    // Read sensor data
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Compute time step
    unsigned long now = millis();
    float dt = (now - lastIMUUpdate) / 1000.0f;
    lastIMUUpdate = now;

    if (dt > 0.0f && dt < 0.5f) {  // Reasonable time step
      // Update filter with new measurements
      filter.updateIMU(ax, ay, az, gx, gy, gz, dt);

      // Get orientation as Euler angles
      filter.getEulerAngles(euler);
    }
  }
}