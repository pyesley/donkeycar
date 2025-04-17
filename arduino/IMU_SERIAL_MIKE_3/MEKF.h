// MEKF.h - Multiplicative Extended Kalman Filter for IMU orientation estimation
#ifndef MEKF_H
#define MEKF_H

#include <Arduino.h>

class MEKF {
private:
  // State variables
  float q[4]; // Quaternion [w, x, y, z]
  float w[3]; // Angular velocity [x, y, z]

  // Filter tuning parameters
  float beta;       // Algorithm gain
  float zeta;       // Gyro drift bias gain
  float w_bias[3];  // Gyro bias estimate

  // Helper method for quaternion normalization
  void normalizeQuaternion();

public:
  // Constructor - initializes the filter
  MEKF();

  // Update state with IMU measurements
  void updateIMU(float ax, float ay, float az, float gx, float gy, float gz, float dt);

  // Get quaternion representation of orientation
  void getQuaternion(float* qout);

  // Get Euler angles in degrees (roll, pitch, yaw)
  void getEulerAngles(float* euler);
};

#endif // MEKF_H