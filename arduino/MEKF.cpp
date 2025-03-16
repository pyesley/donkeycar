#include "MEKF.h"

// Constructor implementation
MEKF::MEKF() {
  // Initialize quaternion to identity
  q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;

  // Initialize angular velocity and bias to zero
  w[0] = 0.0f; w[1] = 0.0f; w[2] = 0.0f;
  w_bias[0] = 0.0f; w_bias[1] = 0.0f; w_bias[2] = 0.0f;

  // Default filter parameters
  beta = 0.1f;   // Algorithm gain
  zeta = 0.015f;  // Bias drift compensation gain
}

void MEKF::normalizeQuaternion() {
  float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if(norm > 0.0f) {
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
  }
}

void MEKF::updateIMU(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
  // Normalized accelerometer readings (unit vector)
  float accel_norm = sqrt(ax*ax + ay*ay + az*az);
  if(accel_norm > 0.0f) {
    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;
  }

  // Correct for gyro bias
  gx -= w_bias[0];
  gy -= w_bias[1];
  gz -= w_bias[2];

  // Convert gyro readings to rad/s if they are in deg/s
  gx *= PI / 180.0f;
  gy *= PI / 180.0f;
  gz *= PI / 180.0f;

  // Reference direction of Earth's gravity
  float vx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
  float vy = 2.0f * (q[0]*q[1] + q[2]*q[3]);
  float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

  // Error is cross product between estimated and measured direction
  float ex = ay * vz - az * vy;
  float ey = az * vx - ax * vz;
  float ez = ax * vy - ay * vx;

  // Apply feedback
  w_bias[0] += zeta * ex * dt;
  w_bias[1] += zeta * ey * dt;
  w_bias[2] += zeta * ez * dt;

  // Adjusted angular rate
  gx += beta * ex;
  gy += beta * ey;
  gz += beta * ez;

  // Rate of change of quaternion
  float qDot[4];
  qDot[0] = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
  qDot[1] = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
  qDot[2] = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
  qDot[3] = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

  // Integrate to get quaternion
  q[0] += qDot[0] * dt;
  q[1] += qDot[1] * dt;
  q[2] += qDot[2] * dt;
  q[3] += qDot[3] * dt;

  // Normalize
  normalizeQuaternion();
}

void MEKF::getQuaternion(float* qout) {
  qout[0] = q[0]; // w
  qout[1] = q[1]; // x
  qout[2] = q[2]; // y
  qout[3] = q[3]; // z
}

void MEKF::getEulerAngles(float* euler) {
  // Roll (x-axis rotation)
  euler[0] = atan2(2.0f * (q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2])) * 180.0f / PI;

  // Pitch (y-axis rotation)
  float sinp = 2.0f * (q[0]*q[2] - q[3]*q[1]);
  if (fabs(sinp) >= 1.0f) {
    euler[1] = copysign(90.0f, sinp); // Use 90° if out of range
  } else {
    euler[1] = asin(sinp) * 180.0f / PI;
  }

  // Yaw (z-axis rotation)
  euler[2] = atan2(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) * 180.0f / PI;
}