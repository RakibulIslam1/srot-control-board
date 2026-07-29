#pragma once

// =============================================================================
//  drivers/bno085 — Adafruit BNO08x (SH-2) IMU on I2C Bus0
//
//  Mirrors the proven example/bno085 example.cpp: GAME_ROTATION_VECTOR primary
//  attitude (robust to thruster magnetic disturbance), + linear accel, gravity,
//  calibrated gyro, magnetometer, raw accel. Polled via getSensorEvent() — our
//  board wires no INT/RESET, so no interrupt.
// =============================================================================

#include <Arduino.h>

namespace bno085 {

// Latest fused sample. Angles in radians; gyro in rad/s; accel/gravity in m/s^2;
// mag in uT. Quaternion is the game rotation vector (w,x,y,z).
struct Sample {
    float qw = 1, qx = 0, qy = 0, qz = 0;
    float roll = 0, pitch = 0, yaw = 0;      // rad, from quaternion
    float lin_x = 0, lin_y = 0, lin_z = 0;   // m/s^2, gravity removed
    float grav_x = 0, grav_y = 0, grav_z = 0;// m/s^2
    float gyro_x = 0, gyro_y = 0, gyro_z = 0;// rad/s
    float mag_x = 0, mag_y = 0, mag_z = 0;   // uT
    uint8_t acc_accuracy = 0;                // 0..3 (from accel/rotation status)
    uint8_t gyro_accuracy = 0;
    uint8_t mag_accuracy = 0;
};

// Blocking init (includes the 3 s cold-boot delay + enableReports). Returns
// false if the sensor is not found on either 0x4A or the configured address.
bool begin();

// Drain all pending SH-2 events into `out`. Returns true if any field updated.
// On a sensor reset it re-enables reports and, for a short recovery window,
// IGNORES rotation-vector samples (the identity/converging garbage) so `out`'s
// attitude keeps its last-good value instead of flickering to 0°.
bool poll(Sample& out);

// Is the sensor present and initialised?
bool healthy();

// True when a real fused attitude has been seen recently and we're not in a
// post-reset recovery window (use this for imu_valid, not healthy()).
bool attitudeValid();

// Number of BNO085 resets observed since boot (surface it to spot a marginal
// 3.3 V supply / weak I2C pull-ups — periodic resets are a hardware symptom).
uint32_t resetCount();

// Self-heal diagnostics. enableFailCount() rising means the sensor is rejecting
// sh2_setSensorConfig (marginal supply / bus); reinitCount() rising means the driver had
// to re-open the whole SH-2 session because attitude stopped arriving. Both should stay
// at 0 on healthy hardware — steady growth is a hardware fault, not a firmware one.
uint32_t enableFailCount();
uint32_t reinitCount();

}  // namespace bno085
