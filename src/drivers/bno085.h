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

// --- BNO085 Dynamic Calibration Data (its OWN flash, not our NVS) -------------
//
// The part converges a hard/soft-iron solution at runtime and keeps it in its own flash,
// but ONLY when the host asks. We never asked, so every power cycle started from zero
// magnetic accuracy -- the operator's "calibration is lost after a power cycle", and the
// reason control/yaw_ref could refuse (it needs mag_accuracy >= 2).
//
// Distinct from our CAL_MAG_* rows in NVS_NS_CAL, which are OUR corrections and always
// persisted correctly. Both are needed.
//
// Rate-limited internally (>= 60 s between writes) because flash endurance is finite and
// the accuracy flag can oscillate around a threshold. `force` bypasses the gap for an
// explicit operator-triggered save. Blocks on a command/response round trip: call only
// from the sensor task, and never while armed.
bool saveCalibration(bool force = false);

bool     dcdSavedThisBoot();
uint32_t dcdSaveCount();
// True once after each successful save, so the caller emits one STATUSTEXT rather than
// spamming. A silent save is indistinguishable from the silent non-save this replaces.
bool     takeDcdAnnounce();
// True once after a save ATTEMPT failed. A silent failure here is indistinguishable from
// "the calibration will not save", which is exactly how this bug presented.
bool     takeDcdFailAnnounce();
// Did sh2_setCalConfig() succeed at begin()? If false the sensor is not running dynamic
// calibration at all, so mag accuracy will never climb and there is nothing to save.
bool     calConfigOk();
// True once when a RETRIED sh2_setCalConfig finally succeeded (it is normally rejected on
// the first attempt, ~90 ms after reset). Lets the caller say so instead of leaving the
// boot-time "calibration OFF" error as the operator's last word on the subject.
bool     takeCalConfigAnnounce();

// Enable/disable the magnetometer REPORT (interval 0 = off).
//
// The mag is never fused into attitude -- the BNO runs 6-DOF GAME_ROTATION_VECTOR -- so
// once control/yaw_ref has taken its one-shot earth reference the report has no consumer.
// Stopping it cuts SHTP traffic on a bus polled WITHOUT the INT line, where the driver's
// own comment notes every extra report multiplies the desync/reset risk.
bool setMagReportEnabled(bool on);

}  // namespace bno085
