#pragma once

// =============================================================================
//  control/attitude_control — cascaded angle→rate attitude controller
//
//  Outer loop: angle error × ANG_*_P → desired body rate.
//  Inner loop: rate PID (RAT_*) → normalized torque demand (−1..1).
//  Gains are pulled live from g_params each update so QGC tuning takes effect.
// =============================================================================

#include <Arduino.h>

namespace attitude {

// Max pilot-commanded magnitudes.
constexpr float MAX_LEAN_RAD  = 0.6f;   // ~34° stabilize lean
constexpr float MAX_YAW_RATE  = 2.5f;   // rad/s yaw-rate command
constexpr float MAX_ACRO_RATE = 4.0f;   // rad/s per-axis acro rate

// Reset all integrators (call on (re)entry to a stabilized mode).
void reset();

// STABILIZE: hold target roll/pitch angles (rad); yaw is fine rate-command when the
// stick is active and heading-hold when centred. Inputs: pilot roll/pitch/yaw sticks
// (−1..1); measured roll/pitch/yaw + body rates. Outputs torque demands (−1..1).
void stabilize(float stick_roll, float stick_pitch, float stick_yaw,
               float meas_roll, float meas_pitch, float meas_yaw,
               float gx, float gy, float gz, float dt,
               float& out_roll, float& out_pitch, float& out_yaw);

// ACRO: rate-only control. Sticks map to body-rate targets.
void acro(float stick_roll, float stick_pitch, float stick_yaw,
          float gx, float gy, float gz, float dt,
          float& out_roll, float& out_pitch, float& out_yaw);

// Direct rate loop for one axis (used by STUNT on its active axis).
// axis: 0=roll,1=pitch,2=yaw. rate_cmd in rad/s. Returns torque (−1..1).
float rateAxis(int axis, float rate_cmd, float meas_rate, float dt);

// Rate-PID integrator state for one axis (0=roll,1=pitch,2=yaw) — read by the
// CoB auto-trim to learn the steady bias holding against static buoyancy torque.
float rateIntegral(int axis);

}  // namespace attitude
