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

// Reset all integrators (call on (re)entry to a stabilized mode). Also arms a
// heading capture: the next centred-yaw cycle latches the current heading, so
// heading-hold engages immediately on arm / mode entry.
void reset();

// Force the heading-hold target to an explicit absolute heading (rad) and engage the
// hold now. Used by AUTO to LOCK the heading a translate leg started with, and to
// settle a completed TURN on the heading that was actually commanded rather than
// wherever the completion threshold happened to stop. A subsequent yaw stick/demand
// still overrides this — the pilot (or a yaw command) always wins.
void holdYaw(float yaw_rad);

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

// Remove `amount` of accumulated rate-PID integrator on one axis (never past zero,
// never sign-flipping). The CoB auto-trim calls this after moving that effort into its
// own feedforward trim, so the effort is HANDED OVER rather than counted twice.
void bleedRateIntegral(int axis, float amount);

}  // namespace attitude
