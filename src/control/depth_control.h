#pragma once

// =============================================================================
//  control/depth_control — barometric depth-hold (heave) controller
//
//  Pilot throttle stick commands a climb rate that moves the depth target; when
//  centred, the target latches and the PID holds depth. Output is a heave demand
//  (−1..1) fed to the mixer's throttle axis.
// =============================================================================

#include <Arduino.h>

namespace depth {

constexpr float MAX_CLIMB_MS = 0.5f;   // m/s at full stick

// Latch the depth target to the current depth (call on entry to DEPTH_HOLD).
void reset(float current_depth);

// One step. stick_throttle −1..1 (up = ascend). Returns heave demand −1..1 and
// writes the current target depth to target_out.
float update(float stick_throttle, float meas_depth, float dt, float& target_out);

// Directly set the held depth target (used by PATTERN headroom moves).
void setTarget(float depth_m);
float target();

// Depth-PID integrator state (for CoB vertical auto-trim).
float integral();

// Read the depth loop's SIGN with the vehicle disarmed and nothing spinning. Runs the same
// PID class over the same error expression against a separate instance (so the flight
// integrator is never wound up while disarmed). See the long note in the .cpp — the
// documented "hand-move the vehicle" check only works in water, which is why this loop went
// so long unverified. Published as DEPTH_CMD.
float preview(float meas_depth, float tgt);

// Last error / output of the REAL controller, so the water test can confirm the armed loop
// agrees with what preview() showed on the bench. Published as DEPTH_ERR / DEPTH_OUT.
float lastError();
float lastOutput();

// Remove `amount` of accumulated integrator (never past zero, never sign-flipping).
// The CoB auto-trim calls this after transferring that effort into its own feedforward
// trim, so the effort is HANDED OVER rather than duplicated.
void bleedIntegral(float amount);

}  // namespace depth
