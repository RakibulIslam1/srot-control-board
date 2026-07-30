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

// Remove `amount` of accumulated integrator (never past zero, never sign-flipping).
// The CoB auto-trim calls this after transferring that effort into its own feedforward
// trim, so the effort is HANDED OVER rather than duplicated.
void bleedIntegral(float amount);

}  // namespace depth
