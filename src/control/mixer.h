#pragma once

// =============================================================================
//  control/mixer — Vectored-8 thrust allocation
//
//  Maps the six body-axis demands (roll, pitch, yaw, throttle/heave, forward,
//  lateral), each in −1..1, onto 8 thruster outputs with the standard vectored
//  6-DOF allocation matrix for this frame. Motors 1–4 = horizontal, mounted at 45°
//  (yaw/surge/sway); motors 5–8 = vertical (heave/roll/pitch). See docs/THRUSTER_MAP.md.
// =============================================================================

#include <Arduino.h>
#include "config.h"

namespace mixer {

// Allocate demands → per-motor normalized outputs (−1..1). If any output
// exceeds ±1 the whole set is scaled down so the mix (relative authority) is
// preserved rather than clipped per-motor.
void mix(float roll, float pitch, float yaw,
         float throttle, float forward, float lateral,
         float out[NUM_THRUSTERS]);

// Convert normalized outputs (−1..1) → bidirectional DShot values, applying
// per-motor direction (`dir[i]` = +1/−1). When !armed all outputs are disarmed.
// 3D DShot mapping: 1049..2047 forward, 48..1047 reverse, ~1048 centre.
void toDshot(const float norm[NUM_THRUSTERS], const int8_t dir[NUM_THRUSTERS],
             bool armed, int16_t dshot[NUM_THRUSTERS]);

// Single-motor helper (motor test): normalized −1..1 → DShot value.
int16_t oneToDshot(float norm, int8_t dir);

// Feed the THRUSTER-battery voltage (volts) for the open-loop thrust linearisation, with
// the seconds elapsed since the last call. Pass 0 volts when no fresh source exists — the
// compensation then does nothing. Internally low-passed at ~0.5 Hz. Must be the thruster
// pack (2nd board / ESP-NOW), NOT the SBC battery on the local ADC.
void setBatteryVoltage(float volts, float dt_s);

// Normalised filtered pack voltage (V/V_max), or 0 when compensation is unavailable.
float batteryScale();

// Expected roll/pitch/yaw response when motor m is driven positive (the mixing
// factors) — used by motor-detect to compare against the measured gyro.
void motorAngular(int m, float& roll, float& pitch, float& yaw);

}  // namespace mixer
