#pragma once

// =============================================================================
//  control/thrust_trim — slow per-thruster thrust normalisation from RPM
//
//  WHY THIS EXISTS
//  Throttle does not command thrust, it commands volts:
//      RPM    ~ (duty * V_batt) / Kv        thrust ~ RPM^2
//  so a T200 at the SAME PWM makes 3.71 kgf at 12 V and 6.7 kgf at 20 V — a 1.8x
//  spread purely from battery state. That is why "20 % for 5 s" travels a
//  different distance on a full vs flat pack. Measured thrust/RPM^2, by contrast,
//  is invariant to ~3 % across that whole range, so RPM is an excellent proxy for
//  thrust and a poor-but-cheap way to detect that a command under-delivered.
//
//  WHY IT IS SLOW, AND OUTSIDE THE ATTITUDE LOOP
//  Using RPM as a *setpoint* inside the stabilisation path made the vehicle
//  oscillate (spin up / stop / spin up on a 1 degree disturbance). Thruster
//  dynamics are a slow nonlinear lag, and shaft-speed control is documented to
//  induce thrust oscillation in dynamic conditions. So this runs at ~10 Hz with a
//  multi-second time constant, applying only a bounded GAIN to the commanded duty.
//  It can never fight the attitude controller: it is 100x slower than it.
//
//  Kept per DIRECTION because a T200 makes only ~75-79 % of its forward thrust in
//  reverse, so one gain cannot describe both.
// =============================================================================

#include <Arduino.h>
#include "config.h"

namespace thrust_trim {

// Advance the learner. Call at ~10 Hz from a comms/low-rate context.
//   duty[]     : commanded normalised output per thruster (-1..1), post-mixer
//   rpm[]      : measured signed RPM per thruster (Pico telemetry)
//   present    : bit i set = thruster i is reporting telemetry
//   dt_s       : seconds since the last call
//   allow_learn: false freezes adaptation entirely (props in air, disarmed, ...)
void update(const float duty[NUM_THRUSTERS], const int16_t rpm[NUM_THRUSTERS],
            uint8_t present, float dt_s, bool allow_learn);

// Gain to multiply thruster i's commanded duty by. Always finite and bounded;
// returns exactly 1.0 when the trim is disabled or nothing has been learned.
float gain(int motor, float duty);

// Reset every learned gain to 1.0 (arm/disarm edges, mode changes, param reset).
void reset();

// Diagnostics: the learned gain for a motor in a given direction (fwd = duty >= 0).
float learned(int motor, bool forward);

}  // namespace thrust_trim
