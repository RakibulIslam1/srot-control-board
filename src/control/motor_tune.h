#pragma once
// =============================================================================
//  control/motor_tune — per-thruster RPM control autotuner (ESP32-orchestrated).
//
//  Runs IN WATER, props on. For each motor in turn it: (1) ramps raw throttle to
//  find the min-spin/idle point, (2) holds a few levels to fit the feedforward
//  slope FF_A (throttle per rpm), (3) relay-tunes the PI on the throttle→RPM plant
//  (Åström–Hägglund → Ziegler–Nichols). Results are averaged across motors and
//  written to g_params (RPM_KP/KI/FF_A/IDLE + MOT_SPIN_MIN), then persisted; the
//  DShot task pushes them to the Pico via the cfg frame.
//
//  It commands ONE motor's raw throttle at a time; the control loop drives that
//  motor and holds the rest stopped, with the safety monitor + keep-alive active.
// =============================================================================

#include <Arduino.h>
#include "config.h"

namespace motor_tune {

void start();                 // begin at motor 0, phase 0
void abort();                 // stop now (safety trip / mode exit)

// One step. `rpm` = latest per-motor signed RPM (Pico telemetry). Returns true while
// running; sets `active_motor` (0..NUM_THRUSTERS-1) and `out_throttle` (0..1 raw) for it.
bool update(const int16_t rpm[NUM_THRUSTERS], float dt, uint32_t now,
            int& active_motor, float& out_throttle);

float progress();             // 0..1

}  // namespace motor_tune
