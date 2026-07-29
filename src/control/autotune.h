#pragma once

// =============================================================================
//  control/autotune — Åström–Hägglund relay auto-tuner for ALL PIDs
//
//  Sequentially tunes: inner rate loops (roll/pitch/yaw) → outer angle-P loops
//  (roll/pitch/yaw) → depth-hold PID. Each phase excites its signal with a relay,
//  measures the limit-cycle period Tu and amplitude a → Ku = 4A/(πa), then sets
//  conservative Ziegler–Nichols gains. Writes the params and persists.
//
//  Requires the vehicle ARMED and free to move (open water). ~1 minute total.
// =============================================================================

#include <Arduino.h>

namespace autotune {

// Begin tuning from the first phase. Call on the rising edge of the request.
// depth_ok = a working depth sensor. Without one the depth phase is SKIPPED: its relay
// is open-loop on the depth signal, so a constant 0 reading makes it hold a fixed heave
// for the whole phase timeout while the depth-runaway guard is also blind.
void start(bool depth_ok);

// One step. Drives torque on the axis/loop under test (and a heave demand during
// the depth phase); returns true while tuning, false when all phases are done.
bool update(float roll, float pitch, float yaw,
            float gx, float gy, float gz, float depth,
            float dt, uint32_t now,
            float& out_roll, float& out_pitch, float& out_yaw, float& out_thr);

// 0..1 across all phases.
float progress();

// Abort immediately (e.g. safety trip) — stops the sequence without saving further.
void abort();

}  // namespace autotune
