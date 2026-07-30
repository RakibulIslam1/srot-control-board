#pragma once

// =============================================================================
//  control/pattern — 4-step complex pattern
//    1. TURN_360  — full yaw turn
//    2. HEADROOM  — move to entry_depth ± headroom clearance
//    3. TASK      — hold and perform the target maneuver
//    4. RETURN    — restore entry depth + heading
// =============================================================================

#include <Arduino.h>
#include "state_types.h"

namespace pattern {

// Begin the pattern. Captures the depth/heading to return to.
void start(float headroom_m, float entry_depth, float entry_yaw);

// Stop the sequence and return to IDLE. Called on the ARMED->DISARMED edge: without it
// the state machine kept its phase across a disarm and resumed driving thrusters the
// instant you re-armed, with no new command issued (the same defect autotune/motor_tune/
// movement had, which is why they all have abort()).
void abort();

// One step. Returns true while running, false once the pattern is complete.
// Writes attitude torque demands and a heave demand (depth output).
bool update(float meas_roll, float meas_pitch, float meas_yaw,
            float gx, float gy, float gz, float meas_depth,
            float dt, uint32_t now,
            float& out_roll, float& out_pitch, float& out_yaw, float& out_thr);

// Current step (for telemetry).
PatternStep step();

}  // namespace pattern
