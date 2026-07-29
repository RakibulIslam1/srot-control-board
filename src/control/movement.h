#pragma once
// =============================================================================
//  control/movement — Jetson-offloaded motion primitives (AUTO mode).
//
//  One robust state machine for every high-level verb: forward / back / strafe /
//  turn / dive / arc / style / stop / hold. Translation runs at an RPM-controlled
//  speed (voltage-independent) for a duration, then the ESP32 BRAKES (reverse
//  thrust scaled by the cruise speed) so distance is repeatable with no drift.
//  Depth changes are ramped (smooth, no splash); turns are rate-limited and pick
//  the shortest direction; style delegates to the spin controller. Heading- and
//  depth-hold run around it (in the control loop). Every command is preemptible.
// =============================================================================

#include <Arduino.h>
#include "config.h"

namespace movement {

enum class Type : uint8_t {
    NONE = 0, FWD, BACK, LEFT, RIGHT, TURN, DIVE, STOP, HOLD, STYLE, ARC
};

// Demand produced each control cycle.
struct Demand {
    float fwd = 0, lat = 0, yaw = 0;   // normalized surge / sway / yaw-stick setpoints
    float roll = 0, pitch = 0;         // torque demands — used only when spin=true (STYLE)
    float depth_target = 0;            // smoothed depth setpoint (m)
    bool  spin = false;                // STYLE: apply roll/pitch/yaw directly (spin controller)
};

// Call on entry to AUTO to latch the depth setpoint to the current depth.
void enter(float cur_depth);

// Start a command. `primary` = duration_s | degrees | depth_m | style-count; `speed` = 0..1
// (translation/arc fwd) | yaw-rate deg/s (turn); `aux` = arc signed yaw-rate / dive descent
// speed; `submode` = turn 0 relative / 1 absolute. cur_yaw (rad), cur_depth (m) at start.
void start(Type t, float primary, float speed, uint8_t submode, float aux,
           float timeout_s, float cur_yaw, float cur_depth);

void abort();   // brake→stop the current command (preemption / safety)

// One control step. Sets `running` true while a command executes (false = idle station-keep).
Demand update(float roll, float pitch, float yaw, float gx, float gy, float gz,
              float depth, const int16_t rpm[NUM_THRUSTERS], float dt, bool& running);

Type    type();       // active command type (NONE when idle)
float   progress();   // 0..1
uint8_t phase();      // internal phase (telemetry)

}  // namespace movement
