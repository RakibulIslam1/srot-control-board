#pragma once
// =============================================================================
//  control/movement — Jetson-offloaded motion primitives (AUTO mode).
//
//  One robust state machine for every high-level verb: forward / back / strafe /
//  turn / dive / arc / style / stop / hold. Translation cruises at an open-loop
//  normalised thrust for a duration, then the ESP32 BRAKES (reverse thrust scaled
//  by the cruise speed) so distance is repeatable with no drift. Run-to-run
//  repeatability as the pack drains comes from the mixer's voltage feedforward and
//  the slow RPM-learned thrust trim (MOT_BAT_V_MAX / THR_TRIM_EN) — NOT from an RPM
//  setpoint; RPM left the attitude path deliberately (it oscillated).
//
//  Depth changes are ramped (smooth, no splash); turns are rate-limited and pick
//  the shortest direction; style delegates to the spin controller. Heading- and
//  depth-hold run around it (in the control loop): every translate leg LOCKS the
//  heading it started with, and a completed turn locks the heading it was told to
//  reach. Every command is preemptible.
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
    // One-shot heading lock. Set on the first cycle of a translate/hold leg (the heading
    // the leg began with) and when a TURN reaches its commanded heading. The control loop
    // forwards it to attitude::holdYaw(). Without this the hold target was whatever the
    // attitude loop happened to be latching — correct in practice, but nothing guaranteed
    // it, and a finished TURN settled up to ~1.7° off the heading it was asked for.
    float yaw_lock = 0;                // absolute heading to hold (rad)
    bool  yaw_lock_valid = false;      // true for exactly one cycle
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
