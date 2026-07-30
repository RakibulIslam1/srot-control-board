#pragma once
// =============================================================================
//  control/yaw_ref — ONE-SHOT magnetic yaw reference ("gyrocompass alignment")
//
//  The attitude solution is SH2_GAME_ROTATION_VECTOR: the BNO085's 6-axis
//  accel+gyro fusion with NO magnetometer. That is deliberate — inside a metal
//  hull with eight thrusters pulling tens of amps, a 9-DOF (mag-fused) yaw jumps
//  the moment you throttle up, because the motor currents swamp the earth field.
//  The cost is that yaw is RELATIVE: it starts at whatever arbitrary zero the
//  fusion initialised at and has no idea where north is.
//
//  This module buys back the earth reference without ever letting the mag into
//  the loop. Once, at boot, while disarmed and still:
//
//      offset = magnetic_heading - game_yaw
//
//  and every consumer thereafter sees `game_yaw + offset`. The mag is read ONCE
//  and then never influences anything again, so thruster fields cannot move the
//  attitude solution — but heading is absolute from the first second.
//
//  WHAT THIS DOES NOT FIX: drift. The game rotation vector's yaw still drifts
//  (order 0.5-3 deg/min), so a long dive slowly loses absolute accuracy. This
//  corrects the REFERENCE, not the rate. Fixing drift would mean keeping a slow
//  mag correction running, which reintroduces exactly the motor sensitivity the
//  6-axis fusion was chosen to avoid. That trade is the whole point.
//
//  Fails SAFE: if the mag looks untrustworthy (poor accuracy, implausible field
//  strength, unstable samples) the alignment is REFUSED, the offset stays 0, and
//  the vehicle behaves exactly as it did before — relative yaw. Both outcomes are
//  announced over STATUSTEXT; a silent refusal would read as "it does nothing".
// =============================================================================

#include <Arduino.h>

namespace yaw_ref {

// Result of the last alignment attempt (also drives the STATUSTEXT wording).
enum class State : uint8_t {
    IDLE = 0,      // MAG_YAW_REF off, or nothing attempted yet
    SAMPLING,      // collecting mag/attitude samples
    LOCKED,        // alignment succeeded — offset() is live
    REFUSED_CAL,   // BNO085 mag accuracy too low (needs a mag calibration)
    REFUSED_FIELD, // field magnitude implausible (hard iron / interference nearby)
    REFUSED_NOISE, // samples disagreed too much (vehicle moving, or interference)
};

// Reset to IDLE and drop any offset. Called at boot and by the MAG_ALIGN re-trigger.
void reset();

// Feed one fresh sample. Call from Task_SensorRead each cycle with the RAW mag vector
// (uT, sensor frame — cal is applied here), the current roll/pitch/yaw (rad) from the
// game rotation vector, and the BNO085's mag accuracy (0..3). `imu_ok` gates sampling.
// Cheap and non-blocking; does nothing once LOCKED or refused.
void update(float mx, float my, float mz, uint8_t mag_accuracy,
            float roll, float pitch, float game_yaw, bool imu_ok);

// Radians to ADD to the game-rotation-vector yaw to get an absolute heading.
// Exactly 0.0 unless an alignment succeeded, so an unaligned vehicle is unchanged.
float offset();

State state();

// True once for each state change, so the caller can emit one STATUSTEXT per event
// instead of spamming. Clears on read.
bool takeStateChange();

// Human-readable reason for the current state (for STATUSTEXT).
const char* stateText();

}  // namespace yaw_ref
