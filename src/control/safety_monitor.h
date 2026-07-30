#pragma once
// =============================================================================
//  control/safety_monitor — anomaly guard for autotune modes.
//
//  While an AUTOTUNE / MOTOR_TUNE mode is armed, the control loop calls ok() each
//  cycle. If the vehicle exceeds any ST_* limit (tumbling angle, spin-out rate,
//  depth runaway, over-RPM, or NaN attitude) it returns false with a reason — the
//  loop then aborts the tune and DISARMS. Angles/rates are radians; limits are deg.
// =============================================================================

#include <stdint.h>

namespace safety_monitor {

// Returns true if all guards pass. On a violation returns false and sets *why.
// Pass rpm=nullptr / n_rpm=0 to skip the RPM check (flight-loop autotune).
//
// `allow_inverted` suppresses ONLY the tumbling-angle guard, for manoeuvres that are
// *meant* to exceed it — a STYLE/STUNT 360 deg roll trips ST_ANGLE_MAX (70 deg) within a
// fraction of a turn, which used to disarm the vehicle mid-spin every single time. The
// rate and RPM guards still apply, and those are what catch a genuine tumble or spin-out;
// a commanded roll is rate-limited by the spin controller.
//
// `depth_ref_valid` says whether `depth0` is a MEANINGFUL reference. Pass false for a mode
// that does not own a depth setpoint: STUNT passes the pilot's throttle straight through and
// never calls depth::setTarget(), so depth0 is frozen at whatever the depth was on mode
// entry. Checking against that would disarm the vehicle mid-roll — possibly inverted — for
// a deliberate pilot climb, which is a fault report for a non-fault. Every other guard still
// applies; only the depth comparison is skipped.
bool ok(float roll, float pitch, float gx, float gy, float gz,
        float depth, float depth0, const int16_t* rpm, int n_rpm, const char** why,
        bool allow_inverted = false, bool depth_ref_valid = true);

}  // namespace safety_monitor
