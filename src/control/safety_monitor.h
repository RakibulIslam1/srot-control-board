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
bool ok(float roll, float pitch, float gx, float gy, float gz,
        float depth, float depth0, const int16_t* rpm, int n_rpm, const char** why);

}  // namespace safety_monitor
