#pragma once

// =============================================================================
//  control/arming — arm/disarm safety gating (ArduPilot-style)
//
//  The thruster kill switch (from the 2nd board) is display-only and does NOT
//  gate arming. Pre-arm checks are gated by the ARMING_CHECK param.
// =============================================================================

#include <Arduino.h>

namespace arming {

// True if arming is permitted. When `reason` is non-null and arming is refused,
// it is filled with a short human-readable pre-arm failure message (<= 41 chars, so
// "PreArm: " + reason fits STATUSTEXT's 50).
//
// Refuses (regardless of ARMING_CHECK) while the mode is STUNT, PATTERN, AUTOTUNE,
// MOTOR_TUNE or MOTOR_DETECT, or an ATUNE/USER_5 autotune is pending: arming there would
// itself start a scripted manoeuvre. Takes mtx_control and mtx_sensors (not nested).
bool canArm(const char** reason = nullptr);

}  // namespace arming
