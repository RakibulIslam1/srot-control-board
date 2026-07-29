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
// it is filled with a short human-readable pre-arm failure message.
bool canArm(const char** reason = nullptr);

}  // namespace arming
