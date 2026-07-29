#pragma once
// =============================================================================
//  control/feedforward — hydrodynamic model-based demand post-processing (2a)
//
//  Called once per control cycle AFTER the mode produces its 6 body demands and
//  BEFORE the mixer. Adds three physically-grounded terms to the roll/pitch/yaw
//  torque demands (and throttle for depth), then clamps to ±1:
//    1. Angular drag feedforward   K·ω·|ω|   (linearizes rotational drag)
//    2. Cross-coupling cancellation (counter roll/pitch when yawing)
//    3. Centre-of-buoyancy auto-trim (bleed the steady I-term into a trim)
//  All gains default 0/off → behaviour unchanged until tuned. See ALGORITHMS.md §5.
// =============================================================================

#include <Arduino.h>
#include "state_types.h"   // FlightMode

namespace feedforward {

// Clear the learned auto-trim (call on mode entry so it isn't carried across modes).
void reset();

// Add the model terms to the demands. gx/gy/gz = measured body rates (rad/s).
// `mode` gates which terms apply; `learn` enables auto-trim learning (pass true only
// when the pilot sticks are near centre, so a maneuver doesn't corrupt the trim).
void apply(float& roll, float& pitch, float& yaw, float& throttle,
           float gx, float gy, float gz, FlightMode mode, bool learn);

}  // namespace feedforward
