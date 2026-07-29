#pragma once

// =============================================================================
//  control/stunt — N×360° spin controller
//
//  Spins the active axis at a commanded rate (its angle limit bypassed) while
//  the two passive axes stay angle-stabilized. Rotation is accumulated from the
//  measured body rate on the active axis.
// =============================================================================

#include <Arduino.h>
#include "state_types.h"

namespace stunt {

// Begin a spin of `target_deg` total about `axis` at `rate_dps` deg/s.
void start(StuntAxis axis, float target_deg, float rate_dps);

// One step. Returns true while spinning, false once complete. Writes the three
// torque demands (−1..1).
bool update(float meas_roll, float meas_pitch,
            float gx, float gy, float gz, float dt,
            float& out_roll, float& out_pitch, float& out_yaw);

// 0..1 completion.
float progress();

}  // namespace stunt
