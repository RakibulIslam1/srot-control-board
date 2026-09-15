#pragma once
// =============================================================================
//  control/dist_leg — measured-distance translation for the DIST movement.
//
//  ESTIMATOR: per-leg 1-D Kalman filter, state [p, v, b] along the leg axis (b = accel
//  bias). Predicted every control cycle with the BNO's gravity-removed acceleration;
//  corrected by each companion velocity on arrival.
//
//  ⛔ NOT DELAY-COMPENSATED, ON MEASUREMENT. The obvious improvement -- form the innovation
//  against the velocity believed at the CAPTURE instant (Larsen et al. 1998) -- was built,
//  and it made things WORSE: with a 250 ms feed it settled 31/54 runs against 51/54 without,
//  and at 400 ms 11/54 against 30/54. That is the measurement. The CAUSE is not verified; the
//  likeliest is that the innovation is taken in the past while the gain and covariance are
//  the present's. A full state replay might win; it was not worth 2.6 kB of RAM on a guess. See bakeoff/probe results in duburi_ws bumblebee-study/rounds/dist_leg.
//
//  CONTROL: cascaded. Position error -> velocity reference, the least of the speed cap, the
//  stopping profile sqrt(2*a*|e|) and a proportional term; then a velocity PI with
//  feed-forward and conditional-integration anti-windup.
//
//  WHY THIS AND NOT ADRC OR SLIDING MODE: a bake-off on one harness (duburi_ws
//  bumblebee-study/rounds/dist_leg/bakeoff.py: 9 hull models x 3 distances x 5 feed/actuator
//  scenarios, scored against truth, with this board's relay-like actuator) found all four
//  families settle; error is set by the velocity feed, not the law. Super-twisting SMC
//  reversed thrust 300-486 times per stop on the relay actuator; linear ADRC 2-3x more
//  than PID for no accuracy gain. PID is the only one of the accurate set that rejects a
//  current by construction. The brake-handoff this replaces overshot by up to 72 cm on a
//  high-drag hull, because the timed brake ends while the hull is still moving.
// =============================================================================
#include <Arduino.h>

namespace dist_leg {

// Start a leg along one body axis. `sign` +1/-1. Resets the filter to p = 0, v = v0.
void  begin(float distance_m, float speed_cap, float v0);
// One control cycle: accel along the axis (m/s^2, gravity removed), dt (s), now (ms).
void  predict(float accel_axis, float dt);
// A companion velocity along the axis. Returns false if it failed the innovation gate.
bool  correct(float v_axis, float var);
// Normalised surge/sway demand toward the goal (signed along the axis).
float demand(float dt);
bool  settled(uint32_t now_ms);
float remaining();   // metres to go along the axis, signed (negative = overshot)
float velocity();
bool  accelTrusted();   // false once the accelerometer was rejected for this leg

}  // namespace dist_leg
