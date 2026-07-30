// =============================================================================
//  control/depth_control — implementation
// =============================================================================

#include "control/depth_control.h"
#include "control/pid.h"
#include "comms/params.h"

namespace depth {

static PID   s_pid;
static float s_target = 0;
static const float STICK_DEADBAND = 0.05f;

void reset(float current_depth) {
    s_target = current_depth;
    s_pid.reset();
}

void setTarget(float depth_m) { s_target = depth_m; }
float target() { return s_target; }
float integral() { return s_pid.integral(); }
void  bleedIntegral(float amount) { s_pid.bleedIntegral(amount); }

float update(float stick_throttle, float meas_depth, float dt, float& target_out) {
    s_pid.setGains(g_params.depth_p, g_params.depth_i, g_params.depth_d);
    s_pid.setLimits(1.0f, 1.0f);

    // Active stick moves the target (up = ascend = decrease depth).
    if (fabsf(stick_throttle) > STICK_DEADBAND) {
        s_target -= stick_throttle * MAX_CLIMB_MS * dt;
        if (s_target < 0) s_target = 0;
    }

    target_out = s_target;

    // Run the PID in ALTITUDE (= −depth), not depth, because that is the frame its
    // OUTPUT lives in: positive heave means ascend.
    //
    // This was inverted. The old code did `update(s_target, meas_depth)`, so a target
    // DEEPER than the current depth gave a POSITIVE error and hence a positive heave
    // demand — which ascends. It commanded the opposite of what it wanted, on every axis
    // of the depth loop. The old comment ("positive error → push down, since the mixer's
    // throttle column is −1") is where the slip happened: the −1 column combined with
    // "a positive motor command pushes the vehicle DOWN" (docs/THRUSTER_MAP.md) means
    // positive throttle ASCENDS. Two negatives, read as one.
    //
    // Corroborating the convention from three independent places: THRUSTER_MAP.md's axis
    // table ("throttle (heave): positive means ascend"), DEPTH_LOST_ASCENT being applied
    // as a positive value to surface without a sensor, and MANUAL mode passing the pilot's
    // throttle straight through (MANUAL_CONTROL z > 500 = up).
    //
    // Negating the INPUTS rather than the output is a readability choice, not a correctness
    // one: for this PID the two are provably identical from zero state (the integrator and
    // filtered derivative are both linear, and negating everything swaps the anti-windup's
    // push_high/push_low tests symmetrically, so the integrate-or-hold decision is
    // unchanged). Working in altitude is preferred because error, output, integrator and
    // derivative then all share ONE frame — which is exactly the confusion that produced the
    // bug in the first place.
    //
    // Worked through, subject named explicitly each time (shorthand is how this went wrong):
    //   err = setpoint - measurement = (-target) - (-measured) = measured - target
    //   measured DEEPER than target   -> err > 0 -> heave > 0 -> ascend  (back toward target)
    //   measured SHALLOWER than target -> err < 0 -> heave < 0 -> descend (back toward target)
    //   SURFACE failsafe, target 0 at 3 m -> err = +3 -> ascend
    //
    // This was never caught in the water because the Bar30 had not been fitted — the loop
    // has never run closed. Verify depth hold on the bench (hand-move the sensor) BEFORE
    // trusting it, and note the SURFACE failsafe depends on it.
    return s_pid.update(-s_target, -meas_depth, dt);
}

}  // namespace depth
