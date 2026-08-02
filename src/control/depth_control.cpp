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
// Last error/output of the REAL controller, for telemetry. Lets the water test confirm the
// armed loop matches what preview() showed on the bench.
static float s_last_err = 0, s_last_out = 0;

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
    const float out = s_pid.update(-s_target, -meas_depth, dt);
    s_last_err  = (-s_target) - (-meas_depth);   // == meas_depth - s_target
    s_last_out  = out;
    return out;
}

// --- OBSERVABILITY: prove the sign without arming ----------------------------
//
// AUDIT R1 has stood since the beginning: this loop has NEVER run closed, because the Bar30
// was not fitted during development. The documented bench check is "enter DEPTH_HOLD and
// hand-move the vehicle" — but that only works in WATER. In air, lifting the vehicle a whole
// metre changes the pressure by ~0.12 mbar, about 1.2 mm of equivalent depth: far too small
// to exercise anything. So the check as written could not actually be performed on a bench,
// which is a large part of why it never was.
//
// This makes the loop READABLE instead. `preview()` runs the same PID class over the same
// error expression against a SEPARATE instance, so the sign can be read straight off
// telemetry with the vehicle DISARMED and nothing spinning. Pressurise the Bar30 by hand
// (a thumb over the port is plenty) and watch DEPTH_CMD.
//
// EXPECTED, and this is the whole question:
//   measured DEEPER than target  -> DEPTH_CMD POSITIVE -> ascend  (back toward target)
//   measured SHALLOWER than target -> DEPTH_CMD NEGATIVE -> descend (back toward target)
// A demand that moves AWAY from the target means the sign is still inverted. Do not dive.
//
// Separate instance on purpose: sharing the flight PID would wind up its integrator while
// disarmed and hand a primed controller to the next arm.
// PROPORTIONAL ONLY, deliberately. The first version of this ran a full PID and it was
// useless: a preview is fed a CONSTANT error every telemetry tick, so the integrator wound
// straight up to the +/-1 limit and DEPTH_CMD sat pinned at 1.0 regardless of the input.
// It reported "inverted" for a loop that is in fact correct.
//
// For a SIGN test the integral and derivative terms are noise — only the sense of the error
// matters, and P alone shows it instantaneously with no state to accumulate. The real
// controller's full PID output is separately observable as DEPTH_OUT once armed.
float preview(float meas_depth, float tgt) {
    // Same error expression as update(): altitude frame, err = (-tgt) - (-meas) = meas - tgt.
    const float err = (-tgt) - (-meas_depth);
    float out = g_params.depth_p * err;
    if (out >  1.0f) out =  1.0f;
    if (out < -1.0f) out = -1.0f;
    return out;
}

float lastError()  { return s_last_err; }
float lastOutput() { return s_last_out; }

}  // namespace depth
