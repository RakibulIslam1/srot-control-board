// =============================================================================
//  control/feedforward — implementation
// =============================================================================

#include "control/feedforward.h"
#include "control/attitude_control.h"
#include "control/depth_control.h"
#include "comms/params.h"

namespace feedforward {

// Learned Centre-of-Buoyancy trim (persistent across cycles; reset on mode entry).
static float s_trim_roll = 0, s_trim_pitch = 0, s_trim_thr = 0;

void reset() { s_trim_roll = s_trim_pitch = s_trim_thr = 0; }

void apply(float& roll, float& pitch, float& yaw, float& throttle,
           float gx, float gy, float gz, FlightMode mode, bool learn) {
    if (mode == FlightMode::MANUAL) return;   // raw passthrough — no model terms

    // AUTO is included in `stabilized`: it runs the SAME attitude::stabilize() cascade, so
    // the drag and cross-coupling terms apply to it identically. Leaving it out meant the
    // model-based feedforward silently did nothing in the one mode the companion computer
    // uses. It is deliberately NOT in `angle_hold` — AUTO commands its own translation, so
    // "sticks centred" is not evidence of a steady trim state and it must not learn.
    const bool stabilized = (mode == FlightMode::STABILIZE || mode == FlightMode::ACRO ||
                             mode == FlightMode::DEPTH_HOLD || mode == FlightMode::SURFACE ||
                             mode == FlightMode::AUTO);
    // `angle_hold` gates TRIM LEARNING, and the rule it encodes is "a human is on the sticks,
    // so centred sticks are evidence of a steady trim state". State it that way rather than
    // enumerating the allowed modes: the caller's `learn` flag is only "sticks are centred"
    // and is mode-blind, so an autonomous mode that never drives sp_* leaves it permanently
    // true and the CoB auto-trim learns against machine-commanded effort. AUTO was excluded
    // by name for exactly this reason; the next autonomous mode (the vision primitive in
    // VISION_API.md) would have had to remember to add itself. Inverting the test means it
    // cannot be forgotten — a new mode is excluded until someone deliberately includes it.
    const bool pilot_mode = (mode == FlightMode::STABILIZE || mode == FlightMode::ACRO ||
                             mode == FlightMode::DEPTH_HOLD || mode == FlightMode::SURFACE ||
                             mode == FlightMode::MANUAL);
    const bool angle_hold = pilot_mode && (mode != FlightMode::ACRO) &&
                            (mode != FlightMode::MANUAL);
    const bool depth_mode = (mode == FlightMode::DEPTH_HOLD || mode == FlightMode::SURFACE);

    // 1) Angular drag feedforward — rotational drag is ~quadratic in rate (τ_drag ∝
    //    ω·|ω|), so pre-inject the counter-torque; the rate PID then sees a ~linear
    //    plant and only cleans up disturbances. Uses the measured body rates.
    // 2) Cross-coupling — a fast yaw induces roll/pitch via hydrodynamic coupling;
    //    proactively counter it (empirical gains) rather than waiting for the gyro.
    if (stabilized) {
        roll  += g_params.atc_drag_rll * gx * fabsf(gx);
        pitch += g_params.atc_drag_pit * gy * fabsf(gy);
        yaw   += g_params.atc_drag_yaw * gz * fabsf(gz);
        roll  += g_params.xc_yaw2rll * gz;
        pitch += g_params.xc_yaw2pit * gz;
    }

    // 3) Centre-of-Buoyancy auto-trim — slowly TRANSFER the steady rate-PID integrator
    //    (and depth integrator) into a persistent feedforward trim, so the integrator
    //    returns toward zero: full I-term headroom for real disturbances, no slow
    //    windup, faster recovery. (It does NOT save energy — the same thrust holds the
    //    sub level regardless; only physical ballast changes the energy cost.)
    //
    //    It must be a TRANSFER, not a copy. This used to add `leak * integral()` to the
    //    trim while leaving the integrator untouched — but an integrator only decays when
    //    the error reverses, so the same charge was re-transferred every cycle and the
    //    trim ramped straight to its clamp (at leak 0.002 x 500 Hz that is under a second),
    //    after which the PID had to fight its own trim. Bleeding the transferred amount out
    //    of the integrator makes the total effort conserved, which is what the design
    //    always claimed to do.
    if (angle_hold && g_params.trim_en > 0.5f && learn) {
        const float leak = g_params.trim_leak;
        const float mx   = g_params.trim_max;
        auto shift = [&](float& trim, float integ) -> float {
            const float want = leak * integ;
            const float before = trim;
            trim = constrain(trim + want, -mx, mx);
            return trim - before;      // what the clamp actually accepted
        };
        attitude::bleedRateIntegral(0, shift(s_trim_roll,  attitude::rateIntegral(0)));
        attitude::bleedRateIntegral(1, shift(s_trim_pitch, attitude::rateIntegral(1)));
        if (depth_mode)
            depth::bleedIntegral(shift(s_trim_thr, depth::integral()));
    }
    roll     += s_trim_roll;
    pitch    += s_trim_pitch;
    throttle += s_trim_thr;

    roll     = constrain(roll,     -1.0f, 1.0f);
    pitch    = constrain(pitch,    -1.0f, 1.0f);
    yaw      = constrain(yaw,      -1.0f, 1.0f);
    throttle = constrain(throttle, -1.0f, 1.0f);
}

}  // namespace feedforward
