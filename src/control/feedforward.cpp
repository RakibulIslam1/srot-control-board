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

    const bool stabilized = (mode == FlightMode::STABILIZE || mode == FlightMode::ACRO ||
                             mode == FlightMode::DEPTH_HOLD || mode == FlightMode::SURFACE);
    const bool angle_hold = (mode == FlightMode::STABILIZE ||
                             mode == FlightMode::DEPTH_HOLD || mode == FlightMode::SURFACE);
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

    // 3) Centre-of-Buoyancy auto-trim — slowly bleed the steady rate-PID integrator
    //    (and depth integrator) into a persistent feedforward trim, so the integrator
    //    returns toward zero: full I-term headroom for real disturbances, no slow
    //    windup, faster recovery. (It does NOT save energy — the same thrust holds the
    //    sub level regardless; only physical ballast changes the energy cost.)
    if (angle_hold && g_params.trim_en > 0.5f && learn) {
        const float leak = g_params.trim_leak;
        const float mx   = g_params.trim_max;
        s_trim_roll  = constrain(s_trim_roll  + leak * attitude::rateIntegral(0), -mx, mx);
        s_trim_pitch = constrain(s_trim_pitch + leak * attitude::rateIntegral(1), -mx, mx);
        if (depth_mode)
            s_trim_thr = constrain(s_trim_thr + leak * depth::integral(), -mx, mx);
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
