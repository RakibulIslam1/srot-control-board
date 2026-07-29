// =============================================================================
//  control/thrust_trim — implementation. See the header for the rationale.
// =============================================================================

#include "control/thrust_trim.h"
#include "comms/params.h"
#include <math.h>

namespace thrust_trim {

// [motor][0] = forward gain, [motor][1] = reverse gain.
static float s_k[NUM_THRUSTERS][2];
static bool  s_init = false;

// Below this |duty| the measurement is dominated by the ESC's own break-away
// behaviour rather than the thrust curve, so learning there teaches nonsense.
static const float MIN_LEARN_DUTY = 0.15f;
// A duty that is still moving is a transient: RPM lags it by 100-300 ms, so the
// ratio would read low and the gain would chase it. Only learn near steady state.
static const float MAX_LEARN_SLEW = 0.05f;   // per update (~10 Hz) = 0.5/s

static float s_prev_duty[NUM_THRUSTERS];

void reset() {
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        s_k[i][0] = s_k[i][1] = 1.0f;
        s_prev_duty[i] = 0.0f;
    }
    s_init = true;
}

float learned(int motor, bool forward) {
    if (!s_init || motor < 0 || motor >= NUM_THRUSTERS) return 1.0f;
    return s_k[motor][forward ? 0 : 1];
}

float gain(int motor, float duty) {
    if (!s_init || motor < 0 || motor >= NUM_THRUSTERS) return 1.0f;
    if (g_params.thr_trim_en < 0.5f) return 1.0f;
    return s_k[motor][(duty >= 0.0f) ? 0 : 1];
}

void update(const float duty[NUM_THRUSTERS], const int16_t rpm[NUM_THRUSTERS],
            uint8_t present, float dt_s, bool allow_learn) {
    if (!s_init) reset();

    // Enabled explicitly only. A prop in AIR has almost no load, so it spins far
    // faster for the same duty; learning there would drive every gain to its clamp
    // and then mis-scale thrust the moment the vehicle is in water.
    if (g_params.thr_trim_en < 0.5f || !allow_learn || dt_s <= 0.0f) {
        for (int i = 0; i < NUM_THRUSTERS; ++i) s_prev_duty[i] = duty[i];
        return;
    }

    const float rpm_full = (g_params.rpm_max > 1.0f) ? g_params.rpm_max : 3600.0f;
    const float tau      = (g_params.thr_trim_tau > 0.2f) ? g_params.thr_trim_tau : 4.0f;
    const float lim      = constrain(g_params.thr_trim_max, 0.0f, 0.9f);
    const float step     = dt_s / tau;    // fraction of the error corrected per update

    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        float d = duty[i];
        float slew = fabsf(d - s_prev_duty[i]);
        s_prev_duty[i] = d;

        if (!(present & (1 << i))) continue;              // no telemetry -> nothing to learn from
        if (fabsf(d) < MIN_LEARN_DUTY) continue;          // below the useful band
        if (slew > MAX_LEARN_SLEW) continue;              // transient, not steady state

        int   band = (d >= 0.0f) ? 0 : 1;
        float meas = fabsf((float)rpm[i]);
        float pred = rpm_full * fabsf(d);
        if (pred < 1.0f) continue;

        // A reversal in progress reports the old direction's speed for a moment.
        if ((d >= 0.0f) != (rpm[i] >= 0) && meas > 0.05f * rpm_full) continue;

        // ratio > 1 means we are spinning FASTER than commanded -> reduce the gain.
        float ratio = meas / pred;
        if (!isfinite(ratio)) continue;
        ratio = constrain(ratio, 0.25f, 4.0f);            // reject wild outliers

        s_k[i][band] += step * (1.0f / ratio - s_k[i][band]);
        s_k[i][band] = constrain(s_k[i][band], 1.0f - lim, 1.0f + lim);
    }
}

}  // namespace thrust_trim
