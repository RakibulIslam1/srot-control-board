// =============================================================================
//  control/dist_leg — implementation. See the header for the design and the evidence.
// =============================================================================
#include "control/dist_leg.h"
#include "comms/params.h"
#include "config.h"
#include <math.h>

namespace dist_leg {

static float s_goal = 0, s_cap = 0;
static float x_p = 0, x_v = 0, x_b = 0;
static float P[3][3];
static float s_integ = 0;
static uint32_t s_still_since = 0;
// A wrong-signed or mis-scaled accel axis does not look like bias -- it fights every velocity
// update, and the gate rejects them in a row. Past DIST_ACCEL_DISTRUST rejections the filter
// stops trusting the accelerometer for the rest of the leg (velocity-only prediction, reopened
// covariance) instead of diverging on an input it cannot explain.
static int      s_rejects = 0;
static bool     s_accel_ok = true;


void begin(float distance_m, float speed_cap, float v0) {
    s_goal = distance_m; s_cap = speed_cap;
    x_p = 0; x_v = v0; x_b = 0;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) P[i][j] = 0;
    P[1][1] = 0.05f * 0.05f;       // velocity from the feed that let the leg start
    P[2][2] = 0.3f * 0.3f;         // accel bias unknown: -17 mg measured on this board
    s_integ = 0; s_still_since = 0;
    s_rejects = 0; s_accel_ok = true;
}

void predict(float a, float dt) {
    if (!isfinite(a) || !(dt > 0.0f) || dt > 0.05f) return;
    const float acc = s_accel_ok ? (a - x_b) : 0.0f;
    x_p += x_v * dt;
    x_v += acc * dt;
    // P = F P F' + Q with F = [[1,dt,0],[0,1,-dt],[0,0,1]], expanded.
    const float p00 = P[0][0], p01 = P[0][1], p02 = P[0][2];
    const float p11 = P[1][1], p12 = P[1][2], p22 = P[2][2];
    const float n00 = p00 + dt * (2 * p01 + dt * p11);
    const float n01 = p01 + dt * p11 - dt * (p02 + dt * p12);
    const float n02 = p02 + dt * p12;
    const float n11 = p11 - 2 * dt * p12 + dt * dt * p22 + DIST_Q_V * DIST_Q_V * dt;
    const float n12 = p12 - dt * p22;
    const float n22 = p22 + DIST_Q_B * DIST_Q_B * dt;
    P[0][0] = n00; P[0][1] = P[1][0] = n01; P[0][2] = P[2][0] = n02;
    P[1][1] = n11; P[1][2] = P[2][1] = n12; P[2][2] = n22;
}

bool correct(float z, float var) {
    if (!isfinite(z) || !(var > 0.0f)) return false;
    const float y = z - x_v;
    const float S = P[1][1] + var;
    if (y * y > DIST_GATE_SIGMA2 * S) {
        if (++s_rejects < DIST_ACCEL_DISTRUST) return false;   // one bad sample: ignore it
        // A run of rejections is not noise: the estimate has left the world. Believe the
        // measurement -- adopt it outright, drop the accelerometer for the rest of the leg
        // (the likeliest cause is its axis or sign), and carry on velocity-only. Rejecting
        // for ever instead is how the estimate stays wrong while the hull keeps moving.
        s_accel_ok = false;
        x_v = z; x_b = 0;
        P[1][1] = var; P[0][1] = P[1][0] = 0; P[1][2] = P[2][1] = 0;
        P[0][2] = P[2][0] = 0; P[2][2] = 0;
        s_rejects = 0;
        return true;
    }
    s_rejects = 0;
    const float k0 = P[0][1] / S, k1 = P[1][1] / S, k2 = P[2][1] / S;
    x_p += k0 * y; x_v += k1 * y; x_b += k2 * y;
    float r0[3] = {P[1][0], P[1][1], P[1][2]};
    const float k[3] = {k0, k1, k2};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) P[i][j] -= k[i] * r0[j];
    return true;
}

float remaining() { return s_goal - x_p; }
float velocity()  { return x_v; }
bool  accelTrusted() { return s_accel_ok; }

float demand(float dt) {
    const float e = remaining();
    const float prof = sqrtf(2.0f * g_params.dist_a_dec * fabsf(e));
    float vref = fabsf(e) * g_params.dist_kp_pos;
    if (prof < vref) vref = prof;
    if (g_params.dist_v_max < vref) vref = g_params.dist_v_max;
    vref = (e < 0) ? -vref : vref;
    const float verr = vref - x_v;
    const float raw = g_params.dist_ff * vref + g_params.dist_kp_v * verr
                    + g_params.dist_ki_v * s_integ;
    // Conditional integration: only while the output is inside the speed cap.
    if (fabsf(raw) < s_cap) s_integ += verr * dt;
    return constrain(raw, -s_cap, s_cap);
}

bool settled(uint32_t now) {
    if (fabsf(remaining()) < DIST_TOL_M && fabsf(x_v) < DIST_V_SETTLE) {
        if (s_still_since == 0) s_still_since = now ? now : 1;
        return (now - s_still_since) >= DIST_SETTLE_MS;
    }
    s_still_since = 0;
    return false;
}

}  // namespace dist_leg
