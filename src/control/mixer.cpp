// =============================================================================
//  control/mixer — implementation
// =============================================================================

#include "control/mixer.h"
#include "comms/params.h"
#include "control/thrust_trim.h"   // slow RPM-based thrust normalisation gain

namespace mixer {

// Inverse thrust curve: map a THRUST demand (0..1) to a THROTTLE ratio (0..1) so
// that command→thrust is ~linear. Propeller thrust ∝ RPM² and throttle→RPM is ~
// linear, so thrust≈(1−e)·thr + e·thr²; this solves that quadratic for thr. e=0 is
// linear. (Same model as ArduPilot MOT_THST_EXPO.)
static float thstExpo(float thrust, float expo) {
    expo = constrain(expo, 0.0f, 1.0f);
    if (expo < 0.001f) return thrust;   // linear
    return ((expo - 1.0f) +
            sqrtf((1.0f - expo) * (1.0f - expo) + 4.0f * expo * thrust)) /
           (2.0f * expo);
}

// Columns: roll, pitch, yaw, throttle, forward, lateral.
// Rows = motors 1..8 (SUB_FRAME_VECTORED_6DOF).
static const float M[NUM_THRUSTERS][6] = {
    //  roll  pitch  yaw   thr   fwd   lat
    {  0,    0,     1,    0,   -1,    1 },  // 1 FR horiz
    {  0,    0,    -1,    0,   -1,   -1 },  // 2 FL horiz
    {  0,    0,    -1,    0,    1,    1 },  // 3 RR horiz
    {  0,    0,     1,    0,    1,   -1 },  // 4 RL horiz
    {  1,   -1,     0,   -1,    0,    0 },  // 5 FR vert
    { -1,   -1,     0,   -1,    0,    0 },  // 6 FL vert
    {  1,    1,     0,   -1,    0,    0 },  // 7 RR vert
    { -1,    1,     0,   -1,    0,    0 },  // 8 RL vert
};

void mix(float roll, float pitch, float yaw,
         float throttle, float forward, float lateral,
         float out[NUM_THRUSTERS]) {
    const float demand[6] = { roll, pitch, yaw, throttle, forward, lateral };
    float maxabs = 1.0f;
    for (int m = 0; m < NUM_THRUSTERS; ++m) {
        float v = 0;
        for (int c = 0; c < 6; ++c) v += M[m][c] * demand[c];
        out[m] = v;
        float a = fabsf(v);
        if (a > maxabs) maxabs = a;
    }
    // Uniform scale-down keeps the relative mix intact when saturated.
    if (maxabs > 1.0f) {
        float inv = 1.0f / maxabs;
        for (int m = 0; m < NUM_THRUSTERS; ++m) out[m] *= inv;
    }
}

// --- thruster-battery voltage feedforward ------------------------------------
// Slowly-filtered normalised pack voltage (v = V/V_max), or 0 when unavailable.
// Filtered at ~0.5 Hz like ArduPilot: the raw value arrives at <=2 Hz, unfiltered and
// ~9 mV-quantised, and must not pass sag transients straight into the motor demand.
static float s_v_norm = 0.0f;

void setBatteryVoltage(float volts, float dt_s) {
    float vmax = g_params.mot_bat_v_max;
    if (vmax < 1.0f || volts < 1.0f) { s_v_norm = 0.0f; return; }   // disabled / no source
    float vmin = constrain(g_params.mot_bat_v_min, 1.0f, vmax);
    float v = constrain(volts, vmin, vmax) / vmax;
    // 0.5 Hz single-pole LPF.
    float a = constrain(dt_s / (dt_s + (1.0f / (2.0f * (float)M_PI * 0.5f))), 0.0f, 1.0f);
    s_v_norm = (s_v_norm <= 0.0f) ? v : s_v_norm + a * (v - s_v_norm);
}

float batteryScale() {
    return (s_v_norm > 0.05f) ? s_v_norm : 0.0f;   // 0 = no compensation
}

int16_t oneToDshot(float norm, int8_t dir) {
    // 3D/bidirectional DShot bands: 1049..2047 forward, 48..1047 reverse, 1048 = the
    // neutral gap = zero thrust while the ESC stays armed. (The reverse band already
    // assumes 3D-configured ESCs.)
    const int16_t NEUTRAL_3D = 1048;

    float n = constrain(norm * (float)dir, -1.0f, 1.0f);
    float t = fabsf(n);

    // Truly-centred stick → armed idle. MOT_SPIN_ARM = 0 (default) means STOPPED
    // (neutral), not min-forward — otherwise every thruster creeps on arm.
    if (t < 0.005f) {
        float arm = constrain(g_params.mot_spin_arm, 0.0f, 1.0f);
        if (arm <= 0.0f) return NEUTRAL_3D;              // stopped
        return (int16_t)(1049.0f + arm * (2047.0f - 1049.0f));   // idle forward
    }

    // Linearize the thrust demand, then lift it into [SPIN_MIN, 1] so the smallest
    // command crosses the prop deadband instead of doing nothing then lurching.
    float spin_min = constrain(g_params.mot_spin_min, 0.0f, 0.9f);
    float expo = g_params.mot_thst_expo;
    float thr;

    // Thruster-battery voltage feedforward (ArduPilot thrust linearisation). Throttle
    // commands VOLTS, not thrust: at the same PWM a T200 makes 3.71 kgf at 12 V and
    // 6.7 kgf at 20 V. Scaling the demand by lift_max and the duty by 1/v keeps duty*V —
    // and therefore thrust — constant as the pack drains, which is what makes a timed
    // command travel the same distance every run. Open-loop, so it adds no lag.
    float v = batteryScale();
    if (v > 0.0f) {
        float lift_max = v * (1.0f - expo) + expo * v * v;
        thr = thstExpo(constrain(t * lift_max, 0.0f, 1.0f), expo) / v;
        thr = constrain(thr, 0.0f, 1.0f);
    } else {
        thr = thstExpo(t, expo);          // no usable pack voltage -> uncompensated
    }
    float shaped = constrain(spin_min + (1.0f - spin_min) * thr, 0.0f, 1.0f);

    // BOTH DShot 3D bands run low->high WITHIN themselves: forward 1049 (min) .. 2047
    // (max), reverse 48 (min) .. 1047 (max). The reverse band is NOT mirrored around the
    // 1048 neutral gap. Mapping it backwards (1047 - shaped*999) makes the smallest
    // reverse demand ~90% reverse and full demand a dead stop, which reads as a violent
    // judder that never spins up. Keep both branches in the same "band-min + magnitude"
    // shape so they can't drift apart again.
    if (n >= 0.0f) {
        return (int16_t)(1049.0f + shaped * (2047.0f - 1049.0f));   // forward: 1049..2047
    } else {
        return (int16_t)(48.0f + shaped * (1047.0f - 48.0f));       // reverse: 48..1047
    }
}

void motorAngular(int m, float& roll, float& pitch, float& yaw) {
    if (m < 0 || m >= NUM_THRUSTERS) { roll = pitch = yaw = 0; return; }
    roll = M[m][0]; pitch = M[m][1]; yaw = M[m][2];
}

void toDshot(const float norm[NUM_THRUSTERS], const int8_t dir[NUM_THRUSTERS],
             bool armed, int16_t dshot[NUM_THRUSTERS]) {
    for (int m = 0; m < NUM_THRUSTERS; ++m) {
        if (!armed) {
            dshot[m] = DSHOT_DISARMED;
        } else {
            // Apply the slow per-thruster thrust normalisation. This is a bounded,
            // multi-second-time-constant GAIN learned from measured RPM — it makes a
            // given command produce the same thrust as the pack drains, without putting
            // RPM anywhere near the attitude loop. Returns exactly 1.0 when disabled.
            float n = norm[m] * thrust_trim::gain(m, norm[m] * (float)dir[m]);
            dshot[m] = oneToDshot(constrain(n, -1.0f, 1.0f), dir[m]);
        }
    }
}

}  // namespace mixer
