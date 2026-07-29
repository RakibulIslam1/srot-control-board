// =============================================================================
//  control/attitude_control — implementation
// =============================================================================

#include "control/attitude_control.h"
#include "control/pid.h"
#include "comms/params.h"

namespace attitude {

static const float DEG2RAD = 0.01745329252f;

// Inner rate PIDs, one per axis.
static PID s_rate_roll, s_rate_pitch, s_rate_yaw;

// Yaw heading-hold: when the yaw stick is centred we hold this captured heading.
static float s_yaw_target = 0.0f;
static bool  s_yaw_hold   = false;
static bool  s_yaw_capture_pending = false;   // capture heading on next centred cycle

// Cubic stick expo: fine resolution near centre, full authority at the ends.
static float applyExpo(float x, float e) {
    e = constrain(e, 0.0f, 1.0f);
    return (1.0f - e) * x + e * x * x * x;
}

// Wrap an angle error to [-π, π]. Guards NaN (a NaN would spin the while-loops
// forever → task watchdog reboot).
static float wrapPi(float a) {
    if (!isfinite(a)) return 0.0f;
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

static void loadGains() {
    s_rate_roll.setGains(g_params.rat_rll_p, g_params.rat_rll_i, g_params.rat_rll_d);
    s_rate_pitch.setGains(g_params.rat_pit_p, g_params.rat_pit_i, g_params.rat_pit_d);
    s_rate_yaw.setGains(g_params.rat_yaw_p, g_params.rat_yaw_i, g_params.rat_yaw_d);
    // Integrator limit from ATC_RAT_*_IMAX (0 → fall back to 0.5 so a zeroed param
    // doesn't kill the I-term); output clamped to ±1 (normalized torque).
    s_rate_roll.setLimits(g_params.rat_rll_imax  > 0 ? g_params.rat_rll_imax  : 0.5f, 1.0f);
    s_rate_pitch.setLimits(g_params.rat_pit_imax > 0 ? g_params.rat_pit_imax : 0.5f, 1.0f);
    s_rate_yaw.setLimits(g_params.rat_yaw_imax   > 0 ? g_params.rat_yaw_imax   : 0.5f, 1.0f);
}

void reset() {
    s_rate_roll.reset();
    s_rate_pitch.reset();
    s_rate_yaw.reset();
    s_yaw_hold = false;
    s_yaw_capture_pending = true;   // capture heading on the next centred-stick cycle
}

void stabilize(float stick_roll, float stick_pitch, float stick_yaw,
               float meas_roll, float meas_pitch, float meas_yaw,
               float gx, float gy, float gz, float dt,
               float& out_roll, float& out_pitch, float& out_yaw) {
    loadGains();
    const float expo = g_params.pilot_expo;

    // Outer angle loop (roll/pitch) with stick expo → desired body rate.
    float tgt_roll  = applyExpo(stick_roll,  expo) * MAX_LEAN_RAD;
    float tgt_pitch = applyExpo(stick_pitch, expo) * MAX_LEAN_RAD;
    float des_roll_rate  = (tgt_roll  - meas_roll)  * g_params.ang_rll_p;
    float des_pitch_rate = (tgt_pitch - meas_pitch) * g_params.ang_pit_p;

    // Yaw: fine rate-command when the stick is active (PILOT_YAW_RATE deg/s, expo);
    // when centred, HOLD the last heading (angle-P → rate, clamped to the same max)
    // so you can nudge a few °/s then lock heading.
    float max_yaw_rate = g_params.pilot_yaw_rate * DEG2RAD;
    if (max_yaw_rate <= 0.0f) max_yaw_rate = MAX_YAW_RATE;
    float yaw_stick = applyExpo(stick_yaw, expo);
    float des_yaw_rate;
    if (fabsf(yaw_stick) > 0.02f) {
        des_yaw_rate = yaw_stick * max_yaw_rate;
        s_yaw_target = meas_yaw;   // remember heading for when the stick releases
        s_yaw_hold   = true;
        s_yaw_capture_pending = false;
    } else {
        // Centred stick: capture the heading on the first cycle after mode entry so
        // hold engages immediately (not only after the pilot nudges yaw once).
        if (s_yaw_capture_pending) {
            s_yaw_target = meas_yaw;
            s_yaw_hold   = true;
            s_yaw_capture_pending = false;
        }
        float err = wrapPi(s_yaw_target - meas_yaw);
        des_yaw_rate = constrain(err * g_params.ang_yaw_p, -max_yaw_rate, max_yaw_rate);
    }

    // Inner rate loop → torque, with feedforward (crisper small commands).
    out_roll  = constrain(s_rate_roll.update(des_roll_rate, gx, dt)
                          + g_params.rat_rll_ff * des_roll_rate, -1.0f, 1.0f);
    out_pitch = constrain(s_rate_pitch.update(des_pitch_rate, gy, dt)
                          + g_params.rat_pit_ff * des_pitch_rate, -1.0f, 1.0f);
    out_yaw   = constrain(s_rate_yaw.update(des_yaw_rate, gz, dt)
                          + g_params.rat_yaw_ff * des_yaw_rate, -1.0f, 1.0f);
}

void acro(float stick_roll, float stick_pitch, float stick_yaw,
          float gx, float gy, float gz, float dt,
          float& out_roll, float& out_pitch, float& out_yaw) {
    loadGains();
    const float expo = g_params.pilot_expo;
    float rr = applyExpo(stick_roll,  expo) * MAX_ACRO_RATE;
    float pr = applyExpo(stick_pitch, expo) * MAX_ACRO_RATE;
    float yr = applyExpo(stick_yaw,   expo) * MAX_ACRO_RATE;
    out_roll  = constrain(s_rate_roll.update(rr, gx, dt)  + g_params.rat_rll_ff * rr, -1.0f, 1.0f);
    out_pitch = constrain(s_rate_pitch.update(pr, gy, dt) + g_params.rat_pit_ff * pr, -1.0f, 1.0f);
    out_yaw   = constrain(s_rate_yaw.update(yr, gz, dt)   + g_params.rat_yaw_ff * yr, -1.0f, 1.0f);
}

float rateIntegral(int axis) {
    switch (axis) {
        case 0:  return s_rate_roll.integral();
        case 1:  return s_rate_pitch.integral();
        default: return s_rate_yaw.integral();
    }
}

float rateAxis(int axis, float rate_cmd, float meas_rate, float dt) {
    loadGains();
    switch (axis) {
        case 0:  return s_rate_roll.update(rate_cmd, meas_rate, dt);
        case 1:  return s_rate_pitch.update(rate_cmd, meas_rate, dt);
        default: return s_rate_yaw.update(rate_cmd, meas_rate, dt);
    }
}

}  // namespace attitude
