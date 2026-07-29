// =============================================================================
//  control/autotune — implementation (relay tuner: rate → angle → depth)
// =============================================================================

#include "control/autotune.h"
#include "control/attitude_control.h"
#include "comms/params.h"
#include "comms/ui_log.h"
#include "comms/mav_stream.h"       // queueStatusText (Core-1 safe)
#include "comms/mavlink_bridge.h"   // MAV_SEVERITY_*

namespace autotune {

// Relay amplitudes per phase kind.
static const float A_TORQUE = 0.30f;   // rate phase: direct torque
static const float A_RATE   = 0.50f;   // angle phase: rate command (rad/s)
static const float A_HEAVE  = 0.30f;   // depth phase: heave demand

// Schmitt-trigger hysteresis on the measured signal (reject sensor noise → clean limit
// cycles): rate rad/s, angle rad, depth m.
static const float H_RATE   = 0.05f;   // ~3 deg/s
static const float H_ANG    = 0.02f;   // ~1 deg
static const float H_DEPTH  = 0.02f;   // 2 cm

// Gain safety clamps (a bad measurement can't write an absurd gain).
static const float KP_MAX = 20.0f, KI_MAX = 10.0f, KD_MAX = 2.0f;

static const uint32_t PHASE_TIMEOUT = 8000;
static const int      MIN_PERIODS   = 6;

static const char* PHASE_NAME[] = {
    "Tune rate roll", "Tune rate pitch", "Tune rate yaw",
    "Tune angle roll", "Tune angle pitch", "Tune angle yaw", "Tune depth" };

enum Phase {
    P_RATE_ROLL = 0, P_RATE_PITCH, P_RATE_YAW,
    P_ANG_ROLL, P_ANG_PITCH, P_ANG_YAW,
    P_DEPTH, P_DONE
};
static const int N_PHASES = 7;

static int      s_phase = -1;
static bool     s_need_init = false;
static uint32_t s_phase_start = 0;

// Limit-cycle measurement.
static int      s_sign = 0;            // Schmitt-trigger sign of the signal
static uint32_t s_last_cross = 0;
static float    s_amp = 0;
static float    s_period_sum_ms = 0;
static int      s_period_n = 0;

// Per-phase reference captured at entry.
static float    s_ref = 0;   // start angle/depth for the signal

static float wrapPi(float e) {
    if (!isfinite(e)) return 0.0f;
    while (e >  (float)M_PI) e -= 2.0f * (float)M_PI;
    while (e < -(float)M_PI) e += 2.0f * (float)M_PI;
    return e;
}

static bool s_depth_ok = false;   // depth phase runs only with a live depth sensor

void start(bool depth_ok) {
    s_phase = P_RATE_ROLL;
    s_need_init = true;
    s_depth_ok = depth_ok;
    if (!depth_ok) {
        mav_stream::queueStatusText(MAV_SEVERITY_WARNING,
                                    "Autotune: no depth sensor - skipping depth phase");
    }
}

static void initPhase(float roll, float pitch, float yaw, float depth, uint32_t now) {
    s_phase_start = now;
    s_sign = 0; s_last_cross = 0; s_amp = 0; s_period_sum_ms = 0; s_period_n = 0;
    if (s_phase >= 0 && s_phase < N_PHASES) ui_log::set(PHASE_NAME[s_phase]);   // OLED progress
    switch (s_phase) {
        case P_ANG_ROLL:  s_ref = roll;  break;
        case P_ANG_PITCH: s_ref = pitch; break;
        case P_ANG_YAW:   s_ref = yaw;   break;
        case P_DEPTH:     s_ref = depth; break;
        default:          s_ref = 0;     break;   // rate phases track rate directly
    }
    attitude::reset();
}

static void applyGains(int phase, float Ku, float Tu) {
    float Kp = 0.33f * Ku;
    float Ki = (Tu > 0) ? Kp / (0.5f * Tu) : 0.0f;
    float Kd = Kp * (Tu / 3.0f);
    // Clamp so a noisy/short limit-cycle measurement can't write an unstable gain.
    Kp = constrain(Kp, 0.0f, KP_MAX);
    Ki = constrain(Ki, 0.0f, KI_MAX);
    Kd = constrain(Kd, 0.0f, KD_MAX);
    switch (phase) {
        case P_RATE_ROLL:  g_params.rat_rll_p = Kp; g_params.rat_rll_i = Ki; g_params.rat_rll_d = Kd; break;
        case P_RATE_PITCH: g_params.rat_pit_p = Kp; g_params.rat_pit_i = Ki; g_params.rat_pit_d = Kd; break;
        case P_RATE_YAW:   g_params.rat_yaw_p = Kp; g_params.rat_yaw_i = Ki; g_params.rat_yaw_d = Kd; break;
        // Outer angle loops are P-only.
        case P_ANG_ROLL:   g_params.ang_rll_p = Kp; break;
        case P_ANG_PITCH:  g_params.ang_pit_p = Kp; break;
        case P_ANG_YAW:    g_params.ang_yaw_p = Kp; break;
        case P_DEPTH:      g_params.depth_p = Kp; g_params.depth_i = Ki; g_params.depth_d = Kd; break;
        default: break;
    }
}

// Track amplitude + Schmitt-trigger half-cycles of the phase signal (hysteresis `hyst`
// rejects sensor noise). Returns true when enough periods are gathered (or timeout).
static bool measure(float signal, uint32_t now, float hyst) {
    s_amp = max(s_amp, fabsf(signal));
    int newsign = s_sign;
    if (signal >  hyst) newsign =  1;
    else if (signal < -hyst) newsign = -1;
    if (newsign != s_sign && s_sign != 0) {          // a half-cycle just completed
        if (s_last_cross != 0) { s_period_sum_ms += (now - s_last_cross); s_period_n++; }
        s_last_cross = now;
    }
    if (newsign != 0) s_sign = newsign;
    return (s_period_n >= MIN_PERIODS) || (now - s_phase_start >= PHASE_TIMEOUT);
}

static void finishPhase() {
    float A = (s_phase <= P_RATE_YAW) ? A_TORQUE
            : (s_phase <= P_ANG_YAW)  ? A_RATE : A_HEAVE;
    if (s_period_n > 0 && s_amp > 1e-3f) {
        float half_ms = s_period_sum_ms / s_period_n;
        float Tu = 2.0f * half_ms / 1000.0f;
        float Ku = 4.0f * A / ((float)M_PI * s_amp);
        applyGains(s_phase, Ku, Tu);
    }
    s_phase++;
    s_need_init = true;
    if (s_phase >= N_PHASES) { s_phase = P_DONE; params::requestSaveAll(); }   // Core-0 writes flash
}

bool update(float roll, float pitch, float yaw,
            float gx, float gy, float gz, float depth,
            float dt, uint32_t now,
            float& out_roll, float& out_pitch, float& out_yaw, float& out_thr) {
    out_roll = out_pitch = out_yaw = out_thr = 0;
    if (s_phase < 0 || s_phase >= P_DONE) return false;

    // Skip the depth phase outright with no depth sensor: its relay switches on the sign
    // of (depth - reference), so a constant 0 never crosses and it would command a fixed
    // +A_HEAVE for the entire phase timeout, open-loop, with the runaway guard blind too.
    if (s_phase == P_DEPTH && !s_depth_ok) {
        s_phase = P_DONE;
        params::requestSaveAll();
        return false;
    }

    if (s_need_init) { initPhase(roll, pitch, yaw, depth, now); s_need_init = false; }

    switch (s_phase) {
        // ---- Inner rate loops: relay torque on the axis rate ----
        case P_RATE_ROLL:
        case P_RATE_PITCH:
        case P_RATE_YAW: {
            float rate = (s_phase == P_RATE_ROLL) ? gx : (s_phase == P_RATE_PITCH) ? gy : gz;
            float relay = (rate > 0) ? -A_TORQUE : A_TORQUE;
            out_roll  = (s_phase == P_RATE_ROLL)  ? relay : attitude::rateAxis(0, 0, gx, dt);
            out_pitch = (s_phase == P_RATE_PITCH) ? relay : attitude::rateAxis(1, 0, gy, dt);
            out_yaw   = (s_phase == P_RATE_YAW)   ? relay : attitude::rateAxis(2, 0, gz, dt);
            if (measure(rate, now, H_RATE)) finishPhase();
            break;
        }
        // ---- Outer angle loops: relay the rate command by angle-error sign ----
        case P_ANG_ROLL:
        case P_ANG_PITCH:
        case P_ANG_YAW: {
            int axis = (s_phase == P_ANG_ROLL) ? 0 : (s_phase == P_ANG_PITCH) ? 1 : 2;
            float ang = (axis == 0) ? roll : (axis == 1) ? pitch : yaw;
            float sig = (axis == 2) ? wrapPi(ang - s_ref) : (ang - s_ref);
            float rate_cmd = (sig > 0) ? -A_RATE : A_RATE;
            float gyro[3] = { gx, gy, gz };
            // Active axis follows the relay rate command; passive axes damp to 0.
            out_roll  = attitude::rateAxis(0, (axis == 0) ? rate_cmd : 0, gx, dt);
            out_pitch = attitude::rateAxis(1, (axis == 1) ? rate_cmd : 0, gy, dt);
            out_yaw   = attitude::rateAxis(2, (axis == 2) ? rate_cmd : 0, gz, dt);
            (void)gyro;
            if (measure(sig, now, H_ANG)) finishPhase();
            break;
        }
        // ---- Depth loop: relay heave by depth-error sign, hold attitude level ----
        case P_DEPTH: {
            float sig = depth - s_ref;
            out_thr = (sig > 0) ? -A_HEAVE : A_HEAVE;
            out_roll  = attitude::rateAxis(0, 0, gx, dt);
            out_pitch = attitude::rateAxis(1, 0, gy, dt);
            out_yaw   = attitude::rateAxis(2, 0, gz, dt);
            if (measure(sig, now, H_DEPTH)) finishPhase();
            break;
        }
        default:
            return false;
    }
    return (s_phase < P_DONE);
}

float progress() {
    if (s_phase < 0) return 0.0f;
    if (s_phase >= P_DONE) return 1.0f;
    return (float)s_phase / (float)N_PHASES;
}

void abort() { s_phase = P_DONE; s_need_init = false; }   // stop now, keep gains tuned so far

}  // namespace autotune
