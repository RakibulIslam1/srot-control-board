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

// Gain safety clamps, PER LOOP FAMILY.
//
// There used to be one set — KP_MAX 20, KI_MAX 10, KD_MAX 2 — applied to all three. Those
// are sane numbers for the ANGLE loop (default P 4.5) and meaningless for the RATE loops
// (defaults 0.135 / 0.090 / 0.0036), where they sit 148x / 111x / 555x above the default.
// A single clamp across families whose natural scales differ by ~30x is not a safety clamp
// at all: for the rate loops the P and D limits could never be reached, and the I limit,
// when it did bind, bound at 111x the default.
//
// These are roughly 7-11x each family's compiled default: loose enough that a genuine tune
// is never truncated, tight enough that a bad measurement cannot write something that will
// not fly. Hitting one is itself a signal, so it is reported (see applyGains).
struct GainLimit { float kp, ki, kd; };
static const GainLimit LIMITS[] = {
    { 1.0f, 0.6f, 0.04f },   // rate roll   (def 0.135 / 0.090 / 0.0036)
    { 1.0f, 0.6f, 0.04f },   // rate pitch  (def 0.135 / 0.090 / 0.0036)
    { 1.2f, 0.6f, 0.04f },   // rate yaw    (def 0.180 / 0.018 / 0)
    { 12.0f, 0.0f, 0.0f },   // angle roll  (def 4.5, P-only)
    { 12.0f, 0.0f, 0.0f },   // angle pitch (def 4.5, P-only)
    { 12.0f, 0.0f, 0.0f },   // angle yaw   (def 4.5, P-only)
    { 10.0f, 2.0f, 0.5f },   // depth       (def 3.0 / 0.5 / 0)
};

static const uint32_t PHASE_TIMEOUT = 8000;
// Half-cycles required for a measurement to count. Six half-cycles = three full periods.
static const int      MIN_PERIODS   = 6;
// Half-cycles discarded before measurement starts. The relay steps to full amplitude the
// instant a phase opens, so the first excursion is an entry transient, not a limit cycle —
// it is typically the largest one and it used to set the amplitude for the whole phase.
static const int      SETTLE_CROSSINGS = 2;
// A real limit cycle has a consistent period. If the longest measured half-cycle is more
// than this multiple of the shortest, the signal was drifting or noise-driven, not
// oscillating, and the numbers derived from it mean nothing.
static const float    PERIOD_SPREAD_MAX = 3.0f;
// The amplitude must clear the Schmitt hysteresis by a real margin. Sitting just above the
// band is chatter: it satisfies the crossing count while carrying no information, and
// because Ku = 4A/(pi*a) a tiny `a` is exactly what produces an enormous gain.
static const float    AMP_MARGIN = 2.0f;

static const char* PHASE_NAME[] = {
    "Tune rate roll", "Tune rate pitch", "Tune rate yaw",
    "Tune angle roll", "Tune angle pitch", "Tune angle yaw", "Tune depth" };

// Short tags for the STATUSTEXT reports. MAVLink's STATUSTEXT.text field is char[50] and
// the cross-core queue slot is 64 B, so 50 characters is the hard budget for a whole
// message — the PHASE_NAME strings above are for the OLED ticker and are too long to
// leave room for the numbers that are the point of the report.
static const char* PHASE_TAG[] = {
    "RollRate", "PitchRate", "YawRate", "RollAng", "PitchAng", "YawAng", "Depth" };

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
static int      s_cross_n = 0;         // total half-cycles seen, incl. the discarded ones
static float    s_half_min = 0, s_half_max = 0;   // period spread (limit-cycle sanity)
static int      s_fail_n = 0;          // phases that produced no usable measurement

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
    s_fail_n = 0;
    if (!depth_ok) {
        mav_stream::queueStatusText(MAV_SEVERITY_WARNING,
                                    "Autotune: no depth sensor - skipping depth phase");
    }
}

static void initPhase(float roll, float pitch, float yaw, float depth, uint32_t now) {
    s_phase_start = now;
    s_sign = 0; s_last_cross = 0; s_amp = 0; s_period_sum_ms = 0; s_period_n = 0;
    s_cross_n = 0; s_half_min = 0; s_half_max = 0;
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
    // Clamp against THIS loop family's envelope, not one global set — see LIMITS.
    const GainLimit& L = LIMITS[phase];
    bool clipped = (Kp > L.kp) || (Ki > L.ki) || (Kd > L.kd);
    Kp = constrain(Kp, 0.0f, L.kp);
    Ki = constrain(Ki, 0.0f, L.ki);
    Kd = constrain(Kd, 0.0f, L.kd);
    // ONE line per phase, inside the 50-char STATUSTEXT budget. A clamp is folded in as a
    // suffix rather than a second message: the cross-core queue is only 4 deep, and two
    // messages per phase risks a genuinely important WARNING being dropped behind an INFO.
    // "CLAMP" matters because it means the tune wanted a gain the envelope refused — the
    // number that landed is a limit, not a result.
    {
        char b[64];
        snprintf(b, sizeof(b), "%s Tu=%.2f P=%.3f I=%.3f D=%.4f%s",
                 PHASE_TAG[phase], (double)Tu,
                 (double)Kp, (double)Ki, (double)Kd, clipped ? " CLAMP" : "");
        mav_stream::queueStatusText(clipped ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO, b);
    }
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
//
// The first SETTLE_CROSSINGS half-cycles are DISCARDED, amplitude included. The relay steps
// to full amplitude the moment a phase opens, so the opening excursion is a step response,
// not a limit cycle — and since s_amp was a running max over the whole phase, that transient
// (typically the largest swing of the run) set the amplitude used for Ku. Relay tuning wants
// the steady-state amplitude; measuring the transient biases `a` high, Ku low, and makes two
// runs on the same vehicle disagree depending on how violent the entry happened to be.
static bool measure(float signal, uint32_t now, float hyst) {
    int newsign = s_sign;
    if (signal >  hyst) newsign =  1;
    else if (signal < -hyst) newsign = -1;
    if (newsign != s_sign && s_sign != 0) {          // a half-cycle just completed
        s_cross_n++;
        if (s_last_cross != 0 && s_cross_n > SETTLE_CROSSINGS) {
            float half = (float)(now - s_last_cross);
            s_period_sum_ms += half;
            s_period_n++;
            if (s_half_min == 0 || half < s_half_min) s_half_min = half;
            if (half > s_half_max) s_half_max = half;
        }
        s_last_cross = now;
    }
    if (newsign != 0) s_sign = newsign;
    // Amplitude only accumulates once the transient has been walked past.
    if (s_cross_n >= SETTLE_CROSSINGS) s_amp = max(s_amp, fabsf(signal));
    return (s_period_n >= MIN_PERIODS) || (now - s_phase_start >= PHASE_TIMEOUT);
}

// Did this phase actually observe a limit cycle, or did it just time out?
//
// This is the check that was missing. The old code applied gains on `s_period_n > 0`, so a
// phase that drifted across the hysteresis band ONCE in eight seconds still wrote a full
// PID. Worked through: one interval of ~4 s gives Tu = 8 s and Kd = Kp*Tu/3 = 2.67*Kp, which
// for a rate loop lands at the clamp — 555x the default D gain, into a derivative term on a
// 500 Hz loop. The mirror case, a fast noise-driven crossing, drives Ki to its clamp instead.
// Both are reachable on a bench in air, which is where anyone would first try this.
// Reasons are short by necessity: they share the 50-char STATUSTEXT budget with the tag.
static bool measurementIsValid(float hyst, const char** why) {
    if (s_period_n < MIN_PERIODS) { *why = "no limit cycle"; return false; }
    if (s_amp < hyst * AMP_MARGIN) { *why = "amplitude too small"; return false; }
    if (s_half_min <= 0.0f) { *why = "no usable period"; return false; }
    if (s_half_max > s_half_min * PERIOD_SPREAD_MAX) { *why = "period unstable"; return false; }
    return true;
}

static void finishPhase() {
    float A = (s_phase <= P_RATE_YAW) ? A_TORQUE
            : (s_phase <= P_ANG_YAW)  ? A_RATE : A_HEAVE;
    float hyst = (s_phase <= P_RATE_YAW) ? H_RATE
               : (s_phase <= P_ANG_YAW)  ? H_ANG : H_DEPTH;

    const char* why = nullptr;
    if (measurementIsValid(hyst, &why)) {
        float half_ms = s_period_sum_ms / s_period_n;
        float Tu = 2.0f * half_ms / 1000.0f;
        float Ku = 4.0f * A / ((float)M_PI * s_amp);
        applyGains(s_phase, Ku, Tu);
    } else {
        // Keep the existing gains and SAY SO. A silently-skipped phase used to be
        // indistinguishable from a successful one: the only message autotune ever emitted
        // was "finished". motor_tune already reports both its results and its failures.
        s_fail_n++;
        char b[64];
        snprintf(b, sizeof(b), "%s FAIL: %s - gains kept", PHASE_TAG[s_phase], why);
        mav_stream::queueStatusText(MAV_SEVERITY_WARNING, b);

        // A failed RATE phase ends the whole tune. Every later phase runs on top of the
        // inner loops: the angle phases relay a rate COMMAND through the rate PID, so if
        // that PID is untrusted the angle measurement is meaningless, and the depth phase
        // holds attitude with it. Continuing would layer derived gains on an unknown base
        // and report success at the end.
        if (s_phase <= P_RATE_YAW) {
            mav_stream::queueStatusText(MAV_SEVERITY_ERROR,
                "Autotune ABORTED: rate loop untuned");
            s_phase = P_DONE;
            s_need_init = false;
            return;                     // no save: nothing worth persisting
        }
    }
    s_phase++;
    s_need_init = true;
    if (s_phase >= N_PHASES) {
        s_phase = P_DONE;
        char b[64];
        if (s_fail_n == 0) {
            snprintf(b, sizeof(b), "Autotune complete: all %d phases tuned", N_PHASES);
            mav_stream::queueStatusText(MAV_SEVERITY_INFO, b);
        } else {
            snprintf(b, sizeof(b), "Autotune done: %d phase(s) FAILED", s_fail_n);
            mav_stream::queueStatusText(MAV_SEVERITY_WARNING, b);
        }
        params::requestSaveAll();   // Core-0 writes flash
    }
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
        char b[64];
        if (s_fail_n == 0) {
            snprintf(b, sizeof(b), "Autotune done: %d tuned, depth skipped", N_PHASES - 1);
            mav_stream::queueStatusText(MAV_SEVERITY_INFO, b);
        } else {
            snprintf(b, sizeof(b), "Autotune done: %d FAILED, depth skipped", s_fail_n);
            mav_stream::queueStatusText(MAV_SEVERITY_WARNING, b);
        }
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
            // Relay must drive TOWARD the reference (negative feedback) to produce the
            // sustained limit cycle the tune measures. sig > 0 means we are DEEPER than the
            // reference, so we need to ascend — and positive heave ascends
            // (docs/THRUSTER_MAP.md). This was `-A_HEAVE`, i.e. it drove further away: the
            // same sign slip as the depth PID, independently written. The result was
            // monotonic divergence instead of an oscillation, so the phase either measured
            // nothing usable or ran until the ST_DEPTH_DELTA guard disarmed it.
            out_thr = (sig > 0) ? A_HEAVE : -A_HEAVE;
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
