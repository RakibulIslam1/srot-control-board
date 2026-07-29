// =============================================================================
//  control/motor_tune — implementation
// =============================================================================

#include "control/motor_tune.h"
#include "comms/params.h"
#include "comms/ui_log.h"
#include "comms/mav_stream.h"       // queueStatusText (Core-1 safe)
#include "comms/mavlink_bridge.h"   // MAV_SEVERITY_*
#include <math.h>

namespace motor_tune {

// --- tuning schedule ---------------------------------------------------------
static const float    RAMP_STEP    = 0.0020f;  // throttle/cycle in the idle ramp (~1 s to full)
static const int16_t  SPIN_RPM     = 200;      // rpm that counts as "spinning"
static const uint32_t SPIN_HOLD_MS = 200;      // must exceed SPIN_RPM this long → min-spin found
static const float    FF_LEVELS[3] = { 0.30f, 0.50f, 0.70f };
static const uint32_t FF_SETTLE_MS = 800;      // hold each FF level this long
static const float    RELAY_A      = 0.12f;    // PI-relay throttle amplitude about the bias
static const int      MIN_PERIODS  = 5;
static const uint32_t PI_TIMEOUT   = 6000;

enum Phase { P_SPINUP = 0, P_FF, P_PI, P_DONE_MOTOR };

static int      s_motor = -1;      // motor under test (-1 = idle)
static int      s_phase = 0;
static uint32_t s_t0    = 0;       // phase start
static float    s_out   = 0;       // current commanded throttle

// spin-up
static uint32_t s_spin_ok_since = 0;
// feedforward
static int      s_ff_i = 0;
static float    s_ff_acc = 0; static int s_ff_n = 0;
// PI relay
static float    s_pi_bias = 0.5f, s_pi_target = 0;
static int      s_sign = 0; static uint32_t s_last_cross = 0;
static float    s_amp = 0, s_period_sum = 0; static int s_period_n = 0;

// accumulators across motors
static float    s_acc_spin = 0, s_acc_idle = 0, s_acc_ff = 0, s_acc_kp = 0, s_acc_ki = 0;
static int      s_acc_n = 0;

static void beginMotor(int m) {
    s_motor = m; s_phase = P_SPINUP; s_out = 0; s_spin_ok_since = 0;
    s_ff_i = 0; s_ff_acc = 0; s_ff_n = 0;
    char b[24]; snprintf(b, sizeof(b), "MTune motor %d", m + 1); ui_log::set(b);
}

void start() {
    s_acc_spin = s_acc_idle = s_acc_ff = s_acc_kp = s_acc_ki = 0; s_acc_n = 0;
    beginMotor(0);
    s_t0 = millis();
}

void abort() { s_motor = -1; }

static void finishAndSave() {
    if (s_acc_n > 0) {
        g_params.mot_spin_min = s_acc_spin / s_acc_n;
        // NOTE: deliberately does NOT write RPM_IDLE. s_acc_idle is the *measured* rpm at
        // break-away (hundreds); writing it into the RPM_IDLE *setpoint* made the Pico
        // command that rpm whenever the sticks were centred
        // (pico/main.cpp: "if (centred && g_rpm_idle >= 1.0f) target = g_rpm_idle"), so
        // after one motor tune every arm spun all 8 thrusters with no pilot input —
        // and it persisted across reboots. Dynamic idle stays an opt-in the pilot sets.
        g_params.rpm_ff_a     = s_acc_ff   / s_acc_n;
        g_params.rpm_kp       = s_acc_kp   / s_acc_n;
        g_params.rpm_ki       = s_acc_ki   / s_acc_n;
        params::requestSaveAll();            // Core 0 writes flash
        char b[64];
        snprintf(b, sizeof(b), "MTune done: spin_min=%.2f ff=%.5f kp=%.2f ki=%.3f",
                 g_params.mot_spin_min, g_params.rpm_ff_a, g_params.rpm_kp, g_params.rpm_ki);
        ui_log::set("MTune complete");
        mav_stream::queueStatusText(MAV_SEVERITY_INFO, b);
    } else {
        ui_log::set("MTune: no data");
        mav_stream::queueStatusText(MAV_SEVERITY_WARNING,
                                    "MTune: no motor spun - check ESCs/arming");
    }
    s_motor = -1;
}

// Advance to the next motor, or finish.
static void nextMotor() {
    if (s_motor + 1 < NUM_THRUSTERS) { beginMotor(s_motor + 1); s_t0 = millis(); }
    else finishAndSave();
}

bool update(const int16_t rpm[NUM_THRUSTERS], float dt, uint32_t now,
            int& active_motor, float& out_throttle) {
    (void)dt;
    active_motor = s_motor; out_throttle = 0;
    if (s_motor < 0) return false;
    int m = s_motor;
    float meas = fabsf((float)rpm[m]);

    switch (s_phase) {
        // (1) Ramp throttle up until the motor reliably spins → min-spin + idle rpm.
        case P_SPINUP: {
            s_out += RAMP_STEP;
            if (meas > SPIN_RPM) { if (s_spin_ok_since == 0) s_spin_ok_since = now; }
            else s_spin_ok_since = 0;
            if (s_spin_ok_since && now - s_spin_ok_since > SPIN_HOLD_MS) {
                s_acc_spin += s_out;                 // min active throttle
                s_acc_idle += meas;                  // idle rpm at that throttle
                s_phase = P_FF; s_ff_i = 0; s_t0 = now; s_out = FF_LEVELS[0];
            } else if (s_out >= 0.95f) {             // never spun → skip this motor
                nextMotor();
            }
            break;
        }
        // (2) Hold a few throttle levels, fit FF_A = throttle / rpm.
        case P_FF: {
            s_out = FF_LEVELS[s_ff_i];
            if (now - s_t0 > FF_SETTLE_MS) {
                if (meas > SPIN_RPM) { s_ff_acc += s_out / meas; s_ff_n++; }
                s_ff_i++; s_t0 = now;
                if (s_ff_i >= 3) {
                    if (s_ff_n > 0) s_acc_ff += s_ff_acc / s_ff_n;
                    // Set up the PI relay about the mid level, target ~ that level's rpm.
                    s_pi_bias = FF_LEVELS[1];
                    s_pi_target = (s_ff_n > 0) ? (s_pi_bias / (s_ff_acc / s_ff_n)) : 2000.0f;
                    s_sign = 0; s_last_cross = 0; s_amp = 0; s_period_sum = 0; s_period_n = 0;
                    s_phase = P_PI; s_t0 = now;
                }
            }
            break;
        }
        // (3) Relay the throttle on the RPM-error sign; measure the limit cycle → kp/ki.
        case P_PI: {
            float err = (float)rpm[m] - s_pi_target;   // signed (positive rpm assumed)
            s_out = s_pi_bias + ((err > 0) ? -RELAY_A : RELAY_A);
            s_out = constrain(s_out, 0.05f, 0.95f);
            // Schmitt-trigger half-cycles of the error.
            float hyst = 0.03f * s_pi_target;
            int ns = s_sign;
            if (err >  hyst) ns = 1; else if (err < -hyst) ns = -1;
            if (ns != s_sign && s_sign != 0) {
                if (s_last_cross) { s_period_sum += (now - s_last_cross); s_period_n++; }
                s_last_cross = now;
                s_amp = max(s_amp, fabsf(err));
            }
            if (ns != 0) s_sign = ns;
            if (s_period_n >= MIN_PERIODS || now - s_t0 > PI_TIMEOUT) {
                if (s_period_n > 0 && s_amp > 1.0f) {
                    float Tu = 2.0f * (s_period_sum / s_period_n) / 1000.0f;
                    // Ultimate gain of the throttle→rpm plant, normalized to throttle units.
                    float Ku = 4.0f * RELAY_A / ((float)M_PI * (s_amp / g_params.rpm_max));
                    float Kp = constrain(0.33f * Ku, 0.0f, 4.0f);
                    float Ki = (Tu > 0) ? constrain(Kp / (0.5f * Tu), 0.0f, 1.0f) : 0.02f;
                    s_acc_kp += Kp; s_acc_ki += Ki;
                }
                s_acc_n++;               // count a completed motor
                nextMotor();
            }
            break;
        }
        default: nextMotor(); break;
    }

    active_motor = s_motor; out_throttle = (s_motor >= 0) ? s_out : 0;
    return (s_motor >= 0);
}

float progress() {
    if (s_motor < 0) return (s_acc_n > 0) ? 1.0f : 0.0f;
    return (float)s_motor / (float)NUM_THRUSTERS;
}

}  // namespace motor_tune
