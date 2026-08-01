// =============================================================================
//  control/movement — implementation
// =============================================================================

#include "control/movement.h"
#include "control/stunt.h"
#include "comms/params.h"
#include <math.h>

namespace movement {

enum { PH_IDLE = 0, PH_CRUISE, PH_BRAKE, PH_TURN, PH_DIVE, PH_STYLE, PH_HOLD, PH_DONE };

static Type     s_type  = Type::NONE;
static uint8_t  s_phase = PH_IDLE;
static uint32_t s_start_ms = 0, s_dur_ms = 0, s_brake_ms = 0, s_timeout_ms = 60000;
static float    s_speed = 0, s_cur_speed = 0;   // commanded / ramped translation speed
static float    s_uf = 0, s_ul = 0;             // surge / sway axis signs
static float    s_yaw_rate = 0;                 // deg/s (turn or arc)
static float    s_target_yaw = 0;               // rad (absolute turn goal)
// Progress support for the two CLOSED-LOOP phases. TURN and DIVE finish on an error test, not
// a timer, so there is no duration to divide by — they both used to fall through progress()'s
// `default:` and report a constant 0.5. Those are the two verbs a mission most wants to wait
// on (JETSON_FEEDBACK.md §6), so MV_PROG and COMMAND_ACK.progress carried no information
// exactly where they mattered. `s_span` is the distance to cover latched at start; `s_remain`
// is refreshed by update() from the same error the completion test already computes.
static float    s_span = 0, s_remain = 0;
static float    s_depth_ramp = 0, s_depth_goal = 0;   // smoothed / commanded depth setpoint
static bool     s_running = false;
// Pending one-shot heading lock, emitted on the next update() and then cleared.
static float    s_yaw_lock = 0;
static bool     s_yaw_lock_pending = false;

static float wrapPi(float e) {
    if (!isfinite(e)) return 0.0f;
    while (e >  (float)M_PI) e -= 2.0f * (float)M_PI;
    while (e < -(float)M_PI) e += 2.0f * (float)M_PI;
    return e;
}
static uint32_t brakeMs(float cruise) { return (uint32_t)(g_params.move_brake_k * fabsf(cruise) * 1000.0f); }

// PILOT_YAW_RATE is a GCS-editable param with no lower clamp, and it is used here as a
// DIVISOR to normalise a deg/s rate into a -1..1 yaw demand. At 0 the division gives Inf
// (constrain() saves it) or, when the numerator is also 0, NaN — which stabilize() reads as
// a centred stick, so the commanded TURN/ARC silently never turns. attitude_control.cpp
// already guards the same param the same way; this brings movement into line.
static float pilotYawRate() {
    float r = g_params.pilot_yaw_rate;
    return (r > 1.0f) ? r : DEF_PILOT_YAW_RATE;
}

void enter(float cur_depth) {
    s_depth_ramp = s_depth_goal = cur_depth;
    s_type = Type::NONE; s_phase = PH_IDLE; s_running = false; s_cur_speed = 0;
    s_yaw_lock_pending = false;
}

void start(Type t, float primary, float speed, uint8_t submode, float aux,
           float timeout_s, float cur_yaw, float cur_depth) {
    s_type = t; s_start_ms = millis(); s_running = true;
    s_timeout_ms = (timeout_s > 0) ? (uint32_t)(timeout_s * 1000.0f) : 60000;
    s_speed = constrain(speed, 0.0f, g_params.move_cruise_max);
    s_depth_goal = s_depth_ramp;                 // hold depth unless DIVE
    // Capture the OUTGOING leg's travel axis and speed before they are cleared: STOP has to
    // brake along the axis it is stopping, and this reset ran before the type switch, so
    // PH_BRAKE computed `-0 * gain * 0` = zero thrust. "Brake to a halt" was a coast.
    // (JETSON_FEEDBACK.md §2 — and only the wire verb was affected, because abort() never
    // touched these fields, so internal preemption braked correctly the whole time.)
    const int8_t prev_uf = s_uf, prev_ul = s_ul;
    const float  prev_speed = s_cur_speed;
    s_uf = s_ul = 0; s_yaw_rate = 0;
    s_span = s_remain = 0;

    // Lock the heading this command starts with. Every verb that is not itself a
    // heading change gets it, so "go forward 3 s" cannot yaw away mid-leg: the
    // attitude loop is handed an explicit absolute target instead of relying on
    // whatever it last latched. ARC and TURN are excluded — they command yaw.
    switch (t) {
        case Type::FWD: case Type::BACK: case Type::LEFT: case Type::RIGHT:
        case Type::HOLD: case Type::STOP: case Type::DIVE:
            s_yaw_lock = cur_yaw; s_yaw_lock_pending = true;
            break;
        default:
            break;
    }

    switch (t) {
        case Type::FWD:   s_uf = +1; break;
        case Type::BACK:  s_uf = -1; break;
        case Type::LEFT:  s_ul = -1; break;
        case Type::RIGHT: s_ul = +1; break;
        case Type::ARC:   s_uf = +1; s_yaw_rate = aux; break;       // fwd + signed yaw rate
        case Type::TURN:
            s_yaw_rate = (speed > 1.0f) ? speed : g_params.move_yaw_rate;   // deg/s
            s_target_yaw = (submode == 1) ? (primary * (float)M_PI / 180.0f)               // absolute
                                          : wrapPi(cur_yaw + primary * (float)M_PI / 180.0f); // relative
            s_span = s_remain = fabsf(wrapPi(s_target_yaw - cur_yaw));   // for progress()
            s_phase = PH_TURN; return;
        case Type::DIVE:
            // Clamp to at or below the surface. JETSON_COMMS.md documented DIVE p2 as
            // "clamped at >= 0 — you cannot command above the surface" and it was not:
            // depth::setTarget() has no clamp either, and the AUTO path never reaches the
            // one inside depth::update(). A negative target produced sustained ascent — and
            // because the runaway guard references depth::target(), the guard TRACKED the
            // bad setpoint instead of catching it. (JETSON_FEEDBACK.md §7.)
            s_depth_goal = (primary > 0.0f) ? primary : 0.0f;
            s_span = s_remain = fabsf(s_depth_goal - cur_depth);          // for progress()
            s_phase = PH_DIVE; return;
        case Type::STYLE: {
            int n = (int)primary; if (n < 1) n = 1;
            stunt::start(StuntAxis::ROLL, n * 360.0f, 90.0f);
            s_phase = PH_STYLE; return;
        }
        case Type::STOP:
            // Restore the outgoing leg's axis and speed so PH_BRAKE has something to push
            // against — see the capture at the top of this function. Without this the verb
            // held zero translation for the brake window and the vehicle coasted, which on a
            // hull with no position or velocity estimate is unrecoverable state: nothing on
            // board or on the host knows how far it travelled.
            s_uf = prev_uf; s_ul = prev_ul;
            s_speed = prev_speed;                    // PH_BRAKE scales by s_speed, not s_cur_speed
            s_phase = PH_BRAKE; s_brake_ms = brakeMs(prev_speed); return;
        case Type::HOLD:
            // Station-keep: zero translation, hold the depth latched above and let the
            // attitude loop hold heading (a centred yaw demand latches the current
            // heading). `primary` = seconds to hold; 0 = until the timeout.
            // This used to fall through to `default:` — the command was ACKed and then
            // silently dropped to idle, so HOLD never actually did anything.
            s_dur_ms = (uint32_t)(primary * 1000.0f);
            s_cur_speed = 0;
            s_phase = PH_HOLD;
            return;
        default:          s_type = Type::NONE; s_phase = PH_IDLE; s_running = false; return;
    }
    // Translation (fwd/back/strafe/arc): cruise for `primary` seconds, then brake.
    s_dur_ms = (uint32_t)(primary * 1000.0f);
    s_cur_speed = 0;
    s_phase = PH_CRUISE;
}

void abort() {
    s_type = Type::STOP; s_phase = PH_BRAKE; s_start_ms = millis();
    s_brake_ms = brakeMs(s_cur_speed); s_running = true;
    // Don't re-latch a heading here: abort is a safety/preemption path and the current
    // hold target is already the right one to brake against.
}

// See the header for why this is not abort(). Straight to the idle terminal state so
// type() reports NONE on the very next publish and the ACK machine can resolve the move.
void cancel() {
    s_type = Type::NONE; s_phase = PH_IDLE; s_running = false;
    s_cur_speed = 0; s_uf = s_ul = 0; s_yaw_rate = 0;
    s_yaw_lock_pending = false;
}

Demand update(float roll, float pitch, float yaw, float gx, float gy, float gz,
              float depth, const int16_t rpm[NUM_THRUSTERS], float dt, bool& running) {
    (void)rpm;   // RPM no longer integrated (distance estimate removed; timer-based only)
    Demand d;
    const uint32_t now = millis();

    // Emit any pending heading lock exactly once, then clear it.
    if (s_yaw_lock_pending) {
        d.yaw_lock = s_yaw_lock;
        d.yaw_lock_valid = true;
        s_yaw_lock_pending = false;
    }

    // Smooth depth-target ramp toward the goal (no splash / lurch).
    const float dstep = g_params.move_depth_rate * dt;
    if      (s_depth_ramp < s_depth_goal) s_depth_ramp = min(s_depth_ramp + dstep, s_depth_goal);
    else                                  s_depth_ramp = max(s_depth_ramp - dstep, s_depth_goal);
    d.depth_target = s_depth_ramp;

    // Global timeout → brake out.
    if (s_running && s_phase != PH_BRAKE && (now - s_start_ms) > s_timeout_ms) {
        s_phase = PH_BRAKE; s_start_ms = now; s_brake_ms = brakeMs(s_cur_speed);
    }

    switch (s_phase) {
        case PH_CRUISE: {
            if (s_cur_speed < s_speed) s_cur_speed = min(s_cur_speed + g_params.move_accel * dt, s_speed);
            d.fwd = s_uf * s_cur_speed;
            d.lat = s_ul * s_cur_speed;
            if (s_yaw_rate != 0.0f)   // ARC: turn while translating
                d.yaw = constrain(s_yaw_rate / pilotYawRate(), -1.0f, 1.0f);
            if (now - s_start_ms >= s_dur_ms) { s_phase = PH_BRAKE; s_start_ms = now; s_brake_ms = brakeMs(s_speed); }
            break;
        }
        case PH_BRAKE: {
            const float g = g_params.move_brake_gain;
            d.fwd = -s_uf * g * s_speed;      // reverse along the travel axis to null momentum
            d.lat = -s_ul * g * s_speed;
            s_cur_speed = 0;
            if (now - s_start_ms >= s_brake_ms) s_phase = PH_DONE;
            break;
        }
        case PH_TURN: {
            float err = wrapPi(s_target_yaw - yaw);
            s_remain = fabsf(err);          // progress() divides this by the latched span
            if (fabsf(err) < 0.03f) {
                d.yaw = 0; s_phase = PH_DONE;
                // Hold the heading we were ASKED for, not the one the ~1.7° completion
                // threshold happened to stop at — otherwise "turn to 90°" settles
                // anywhere in 88.3..91.7° and then holds that error forever, and the
                // error compounds across a sequence of turns.
                d.yaw_lock = s_target_yaw;
                d.yaw_lock_valid = true;
            } else {
                d.yaw = constrain((err > 0 ? 1.0f : -1.0f) * s_yaw_rate / pilotYawRate(), -1.0f, 1.0f);
            }
            break;
        }
        case PH_DIVE:
            s_remain = fabsf(depth - s_depth_goal);   // progress() divides by the latched span
            if (fabsf(s_depth_ramp - s_depth_goal) < 0.01f && fabsf(depth - s_depth_goal) < 0.15f)
                s_phase = PH_DONE;
            break;
        case PH_STYLE: {
            bool spinning = stunt::update(roll, pitch, gx, gy, gz, dt, d.roll, d.pitch, d.yaw);
            d.spin = true;
            if (!spinning) { s_phase = PH_DONE; d.spin = false; }
            break;
        }
        case PH_HOLD:
            // Everything stays zero: d.fwd/lat/yaw default to 0, so the attitude loop
            // holds heading and the depth ramp holds the latched depth. A 0-second hold
            // runs until the global timeout brakes it out.
            if (s_dur_ms && (now - s_start_ms) >= s_dur_ms) s_phase = PH_DONE;
            break;
        case PH_DONE:
            s_type = Type::NONE; s_phase = PH_IDLE; s_running = false;
            break;
        case PH_IDLE:
        default:
            s_running = false;
            break;
    }
    running = s_running;
    return d;
}

Type    type()     { return s_type; }
uint8_t phase()    { return s_phase; }

float progress() {
    if (!s_running) return 1.0f;
    const uint32_t now = millis();
    switch (s_phase) {
        case PH_CRUISE: return s_dur_ms ? constrain((float)(now - s_start_ms) / (float)s_dur_ms, 0.0f, 0.9f) : 0.5f;
        case PH_BRAKE:  return 0.9f + (s_brake_ms ? 0.1f * constrain((float)(now - s_start_ms) / (float)s_brake_ms, 0.0f, 1.0f) : 0.1f);
        case PH_STYLE:  return stunt::progress();
        case PH_HOLD:   return s_dur_ms ? constrain((float)(now - s_start_ms) / (float)s_dur_ms, 0.0f, 0.99f) : 0.5f;
        // TURN and DIVE are closed-loop, so progress is the fraction of the latched span
        // covered, not elapsed time. Capped at 0.99 like PH_HOLD: only the terminal state
        // reports 1.0, so a client can distinguish "nearly there" from "done".
        case PH_TURN:
        case PH_DIVE:   return (s_span > 1e-3f)
                               ? constrain(1.0f - s_remain / s_span, 0.0f, 0.99f)
                               : 0.99f;   // asked to go where we already are
        default:        return 0.5f;
    }
}

}  // namespace movement
