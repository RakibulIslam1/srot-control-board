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
static float    s_depth_ramp = 0, s_depth_goal = 0;   // smoothed / commanded depth setpoint
static bool     s_running = false;

static float wrapPi(float e) {
    if (!isfinite(e)) return 0.0f;
    while (e >  (float)M_PI) e -= 2.0f * (float)M_PI;
    while (e < -(float)M_PI) e += 2.0f * (float)M_PI;
    return e;
}
static uint32_t brakeMs(float cruise) { return (uint32_t)(g_params.move_brake_k * fabsf(cruise) * 1000.0f); }

void enter(float cur_depth) {
    s_depth_ramp = s_depth_goal = cur_depth;
    s_type = Type::NONE; s_phase = PH_IDLE; s_running = false; s_cur_speed = 0;
}

void start(Type t, float primary, float speed, uint8_t submode, float aux,
           float timeout_s, float cur_yaw, float cur_depth) {
    (void)cur_depth;
    s_type = t; s_start_ms = millis(); s_running = true;
    s_timeout_ms = (timeout_s > 0) ? (uint32_t)(timeout_s * 1000.0f) : 60000;
    s_speed = constrain(speed, 0.0f, g_params.move_cruise_max);
    s_depth_goal = s_depth_ramp;                 // hold depth unless DIVE
    s_uf = s_ul = 0; s_yaw_rate = 0;

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
            s_phase = PH_TURN; return;
        case Type::DIVE:  s_depth_goal = primary; s_phase = PH_DIVE; return;
        case Type::STYLE: {
            int n = (int)primary; if (n < 1) n = 1;
            stunt::start(StuntAxis::ROLL, n * 360.0f, 90.0f);
            s_phase = PH_STYLE; return;
        }
        case Type::STOP:  s_phase = PH_BRAKE; s_brake_ms = brakeMs(s_cur_speed); return;
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
}

Demand update(float roll, float pitch, float yaw, float gx, float gy, float gz,
              float depth, const int16_t rpm[NUM_THRUSTERS], float dt, bool& running) {
    (void)rpm;   // RPM no longer integrated (distance estimate removed; timer-based only)
    Demand d;
    const uint32_t now = millis();

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
                d.yaw = constrain(s_yaw_rate / g_params.pilot_yaw_rate, -1.0f, 1.0f);
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
            if (fabsf(err) < 0.03f) { d.yaw = 0; s_phase = PH_DONE; }   // ~1.7° → hold new heading
            else d.yaw = constrain((err > 0 ? 1.0f : -1.0f) * s_yaw_rate / g_params.pilot_yaw_rate, -1.0f, 1.0f);
            break;
        }
        case PH_DIVE:
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
        default:        return 0.5f;
    }
}

}  // namespace movement
