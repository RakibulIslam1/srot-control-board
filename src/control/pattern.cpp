// =============================================================================
//  control/pattern — implementation
// =============================================================================

#include "control/pattern.h"
#include "control/attitude_control.h"
#include "control/depth_control.h"
#include "comms/params.h"

namespace pattern {

static const float DEG2RAD = 0.01745329f;
static const float RAD2DEG = 57.2957795f;

static PatternStep s_step = PatternStep::IDLE;
static float       s_entry_depth = 0, s_entry_yaw = 0, s_headroom = 0;
static float       s_turn_done_deg = 0;
static uint32_t    s_step_start = 0;

// Shortest signed angular error a→b (rad), wrapped to [−π, π]. Guards NaN.
static float yawError(float target, float meas) {
    float e = target - meas;
    if (!isfinite(e)) return 0.0f;
    while (e >  M_PI) e -= 2 * M_PI;
    while (e < -M_PI) e += 2 * M_PI;
    return e;
}

// Level roll/pitch + hold a target heading; returns torques.
static void holdAttitude(float target_yaw, float meas_roll, float meas_pitch, float meas_yaw,
                         float gx, float gy, float gz, float dt,
                         float& r, float& p, float& y) {
    float roll_rate  = (0 - meas_roll)  * g_params.ang_rll_p;
    float pitch_rate = (0 - meas_pitch) * g_params.ang_pit_p;
    float yaw_rate   = yawError(target_yaw, meas_yaw) * g_params.ang_yaw_p;
    r = attitude::rateAxis(0, roll_rate, gx, dt);
    p = attitude::rateAxis(1, pitch_rate, gy, dt);
    y = attitude::rateAxis(2, yaw_rate, gz, dt);
}

void start(float headroom_m, float entry_depth, float entry_yaw) {
    s_step = PatternStep::TURN_360;
    s_entry_depth = entry_depth;
    s_entry_yaw = entry_yaw;
    s_headroom = headroom_m;
    s_turn_done_deg = 0;
    s_step_start = millis();
    attitude::reset();
    depth::reset(entry_depth);
}

PatternStep step() { return s_step; }

bool update(float meas_roll, float meas_pitch, float meas_yaw,
            float gx, float gy, float gz, float meas_depth,
            float dt, uint32_t now,
            float& out_roll, float& out_pitch, float& out_yaw, float& out_thr) {
    float tgt;
    switch (s_step) {
        case PatternStep::TURN_360: {
            // Spin yaw at STUNT_RATE, level roll/pitch, hold current depth.
            float spin = g_params.stunt_rate * DEG2RAD;
            out_roll  = attitude::rateAxis(0, (0 - meas_roll)  * g_params.ang_rll_p, gx, dt);
            out_pitch = attitude::rateAxis(1, (0 - meas_pitch) * g_params.ang_pit_p, gy, dt);
            out_yaw   = attitude::rateAxis(2, spin, gz, dt);
            depth::setTarget(s_entry_depth);
            out_thr = depth::update(0, meas_depth, dt, tgt);
            s_turn_done_deg += fabsf(gz) * dt * RAD2DEG;
            if (s_turn_done_deg >= 360.0f) { s_step = PatternStep::HEADROOM; s_step_start = now; }
            return true;
        }
        case PatternStep::HEADROOM: {
            depth::setTarget(s_entry_depth + s_headroom);
            holdAttitude(s_entry_yaw, meas_roll, meas_pitch, meas_yaw, gx, gy, gz, dt,
                         out_roll, out_pitch, out_yaw);
            out_thr = depth::update(0, meas_depth, dt, tgt);
            if (fabsf(meas_depth - (s_entry_depth + s_headroom)) < 0.1f) {
                s_step = PatternStep::TASK; s_step_start = now;
            }
            return true;
        }
        case PatternStep::TASK: {
            // Placeholder target task: hold pose+depth for 3 s.
            depth::setTarget(s_entry_depth + s_headroom);
            holdAttitude(s_entry_yaw, meas_roll, meas_pitch, meas_yaw, gx, gy, gz, dt,
                         out_roll, out_pitch, out_yaw);
            out_thr = depth::update(0, meas_depth, dt, tgt);
            if (now - s_step_start >= 3000) { s_step = PatternStep::RETURN; s_step_start = now; }
            return true;
        }
        case PatternStep::RETURN: {
            depth::setTarget(s_entry_depth);
            holdAttitude(s_entry_yaw, meas_roll, meas_pitch, meas_yaw, gx, gy, gz, dt,
                         out_roll, out_pitch, out_yaw);
            out_thr = depth::update(0, meas_depth, dt, tgt);
            bool depth_ok = fabsf(meas_depth - s_entry_depth) < 0.1f;
            bool yaw_ok = fabsf(yawError(s_entry_yaw, meas_yaw)) < 0.05f;
            if (depth_ok && yaw_ok) { s_step = PatternStep::DONE; }
            return true;
        }
        case PatternStep::DONE:
        default:
            out_roll = out_pitch = out_yaw = out_thr = 0;
            return false;
    }
}

}  // namespace pattern
