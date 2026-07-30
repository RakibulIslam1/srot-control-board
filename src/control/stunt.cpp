// =============================================================================
//  control/stunt — implementation
// =============================================================================

#include "control/stunt.h"
#include "control/attitude_control.h"
#include "comms/params.h"

namespace stunt {

static const float DEG2RAD = 0.01745329f;
static const float RAD2DEG = 57.2957795f;

static StuntAxis s_axis = StuntAxis::NONE;
static float     s_target_deg = 0;
static float     s_done_deg = 0;
static float     s_rate_dps = 0;

void start(StuntAxis axis, float target_deg, float rate_dps) {
    s_axis = axis;
    s_target_deg = target_deg;
    s_done_deg = 0;
    s_rate_dps = rate_dps;
    attitude::reset();
}

void abort() { s_axis = StuntAxis::NONE; s_done_deg = 0; s_target_deg = 0; }

bool update(float meas_roll, float meas_pitch,
            float gx, float gy, float gz, float dt,
            float& out_roll, float& out_pitch, float& out_yaw) {
    float spin = s_rate_dps * DEG2RAD;   // rad/s on the active axis

    // Rate command per axis: active axis spins; passive angle axes hold level;
    // passive yaw holds zero rate.
    float roll_rate  = (s_axis == StuntAxis::ROLL)  ? spin : (0 - meas_roll)  * g_params.ang_rll_p;
    float pitch_rate = (s_axis == StuntAxis::PITCH) ? spin : (0 - meas_pitch) * g_params.ang_pit_p;
    float yaw_rate   = (s_axis == StuntAxis::YAW)   ? spin : 0.0f;

    out_roll  = attitude::rateAxis(0, roll_rate, gx, dt);
    out_pitch = attitude::rateAxis(1, pitch_rate, gy, dt);
    out_yaw   = attitude::rateAxis(2, yaw_rate, gz, dt);

    // Accumulate actual rotation from the measured rate on the active axis.
    float axis_rate = (s_axis == StuntAxis::ROLL) ? gx
                    : (s_axis == StuntAxis::PITCH) ? gy : gz;
    s_done_deg += fabsf(axis_rate) * dt * RAD2DEG;

    return s_done_deg < s_target_deg;
}

float progress() {
    return (s_target_deg > 0) ? constrain(s_done_deg / s_target_deg, 0.0f, 1.0f) : 1.0f;
}

}  // namespace stunt
