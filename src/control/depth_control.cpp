// =============================================================================
//  control/depth_control — implementation
// =============================================================================

#include "control/depth_control.h"
#include "control/pid.h"
#include "comms/params.h"

namespace depth {

static PID   s_pid;
static float s_target = 0;
static const float STICK_DEADBAND = 0.05f;

void reset(float current_depth) {
    s_target = current_depth;
    s_pid.reset();
}

void setTarget(float depth_m) { s_target = depth_m; }
float target() { return s_target; }
float integral() { return s_pid.integral(); }
void  bleedIntegral(float amount) { s_pid.bleedIntegral(amount); }

float update(float stick_throttle, float meas_depth, float dt, float& target_out) {
    s_pid.setGains(g_params.depth_p, g_params.depth_i, g_params.depth_d);
    s_pid.setLimits(1.0f, 1.0f);

    // Active stick moves the target (up = ascend = decrease depth).
    if (fabsf(stick_throttle) > STICK_DEADBAND) {
        s_target -= stick_throttle * MAX_CLIMB_MS * dt;
        if (s_target < 0) s_target = 0;
    }

    target_out = s_target;
    // Positive error (target deeper than measured) → push down (negative heave,
    // since the mixer's throttle column is −1 on the vertical thrusters).
    return s_pid.update(s_target, meas_depth, dt);
}

}  // namespace depth
