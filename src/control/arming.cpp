// =============================================================================
//  control/arming — implementation
// =============================================================================

#include "control/arming.h"
#include "state_types.h"
#include "comms/params.h"

namespace arming {

bool canArm(const char** reason) {
    // ARMING_CHECK == 0 → skip pre-arm checks (arm always allowed).
    if (g_params.arming_check < 0.5f) return true;

    bool imu_ok = false;
    {
        StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(5));
        if (!lk.ok()) { if (reason) *reason = "state busy"; return false; }
        imu_ok = g_state.sensors.imu_valid;
    }
    if (!imu_ok) { if (reason) *reason = "IMU not healthy"; return false; }

    bool cal_active = false;
    {
        StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(5));
        if (lk.ok()) cal_active = (g_state.cal.routine != CalRoutine::NONE);
    }
    if (cal_active) { if (reason) *reason = "calibrating"; return false; }

    return true;
}

}  // namespace arming
