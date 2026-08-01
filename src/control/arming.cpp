// =============================================================================
//  control/arming — implementation
// =============================================================================

#include "control/arming.h"
#include "state_types.h"
#include "comms/params.h"
#include "config.h"          // ESPNOW_STALE_MS — thruster-voltage freshness

namespace arming {

bool canArm(const char** reason) {
    // ARMING_CHECK == 0 → skip pre-arm checks (arm always allowed).
    if (g_params.arming_check < 0.5f) return true;

    bool imu_ok = false, leak = false, aux_fresh = false;
    float thr_v = 0.0f;
    {
        StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(5));
        if (!lk.ok()) { if (reason) *reason = "state busy"; return false; }
        const SensorState& s = g_state.sensors;
        imu_ok = s.imu_valid;
        leak   = s.leak;
        // Same freshness triple the control loop uses — a held voltage must not be read as
        // a live one, here least of all.
        aux_fresh = s.aux_valid && (s.aux_stamp_ms != 0) &&
                    (millis() - s.aux_stamp_ms < ESPNOW_STALE_MS);
        thr_v = aux_fresh ? s.aux_voltage : 0.0f;
    }
    if (!imu_ok) { if (reason) *reason = "IMU not healthy"; return false; }

    // Refuse to arm into an active leak. The leak failsafe would catch it a moment later and
    // surface the vehicle, so this is not new protection — but "arm refused: leak" is a
    // fact the operator can act on, where an unexplained surface right after arming is a
    // fault to diagnose. Gated on LEAK_EN like the failsafe, so an unwired sensor floating
    // active cannot block arming on a vehicle that never opted in.
    if (g_params.leak_en > 0.5f && leak) { if (reason) *reason = "leak detected"; return false; }

    // Refuse to arm on a thruster pack that is ALREADY below the failsafe threshold —
    // arming would spin up and immediately trip the battery failsafe into a SURFACE. Only
    // when a fresh ESP-NOW source exists: with no source thr_v is 0, and 0 means "no data",
    // never "flat" (the same convention the failsafe itself uses).
    //
    // Gated on FS_BAT_ENABLE exactly as the runtime failsafe is. Without that gate this
    // check would block arming for a threshold the pilot had explicitly switched off — a
    // pre-arm must never be stricter than the failsafe it is anticipating.
    if (g_params.fs_bat_enable > 0.5f && aux_fresh && thr_v > 1.0f &&
        g_params.fs_bat_voltage > 1.0f && thr_v < g_params.fs_bat_voltage) {
        if (reason) *reason = "thruster battery low";
        return false;
    }

    bool cal_active = false;
    {
        StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(5));
        if (lk.ok()) cal_active = (g_state.cal.routine != CalRoutine::NONE);
    }
    if (cal_active) { if (reason) *reason = "calibrating"; return false; }

    return true;
}

}  // namespace arming
