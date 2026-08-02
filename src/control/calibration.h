#pragma once

// =============================================================================
//  control/calibration — sensor & motor calibration state machine
//
//  Routines are requested by MAVLink commands (which set g_state.cal.routine)
//  and advanced here from the control loop. Results land in g_state.cal and are
//  applied live by Task_SensorRead; PREFLIGHT_STORAGE persists them to NVS.
//
//  Runs on Core 1 — it must NOT send MAVLink directly (that is the comms core's
//  job). Progress is published via g_state.cal.progress and streamed by the
//  MAVLink task.
// =============================================================================

#include <Arduino.h>

namespace calibration {

// Load persisted calibration from NVS (ns "srot_cal") into g_state.cal. Call in
// setup() before tasks start.
void loadFromNVS();

// Outcome of the last saveToNVS(). Every Preferences write is now checked (they return 0
// on failure and every return was previously discarded) AND a representative subset is
// read back, because a write that "succeeds" into a full partition is exactly the case
// that made R14 take several rounds to find. 0 fails = the calibration really is on flash.
uint16_t    nvsFailCount();
const char* nvsFailKey();     // first key that failed, or nullptr
uint32_t    nvsSaveOkCount();

// Persist g_state.cal results to NVS.
void saveToNVS();

// Advance the active routine one control step. Reads the freshly published
// sensor sample (passed in to avoid re-locking). `now` = millis().
// If a motor drive is requested (MOTOR_DETECT/TEST), sets *test_motor/*test_thr.
void update(float roll, float pitch, float yaw,
            float gx, float gy, float gz,
            float grav_x, float grav_y, float grav_z,
            float mx, float my, float mz,
            float pressure_mbar, uint32_t now);

}  // namespace calibration
