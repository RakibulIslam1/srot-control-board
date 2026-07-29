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
