// =============================================================================
//  Task_SensorRead (Core 1) — BNO085 IMU + Bar30 depth on Bus0 (Wire, 400 kHz)
//  P1: real driver reads → SensorState. Real-time; never blocks long.
//
//  BNO085 polled every cycle (500 Hz task, 400 Hz reports, INT-gated). Bar30 is slow
//  (~a few ms/conversion) so it is decimated to ~20 Hz to protect IMU cadence.
//  Sensor-frame calibration offsets from CalState are applied before publishing.
// =============================================================================

#include "tasks.h"
#include "config.h"
#include "state_types.h"
#include "comms/params.h"
#include "drivers/bno085.h"
#include "drivers/bar30.h"
#include "drivers/analog_mon.h"

extern volatile uint32_t g_stack_hw[6];   // [1] = this task's stack high-water

void Task_SensorRead(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_SENSOR_HZ);
    TickType_t last = xTaskGetTickCount();
    uint32_t stk_cnt = 0;

    analog_mon::begin();

    // Sensor bring-up runs here (Core 1) so the 3 s BNO085 cold-boot delay does
    // NOT block task creation / the MAVLink heartbeat on Core 0.
    bno085::begin();
    bar30::begin();

    bno085::Sample imu;
    bar30::Sample  baro;
    analog_mon::Sample an;

    // Decimation counters (200 Hz base). Bar30 read busy-waits ~2-3 ms, so keep it
    // well off the 200 Hz path: ~20 Hz is plenty for depth.
    const uint16_t BARO_DIV  = TASK_SENSOR_HZ / 20;   // ~20 Hz when healthy
    const uint8_t  ANLG_DIV  = TASK_SENSOR_HZ / 10;   // ~10 Hz
    uint16_t baro_cnt = 0, baro_div = BARO_DIV;       // baro_div backs off if reads fail
    uint8_t  anlg_cnt = 0;

    for (;;) {
        // --- Snapshot the calibration inputs we need (short lock) ---
        float baro_zero = 1013.25f;
        Vec3f gyro_off;
        float level_r = 0, level_p = 0;
        {
            StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
            if (lk.ok()) {
                baro_zero = (g_state.cal.baro_zero_mbar > 0) ? g_state.cal.baro_zero_mbar : 1013.25f;
                gyro_off  = g_state.cal.gyro_offset;
                level_r   = g_state.cal.level_roll_off;
                level_p   = g_state.cal.level_pitch_off;
            }
        }

        // --- IMU every cycle ---
        bool imu_new = bno085::poll(imu);

        // --- Bar30 decimated, with back-off ---
        // A flaky/failing Bar30 can busy-wait each read; if it fails, back off to ~1/2 s so it
        // can't repeatedly stall Core 1 (which would starve the DShot task → Pico link flap).
        bool baro_new = false;
        if (++baro_cnt >= baro_div) {
            baro_cnt = 0;
            baro_new = bar30::read(baro, baro_zero);
            baro_div = baro_new ? BARO_DIV : (uint16_t)(TASK_SENSOR_HZ / 2);
        }

        // --- Battery / discrete decimated ---
        if (++anlg_cnt >= ANLG_DIV) {
            anlg_cnt = 0;
            analog_mon::read(an, g_params.pm1_vmult, BATT_CURR_MULT,
                             g_params.leak_en > 0.5f);
        }

        // --- Apply calibration (sensor frame) ---
        float gx = imu.gyro_x - gyro_off.x;
        float gy = imu.gyro_y - gyro_off.y;
        float gz = imu.gyro_z - gyro_off.z;

        // --- Publish under mtx_sensors (short lock, skip cycle if contended) ---
        // The extra braces MATTER: without them the StateLock lives to the end of the
        // for-body and the mutex is held across vTaskDelayUntil() below — i.e. this task
        // sleeps to its next deadline still owning mtx_sensors. That pushed the lock's
        // duty cycle to ~100 %, so every other task's timed take could miss, which is
        // where arming's "state busy" disarm came from. Keep the guard scoped.
        {
        StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(2));
        if (lk.ok()) {
            SensorState& s = g_state.sensors;
            if (imu_new) {
                s.quat_w = imu.qw; s.quat_x = imu.qx; s.quat_y = imu.qy; s.quat_z = imu.qz;
                // Apply AHRS mounting trim to roll/pitch. Mag/accel are published
                // raw; their calibration is applied by the consumers that use them.
                s.roll = imu.roll - level_r; s.pitch = imu.pitch - level_p; s.yaw = imu.yaw;
                s.lin_accel  = { imu.lin_x, imu.lin_y, imu.lin_z };
                s.gravity    = { imu.grav_x, imu.grav_y, imu.grav_z };
                s.gyro       = { gx, gy, gz };
                s.mag        = { imu.mag_x, imu.mag_y, imu.mag_z };
                s.cal_accel  = imu.acc_accuracy;
                s.cal_gyro   = imu.gyro_accuracy;
                s.cal_mag    = imu.mag_accuracy;
                s.imu_stamp_ms = millis();
            }
            // Freshness-based validity (false during a BNO reset-recovery window);
            // roll/pitch/yaw above hold their last-good value across a reset.
            s.imu_valid = bno085::attitudeValid();
            if (s.imu_valid) s.imu_ever_valid = true;
            s.bno_reset_count = bno085::resetCount();
            if (baro_new) {
                s.depth_m       = baro.depth_m;
                s.pressure_mbar = baro.pressure_mbar;
                s.water_temp_c  = baro.temp_c;
                s.baro_valid    = bar30::healthy();
                s.baro_stamp_ms = millis();
            }
            // Battery source (0=off, 1=local ADC, 2=ESP-NOW aux). There is one
            // local battery-voltage ADC (BATT_VOLT); the 2nd battery is ESP-NOW.
            // aux = THRUSTER battery (2nd board, ESP-NOW). Treat it as absent unless the
            // link is actually fresh — it holds its last value on link loss, and a held
            // voltage must not keep a failsafe (or motor compensation) alive.
            bool  aux_ok = s.aux_valid && (s.aux_stamp_ms != 0) &&
                           (millis() - s.aux_stamp_ms < ESPNOW_STALE_MS);
            float aux = aux_ok ? s.aux_voltage : 0.0f;
            int src1 = (int)g_params.pm1_src, src2 = (int)g_params.pm2_src;
            if      (src1 == 1) { s.pm1_voltage = an.volt; s.pm1_present = true; }
            else if (src1 == 2) { s.pm1_voltage = aux;     s.pm1_present = (aux > 0.05f); }
            else                { s.pm1_voltage = 0;       s.pm1_present = false; }
            if      (src2 == 2) { s.pm2_voltage = aux;     s.pm2_present = (aux > 0.05f); }
            else                { s.pm2_voltage = 0;       s.pm2_present = false; }  // no 2nd local ADC
            s.curr_a      = an.curr;
            s.leak        = an.leak;
            // s.kill_switch is set from the 2nd board over ESP-NOW, not here.
        }
        }   // <- mtx_sensors released HERE, before the delay below

        // Report stack headroom ~1 Hz (diagnostic for overflow reboots).
        if (++stk_cnt >= TASK_SENSOR_HZ) { stk_cnt = 0; g_stack_hw[1] = uxTaskGetStackHighWaterMark(nullptr); }

        vTaskDelayUntil(&last, period);
    }
}
