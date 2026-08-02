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
#include "comms/mavlink_bridge.h"   // MAV_SEVERITY_* for the notices below
#include "comms/mav_stream.h"       // queueStatusText: Core-1-safe alignment notices
#include "control/yaw_ref.h"
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

    // Decimation counters off the TASK_SENSOR_HZ (500 Hz) base. A Bar30 read busy-waits
    // ~2-3 ms, so keep it well off the fast path: ~20 Hz is plenty for depth.
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

        // --- One-shot magnetic yaw reference (MAG_YAW_REF) ---
        // Runs BEFORE the mtx_sensors lock on purpose: yaw_ref::update() takes mtx_cal
        // and mtx_control, and nesting those inside mtx_sensors is how you build a
        // lock-order deadlock. It only needs this cycle's local `imu` sample anyway.
        //
        // MAG_ALIGN is a momentary re-trigger. Honour it only while DISARMED — dropping
        // the offset mid-dive would step the heading-hold target and swing the vehicle.
        if (g_params.mag_align >= 0.5f) {
            bool armed_now = false;
            { StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
              if (lk.ok()) armed_now = g_state.control.armed; }
            if (armed_now) {
                mav_stream::queueStatusText(MAV_SEVERITY_WARNING,
                                            "MAG_ALIGN refused: disarm first");
            } else {
                yaw_ref::reset();
            }
            g_params.mag_align = 0.0f;      // momentary: consume the trigger either way
        }
        yaw_ref::update(imu.mag_x, imu.mag_y, imu.mag_z, imu.mag_accuracy,
                        imu.roll - level_r, imu.pitch - level_p, imu.yaw,
                        bno085::attitudeValid());
        // One line per transition, not per cycle — including refusals, so a failed
        // alignment is never silent (it would otherwise look like the param does nothing).
        if (yaw_ref::takeStateChange()) {
            yaw_ref::State st = yaw_ref::state();
            uint8_t sev = (st == yaw_ref::State::LOCKED)   ? MAV_SEVERITY_INFO
                        : (st == yaw_ref::State::SAMPLING) ? MAV_SEVERITY_INFO
                                                           : MAV_SEVERITY_WARNING;
            mav_stream::queueStatusText(sev, yaw_ref::stateText());
        }
        const float yaw_abs_off = yaw_ref::offset();   // 0 unless an alignment locked

        // --- Persist the BNO085's OWN calibration once it is worth persisting ---------
        //
        // This is the fix for "the magnetometer calibration is lost after a power cycle".
        // The part keeps its hard/soft-iron solution in its own flash and only writes it
        // when asked; nothing ever asked, so every boot restarted at accuracy 0.
        //
        // Trigger on accuracy 3 (fully converged) while DISARMED. Disarmed matters twice:
        // sh2_saveDcdNow() blocks on a round trip, and a solution converged next to eight
        // thrusters pulling current is not one worth freezing into flash.
        {
            static bool armed_cache = false;
            { StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
              if (lk.ok()) armed_cache = g_state.control.armed; }
            if (!armed_cache && imu.mag_accuracy >= 3 && !bno085::dcdSavedThisBoot()) {
                bno085::saveCalibration();
            }
        }
        if (bno085::takeDcdAnnounce()) {
            mav_stream::queueStatusText(MAV_SEVERITY_INFO,
                                        "IMU calibration saved to sensor flash");
        }

        // --- Stop the mag report once the earth reference is taken --------------------
        //
        // The mag never feeds attitude (6-DOF GAME_ROTATION_VECTOR), so after yaw_ref has
        // locked it has no consumer at all. Dropping the report cuts SHTP traffic on an
        // I2C bus polled without the INT line, where extra reports are the documented
        // desync/reset risk. MAG_ALIGN re-enables it (yaw_ref::reset() -> not LOCKED).
        //
        // Ordered AFTER the DCD save above on purpose: stop the report first and the
        // accuracy flag stops updating, so the save would never fire.
        {
            static bool mag_report_on = true;
            const bool want_on = (yaw_ref::state() != yaw_ref::State::LOCKED) ||
                                 !bno085::dcdSavedThisBoot();
            if (want_on != mag_report_on) {
                if (bno085::setMagReportEnabled(want_on)) {
                    mag_report_on = want_on;
                    mav_stream::queueStatusText(MAV_SEVERITY_INFO,
                        want_on ? "Mag report ON (re-aligning)"
                                : "Mag report OFF - heading is now inertial");
                }
            }
        }

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
                s.roll = imu.roll - level_r; s.pitch = imu.pitch - level_p;
                // Yaw carries the one-shot magnetic offset (0 unless MAG_YAW_REF locked),
                // so heading-hold, absolute TURNs, ATTITUDE and VFR_HUD all see ONE
                // consistent yaw. Wrap here, not at each consumer.
                {
                    float y = imu.yaw + yaw_abs_off;
                    while (y >  (float)M_PI) y -= 2.0f * (float)M_PI;
                    while (y < -(float)M_PI) y += 2.0f * (float)M_PI;
                    s.yaw = y;
                }
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
                s.baro_stamp_ms = millis();
            }
            // OUTSIDE the `if (baro_new)`: health must be able to go FALSE without a
            // successful read, or the flag can only ever be cleared by the very event that
            // is failing to happen. `healthy()` is now a live consecutive-failure model, not
            // the boot-time constant it used to be, so this is the point where a sensor that
            // has stopped answering actually withdraws depth from the rest of the stack.
            s.baro_valid = bar30::healthy();
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
            // src2 == 1 is the DEPRECATED alias for 2, not a second ADC. There is exactly one
            // voltage divider on this board, so "1" never had an implementation and fell
            // through to off IN SILENCE — blanking the OLED box, Battery 2, the mixer's
            // voltage linearisation AND the low-thruster-battery failsafe from one stale
            // parameter. Found set to 1 on the vehicle's own board (2026-08-02) while ESP-NOW
            // was up and delivering 15.4 V. Accepting it costs nothing and cannot pick a wrong
            // source, because ESP-NOW is the only source PM2 can physically have.
            if      (src2 == 2 || src2 == 1) { s.pm2_voltage = aux; s.pm2_present = (aux > 0.05f); }
            else                             { s.pm2_voltage = 0;   s.pm2_present = false; }
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
