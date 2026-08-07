// =============================================================================
//  comms/mav_stream — implementation
// =============================================================================

#include "comms/mav_stream.h"
#include "comms/mavlink_bridge.h"
#include "comms/mav_commands.h"
#include "comms/params.h"            // g_params.leak_en — the LEAK health bit's "enabled" state
#include "control/depth_control.h"   // preview()/lastError() — depth-loop observability
#include "control/mixer.h"           // mix() is pure — preview the depth chain disarmed
#include "control/yaw_ref.h"         // state() — is the heading absolute or boot-relative?
#include "drivers/bar30.h"           // jitterP2P() — why depth withdrew
#include "comms/ui_log.h"
#include "config.h"
#include "state_types.h"

// Diagnostics globals (defined in main.cpp).
extern volatile uint32_t g_stack_hw[6];

#ifndef MAV_CMD_SROT_MOVE
#define MAV_CMD_SROT_MOVE 31000   // custom high-level move (see docs/JETSON_COMMS.md)
#endif

namespace mav_stream {

// --- state snapshot (taken under mutexes, then used lock-free) ---------------
struct Snap {
    bool    armed = false;
    uint8_t mode = 0;
    float   roll = 0, pitch = 0, yaw = 0, gx = 0, gy = 0, gz = 0;
    float   lx = 0, ly = 0, lz = 0, grx = 0, gry = 0, grz = 0, mx = 0, my = 0, mz = 0;
    float   depth = 0, pressure = 0, wtemp = 0, throttle_pct = 0;
    float   pm1 = 0, pm2 = 0, curr = 0;
    bool    pm1_present = false, pm2_present = false;
    bool    leak = false, kill = false;
    uint8_t mag_acc = 0;        // BNO085 magnetic accuracy 0..3 (published as MAGACC)
    bool    imu_ok = false, depth_ok = false;   // real sensor health for SYS_STATUS
    float   stunt_prog = 0;
    bool    atune = false;
    int16_t rpm[NUM_THRUSTERS] = {0};   // per-thruster RPM (Pico backend; 0 otherwise)
    uint8_t esc_fault = 0;              // bit i = thruster i faulted
    uint8_t esc_present = 0;            // bit i = thruster i sending telemetry (connected)
    bool    thr_link_ok = false;        // Pico link alive
    // AUTO movement (SROT_MOVE progress/telemetry)
    bool     mv_active = false;
    uint8_t  mv_type = 0, mv_state = 0, mv_src_sys = 0, mv_src_comp = 0;
    float    mv_progress = 0;
    uint32_t mv_seq = 0, mv_done_seq = 0;
};

static void snapshot(Snap& s) {
    {
        StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(5));
        if (lk.ok()) {
            const SensorState& x = g_state.sensors;
            s.roll = x.roll; s.pitch = x.pitch; s.yaw = x.yaw;
            s.gx = x.gyro.x; s.gy = x.gyro.y; s.gz = x.gyro.z;
            s.lx = x.lin_accel.x; s.ly = x.lin_accel.y; s.lz = x.lin_accel.z;
            s.grx = x.gravity.x; s.gry = x.gravity.y; s.grz = x.gravity.z;
            s.mx = x.mag.x; s.my = x.mag.y; s.mz = x.mag.z;
            s.depth = x.depth_m; s.pressure = x.pressure_mbar; s.wtemp = x.water_temp_c;
            s.pm1 = x.pm1_voltage; s.pm2 = x.pm2_voltage; s.curr = x.curr_a;
            s.pm1_present = x.pm1_present; s.pm2_present = x.pm2_present;
            s.leak = x.leak; s.kill = x.kill_switch;
            s.mag_acc = x.cal_mag;
            s.imu_ok = x.imu_valid;
            s.depth_ok = x.baro_valid && (x.baro_stamp_ms != 0) &&
                         (millis() - x.baro_stamp_ms < DEPTH_STALE_MS);
        }
    }
    {
        StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(5));
        if (lk.ok()) {
            const ControlState& c = g_state.control;
            s.armed = c.armed;
            s.mode = (uint8_t)c.mode;
            s.throttle_pct = fabsf(c.out_throttle) * 100.0f;
            s.stunt_prog = c.stunt_progress;
            s.atune = c.autotune_active;
            s.mv_active = c.mv_active; s.mv_type = c.mv_type; s.mv_state = c.mv_state;
            s.mv_progress = c.mv_progress;
            s.mv_seq = c.mv_seq; s.mv_done_seq = c.mv_done_seq;
            s.mv_src_sys = c.mv_src_sys; s.mv_src_comp = c.mv_src_comp;
        }
    }
    {
        // mtx_thrusters is contended by two 500 Hz tasks, so a 3 ms miss is not rare.
        // Keep the LAST good values on a miss — the previous behaviour left the
        // default-constructed Snap in place and transmitted all-zero RPM, which made a
        // perfectly healthy thruster read 0 in the GCS at random.
        static int16_t  s_rpm_last[NUM_THRUSTERS] = {0};
        static uint8_t  s_fault_last = 0, s_present_last = 0;
        static bool     s_link_last = false;
        StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(3));
        if (lk.ok()) {
            for (int i = 0; i < NUM_THRUSTERS; ++i) s_rpm_last[i] = g_state.thrusters.rpm[i];
            s_fault_last   = g_state.thrusters.esc_fault;
            s_present_last = g_state.thrusters.esc_present;
            s_link_last    = g_state.thrusters.link_ok;
        }
        for (int i = 0; i < NUM_THRUSTERS; ++i) s.rpm[i] = s_rpm_last[i];
        s.esc_fault   = s_fault_last;
        s.esc_present = s_present_last;
        s.thr_link_ok = s_link_last;
    }
}

// (Thruster stalls are announced by Task_DShot_RMT, which is the only place the
//  commanded value and the measured rpm are in scope together — see the
//  "Thruster N STALLED: ..." message there. Nothing to do here.)

// Announce thrusters that return NO telemetry while armed — i.e. "not detected"
// (ESC unplugged, OR an ESC without bidirectional DShot). Distinct from a fault, and
// edge-triggered so it announces at most once per arm session (resets when disarmed
// or the link drops). This is why a bare bench (no ESCs) reads "not detected", not "fault".
static void reportEscNotDetected(const Snap& s) {
    static uint8_t s_ever_seen = 0;      // thrusters that have EVER reported since boot
    static uint8_t s_last_lost = 0;
    static bool    s_summary_sent = false;

    if (!(s.armed && s.thr_link_ok)) return;
    s_ever_seen |= s.esc_present;
    const uint8_t all = (uint8_t)((1u << NUM_THRUSTERS) - 1);

    // One summary at the first arm instead of a per-motor warning every session. On a
    // partly-populated frame (e.g. 4 of 8 wired) the old code warned about the same
    // missing thrusters forever, which just trains you to ignore the warnings.
    if (!s_summary_sent) {
        s_summary_sent = true;
        char wired[24] = "", absent[24] = "";
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            char t[4]; snprintf(t, sizeof(t), "%d", i + 1);
            char* dst = (s.esc_present & (1 << i)) ? wired : absent;
            if (dst[0]) strncat(dst, ",", 23 - strlen(dst));
            strncat(dst, t, 23 - strlen(dst));
        }
        char buf[80];
        if (absent[0]) snprintf(buf, sizeof(buf), "Thrusters wired: %s (absent: %s)",
                                wired[0] ? wired : "none", absent);
        else           snprintf(buf, sizeof(buf), "Thrusters wired: all %d", NUM_THRUSTERS);
        sendStatusText(absent[0] ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO, buf);
    }

    // After that, only a REGRESSION is worth a warning: a thruster that was talking to us
    // and then stopped. Never-present channels stay quiet.
    uint8_t lost = (uint8_t)(s_ever_seen & ~s.esc_present & all);
    uint8_t rising = (uint8_t)(lost & ~s_last_lost);
    s_last_lost = lost;
    for (int i = 0; i < NUM_THRUSTERS; ++i)
        if (rising & (1 << i)) {
            char buf[56];
            snprintf(buf, sizeof(buf), "Thruster %d LOST telemetry", i + 1);
            sendStatusText(MAV_SEVERITY_ERROR, buf);
        }
}

// Per-thruster RPM. Sent as BOTH ESC_STATUS and ESC_TELEMETRY_*, for a specific reason.
//
// ESC_STATUS (291) is a WIP message that upstream MAVLink REMOVED from `common`. pymavlink
// silently discards any message whose id is missing from its CRC-extra table — no error, no
// callback — so a standard companion cannot decode it in any dialect, and the board reporting
// RPM perfectly looks exactly like an ESC or Bluejay fault. The companion verified this on
// pymavlink 2.4.49 against common/ardupilotmega/all/development (JETSON_FEEDBACK.md §8).
//
// ESC_TELEMETRY_1_TO_4 (11030) / _5_TO_8 (11031) are in the ardupilotmega dialect we already
// vendor and include, carry exactly 4 ESCs each — an exact fit for 8 thrusters — and decode
// everywhere. ESC_STATUS is kept alongside because QGC/BlueOS render it and Bondor already
// consumes it; the pair costs ~140 B at 5 Hz, which is under 6 % of the link.
//
// Voltage/current/temperature are 0: the Pico link does not report them yet. They are real
// fields in ESC_TELEMETRY (unlike ESC_STATUS, where we were sending float zeros), so a
// consumer should treat 0 as "not instrumented", not "measured zero".
static void sendEscStatus(const Snap& s, uint32_t t) {
    for (int base = 0; base < NUM_THRUSTERS; base += 4) {
        int32_t rpm[4]; float volt[4] = {0}, curr[4] = {0};
        for (int i = 0; i < 4; ++i) rpm[i] = (base + i < NUM_THRUSTERS) ? s.rpm[base + i] : 0;
        mavlink_message_t m;
        mavlink_msg_esc_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
            (uint8_t)base, (uint64_t)t * 1000ULL, rpm, volt, curr);
        mav::tx(m);
    }

    // The decodable pair. Fields are per-ESC arrays of 4: temperature (degC), voltage (cV),
    // current (cA), totalcurrent (mAh), rpm, count. RPM is uint16 on this message, so a
    // reversing thruster reports magnitude — the sign lives in the commanded direction, which
    // the companion already knows.
    uint8_t  temp[4] = {0};
    uint16_t volt_cv[4] = {0}, curr_ca[4] = {0}, totc[4] = {0}, erpm[4] = {0}, cnt[4] = {0};
    for (int i = 0; i < 4; ++i) erpm[i] = (uint16_t)abs((int)s.rpm[i]);
    mavlink_message_t e1;
    mavlink_msg_esc_telemetry_1_to_4_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &e1,
        temp, volt_cv, curr_ca, totc, erpm, cnt);
    mav::tx(e1);

    for (int i = 0; i < 4; ++i)
        erpm[i] = (4 + i < NUM_THRUSTERS) ? (uint16_t)abs((int)s.rpm[4 + i]) : 0;
    mavlink_message_t e2;
    mavlink_msg_esc_telemetry_5_to_8_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &e2,
        temp, volt_cv, curr_ca, totc, erpm, cnt);
    mav::tx(e2);
}

// --- individual senders ------------------------------------------------------
static void sendHeartbeat(const Snap& s) {
    mavlink_message_t m;
    // base_mode advertises: we use a custom mode enum, we stabilize, and we accept
    // manual input. SAFETY_ARMED reflects the real arm state.
    uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                   MAV_MODE_FLAG_STABILIZE_ENABLED |
                   MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (s.armed) base |= MAV_MODE_FLAG_SAFETY_ARMED;
    uint8_t st = s.leak ? MAV_STATE_CRITICAL
               : (s.armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    // Honest identity: a GENERIC autopilot on a submarine frame. Hengla is not
    // ArduPilot and does not claim to be, which is why a GCS's ArduPilot-gated setup
    // pages stay empty (see docs/BLUEOS.md). custom_mode carries our own FlightMode.
    mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        MAV_TYPE_SUBMARINE, MAV_AUTOPILOT_GENERIC, base, (uint32_t)s.mode, st);
    mav::tx(m);
}

static void sendAttitude(const Snap& s, uint32_t t) {
    mavlink_message_t m;
    mavlink_msg_attitude_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        t, s.roll, s.pitch, s.yaw, s.gx, s.gy, s.gz);
    mav::tx(m);
}

static void sendScaledImu(const Snap& s, uint32_t t) {
    mavlink_message_t m;
    // accel (incl. gravity) in mg; gyro in mrad/s; mag in mgauss (uT×10).
    int16_t xa = (int16_t)((s.lx + s.grx) / 9.80665f * 1000.0f);
    int16_t ya = (int16_t)((s.ly + s.gry) / 9.80665f * 1000.0f);
    int16_t za = (int16_t)((s.lz + s.grz) / 9.80665f * 1000.0f);
    mavlink_msg_scaled_imu2_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m, t,
        xa, ya, za,
        (int16_t)(s.gx * 1000.0f), (int16_t)(s.gy * 1000.0f), (int16_t)(s.gz * 1000.0f),
        (int16_t)(s.mx * 10.0f), (int16_t)(s.my * 10.0f), (int16_t)(s.mz * 10.0f),
        // Temperature comes from the Bar30, not the IMU. 0 is the MAVLink-defined
        // "temperature not provided" sentinel for this field, so suppressing it when the
        // baro is unhealthy is spec-correct rather than a local convention. Sending the
        // number anyway is what let a -160 C reading look like a measurement.
        s.depth_ok ? (int16_t)(s.wtemp * 100.0f) : (int16_t)0);
    mav::tx(m);
}

static void sendScaledPressure(const Snap& s, uint32_t t) {
    mavlink_message_t m;
    mavlink_msg_scaled_pressure2_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m, t,
        s.pressure, 0.0f, (int16_t)(s.wtemp * 100.0f), 0);
    mav::tx(m);
}

static void sendVfrHud(const Snap& s) {
    mavlink_message_t m;
    float hdg = s.yaw * 57.2957795f;
    while (hdg < 0) hdg += 360.0f;
    while (hdg >= 360.0f) hdg -= 360.0f;
    mavlink_msg_vfr_hud_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        0.0f, 0.0f, (int16_t)hdg, (uint16_t)s.throttle_pct, -s.depth, 0.0f);
    mav::tx(m);
}

static void sendBattery(uint8_t id, float volts, float amps) {
    mavlink_message_t m;
    uint16_t voltages[10];
    voltages[0] = (uint16_t)(volts * 1000.0f);
    for (int i = 1; i < 10; ++i) voltages[i] = UINT16_MAX;
    uint16_t voltages_ext[4] = {0, 0, 0, 0};
    int16_t curr_ca = (amps > 0.001f) ? (int16_t)(amps * 100.0f) : -1;   // cA, -1 = unknown
    mavlink_msg_battery_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        id, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_UNKNOWN, INT16_MAX,
        voltages, curr_ca, -1, -1, -1, 0, MAV_BATTERY_CHARGE_STATE_UNDEFINED,
        voltages_ext, 0, 0);
    mav::tx(m);
}

static void sendSysStatus(const Snap& s) {
    mavlink_message_t m;
    // Advertise IMU + baro + control subsystems (mag omitted — it's disabled).
    uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                       MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_AHRS |
                       MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
                       MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                       MAV_SYS_STATUS_SENSOR_YAW_POSITION |
                       MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    // Health must reflect reality — this used to report every subsystem healthy
    // unconditionally, so the GCS showed a good AHRS even with the IMU dropped out.
    uint32_t health = sensors;
    if (!s.imu_ok) {
        health &= ~(uint32_t)(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                              MAV_SYS_STATUS_AHRS |
                              MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
                              MAV_SYS_STATUS_SENSOR_YAW_POSITION);
    }
    if (!s.depth_ok) {
        health &= ~(uint32_t)(MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                              MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL);
    }
    // Only report a battery voltage when PM1 is actually present.
    uint16_t vbat = s.pm1_present ? (uint16_t)(s.pm1 * 1000.0f) : UINT16_MAX;
    // LEAK on the EXTENDED sensor-health bitfield. It was carried only as a
    // NAMED_VALUE_FLOAT, multiplexed with MV_STATE/WTEMP/GAIN -- and pymavlink caches exactly
    // one message per msgid, so whichever arrived last won and leak DETECTION on the companion
    // was probabilistic. A flooding hull is not something to report by lottery. Here it is a
    // dedicated bit that a temperature reading cannot overwrite.
    //
    // Health-bit semantics are inverted by MAVLink convention: bit SET = healthy = DRY.
    // The vehicle-side failsafe (task_control_loop) and the pre-arm (arming.cpp) are
    // untouched and remain the actual safety mechanism -- this is a REPORTING fix.
    const uint32_t ext_present = MAV_SYS_STATUS_SENSOR_LEAK;
    const uint32_t ext_enabled = (g_params.leak_en > 0.5f) ? MAV_SYS_STATUS_SENSOR_LEAK : 0u;
    const uint32_t ext_health  = s.leak ? 0u : MAV_SYS_STATUS_SENSOR_LEAK;
    // EXTENSION_USED belongs in `present` only -- it announces that the *_extended words are
    // meaningful, and is not itself a sensor that can be enabled or healthy.
    // Arg order after msg: present, enabled, health, load, voltage, current,
    // battery_remaining, drop_rate, errors_comm, errors_count1..4, then the THREE extended
    // words. The trailing zeros here WERE those extended words -- they are replaced, not
    // appended to.
    mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        sensors | (uint32_t)MAV_SYS_STATUS_EXTENSION_USED, sensors, health, 0,
        vbat, -1, -1, 0, 0, 0, 0, 0, 0,
        ext_present, ext_enabled, ext_health);
    mav::tx(m);
}

static void sendPowerStatus(const Snap& s) {
    mavlink_message_t m;
    mavlink_msg_power_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        5000, (uint16_t)(s.pm2 * 1000.0f), 0);
    mav::tx(m);
}

static void sendNamed(uint32_t t, const char* name, float v) {
    mavlink_message_t m;
    mavlink_msg_named_value_float_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m, t, name, v);
    mav::tx(m);
}

void sendStatusText(uint8_t severity, const char* text) {
    ui_log::set(text);    // mirror onto the OLED log ticker
    mavlink_message_t m;
    mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        severity, text, 0, 0);
    mav::txReliable(m);   // user-facing messages (arm/cal/reset) shouldn't drop
}

// --- cross-core STATUSTEXT queue ---------------------------------------------
// The Core-1 control loop must not write UART0 (it would add a third writer to the
// tx mutex and stall the real-time loop), but it is where auto-disarms and failsafes
// are decided. So it queues the text here and Task_MAVLink drains it below.
static portMUX_TYPE s_qmux = portMUX_INITIALIZER_UNLOCKED;
static const uint8_t QCAP = 4;
static char    s_q[QCAP][64];
static uint8_t s_qsev[QCAP];
static uint8_t s_qhead = 0, s_qcount = 0;

void queueStatusText(uint8_t severity, const char* text) {
    if (!text) return;
    portENTER_CRITICAL(&s_qmux);
    if (s_qcount < QCAP) {
        uint8_t slot = (uint8_t)((s_qhead + s_qcount) % QCAP);
        strncpy(s_q[slot], text, sizeof(s_q[0]) - 1);
        s_q[slot][sizeof(s_q[0]) - 1] = '\0';
        s_qsev[slot] = severity;
        s_qcount++;
    }
    portEXIT_CRITICAL(&s_qmux);
}

// Drain the queue from Task_MAVLink. Copies one entry out per iteration so the
// spinlock is never held across the (slow) UART write.
static void flushQueuedStatusText() {
    for (;;) {
        char text[64];
        uint8_t sev;
        portENTER_CRITICAL(&s_qmux);
        if (s_qcount == 0) { portEXIT_CRITICAL(&s_qmux); return; }
        strncpy(text, s_q[s_qhead], sizeof(text));
        text[sizeof(text) - 1] = '\0';
        sev = s_qsev[s_qhead];
        s_qhead = (uint8_t)((s_qhead + 1) % QCAP);
        s_qcount--;
        portEXIT_CRITICAL(&s_qmux);
        sendStatusText(sev, text);
    }
}

// --- QGC compass-cal streaming (ardupilotmega MAG_CAL_*) ---------------------
static void sendMagProgress(uint8_t pct) {
    mavlink_message_t m;
    uint8_t completion_mask[10] = {0};
    mavlink_msg_mag_cal_progress_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        0 /*compass_id*/, 1 /*cal_mask*/, MAG_CAL_RUNNING_STEP_TWO,
        0 /*attempt*/, pct, completion_mask, 0, 0, 0);
    mav::tx(m);
}

static void sendMagReport(bool ok, float ox, float oy, float oz) {
    mavlink_message_t m;
    mavlink_msg_mag_cal_report_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        0 /*compass_id*/, 1 /*cal_mask*/, ok ? MAG_CAL_SUCCESS : MAG_CAL_FAILED,
        1 /*autosaved*/, ok ? 5.0f : 100.0f, ox, oy, oz,
        1.0f, 1.0f, 1.0f, 0, 0, 0, 0, 0, 0, 1.0f);
    mav::tx(m);
}

// Watch the calibration state and emit QGC-facing progress/result messages.
static void updateCalReports(uint32_t now) {
    static uint8_t s_last_seq = 0;
    static uint32_t s_last_prog = 0;

    CalRoutine routine = CalRoutine::NONE, last = CalRoutine::NONE;
    CalResult result = CalResult::NONE;
    uint8_t seq = 0;
    float prog = 0, ox = 0, oy = 0, oz = 0;
    int8_t mdir[NUM_THRUSTERS];
    {
        StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(3));
        if (!lk.ok()) return;
        routine = g_state.cal.routine;
        last = g_state.cal.last_routine;
        result = g_state.cal.result;
        seq = g_state.cal.result_seq;
        prog = g_state.cal.progress;
        ox = g_state.cal.mag_offset.x; oy = g_state.cal.mag_offset.y; oz = g_state.cal.mag_offset.z;
        for (int i = 0; i < NUM_THRUSTERS; ++i) mdir[i] = g_state.cal.motor_dir[i];
    }

    // Live compass progress while the MAG routine runs.
    if (routine == CalRoutine::MAG && now - s_last_prog >= 200) {
        s_last_prog = now;
        sendMagProgress((uint8_t)(prog * 100.0f));
    }

    // On completion (result_seq bumped), report the outcome.
    if (seq != s_last_seq) {
        s_last_seq = seq;
        bool ok = (result == CalResult::SUCCESS);
        switch (last) {
            case CalRoutine::MAG:
                sendMagReport(ok, ox, oy, oz);
                break;
            case CalRoutine::ACCEL_6PT:
                sendStatusText(MAV_SEVERITY_INFO, ok ? "Calibration successful" : "Calibration FAILED");
                break;
            case CalRoutine::GYRO:
                sendStatusText(MAV_SEVERITY_INFO, "Gyro calibration complete");
                break;
            case CalRoutine::BARO_ZERO:
                sendStatusText(MAV_SEVERITY_INFO, "Barometer calibration complete");
                break;
            case CalRoutine::LEVEL:
                sendStatusText(MAV_SEVERITY_INFO, "Level calibration complete");
                break;
            case CalRoutine::MOTOR_DETECT: {
                // Show what it decided, so the result is visible without digging through
                // NVS. NOTE: these multiply with the MOT_n_DIRECTION params — don't also
                // flip a param for a motor that detect already reversed.
                // Print DETECTED and EFFECTIVE. Printing only the detected value is how the
                // 2026-08-07 confusion happened: the log showed "+ + + + - - - -" while
                // Bondor's Setup tab showed MOT_n_DIRECTION unchanged, and the operator
                // reasonably read that as the log not being applied. Both were truthful --
                // detect writes CAL_MDIR and never touches the param -- but the two MULTIPLY,
                // and neither display showed the product that actually reaches the mixer.
                char b[96] = "MotorDetect:";
                for (int i = 0; i < NUM_THRUSTERS; ++i) {
                    char t[6];
                    snprintf(t, sizeof(t), " %c", mdir[i] < 0 ? '-' : '+');
                    strncat(b, t, sizeof(b) - strlen(b) - 1);
                }
                strncat(b, " | eff:", sizeof(b) - strlen(b) - 1);
                for (int i = 0; i < NUM_THRUSTERS; ++i) {
                    const int8_t pdir = (g_params.mot_dir[i] < 0) ? -1 : 1;
                    const int8_t eff  = (int8_t)(mdir[i] * pdir);
                    char t[6];
                    snprintf(t, sizeof(t), " %c", eff < 0 ? '-' : '+');
                    strncat(b, t, sizeof(b) - strlen(b) - 1);
                }
                sendStatusText(MAV_SEVERITY_INFO, b);
                break;
            }
            default:
                break;
        }
    }
}

// --- public ------------------------------------------------------------------
void sendHeartbeatNow() {
    Snap s; snapshot(s); sendHeartbeat(s);
}

void sendBootIdentity() {
    // Honest own-name boot banner — Hengla firmware, no autopilot spoof of any kind.
    mavlink_message_t m;
    char banner[50];
    snprintf(banner, sizeof(banner), "%s ready", HENGLA_FW_VERSION_STR);
    mavlink_msg_statustext_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        MAV_SEVERITY_INFO, banner, 0, 0);
    mav::tx(m);
}

void sendAutopilotVersion() {
    mavlink_message_t m;
    // flight_custom_version is 8 bytes, NOT null-terminated by the protocol — copy at
    // most 8 and leave the rest zeroed rather than relying on strncpy semantics.
    uint8_t custom[8] = {0};
    { const char* n = HENGLA_MAV_CUSTOM_VER;
      for (int i = 0; i < 8 && n[i]; ++i) custom[i] = (uint8_t)n[i]; }
    uint8_t zeros[8]   = {0};
    uint64_t caps = MAV_PROTOCOL_CAPABILITY_MAVLINK2 |
                    MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                    MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
                    MAV_PROTOCOL_CAPABILITY_MISSION_INT;
    // Real Hengla version. Vendor id = the SROT board, product id = this firmware.
    //
    // middleware_sw_version carries SROT_FW_BEHAVIOUR_REV. We have no middleware, so the
    // field was 0 and free, and this is the ONLY place a companion can learn what this
    // firmware DOES rather than what version string it calls itself.
    //
    // Why it has to be on the wire at all: duburi_ws removed its host-side MOVE_STOP brake
    // because rev 2 brakes on-board. Old firmware + new host = the hull COASTS on every
    // stop and abort, silently. Their check for that was a pytest that reads this repo's
    // headers off disk -- which skips on the Jetson, where the firmware source is not
    // checked out, i.e. it protects nothing on the vehicle. A number they can read at
    // connect time does.
    //
    // 0 therefore means "firmware older than 2026-08-01", NOT "unknown" -- a companion is
    // correct to treat 0 as rev 1 and fail closed. Never send 0 from a build that has a
    // meaningful revision.
    mavlink_msg_autopilot_version_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        caps, HENGLA_FW_VERSION_PACKED, SROT_FW_BEHAVIOUR_REV, 0, 0,
        custom, zeros, zeros, HENGLA_MAV_VENDOR_ID, HENGLA_MAV_PRODUCT_ID, 0, nullptr);
    mav::tx(m);
}

// COMMAND_ACK addressed back to the move's sender (Jetson), carrying progress %.
static void sendMoveAck(const Snap& s, uint8_t result, uint8_t progress) {
    mavlink_message_t m;
    mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        MAV_CMD_SROT_MOVE, result, progress, 0, s.mv_src_sys, s.mv_src_comp);
    mav::txReliable(m);   // ACK/completion must not drop — it drives the ROS action result
}

// Stream SROT_MOVE feedback: IN_PROGRESS ACKs (~3 Hz) with % + live state via
// NAMED_VALUE_FLOAT, then one ACCEPTED ACK on completion → the companion's move-action result.
static void updateMove(const Snap& s, uint32_t now) {
    static uint32_t s_seq = 0;          // seq currently being tracked (0 = idle)
    static uint32_t t_ack = 0;
    static bool     s_seen_active = false;   // the control loop confirmed it started
    static uint32_t s_track_ms = 0;
    // Who sent the move we are tracking. Cached because ControlState's mv_src_* is
    // OVERWRITTEN the instant a preempting command arrives — so by the time we notice the
    // preemption, the address of the command being cancelled is already gone.
    static uint8_t  s_src_sys = 0, s_src_comp = 0;
    static bool     s_resolved = false;   // this seq already got its terminal ACK

    // A new (or preempting) command started → track it, ACK immediately.
    // Track on the seq change ALONE — do not require mv_active. A short move can start
    // AND finish before this function next runs (it used to be skipped entirely for the
    // duration of a parameter download), and mv_active is false again by then. Gating on
    // it left s_seq at 0, so no terminal ACK was ever sent and the companion's action
    // waited forever on a result that had already happened.
    if (s.mv_seq != 0 && s.mv_seq != s_seq) {
        // PREEMPTION: a move was still being tracked and a new one displaced it. Resolve the
        // old one before adopting the new, or its action never gets a terminal result and
        // hangs — the same failure as a lost completion ACK, just triggered by the
        // documented "a new move preempts the running one" behaviour rather than by timing.
        //
        // Report what actually happened. If the control loop had already latched the old
        // sequence as COMPLETE (mv_done_seq) in the same window the new command arrived, then
        // it finished and was not cancelled — saying CANCELLED there would be a lie the
        // companion may well act on.
        // ...but only if it had NOT already been resolved. A command that already got its
        // terminal ACK is finished business; ACKing it again as CANCELLED would contradict
        // the ACCEPTED the companion already acted on.
        if (s_seq != 0 && !s_resolved) {
            const bool old_completed = (s.mv_done_seq == s_seq);
            mavlink_message_t m;
            mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
                MAV_CMD_SROT_MOVE,
                old_completed ? MAV_RESULT_ACCEPTED : MAV_RESULT_CANCELLED,
                old_completed ? 100 : 0, 0, s_src_sys, s_src_comp);
            mav::txReliable(m);
        }
        s_seq = s.mv_seq; t_ack = 0; s_seen_active = false; s_track_ms = now;
        s_resolved = false;
        s_src_sys = s.mv_src_sys; s_src_comp = s.mv_src_comp;
    }
    // Nothing to track, or this command is already finished.
    //
    // `s_resolved` replaces the old `s_seq = 0` sentinel, which caused an INFINITE terminal-ACK
    // LOOP: zeroing s_seq made the very next cycle see `mv_seq != s_seq` and re-adopt the SAME
    // completed command as if it were new, whereupon `mv_done_seq == s_seq` was still true, so
    // it re-sent the terminal and zeroed s_seq again. Measured on the bench at ~14 Hz,
    // indefinitely, after every completed move — on a 115200 link shared with all telemetry.
    // Keeping s_seq means the re-adopt test only fires for a genuinely new sequence.
    if (s_seq == 0 || s_resolved) return;
    if (s.mv_active) s_seen_active = true;

    // Never resolve on "!mv_active" until we have actually SEEN it active: on the first
    // cycle after a command, mv_active is still false simply because the control loop has
    // not run yet, and treating that as completion would ACK 100% before the move began.
    // mv_done_seq is latched by the control loop, so a completion we missed is still
    // recoverable however late we get here.
    //
    // The escape hatch: the command handler forces AUTO, but the control loop REFUSES
    // AUTO when there is no depth sensor and falls back to STABILIZE. The move then never
    // starts and nothing would ever resolve it — so fail it explicitly instead of
    // streaming IN_PROGRESS for ever.
    if (!s_seen_active && s.mode != (uint8_t)FlightMode::AUTO && (now - s_track_ms) > 500) {
        sendMoveAck(s, MAV_RESULT_FAILED, 0);
        sendNamed(now, "MV_STATE", 0);
        s_resolved = true;
        return;
    }

    bool done = (s.mv_done_seq == s_seq) || (s_seen_active && !s.mv_active);
    if (done) {
        sendMoveAck(s, MAV_RESULT_ACCEPTED, 100);
        sendNamed(now, "MV_STATE", 0);
        sendNamed(now, "MV_PROG",  1.0f);
        s_resolved = true;
        return;
    }
    if (now - t_ack >= 300) {
        t_ack = now;
        uint8_t prog = (uint8_t)constrain(s.mv_progress * 100.0f, 0.0f, 99.0f);
        sendMoveAck(s, MAV_RESULT_IN_PROGRESS, prog);
        sendNamed(now, "MV_STATE", (float)s.mv_state);
        sendNamed(now, "MV_PROG",  s.mv_progress);
        sendNamed(now, "MV_TYPE",  (float)s.mv_type);
    }
}

// --- runtime stream rates (MAV_CMD_SET_MESSAGE_INTERVAL, 511) -----------------
//
// These were hardcoded literals, which made the fixed 10 Hz ATTITUDE the hard ceiling for
// every loop the companion runs — it could not turn it up for a control loop or down for a
// slow link, and with neither 511 nor REQUEST_DATA_STREAM (66) implemented there was no
// standard mechanism at all (JETSON_FEEDBACK.md §5).
//
// `ms` is the live interval; `def_ms` is what a `0` request restores. 0 in `ms` means the
// stream is DISABLED (a -1 request), which is why the send tests below check it.
struct StreamRate { uint32_t msgid; uint16_t ms; uint16_t def_ms; };
static StreamRate s_rates[] = {
    { MAVLINK_MSG_ID_HEARTBEAT,         1000, 1000 },
    { MAVLINK_MSG_ID_SYS_STATUS,         500,  500 },
    { MAVLINK_MSG_ID_ATTITUDE,           100,  100 },
    { MAVLINK_MSG_ID_SCALED_IMU2,        100,  100 },
    { MAVLINK_MSG_ID_SCALED_PRESSURE2,   200,  200 },
    { MAVLINK_MSG_ID_VFR_HUD,            200,  200 },
    { MAVLINK_MSG_ID_BATTERY_STATUS,    1000, 1000 },
    { MAVLINK_MSG_ID_POWER_STATUS,      1000, 1000 },
    { MAVLINK_MSG_ID_ESC_STATUS,         200,  200 },
    { MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,  500,  500 },
};
static const int N_RATES = (int)(sizeof(s_rates) / sizeof(s_rates[0]));

// Floor on any requested interval. 20 ms = 50 Hz; ATTITUDE at that rate is ~1.8 kB/s, about
// 16 % of a 115200 link. Without a floor a companion can ask for 1 kHz and starve the
// PARAM_VALUE and COMMAND_ACK traffic that missions depend on.
static const uint16_t RATE_MIN_MS = 20;
static const uint16_t RATE_MAX_MS = 10000;

static uint16_t rateMs(uint32_t msgid) {
    for (int i = 0; i < N_RATES; ++i) if (s_rates[i].msgid == msgid) return s_rates[i].ms;
    return 0;
}

bool setMessageInterval(uint32_t msgid, int32_t interval_us) {
    // HEARTBEAT may be re-rated but never disabled, and the REFUSAL is explicit rather than
    // being swallowed by the output-side fallback. Accepting a disable and then transmitting
    // anyway would make GET_MESSAGE_INTERVAL report a state the vehicle is not in — the
    // companion would believe it had silenced the one message its liveness test keys on.
    if (msgid == MAVLINK_MSG_ID_HEARTBEAT && interval_us < 0) return false;
    for (int i = 0; i < N_RATES; ++i) {
        if (s_rates[i].msgid != msgid) continue;
        if (interval_us < 0)       s_rates[i].ms = 0;                  // disable
        else if (interval_us == 0) s_rates[i].ms = s_rates[i].def_ms;  // restore default
        else {
            uint32_t ms = (uint32_t)interval_us / 1000u;
            if (ms < RATE_MIN_MS) ms = RATE_MIN_MS;
            if (ms > RATE_MAX_MS) ms = RATE_MAX_MS;
            s_rates[i].ms = (uint16_t)ms;
        }
        return true;
    }
    return false;   // not a stream we own -> caller replies DENIED
}

int32_t getMessageInterval(uint32_t msgid) {
    for (int i = 0; i < N_RATES; ++i) {
        if (s_rates[i].msgid != msgid) continue;
        return s_rates[i].ms ? (int32_t)s_rates[i].ms * 1000 : -1;   // -1 = disabled
    }
    return 0;   // unknown -> "default", per the MAVLink convention
}

// CONFIG BANNER — announce the stored config that changes how the vehicle MOVES.
//
// Asked for by duburi_ws (Round 8 §8.3/§8.4) after a real near-miss: FRAME_REVERSE lives in
// NVS, so SROT_FW_BEHAVIOUR_REV cannot describe it. Five commits shipped inside one 6->7 bump
// and their drift suite passed clean -- correctly, since it reads headers and no constant
// moved -- while the meaning of every axis command on that hull had changed. A monotonic
// integer describes COMPILED behaviour; on an architecture where we own every control loop,
// CONFIGURATION IS BEHAVIOUR, and it has to be visible to whoever is deciding whether to arm.
//
// Repeated, not boot-only. A boot-only line is missed by every GCS that connects after the
// board powers up, which is the normal case. The INFO line stops after a few repeats so a
// correctly configured vehicle goes quiet; the WARNING repeats for as long as the failsafe
// is actually widened, because a widened safety margin should stay noisy.
static void configBanner(uint32_t now) {
    static uint32_t t_last  = 0;
    static uint8_t  info_n  = 0;
    const uint32_t  PERIOD_MS = 60000;

    if (t_last != 0 && (now - t_last) < PERIOD_MS) return;
    t_last = now;

    const bool wildcard = (g_params.fs_gcs_enable > 0.5f) &&
                          ((int)lroundf(g_params.fs_gcs_compid) == 0);

    if (info_n < 3) {
        info_n++;
        char buf[90];
        int8_t d[NUM_THRUSTERS];
        { StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
          for (int i = 0; i < NUM_THRUSTERS; ++i)
              d[i] = lk.ok() ? g_state.cal.motor_dir[i] : 1; }
        snprintf(buf, sizeof(buf),
                 "CFG rev%d FRAME_REVERSE=%d MOT_DIR=%d,%d,%d,%d,%d,%d,%d,%d",
                 SROT_FW_BEHAVIOUR_REV,
                 g_params.frame_reverse > 0.5f ? 1 : 0,
                 (int)lroundf(g_params.mot_dir[0]) * d[0], (int)lroundf(g_params.mot_dir[1]) * d[1],
                 (int)lroundf(g_params.mot_dir[2]) * d[2], (int)lroundf(g_params.mot_dir[3]) * d[3],
                 (int)lroundf(g_params.mot_dir[4]) * d[4], (int)lroundf(g_params.mot_dir[5]) * d[5],
                 (int)lroundf(g_params.mot_dir[6]) * d[6], (int)lroundf(g_params.mot_dir[7]) * d[7]);
        queueStatusText(MAV_SEVERITY_INFO, buf);
    }

    // A bench affordance that became permanent is the failure this catches: with COMPID = 0
    // any station satisfies companionLost(), so in water a dead Jetson is indistinguishable
    // from a live GCS and the vehicle station-keeps instead of surfacing.
    if (wildcard) {
        queueStatusText(MAV_SEVERITY_WARNING,
                        "FS_GCS_COMPID=0 (wildcard) - ANY station feeds the companion "
                        "failsafe. Set 191 before diving.");
    }
}

void update(uint32_t now) {
    configBanner(now);
    static uint32_t t_hb = 0, t_sys = 0, t_att = 0, t_imu = 0,
                    t_prs = 0, t_hud = 0, t_bat = 0, t_pwr = 0, t_nvf = 0, t_diag = 0, t_esc = 0;
    // Live intervals. HEARTBEAT is deliberately NOT allowed to be disabled: a GCS that turned
    // it off would look identical to a dead vehicle, and the companion's own liveness test
    // keys on it.
    const uint16_t iv_hb  = rateMs(MAVLINK_MSG_ID_HEARTBEAT) ? rateMs(MAVLINK_MSG_ID_HEARTBEAT) : 1000;
    const uint16_t iv_sys = rateMs(MAVLINK_MSG_ID_SYS_STATUS);
    const uint16_t iv_att = rateMs(MAVLINK_MSG_ID_ATTITUDE);
    const uint16_t iv_imu = rateMs(MAVLINK_MSG_ID_SCALED_IMU2);
    const uint16_t iv_prs = rateMs(MAVLINK_MSG_ID_SCALED_PRESSURE2);
    const uint16_t iv_hud = rateMs(MAVLINK_MSG_ID_VFR_HUD);
    const uint16_t iv_bat = rateMs(MAVLINK_MSG_ID_BATTERY_STATUS);
    const uint16_t iv_pwr = rateMs(MAVLINK_MSG_ID_POWER_STATUS);
    const uint16_t iv_esc = rateMs(MAVLINK_MSG_ID_ESC_STATUS);
    const uint16_t iv_nvf = rateMs(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
    Snap s; snapshot(s);

    flushQueuedStatusText();   // auto-disarm / failsafe notices from the Core-1 loop
    updateCalReports(now);
    reportEscNotDetected(s);   // edge-triggered "not detected" (no telemetry) announcements

    // Move progress/completion ACKs run BEFORE the param-download throttle below. They
    // are ~40 B at 3 Hz, so the bandwidth argument for suppressing them does not apply —
    // and suppressing them meant a move that completed inside a download window never got
    // its terminal ACK at all, hanging the companion's action. A GCS opening its Setup tab
    // (which triggers a download) must not be able to strand a move in flight.
    updateMove(s, now);        // AUTO move progress/completion ACKs (SROT_MOVE)

    // While the GCS is downloading the parameter list, throttle everything except the
    // heartbeat so the download completes fast (→ setup tabs latch) and doesn't starve
    // the reliable PARAM_VALUE sends of TX bandwidth.
    // ESC_STATUS is deliberately EXEMPT: it is only ~70 B x2 at 5 Hz, and suppressing it
    // here is why the motor-test tab read 0 rpm — Bondor pulls the param list when you
    // open Setup, which is exactly when you are watching the motors.
    //
    // ATTITUDE is exempt for the same class of reason: it is the companion's control-loop
    // input, and a 10-15 s blackout mid-mission is a 10-15 s hole in every host-side loop
    // that closes on attitude. An operator opening Bondor's Setup tab must not be able to
    // blind an autonomous run. Anything a MISSION depends on belongs above this guard;
    // anything only a human reads belongs below it.
    if (mav_commands::paramDownloadActive()) {
        if (now - t_hb >= iv_hb) { t_hb = now; sendHeartbeat(s); }
        if (iv_esc && now - t_esc >= iv_esc) { t_esc = now; sendEscStatus(s, now); }
        if (iv_att && now - t_att >= iv_att) { t_att = now; sendAttitude(s, now); }
        return;
    }

    if (now - t_hb  >= iv_hb) { t_hb  = now; sendHeartbeat(s); }
    if (iv_sys && now - t_sys >= iv_sys) { t_sys = now; sendSysStatus(s); }
    if (iv_att && now - t_att >= iv_att) { t_att = now; sendAttitude(s, now); }
    if (iv_imu && now - t_imu >= iv_imu) { t_imu = now; sendScaledImu(s, now); }
    // SCALED_PRESSURE2 is SUPPRESSED, not zeroed, when the baro is unhealthy or stale: the
    // message has no validity field, and press_abs is the same corrupted quantity as depth.
    // Sending it would be asserting a pressure we know is unreliable. Absence is the signal.
    if (iv_prs && now - t_prs >= iv_prs && s.depth_ok) { t_prs = now; sendScaledPressure(s, now); }
    if (iv_hud && now - t_hud >= iv_hud) { t_hud = now; sendVfrHud(s); }
    if (iv_bat && now - t_bat >= iv_bat) {
        t_bat = now;
        if (s.pm1_present) sendBattery(0, s.pm1, s.curr);   // current on the main battery
        if (s.pm2_present) sendBattery(1, s.pm2, 0);
    }
    if (iv_pwr && now - t_pwr >= iv_pwr) { t_pwr = now; sendPowerStatus(s); }
    if (iv_esc && now - t_esc >= iv_esc) { t_esc = now; sendEscStatus(s, now); }  // per-thruster RPM
    if (iv_nvf && now - t_nvf >= iv_nvf) {
        t_nvf = now;
        // LEAK also rides SYS_STATUS's extended health bits now (see sendSysStatus). Kept here
        // for ONE release as a deprecated duplicate so the companion has a migration window --
        // pymavlink caches one message per msgid, so LEAK sharing NAMED_VALUE_FLOAT with
        // MV_STATE/WTEMP/GAIN made leak REPORTING probabilistic. Their finding, and a good one.
        sendNamed(now, "LEAK", s.leak ? 1.0f : 0.0f);
        // Suppressed when the baro is unhealthy or stale. A consumer caching NAMED_VALUE_FLOAT
        // simply ages it out; a fabricated -160 it cannot distinguish from a measurement.
        if (s.depth_ok) sendNamed(now, "WTEMP", s.wtemp);
        sendNamed(now, "STUNT_PRG", s.stunt_prog);
        sendNamed(now, "ATUNE", s.atune ? 1.0f : 0.0f);
        sendNamed(now, "KILL", s.kill ? 1.0f : 0.0f);
        sendNamed(now, "CURR", s.curr);
        // Live pilot gain — the gain buttons change this at runtime, so a joystick UI has
        // no other way to know the current value (JS_GAIN_DEFAULT is only the boot value).
        sendNamed(now, "GAIN", mav_commands::pilotGain());
        // BNO085 magnetic accuracy 0..3. Published because "mag not calibrated" was
        // previously an unfalsifiable claim: the operator could calibrate repeatedly with no
        // way to see whether the number the refusal keys on was moving at all. yaw_ref needs
        // >= 2, and the DCD save fires at >= 2.
        sendNamed(now, "MAGACC", (float)s.mag_acc);
        // YAW_REF — the alignment state itself (yaw_ref::State enum), which nothing published.
        //
        // Without it there is no way to tell an ABSOLUTE heading from a relative one. If the
        // reference never locked, ATTITUDE.yaw is measured from the BNO's boot orientation and
        // an absolute MOVE_TURN turns to a meaningless number, silently. duburi_ws needs this
        // to gate absolute turns and fall back to relative on its own.
        //
        // MAGACC is NOT a proxy for it and must not be used as one (see yaw_ref.cpp:179-182):
        // the alignment is protected by the |B| plausibility band and the sample-agreement
        // test, neither of which consults the sensor's self-assessment, and both of which can
        // refuse at MAGACC 3. With a stored CAL_MAG_* need_acc drops to 0, so accuracy stops
        // correlating with lock at all.
        //
        // 0 IDLE, 1 SAMPLING, 2 LOCKED, 3 REFUSED_CAL, 4 REFUSED_FIELD, 5 REFUSED_NOISE.
        // Only 2 means the heading is absolute.
        sendNamed(now, "YAW_REF", (float)yaw_ref::state());
        // Baro peak-to-peak over the jitter window. Published unconditionally (NOT gated on
        // depth_ok) because its whole job is to explain WHY depth just withdrew -- gating it
        // on the health it reports on would hide it exactly when it matters.
        sendNamed(now, "BARO_P2P", bar30::jitterP2P());
        // COMP_SEEN: has the CONFIGURED companion (FS_GCS_SYSID/COMPID) been heard since
        // boot? 0 = never, 1 = yes.
        //
        // This exists so a GCS can tell which side of the seen-then-lost latch it is on.
        // companionLost() only arms once the named companion has actually appeared, so a
        // freshly-booted board with only Bondor connected can arm perfectly safely -- but
        // Bondor had no way to know that, so it warned on EVERY session and offered to
        // wildcard the failsafe (bench mode) even when nothing was wrong. A warning that
        // fires when it does not apply teaches operators to widen a failsafe by reflex.
        //
        // With this, the warning and the bench-mode offer can be gated on the one condition
        // that actually matters: the companion has been seen, so arming Bondor-only really
        // would trip "companion lost".
        sendNamed(now, "COMP_SEEN", mav_commands::companionSeen() ? 1.0f : 0.0f);
        // DEPTH LOOP OBSERVABILITY (AUDIT R1). This loop has never run closed, and the
        // documented bench check ("hand-move the vehicle in DEPTH_HOLD") only works in
        // WATER — in air a metre of height is ~1.2 mm of equivalent depth. So the sign was
        // untestable on a bench, which is much of why it stayed unverified.
        //
        // DEPTH_CMD is the demand a DISARMED vehicle WOULD make for the current depth vs a
        // target 0.5 m below it. Pressurise the Bar30 by hand and read the sign, with
        // nothing spinning:
        //   measured deeper than target  -> POSITIVE -> ascend   (correct)
        //   measured shallower           -> NEGATIVE -> descend  (correct)
        // Moving AWAY from the target means the sign is inverted. Do not dive.
        //
        // DEPTH_ERR/DEPTH_OUT are the REAL controller's last error and output, so the water
        // test can confirm the armed loop agrees with what the bench preview showed.
        if (s.depth_ok) {
            // FIXED target of 0.10 m depth, not an offset from the measurement.
            //
            // An offset target holds the error CONSTANT, so DEPTH_CMD never moves and only
            // the sign is readable — and at 0.5 m the proportional term also saturates the
            // clamp, hiding even that. A fixed shallow target keeps the output in its LINEAR
            // range and makes it TRACK the measurement, so the response is visible too:
            // s.depth is POSITIVE-DOWN, so at the surface (~0 m) the demand is negative
            // (descend toward 0.10 m) and rises toward 0 as the vehicle actually gets deeper.
            sendNamed(now, "DEPTH_CMD", depth::preview(s.depth, 0.10f));
            // DEPTH_ERR / DEPTH_OUT come from the REAL controller, so they are only
            // meaningful while it is running. Publishing them unconditionally made them a
            // frozen register whenever it was not: disarmed they held whatever they had when
            // the loop last executed, streamed at full rate, and looked completely live.
            //
            // That is worse than saying nothing. duburi_ws refuses to arm while
            // |DEPTH_OUT| >= 0.90, so a stale 0.00 passed that guard for the wrong reason --
            // "settled" and "never ran" were indistinguishable. Absence fails it closed.
            //
            // Deliberately NOT fixed by running the loop while disarmed: a depth controller
            // integrating against a stationary hull on the bench is a worse answer than a
            // gap. DEPTH_CMD above is unaffected -- it is a preview() computed on demand and
            // is genuinely live at all times.
            if (depth::outputFresh()) {
                sendNamed(now, "DEPTH_ERR", depth::lastError());
                sendNamed(now, "DEPTH_OUT", depth::lastOutput());
            }
        }
        // MIXER SIGN, previewed with NOTHING SPINNING.
        //
        // depth::update()'s output goes straight into mixer::mix()'s THROTTLE column
        // (task_control_loop: `mixer::mix(roll, pitch, yaw, thr, fwd, lat, norm)`), and
        // mix() is a PURE FUNCTION with no state — so the mixer's contribution to the depth
        // chain can be read exactly, disarmed, by calling it with a known demand.
        //
        // This matters because the chain has four arrows and the original R1 inversion could
        // live in any of them:
        //     depth error -> heave demand -> MIXER -> motor output -> MOT_n_DIRECTION -> spin
        // DEPTH_CMD closed the first. This closes the second. Only the last (a physical
        // motor direction) then needs power, which keeps the armed bench test as short and
        // as low-risk as possible.
        //
        // Convention, from docs/THRUSTER_MAP.md: positive throttle demand = ASCEND, and the
        // vertical rows carry a -1 throttle column, so an ascend demand must produce a
        // NEGATIVE normalized output on all four verticals (a negative motor command pushes
        // the vehicle up). Two negations that must not be read as one — which is precisely
        // how the original bug happened.
        {
            float probe[NUM_THRUSTERS] = {0};
            mixer::mix(0, 0, 0, /*throttle=ascend*/ 1.0f, 0, 0, probe);
            // MIX_VERT: vertical thruster 5 (index 4) for a full ASCEND demand. Expect -1.
            sendNamed(now, "MIX_VERT", probe[4]);
            // MIX_VSGN: how many of the four verticals got the correct (negative) sign.
            // Expect 4. Anything less is a mixer matrix error, not a wiring one.
            int agree = 0;
            for (int i = 4; i < NUM_THRUSTERS; ++i) if (probe[i] < 0.0f) agree++;
            sendNamed(now, "MIX_VSGN", (float)agree);
        }
    }
    // Diagnostics: task stack high-water (words free, min ever) + free heap.
    // A stack value trending toward 0 identifies a task about to overflow/reboot.
    if (now - t_diag >= 2000) {
        t_diag = now;
        g_stack_hw[0] = uxTaskGetStackHighWaterMark(nullptr);   // this = MAVLink task
        sendNamed(now, "STK_MAV",  (float)g_stack_hw[0]);
        sendNamed(now, "STK_SEN",  (float)g_stack_hw[1]);
        sendNamed(now, "STK_CTL",  (float)g_stack_hw[2]);
        sendNamed(now, "STK_UI",   (float)g_stack_hw[3]);
        sendNamed(now, "STK_LORA", (float)g_stack_hw[4]);
        sendNamed(now, "STK_DSH",  (float)g_stack_hw[5]);
        sendNamed(now, "HEAP", (float)ESP.getFreeHeap());
    }
}

}  // namespace mav_stream
