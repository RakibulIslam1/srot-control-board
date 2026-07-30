// =============================================================================
//  comms/mav_stream — implementation
// =============================================================================

#include "comms/mav_stream.h"
#include "comms/mavlink_bridge.h"
#include "comms/mav_commands.h"
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

// Per-thruster RPM → ESC_STATUS (4 ESCs/msg, so index 0 and 4 cover all 8). QGC/BlueOS
// display these. Voltage/current unknown here (0) until the Pico reports them.
static void sendEscStatus(const Snap& s, uint32_t t) {
    for (int base = 0; base < NUM_THRUSTERS; base += 4) {
        int32_t rpm[4]; float volt[4] = {0}, curr[4] = {0};
        for (int i = 0; i < 4; ++i) rpm[i] = (base + i < NUM_THRUSTERS) ? s.rpm[base + i] : 0;
        mavlink_message_t m;
        mavlink_msg_esc_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
            (uint8_t)base, (uint64_t)t * 1000ULL, rpm, volt, curr);
        mav::tx(m);
    }
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
        (int16_t)(s.wtemp * 100.0f));
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
    mavlink_msg_sys_status_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        sensors, sensors, health, 0,
        vbat, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
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
                char b[64] = "MotorDetect:";
                for (int i = 0; i < NUM_THRUSTERS; ++i) {
                    char t[6];
                    snprintf(t, sizeof(t), " %c", mdir[i] < 0 ? '-' : '+');
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
    mavlink_msg_autopilot_version_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        caps, HENGLA_FW_VERSION_PACKED, 0, 0, 0,
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

    // A new (or preempting) command started → track it, ACK immediately.
    // Track on the seq change ALONE — do not require mv_active. A short move can start
    // AND finish before this function next runs (it used to be skipped entirely for the
    // duration of a parameter download), and mv_active is false again by then. Gating on
    // it left s_seq at 0, so no terminal ACK was ever sent and the companion's action
    // waited forever on a result that had already happened.
    if (s.mv_seq != 0 && s.mv_seq != s_seq) {
        s_seq = s.mv_seq; t_ack = 0; s_seen_active = false; s_track_ms = now;
    }
    if (s_seq == 0) return;
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
        s_seq = 0;
        return;
    }

    bool done = (s.mv_done_seq == s_seq) || (s_seen_active && !s.mv_active);
    if (done) {
        sendMoveAck(s, MAV_RESULT_ACCEPTED, 100);
        sendNamed(now, "MV_STATE", 0);
        sendNamed(now, "MV_PROG",  1.0f);
        s_seq = 0;
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

void update(uint32_t now) {
    static uint32_t t_hb = 0, t_sys = 0, t_att = 0, t_imu = 0,
                    t_prs = 0, t_hud = 0, t_bat = 0, t_pwr = 0, t_nvf = 0, t_diag = 0, t_esc = 0;
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
    if (mav_commands::paramDownloadActive()) {
        if (now - t_hb >= 1000) { t_hb = now; sendHeartbeat(s); }
        if (now - t_esc >= 200) { t_esc = now; sendEscStatus(s, now); }
        return;
    }

    if (now - t_hb  >= 1000) { t_hb  = now; sendHeartbeat(s); }
    if (now - t_sys >= 500)  { t_sys = now; sendSysStatus(s); }
    if (now - t_att >= 100)  { t_att = now; sendAttitude(s, now); }
    if (now - t_imu >= 100)  { t_imu = now; sendScaledImu(s, now); }
    if (now - t_prs >= 200)  { t_prs = now; sendScaledPressure(s, now); }
    if (now - t_hud >= 200)  { t_hud = now; sendVfrHud(s); }
    if (now - t_bat >= 1000) {
        t_bat = now;
        if (s.pm1_present) sendBattery(0, s.pm1, s.curr);   // current on the main battery
        if (s.pm2_present) sendBattery(1, s.pm2, 0);
    }
    if (now - t_pwr >= 1000) { t_pwr = now; sendPowerStatus(s); }
    if (now - t_esc >= 200)  { t_esc = now; sendEscStatus(s, now); }   // per-thruster RPM (5 Hz)
    if (now - t_nvf >= 500) {
        t_nvf = now;
        sendNamed(now, "LEAK", s.leak ? 1.0f : 0.0f);
        sendNamed(now, "WTEMP", s.wtemp);
        sendNamed(now, "STUNT_PRG", s.stunt_prog);
        sendNamed(now, "ATUNE", s.atune ? 1.0f : 0.0f);
        sendNamed(now, "KILL", s.kill ? 1.0f : 0.0f);
        sendNamed(now, "CURR", s.curr);
        // Live pilot gain — the gain buttons change this at runtime, so a joystick UI has
        // no other way to know the current value (JS_GAIN_DEFAULT is only the boot value).
        sendNamed(now, "GAIN", mav_commands::pilotGain());
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
