// =============================================================================
//  comms/mav_commands — implementation
// =============================================================================

#include "comms/mav_commands.h"
#include "comms/mav_stream.h"
#include "comms/params.h"
#include "comms/mission.h"
#include "control/arming.h"
#include "control/calibration.h"
#include "control/movement.h"
#include "drivers/analog_mon.h"   // voltMultLooksStale() — PM1_VMULT advisory on set
#include "config.h"
#include "state_types.h"

// Custom high-level move command (COMMAND_LONG, no dialect regen). See docs/JETSON_COMMS.md.
#ifndef MAV_CMD_SROT_MOVE
#define MAV_CMD_SROT_MOVE 31000
#endif

// ArduPilot AP_JSButton function IDs (subset we act on). Verify against your
// QGC joystick-setup UI if a button does the wrong thing.
enum : int {
    JS_NONE = 0, JS_SHIFT = 1, JS_ARM_TOGGLE = 2, JS_ARM = 3, JS_DISARM = 4,
    JS_MODE_MANUAL = 5, JS_MODE_STABILIZE = 6, JS_MODE_DEPTH_HOLD = 7,
    JS_MODE_SURFACE = 9, JS_MODE_AUTO = 10,
    JS_MODE_ACRO = 12,
    JS_LIGHTS1_CYCLE = 31, JS_LIGHTS1_BRIGHTER = 32, JS_LIGHTS1_DIMMER = 33,
    JS_GAIN_TOGGLE = 41, JS_GAIN_INC = 42, JS_GAIN_DEC = 43,
    JS_RELAY1_ON = 51, JS_RELAY1_OFF = 52, JS_RELAY1_TOGGLE = 53,
    JS_RELAY2_ON = 54, JS_RELAY2_OFF = 55, JS_RELAY2_TOGGLE = 56,
};

// Live pilot gain, stepped by the gain buttons. RUNTIME ONLY — JS_GAIN_DEFAULT stays the
// power-on value and nothing is written to NVS, so the vehicle always boots at a known
// gain (same behaviour as ArduSub). 0 = "not initialised yet, adopt the param".
static float s_gain_live = 0.0f;
static const float GAIN_MIN = 0.10f, GAIN_MAX = 1.00f, GAIN_STEP = 0.10f;
static const float GAIN_LOW = 0.25f, GAIN_HIGH = 0.75f;   // JS_GAIN_TOGGLE endpoints

namespace mav_commands {

// --- PARAM_REQUEST_LIST streaming state --------------------------------------
static bool     s_param_streaming = false;
static uint16_t s_param_index = 0;
static uint32_t s_param_stream_start_ms = 0;
// Hard stop for a param download. The stream advances only on a successful queue, so a
// GCS that disconnects mid-list (or a persistently full TX) would leave the flag set
// forever — and that flag throttles telemetry. Full 191-param list takes ~10-15 s.
static const uint32_t PARAM_STREAM_MAX_MS = 30000;

// --- GCS link failsafe -------------------------------------------------------
//
// TWO independent liveness timers, deliberately not one.
//
//   s_gcs_*        — ANY non-self heartbeat. Unchanged behaviour, and it is what keeps the
//                    vehicle alive on the bench: Bondor is 255/190 over USB, and the LoRa
//                    ground station synthesises 255/190 too.
//   s_companion_*  — heartbeats matching FS_GCS_SYSID / FS_GCS_COMPID (the Jetson, 255/191).
//
// The failure this closes, raised by the companion team: a DEAD JETSON with Bondor still
// connected currently holds the GCS failsafe open, so the vehicle station-keeps at depth
// when it should surface. Tracking only "any heartbeat" cannot see that.
//
// But scoping the ONE timer to the companion would break every bench session that has no
// Jetson attached — Bondor alone would stop feeding the failsafe and the vehicle would
// surface on the bench. Hence the SEEN-THEN-LOST latch: the companion timer can only ever
// fail after the companion has been heard at least once. No Jetson on the bench => it was
// never seen => it cannot be "lost" => nothing changes. Jetson dies mid-mission => it WAS
// seen => the failsafe fires even though Bondor is still chattering. Both cases correct.
static uint32_t s_gcs_last_ms = 0;
static bool     s_gcs_seen = false;
static uint32_t s_companion_last_ms = 0;
static bool     s_companion_seen = false;

bool gcsLinkOk() {
    return s_gcs_seen && (millis() - s_gcs_last_ms < GCS_FAILSAFE_MS);
}

// True when a NAMED companion has been seen and has now gone quiet. False both when no
// companion is configured and when one is configured but has never appeared.
bool companionLost() {
    if (!s_companion_seen) return false;                       // never seen => cannot be lost
    return (millis() - s_companion_last_ms) >= GCS_FAILSAFE_MS;
}

bool companionConfigured() {
    return ((int)g_params.fs_gcs_sysid != 0) || ((int)g_params.fs_gcs_compid != 0);
}

bool companionSeen() { return s_companion_seen; }

// Does this heartbeat come from the configured companion? A field set to 0 is a WILDCARD, so
// 0/0 restores the old "any non-self heartbeat" behaviour exactly — the escape hatch for an
// operator whose companion is still on the old 255/190.
static bool matchesCompanion(uint8_t sysid, uint8_t compid) {
    const int want_s = (int)g_params.fs_gcs_sysid, want_c = (int)g_params.fs_gcs_compid;
    return (want_s == 0 || sysid == want_s) && (want_c == 0 || compid == want_c);
}

void feedGcs() {
    s_gcs_seen = true;
    s_gcs_last_ms = millis();
}

bool paramDownloadActive() { return s_param_streaming; }

void paramDownloadProgress(uint16_t& n, uint16_t& total) {
    if (s_param_streaming) { n = s_param_index; total = params::count(); }
    else                   { n = 0; total = 0; }
}

// --- helpers -----------------------------------------------------------------
// Returns false if the message couldn't be queued (TX buffer full) — caller
// must not advance the stream index in that case.
// reliable=true: bounded-blocking send (for single PARAM_REQUEST_READ replies that
// must not drop). reliable=false: non-blocking (used by the bulk stream so a full TX
// buffer never stalls RX — the caller just retries the same index next cycle).
static bool sendParam(uint16_t index, bool reliable = true) {
    char name[17];
    float val;
    if (!params::getByIndex(index, name, val)) return true;   // OOR — treat as done
    mavlink_message_t m;
    // Advertise REAL32 for EVERY param, exactly like ArduPilot (GCS_Param.cpp):
    // the value is carried as a float and the GCS casts to the real type from its
    // own metadata. Sending true INT types here can break QGC/MP download-complete
    // tracking (→ Motor Test tab stays greyed), so we do NOT use params::paramType.
    mavlink_msg_param_value_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        name, val, MAV_PARAM_TYPE_REAL32, params::count(), index);
    return reliable ? mav::txReliable(m) : mav::tx(m);
}

static void sendParamByName(const char* name) {
    int idx = params::indexOf(name);
    if (idx >= 0) sendParam((uint16_t)idx);
}

// Send the QGC accel-cal orientation prompt for a position (1..6). The keywords
// (level/left/right/nose down/nose up/back) are what QGC parses to drive its UI.
static void sendAccelPrompt(int pos) {
    const char* t = nullptr;
    switch (pos) {
        case 1: t = "Place vehicle level and press any key."; break;
        case 2: t = "Place vehicle on its LEFT side and press any key."; break;
        case 3: t = "Place vehicle on its RIGHT side and press any key."; break;
        case 4: t = "Place vehicle nose DOWN and press any key."; break;
        case 5: t = "Place vehicle nose UP and press any key."; break;
        case 6: t = "Place vehicle on its BACK and press any key."; break;
        default: return;
    }
    mav_stream::sendStatusText(MAV_SEVERITY_INFO, t);
}

static void ackCommand(uint16_t command, uint8_t result) {
    mavlink_message_t m;
    mavlink_msg_command_ack_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
        command, result, 0, 0, 0, 0);
    mav::txReliable(m);   // a dropped ACK makes QGC retry → looks like a hang
}

static void setArmed(bool armed);   // forward decl (defined below)

// Modes the GCS dropdown may select. STUNT/PATTERN are command-only.
static bool isSupportedMode(uint8_t m) {
    switch (m) {
        case (uint8_t)FlightMode::STABILIZE:
        case (uint8_t)FlightMode::ACRO:
        case (uint8_t)FlightMode::DEPTH_HOLD:
        case (uint8_t)FlightMode::SURFACE:
        case (uint8_t)FlightMode::MANUAL:
        case (uint8_t)FlightMode::MOTOR_DETECT:
        case (uint8_t)FlightMode::AUTOTUNE:
        case (uint8_t)FlightMode::MOTOR_TUNE:
        case (uint8_t)FlightMode::AUTO:
            return true;
        default:
            return false;
    }
}

// Set the flight mode (ControlState). Returns false if unsupported.
static bool setMode(uint8_t mode) {
    if (!isSupportedMode(mode)) return false;
    StateLock lk(g_state.mtx_control);
    if (lk.ok()) g_state.control.mode = (FlightMode)mode;
    return true;
}

// Toggle/set a payload MOSFET/relay on a PCA switch channel.
static void setRelay(int relay_idx /*0-based*/, int action /*1=on 0=off -1=toggle*/) {
    int ch = PCA_RELAY_BASE_CH + relay_idx;
    if (ch < 0 || ch >= PCA9685_NUM_CH) return;
    StateLock lk(g_state.mtx_aux);
    if (!lk.ok()) return;
    bool cur = g_state.aux.switch_on[ch];
    g_state.aux.switch_on[ch] = (action < 0) ? !cur : (action > 0);
    g_state.aux.dirty = true;
}

// Adjust a lights (servo-role) channel brightness by a signed fraction step.
static void adjustLights(int ch, float dir) {
    if (ch < 0 || ch >= PCA9685_NUM_CH) return;
    StateLock lk(g_state.mtx_aux);
    if (!lk.ok()) return;
    int span = SERVO_MAX_US - SERVO_MIN_US;
    int step = (int)(g_params.lights_step * span);
    int cur = g_state.aux.servo_us[ch];
    if (cur == 0) cur = SERVO_MIN_US;
    cur = constrain(cur + (int)(dir * step), SERVO_MIN_US, SERVO_MAX_US);
    g_state.aux.servo_us[ch] = (uint16_t)cur;
    g_state.aux.dirty = true;
}

// --- pilot gain (runtime, stepped by the gain buttons) -----------------------
// Lazily adopts JS_GAIN_DEFAULT on first use so the param remains the power-on value.
static void setGain(float g);

float pilotGain() {
    if (s_gain_live <= 0.0f) {
        float d = g_params.js_gain_default;
        s_gain_live = (d > 0.0f) ? constrain(d, GAIN_MIN, GAIN_MAX) : 1.0f;
    }
    return s_gain_live;
}

static void setGain(float g) {
    s_gain_live = constrain(g, GAIN_MIN, GAIN_MAX);
    char b[32];
    snprintf(b, sizeof(b), "Gain %d%%", (int)(s_gain_live * 100.0f + 0.5f));
    mav_stream::sendStatusText(MAV_SEVERITY_INFO, b);   // so a button press is visible
}

static void adjustGain(float delta) { setGain(pilotGain() + delta); }

// Run one joystick button function (called on button press edge).
static void runButtonFunction(int func) {
    switch (func) {
        case JS_ARM:              setArmed(true);  mav_stream::sendHeartbeatNow(); break;
        case JS_DISARM:           setArmed(false); mav_stream::sendHeartbeatNow(); break;
        case JS_ARM_TOGGLE: {
            bool armed = false;
            { StateLock lk(g_state.mtx_control); if (lk.ok()) armed = g_state.control.armed; }
            setArmed(!armed); mav_stream::sendHeartbeatNow(); break;
        }
        case JS_MODE_MANUAL:      setMode((uint8_t)FlightMode::MANUAL); mav_stream::sendHeartbeatNow(); break;
        case JS_MODE_STABILIZE:   setMode((uint8_t)FlightMode::STABILIZE); mav_stream::sendHeartbeatNow(); break;
        case JS_MODE_DEPTH_HOLD:  setMode((uint8_t)FlightMode::DEPTH_HOLD); mav_stream::sendHeartbeatNow(); break;
        case JS_MODE_ACRO:        setMode((uint8_t)FlightMode::ACRO); mav_stream::sendHeartbeatNow(); break;
        case JS_MODE_SURFACE:     setMode((uint8_t)FlightMode::SURFACE); mav_stream::sendHeartbeatNow(); break;
        case JS_MODE_AUTO:        setMode((uint8_t)FlightMode::AUTO); mav_stream::sendHeartbeatNow(); break;
        case JS_GAIN_INC:         adjustGain(+GAIN_STEP); break;
        case JS_GAIN_DEC:         adjustGain(-GAIN_STEP); break;
        case JS_GAIN_TOGGLE:      setGain(pilotGain() > (GAIN_LOW + GAIN_HIGH) * 0.5f
                                              ? GAIN_LOW : GAIN_HIGH); break;
        case JS_LIGHTS1_BRIGHTER: adjustLights(PCA_LIGHTS1_CH, +1.0f); break;
        case JS_LIGHTS1_DIMMER:   adjustLights(PCA_LIGHTS1_CH, -1.0f); break;
        case JS_RELAY1_ON:        setRelay(0, 1); break;
        case JS_RELAY1_OFF:       setRelay(0, 0); break;
        case JS_RELAY1_TOGGLE:    setRelay(0, -1); break;
        case JS_RELAY2_ON:        setRelay(1, 1); break;
        case JS_RELAY2_OFF:       setRelay(1, 0); break;
        case JS_RELAY2_TOGGLE:    setRelay(1, -1); break;
        default: break;   // unhandled / none
    }
}

static void setArmed(bool armed) {
    StateLock lk(g_state.mtx_control);
    if (lk.ok()) g_state.control.armed = armed;
}

// Source of the in-flight COMMAND_LONG/INT (captured before dispatch) so a
// long-running command (SROT_MOVE) can address its progress ACKs back to the sender.
static uint8_t s_cmd_src_sys = 0, s_cmd_src_comp = 0;

// --- command dispatch --------------------------------------------------------
// p[0..6] = param1..param7.
static uint8_t dispatchCommand(uint16_t command, const float p[7]) {
    // Reject non-finite parameters up front, for EVERY command. This is the single
    // choke point where GCS/companion floats enter the vehicle, and Arduino's
    // constrain() cannot stop a NaN: it is built from < and >, both of which are false
    // against NaN, so `constrain(NaN, -1, 1)` returns NaN unchanged. A NaN that reached
    // DO_MOTOR_TEST's throttle propagated all the way to `(int16_t)NaN` in
    // mixer::oneToDshot() — an undefined cast producing an arbitrary DShot value on an
    // armed thruster. Guarding here fixes the whole class rather than one command.
    //
    // Safe to apply blanket: no command this firmware implements uses NaN as a
    // meaningful "leave unchanged" sentinel (some ArduPilot NAV/REPOSITION commands do,
    // and none of those are implemented here). Revisit if one is ever added.
    for (int i = 0; i < 7; ++i) {
        if (!isfinite(p[i])) {
            mav_stream::sendStatusText(MAV_SEVERITY_WARNING, "Command rejected: non-finite param");
            return MAV_RESULT_DENIED;
        }
    }
    switch (command) {
        case MAV_CMD_COMPONENT_ARM_DISARM: {
            bool want = p[0] > 0.5f;
            if (want) {
                const char* reason = nullptr;
                if (!arming::canArm(&reason)) {
                    char buf[50];
                    snprintf(buf, sizeof(buf), "PreArm: %s", reason ? reason : "check failed");
                    mav_stream::sendStatusText(MAV_SEVERITY_ERROR, buf);
                    return MAV_RESULT_FAILED;
                }
                setArmed(true);
                mav_stream::sendStatusText(MAV_SEVERITY_INFO, "SROT: Armed");
            } else {
                setArmed(false);
                mav_stream::sendStatusText(MAV_SEVERITY_INFO, "SROT: Disarmed");
            }
            mav_stream::sendHeartbeatNow();
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_DO_SET_MODE:
            if (!setMode((uint8_t)p[1])) return MAV_RESULT_DENIED;  // param2 = custom_mode
            mav_stream::sendHeartbeatNow();
            return MAV_RESULT_ACCEPTED;

        case MAV_CMD_SROT_MOVE: {
            // High-level Jetson move. p1=type(0..9) p2=primary p3=speed|yawrate
            //   p4=turn submode | arc yawrate  p5=timeout_s. Auto-enters AUTO; preempts.
            //   Braking is on-board. See docs/JETSON_COMMS.md.  (wire type + 1 = movement::Type)
            int wire = (int)p[0];
            if (wire < 0 || wire > 9) return MAV_RESULT_DENIED;
            StateLock lk(g_state.mtx_control);
            if (!lk.ok()) return MAV_RESULT_TEMPORARILY_REJECTED;
            auto& c = g_state.control;
            c.mode        = FlightMode::AUTO;
            c.mv_type     = (uint8_t)(wire + 1);        // → movement::Type
            c.mv_primary  = p[1];
            c.mv_speed    = p[2];
            c.mv_submode  = (uint8_t)p[3];              // turn: 0 rel / 1 abs
            c.mv_aux      = p[3];                       // arc: signed yaw rate (deg/s)
            c.mv_timeout  = p[4];
            c.mv_src_sys  = s_cmd_src_sys;
            c.mv_src_comp = s_cmd_src_comp;
            c.mv_seq++;                                 // edge → control loop starts it
            mav_stream::sendHeartbeatNow();             // reflect AUTO immediately
            return MAV_RESULT_IN_PROGRESS;              // progress ACKs stream until done
        }

        case MAV_CMD_DO_MOTOR_TEST: {
            // Standard MAV_CMD_DO_MOTOR_TEST: p1=motor (1-based), p2=throttle type
            //   (0=percent 0..100, 1=raw PWM µs, 2=pilot→reject), p3=throttle.
            //   p4/p5 ignored; timeout is a ≥2 Hz keep-alive (auto-disarm on gap).
            //   Motors must be ARMED.
            int8_t motor = (int8_t)p[0] - 1;
            int    ttype = (int)p[1];
            if (motor < 0 || motor >= NUM_THRUSTERS) return MAV_RESULT_DENIED;
            if (ttype == 2) {   // MOTOR_TEST_THROTTLE_PILOT — not supported
                mav_stream::sendStatusText(MAV_SEVERITY_WARNING, "bad throttle type");
                return MAV_RESULT_DENIED;
            }
            { bool armed = false;
              StateLock lk(g_state.mtx_control); if (lk.ok()) armed = g_state.control.armed;
              if (!armed) { mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                              "Arm motors before testing motors."); return MAV_RESULT_FAILED; } }
            // Normalize to −1..1 for the mixer/DShot mapping.
            float thr;
            if (ttype == 1) thr = ((float)p[2] - 1500.0f) / 400.0f;  // PWM µs (1100..1900) → −1..1
            else            thr = p[2] / 100.0f;                     // percent 0..100 → 0..1
            StateLock lk(g_state.mtx_thrusters);
            if (lk.ok()) {
                g_state.thrusters.test_override = true;
                g_state.thrusters.test_motor = motor;
                g_state.thrusters.test_throttle = constrain(thr, -1.0f, 1.0f);
                // Keep-alive window: the GCS resends at >=2 Hz; expire (and auto-disarm)
                // once it stops. Honour param4 (timeout seconds) instead of a flat 600 ms —
                // Bondor asks for 1 s while refreshing every 300 ms, so only ~2 refreshes
                // fit in a 600 ms window and ONE late/dropped packet chopped the test
                // mid-press (the Pico then forces DShot 0, which reads as a wobble).
                // Floor keeps a 0/absent param safe; ceiling stops a stale test holding
                // the vehicle armed indefinitely.
                uint32_t win = (p[3] > 0.0f) ? (uint32_t)(p[3] * 1000.0f) : 600u;
                if (win < 600u)  win = 600u;
                if (win > 3000u) win = 3000u;
                g_state.thrusters.test_expire_ms = millis() + win;
            }
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_PREFLIGHT_CALIBRATION: {
            // ArduPilot param semantics:
            //   p1=1 gyro, p2=1 mag, p3=1 baro, p4=1 radio(ignored),
            //   p5=1 full accel (6-point), p5=2/4 quick LEVEL (AHRS trim).
            CalRoutine r = CalRoutine::NONE;
            if (p[0] == 1)      { r = CalRoutine::GYRO;      mav_stream::sendStatusText(MAV_SEVERITY_INFO, "Calibrating gyros"); }
            else if (p[1] == 1) { r = CalRoutine::MAG; }
            else if (p[2] == 1) { r = CalRoutine::BARO_ZERO; mav_stream::sendStatusText(MAV_SEVERITY_INFO, "Calibrating barometer"); }
            else if (p[4] == 1) { r = CalRoutine::ACCEL_6PT; sendAccelPrompt(1); }        // full 6-point
            else if (p[4] == 2 || p[4] == 4) { r = CalRoutine::LEVEL; mav_stream::sendStatusText(MAV_SEVERITY_INFO, "Calibrating level"); }
            if (r == CalRoutine::NONE) return MAV_RESULT_DENIED;
            StateLock lk(g_state.mtx_cal);
            if (lk.ok()) {
                g_state.cal.routine = r;
                // 6-point accel: 255 = "no position captured yet" so the first
                // face isn't grabbed until the GCS sends ACCELCAL_VEHICLE_POS.
                g_state.cal.step = (r == CalRoutine::ACCEL_6PT) ? 255 : 0;
                g_state.cal.progress = 0;
                g_state.cal.result = CalResult::NONE;
            }
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_ACCELCAL_VEHICLE_POS: {
            int pos = (int)p[0];                 // 1..6 (LEVEL..BACK)
            if (pos < 1 || pos > 6) return MAV_RESULT_DENIED;
            bool applied = false;
            { StateLock lk(g_state.mtx_cal);
              if (lk.ok()) { g_state.cal.step = (uint8_t)(pos - 1); applied = true; } }
            // Do NOT ACK success when nothing changed. This used to always return ACCEPTED,
            // including on an out-of-range position or a missed lock — so the GCS's accel-cal
            // wizard advanced its step while the vehicle stayed on the previous face, and the
            // two silently desynced for the rest of the calibration.
            if (!applied) return MAV_RESULT_TEMPORARILY_REJECTED;
            if (pos < 6) sendAccelPrompt(pos + 1);   // prompt next face
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_DO_START_MAG_CAL: {
            StateLock lk(g_state.mtx_cal);
            if (lk.ok()) { g_state.cal.routine = CalRoutine::MAG; g_state.cal.step = 0; }
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_PREFLIGHT_STORAGE:
            // ArduPilot convention: param1 1 = write, 2 = reset all params to defaults.
            if (p[0] == 2) {
                // Recovery path when a stored value is wrong and you don't want to
                // reflash (e.g. a stale RPM_LOOP=0 frozen in by a MOTOR_TUNE saveAll).
                params::resetAllToDefaults();
                // Also clear the SEPARATE per-motor direction learned by MOTOR_DETECT.
                // It multiplies with MOT_n_DIRECTION, so a bad bench detect (gyro reads
                // noise out of water) silently cancels the operator's parameter change and
                // looks like "the direction parameter does nothing". Reset both together.
                { StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(5));
                  if (lk.ok()) {
                      for (int i = 0; i < NUM_THRUSTERS; ++i) g_state.cal.motor_dir[i] = 1;
                      g_state.cal.persist_pending = true;
                  } }
                mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                                           "Params + motor directions reset to defaults");
                return MAV_RESULT_ACCEPTED;
            }
            // Defer both flash writes to update() (serviced this same Core-0 task).
            // Only on an actual WRITE request (param1 == 1). This used to also fire the
            // calibration flash write for param1 == 0 ("read"), i.e. burning an NVS write
            // cycle on a command that asked for nothing — pointless wear on a namespace the
            // code elsewhere notes is already tight.
            if (p[0] == 1) {
                params::requestSaveAll();
                StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(5));
                if (lk.ok()) g_state.cal.persist_pending = true;
            }
            return MAV_RESULT_ACCEPTED;

        case MAV_CMD_DO_SET_SERVO: {
            int ch = (int)p[0] - 1;            // channel (1-based in MAVLink)
            if (ch < 0 || ch >= PCA9685_NUM_CH) return MAV_RESULT_DENIED;
            StateLock lk(g_state.mtx_aux);
            if (!lk.ok()) return MAV_RESULT_TEMPORARILY_REJECTED;
            // A role-2 (MOSFET/switch) channel has no pulse width — Task_UI_Status forces
            // servo_us to 0 for it and drives it from switch_on. So writing servo_us here
            // did nothing, and DO_SET_RELAY could not reach it either: relay INSTANCE n maps
            // to the fixed channel PCA_RELAY_BASE_CH + n, i.e. only channels 9-16. A channel
            // in the 1-8 range set to on/off was therefore addressable by NEITHER command.
            //
            // Treat the pulse width as a level for those channels: >= 1500 us = ON. That
            // makes every channel reachable by its own channel number whatever its role,
            // which is the intuitive thing, and leaves DO_SET_RELAY (and the joystick relay
            // buttons that share its mapping) working exactly as before.
            if ((int)g_params.servo_role[ch] == 2) {
                g_state.aux.switch_on[ch] = (p[1] >= 1500.0f);
            } else {
                g_state.aux.servo_us[ch] = (uint16_t)p[1];
            }
            g_state.aux.dirty = true;
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_DO_SET_RELAY: {
            // param1 = relay INSTANCE (0-based) → PCA switch channel, matching the
            // joystick relay-button mapping (setRelay uses PCA_RELAY_BASE_CH + idx).
            int ch = PCA_RELAY_BASE_CH + (int)p[0];
            if (ch < 0 || ch >= PCA9685_NUM_CH) return MAV_RESULT_DENIED;
            StateLock lk(g_state.mtx_aux);
            if (lk.ok()) { g_state.aux.switch_on[ch] = (p[1] > 0.5f); g_state.aux.dirty = true; }
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_USER_1:   // yaw spin
        case MAV_CMD_USER_2:   // pitch spin
        case MAV_CMD_USER_3: { // roll spin
            StuntAxis axis = (command == MAV_CMD_USER_1) ? StuntAxis::YAW
                           : (command == MAV_CMD_USER_2) ? StuntAxis::PITCH
                                                         : StuntAxis::ROLL;
            float spins = (p[0] > 0) ? p[0] : g_params.stunt_spin_cnt;
            StateLock lk(g_state.mtx_control);
            if (lk.ok()) {
                g_state.control.mode = FlightMode::STUNT;
                g_state.control.stunt_axis = axis;
                g_state.control.stunt_target_deg = spins * 360.0f;
                g_state.control.stunt_done_deg = 0;
                g_state.control.stunt_progress = 0;
            }
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_USER_4: {  // complex pattern
            float headroom = (p[0] > 0) ? p[0] : g_params.headroom_depth;
            StateLock lk(g_state.mtx_control);
            if (lk.ok()) {
                g_state.control.mode = FlightMode::PATTERN;
                g_state.control.pattern_step = PatternStep::TURN_360;
                g_state.control.pattern_return_depth = headroom;  // provisional; SM refines
            }
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_USER_5: {  // start relay auto-tune
            StateLock lk(g_state.mtx_control);
            if (lk.ok()) g_state.control.autotune_active = true;
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_SET_MESSAGE_INTERVAL:
            // p1 = msgid, p2 = interval µs (>0 set, 0 default, <0 disable). The companion's
            // whole loop rate ceiling was this command's absence.
            return mav_stream::setMessageInterval((uint32_t)p[0], (int32_t)p[1])
                   ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED;

        case MAV_CMD_GET_MESSAGE_INTERVAL: {
            // Reply in the ACK's param field is not a thing, so answer the way ArduPilot
            // does: emit the current value as a MESSAGE_INTERVAL message.
            uint32_t id = (uint32_t)p[0];
            mavlink_message_t m;
            mavlink_msg_message_interval_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &m,
                                              (uint16_t)id, mav_stream::getMessageInterval(id));
            mav::tx(m);
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_REQUEST_MESSAGE:
            if ((uint32_t)p[0] == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
                mav_stream::sendAutopilotVersion();
            }
            return MAV_RESULT_ACCEPTED;

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            mav_stream::sendAutopilotVersion();
            return MAV_RESULT_ACCEPTED;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            // param1: 1 = reboot autopilot, 3 = reboot to bootloader.
            if ((int)p[0] == 1 || (int)p[0] == 3) {
                ackCommand(command, MAV_RESULT_ACCEPTED);      // ack now — restart never returns
                mav_stream::sendStatusText(MAV_SEVERITY_INFO, "SROT: rebooting");
                MAVLINK_SERIAL.flush();
                vTaskDelay(pdMS_TO_TICKS(200));
                ESP.restart();
            }
            return MAV_RESULT_DENIED;

        default:
            return MAV_RESULT_UNSUPPORTED;
    }
}

// --- message handlers --------------------------------------------------------
static void onCommandLong(const mavlink_message_t& msg) {
    mavlink_command_long_t c;
    mavlink_msg_command_long_decode(&msg, &c);
    if (c.target_system != 0 && c.target_system != MAV_SYSTEM_ID) return;
    s_cmd_src_sys = msg.sysid; s_cmd_src_comp = msg.compid;
    float p[7] = { c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7 };
    uint8_t res = dispatchCommand(c.command, p);
    ackCommand(c.command, res);
}

static void onCommandInt(const mavlink_message_t& msg) {
    mavlink_command_int_t c;
    mavlink_msg_command_int_decode(&msg, &c);
    if (c.target_system != 0 && c.target_system != MAV_SYSTEM_ID) return;
    s_cmd_src_sys = msg.sysid; s_cmd_src_comp = msg.compid;
    float p[7] = { c.param1, c.param2, c.param3, c.param4,
                   (float)c.x, (float)c.y, c.z };
    uint8_t res = dispatchCommand(c.command, p);
    ackCommand(c.command, res);
}

static void onParamSet(const mavlink_message_t& msg) {
    mavlink_param_set_t ps;
    mavlink_msg_param_set_decode(&msg, &ps);
    if (ps.target_system != 0 && ps.target_system != MAV_SYSTEM_ID) return;
    char name[17];
    strncpy(name, ps.param_id, 16);
    name[16] = '\0';
    // Reject non-finite values here too. dispatchCommand() guards COMMAND_LONG/INT, but
    // PARAM_SET is a separate entry point with its own path into params::set() — and a NaN
    // reaching it is not harmless: the skip-if-already-stored compare in set() reads an
    // ABSENT key back as the NAN sentinel, so a NaN write would match, take the skip branch,
    // and report persisted=true for a key that was never written at all. It would then
    // silently revert to its compiled default on the next boot.
    if (!isfinite(ps.param_value)) {
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING, "PARAM_SET rejected: non-finite value");
        return;
    }
    bool persisted = false;
    if (params::set(name, ps.param_value, &persisted)) {
        if (!persisted) {
            // Applied and live, but it will NOT survive a reboot. Silence here meant the
            // operator tuned in the pool, power-cycled, and lost it with no clue why.
            // ERROR, not WARNING. A companion has no other way to observe a failed write —
            // PARAM_VALUE still echoes the accepted value, so from its side the set looks
            // fine and then silently reverts at the next boot. They lost a whole pool
            // session to exactly this: a JS_GAIN_DEFAULT = 1.0 write that never persisted,
            // leaving MANUAL_CONTROL at half authority with nothing to indicate it.
            char wb[64];
            snprintf(wb, sizeof(wb), "%s set but NOT saved (NVS full?)", name);
            mav_stream::sendStatusText(MAV_SEVERITY_ERROR, wb);
        }
        // ATUNE is a momentary trigger: setting it ≥1 starts the relay auto-tune.
        if (strncmp(name, "ATUNE", 16) == 0 && ps.param_value >= 1.0f) {
            StateLock lk(g_state.mtx_control);
            if (lk.ok()) g_state.control.autotune_active = true;
        }
        // Pin assignments only take effect after a reboot (ArduPilot-style).
        if (strncmp(name, "PIN_", 4) == 0) {
            mav_stream::sendStatusText(MAV_SEVERITY_WARNING, "Pin changed - reboot required");
        }
        // PM1_VMULT is a divider RATIO (~11), not the old volts-per-count (~0.009). The value
        // is applied as typed — this only says it looks like the old units. Warning HERE and
        // not just at boot is the point: calibration is an interactive loop (type a value,
        // watch the voltage), so advice that only appears at startup arrives too late to use.
        if (strncmp(name, "PM1_VMULT", 16) == 0 && analog_mon::voltMultLooksStale(ps.param_value)) {
            char wb[72];
            snprintf(wb, sizeof(wb), "PM1_VMULT=%.4f applied, but a divider ratio is ~11",
                     (double)ps.param_value);
            mav_stream::sendStatusText(MAV_SEVERITY_WARNING, wb);
        }
        // Same courtesy for PM2_VMULT, whose units also changed. Applied and KEPT — the
        // boot-time migration is one-shot and has already run, so a small value set now is
        // deliberate and survives the next power cycle.
        if (strncmp(name, "PM2_VMULT", 16) == 0 && ps.param_value < PM2_VMULT_LEGACY_MAX) {
            char wb[72];
            snprintf(wb, sizeof(wb), "PM2_VMULT=%.4f applied, but a trim is ~1.0",
                     (double)ps.param_value);
            mav_stream::sendStatusText(MAV_SEVERITY_WARNING, wb);
        }
        sendParamByName(name);   // echo the accepted value
    }
}

static void onParamRequestRead(const mavlink_message_t& msg) {
    mavlink_param_request_read_t pr;
    mavlink_msg_param_request_read_decode(&msg, &pr);
    if (pr.param_index >= 0) {
        sendParam((uint16_t)pr.param_index);
    } else {
        char name[17];
        strncpy(name, pr.param_id, 16);
        name[16] = '\0';
        sendParamByName(name);
    }
}

static uint16_t s_prev_buttons = 0;

static void onManualControl(const mavlink_message_t& msg) {
    mavlink_manual_control_t mc;
    mavlink_msg_manual_control_decode(&msg, &mc);
    // x=forward, y=lateral, z=throttle/heave (500 neutral), r=yaw.
    //
    // CLAMP FIRST. These are raw int16_t on the wire and MAVLink does not enforce the
    // recommended ranges, so a glitching joystick bridge or a companion bug can legally
    // send x = 32767. That became sp_forward = 32.7, and PILOT_EXPO cubes it
    // (32.7^3 ~ 35000) before the mixer's uniform scale-down — turning what should be a
    // small stick nudge into an instant full-authority burst in whichever direction the
    // mix resolves. The per-motor clamp downstream bounds the OUTPUT but not the
    // behaviour. ControlState documents these setpoints as -1..1; enforce it here, at the
    // one place they enter the vehicle.
    const float x = constrain((float)mc.x, -1000.0f, 1000.0f);
    const float y = constrain((float)mc.y, -1000.0f, 1000.0f);
    const float z = constrain((float)mc.z,     0.0f, 1000.0f);   // 500 = neutral
    const float r = constrain((float)mc.r, -1000.0f, 1000.0f);
    // Apply the joystick gain to the translational axes (JS_GAIN). This is the LIVE gain,
    // which the gain buttons step at runtime; JS_GAIN_DEFAULT is only the power-on value.
    float gain = pilotGain();
    {
        StateLock lk(g_state.mtx_control);
        if (lk.ok()) {
            g_state.control.sp_forward  = (x / 1000.0f) * gain;
            g_state.control.sp_lateral  = (y / 1000.0f) * gain;
            g_state.control.sp_throttle = ((z - 500.0f) / 500.0f) * gain;
            g_state.control.sp_yaw      = (r / 1000.0f) * gain;
            // Stamp AT RECEIPT so the flight loop can age these out. See sp_stamp_ms.
            g_state.control.sp_stamp_ms = millis();
        }
    }

    // Joystick buttons → ArduSub button functions, on the press edge.
    uint16_t pressed = mc.buttons & ~s_prev_buttons;
    for (int i = 0; i < 16; ++i) {
        if (pressed & (1 << i)) {
            int func = (int)g_params.btn_func[i];
            if (func != JS_NONE) runButtonFunction(func);
        }
    }
    s_prev_buttons = mc.buttons;
}

static void onSetMode(const mavlink_message_t& msg) {
    mavlink_set_mode_t sm;
    mavlink_msg_set_mode_decode(&msg, &sm);
    setMode((uint8_t)sm.custom_mode);
}

void handle(const mavlink_message_t& msg) {
    // MAVLink mission upload/download (ArduSub-style) is handled first.
    if (mission::handle(msg)) return;

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // Any heartbeat that isn't ours counts as a live GCS/companion link.
            if (msg.compid != MAV_COMPONENT_ID || msg.sysid != MAV_SYSTEM_ID) {
                s_gcs_seen = true;
                s_gcs_last_ms = millis();
                // ...and separately, is this the companion the failsafe is scoped to?
                if (companionConfigured() && matchesCompanion(msg.sysid, msg.compid)) {
                    s_companion_seen = true;
                    s_companion_last_ms = millis();
                }
            }
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            // Start from 0 only if not already mid-stream — a duplicate request
            // while streaming must NOT restart (that can loop and never finish).
            if (!s_param_streaming) {
                s_param_streaming = true; s_param_index = 0;
                s_param_stream_start_ms = millis();
            }
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            onParamRequestRead(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_SET:
            onParamSet(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            onCommandLong(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_INT:
            onCommandInt(msg);
            break;
        case MAVLINK_MSG_ID_SET_MODE:
            onSetMode(msg);
            break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            onManualControl(msg);
            break;
        default:
            break;
    }
}

void update(uint32_t now) {
    (void)now;
    // Deferred NVS flash writes run HERE (Core 0 / comms task) so a calibration- or
    // autotune-complete never stalls the 500 Hz flight loop on Core 1.
    {
        bool cal_pending = false;
        {   // scope the lock so it releases BEFORE the flash write
            StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(5));
            if (lk.ok() && g_state.cal.persist_pending) {
                g_state.cal.persist_pending = false;
                cal_pending = true;
            }
        }
        if (cal_pending) {
            calibration::saveToNVS();
            // A save that silently did nothing is the failure mode this vehicle already
            // lived through once (R14). Say so, loudly, naming the key that failed.
            if (calibration::nvsFailCount() != 0) {
                char b[64];
                snprintf(b, sizeof(b), "CAL SAVE FAILED (%u keys, first '%s') - NVS full?",
                         (unsigned)calibration::nvsFailCount(),
                         calibration::nvsFailKey() ? calibration::nvsFailKey() : "?");
                mav_stream::sendStatusText(MAV_SEVERITY_ERROR, b);
            } else {
                mav_stream::sendStatusText(MAV_SEVERITY_INFO,
                                           "Calibration saved + verified on flash");
            }
        }
    }
    // A bulk save that failed must not pass in silence. PREFLIGHT_STORAGE already ACKed
    // ACCEPTED (the request WAS accepted; the write is deferred to here), so this message
    // is the only signal that the tune did not persist — it is what the GCS Save button,
    // autotune and motor_tune all rely on.
    if (!params::serviceSaveAll()) {
        mav_stream::sendStatusText(MAV_SEVERITY_CRITICAL,
                                   "SAVE FAILED - params NOT written (NVS full?)");
    }

    // Stream the parameter list with NON-BLOCKING tx so a full TX buffer never
    // stalls RX draining (a blocking send here can drop inbound QGC commands).
    // Advance the index only on a successful queue.
    if (s_param_streaming) {
        for (int i = 0; i < 5 && s_param_streaming; ++i) {
            if (!sendParam(s_param_index, false)) break;   // TX full — resume next cycle
            if (++s_param_index >= params::count()) s_param_streaming = false;
        }
        // Give up if the list never completes, so a stalled download can't throttle
        // telemetry indefinitely (the GCS can always re-request).
        if (s_param_streaming && millis() - s_param_stream_start_ms > PARAM_STREAM_MAX_MS) {
            s_param_streaming = false;
        }
    }
}

}  // namespace mav_commands
