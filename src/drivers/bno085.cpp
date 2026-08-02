// =============================================================================
//  drivers/bno085 — implementation (Adafruit BNO08x)
//  Reference: example/bno085 example.cpp
// =============================================================================

#include "drivers/bno085.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <sh2.h>          // sh2_setCalConfig / sh2_saveDcdNow — see saveCalibration()
#include <esp_system.h>
#include "config.h"

namespace bno085 {

// No RESET pin wired on our board → construct with PIN_BNO_RESET (-1). There is
// also no INT pin (PIN_BNO_INT == -1), so begin_I2C() is called without one and
// data is polled via getSensorEvent().
static Adafruit_BNO08x s_bno(PIN_BNO_RESET);
static sh2_SensorValue_t s_val;
static bool s_ok = false;

// Reset-recovery state: after a reset, ignore rotation vectors for a short window
// (the identity/converging garbage) and hold the last-good attitude. Kept short
// (150 ms) so any residual reset is a brief blip, not a multi-second freeze — the
// primary defence is now fewer reports + a quieter bus (below), which should make
// resets rare in the first place.
static const uint32_t RECOVERY_MS = 150;
static uint32_t s_recovery_until = 0;
static uint32_t s_reset_count = 0;
static uint32_t s_last_attitude_ms = 0;
static bool     s_attitude_ever = false;

// Report intervals. Attitude + rate run at the sensor's 400 Hz maximum, which the
// 500 Hz sensor task oversamples cleanly (the 500 Hz control loop is what consumes
// them). Gravity is only used by level/accel calibration and changes slowly, so it
// stays at 50 Hz to keep the SHTP link lightly loaded. 400 Hz is only safe because
// the INT pin IS wired (PIN_BNO_INT) and reads are INT-gated — without INT this rate
// pushes the polled link toward desync/resets, so drop to 200 Hz if INT is ever lost.
static const uint32_t REPORT_FAST_US = 2500;    // 400 Hz — rotation + gyro (sensor max)
static const uint32_t REPORT_SLOW_US = 20000;   // 50 Hz  — gravity (cal only)

// INT pin (H_INTN, active-LOW). The Adafruit I2C driver ignores INT, so we gate our
// own polling on it: read only when the sensor asserts data-ready. -1 = poll blind.
static int s_int_pin = PIN_BNO_INT;
static uint32_t s_last_event_ms = 0;   // last time getSensorEvent() returned data

static uint32_t s_enable_fail = 0;     // enableReport() rejections (diagnostic)

// --- BNO085 Dynamic Calibration Data (DCD) -----------------------------------
//
// THE BUG THIS FIXES: the BNO085 keeps its own hard/soft-iron solution in ITS OWN flash,
// and it is only written when the host explicitly asks. We never asked. So every power
// cycle the part restarted with no magnetic calibration and `mag_accuracy` climbed from 0
// again -- which is exactly the operator's "after some power cycles the magnetometer
// calibration is lost", and also why the one-shot yaw reference (control/yaw_ref) could
// refuse: it requires accuracy >= 2 and accuracy was always 0 for the first minutes.
//
// Note this is a DIFFERENT thing from our CAL_MAG_* rows in NVS_NS_CAL. Those persist
// correctly and always did; they are OUR corrections, applied on top of whatever solution
// the BNO itself has converged to. Saving ours while never saving the sensor's is why the
// symptom looked like "calibration is there but useless".
static uint32_t s_dcd_saves = 0;
static uint32_t s_dcd_last_ms = 0;
static bool     s_dcd_saved_this_boot = false;
static bool     s_dcd_pending_announce = false;
static bool     s_dcd_pending_fail = false;
static bool     s_calcfg_ok = false;
static bool     s_calcfg_became_ok = false;   // announce the late success once
// Never write the part's flash more often than this. Flash has finite endurance and the
// accuracy flag can oscillate around a threshold; a save loop would wear it out.
static const uint32_t DCD_MIN_GAP_MS = 60000;
static uint32_t s_reinit_count = 0;    // full begin_I2C() recoveries
static uint32_t s_last_heal_ms = 0;    // rate-limit the self-heal attempts

// Re-issue the report set, checking every return. The BNO08x forgets its enabled
// reports on reset, and enableReport() CAN be rejected (it needs ~90 ms after a reset
// before it accepts sh2_setSensorConfig). wasReset() is a self-clearing latch, so a
// rejected re-enable used to mean the sensor went silent FOREVER with nothing left to
// retry it. Returns true only if every report was accepted.
static bool enableReportsChecked();

static void enableReports() {
    // Minimal report set. Polling BNO08x SHTP over I2C WITHOUT the INT line is
    // fragile — every enabled report multiplies the traffic that can desync and
    // reset the sensor (the "r8" freeze). We enable only what the firmware needs:
    //   GAME_ROTATION_VECTOR → attitude (roll/pitch/yaw)
    //   GYROSCOPE_CALIBRATED → rate control
    //   GRAVITY              → level / accel calibration (calibration::update)
    // Dropped vs. before: LINEAR_ACCELERATION (telemetry-only) and raw
    // ACCELEROMETER (only fed the accuracy flag, now taken from the rotation
    // vector status). This roughly halves the SHTP load.
    enableReportsChecked();
}

static bool enableReportsChecked() {
    bool ok = true;
    if (!s_bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_FAST_US)) ok = false;
    if (!s_bno.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_FAST_US)) ok = false;
    if (!s_bno.enableReport(SH2_GRAVITY, REPORT_SLOW_US))              ok = false;
#if BNO_USE_MAG
    if (!s_bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, REPORT_SLOW_US)) ok = false;
#endif
    if (!ok) s_enable_fail++;
    return ok;
}

// Quaternion → euler (radians). Same axis convention as the example, minus its
// RAD_TO_DEG scaling so we store radians for MAVLink ATTITUDE.
static void quatToEuler(float qr, float qi, float qj, float qk,
                        float& roll, float& pitch, float& yaw) {
    float sqr = sq(qr), sqi = sq(qi), sqj = sq(qj), sqk = sq(qk);
    yaw   = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    // Clamp the asinf argument: float rounding on a near-unit quaternion can push it
    // just past ±1 → NaN pitch that would propagate through the whole control loop.
    float sarg = -2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr);
    pitch = asinf(constrain(sarg, -1.0f, 1.0f));
    roll  = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}

bool begin() {
    // Let the BNO085's internal processor boot. Only a true power-on needs the full 3 s;
    // on a warm reset (EN button / brown-out) the sensor is already booted, so a short
    // settle is enough — this makes the board recover fast after a warm reset instead of
    // stalling the sensor task for 3 s.
    delay(esp_reset_reason() == ESP_RST_POWERON ? 3000 : 300);

    // INT (H_INTN) is an active-LOW push-pull output from the sensor, idle HIGH.
    if (s_int_pin >= 0) pinMode(s_int_pin, INPUT_PULLUP);

    // Wire is already begun by main.cpp on the I2C0 pins at I2C0_FREQ.
    if (!s_bno.begin_I2C(I2C_ADDR_BNO085, &Wire)) {
        // Fall back to the alternate SH-2 I2C address.
        if (!s_bno.begin_I2C(0x4B, &Wire)) {
            s_ok = false;
            return false;
        }
    }
    enableReports();

    // Ask the part to run dynamic calibration for accel + mag. Without this the DCD never
    // improves, so there would be nothing worth saving. Failure is non-fatal: the sensor
    // still works, it just will not self-calibrate, and saveCalibration() will report it.
    // Without this the DCD never improves, so there is nothing worth saving and mag
    // accuracy stays pinned at 0 for ever. Its result is REPORTED, not assumed: a silent
    // failure here looks exactly like "the calibration will not save".
    // First attempt. It USUALLY FAILS HERE and that is expected: the part needs ~90 ms
    // after a reset before it accepts a config command -- the same reason enableReport()
    // can be rejected right after begin() (see enableReportsChecked). Retried from poll()
    // until it takes; without it the sensor never runs dynamic calibration, mag accuracy is
    // pinned near 0 for ever, and every mag calibration the operator performs is discarded
    // on the next boot. That was the actual cause of "the compass calibration will not save".
    s_calcfg_ok = (sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_MAG) == SH2_OK);

    s_ok = true;
    return true;
}

bool saveCalibration(bool force) {
    if (!s_ok) return false;
    const uint32_t now = millis();
    if (!force && s_dcd_last_ms != 0 && (now - s_dcd_last_ms) < DCD_MIN_GAP_MS) return false;
    // sh2_saveDcdNow() blocks on a command/response round trip; only ever call it from the
    // sensor task, disarmed, never from the flight loop.
    const int rc = sh2_saveDcdNow();
    s_dcd_last_ms = now;
    if (rc == SH2_OK) {
        s_dcd_saves++;
        s_dcd_saved_this_boot = true;
        s_dcd_pending_announce = true;
        return true;
    }
    s_dcd_pending_fail = true;
    return false;
}

bool takeDcdFailAnnounce() { bool a = s_dcd_pending_fail; s_dcd_pending_fail = false; return a; }
bool calConfigOk()         { return s_calcfg_ok; }
bool takeCalConfigAnnounce() { bool a = s_calcfg_became_ok; s_calcfg_became_ok = false; return a; }

bool dcdSavedThisBoot()   { return s_dcd_saved_this_boot; }
uint32_t dcdSaveCount()   { return s_dcd_saves; }
bool takeDcdAnnounce()    { bool a = s_dcd_pending_announce; s_dcd_pending_announce = false; return a; }

bool setMagReportEnabled(bool on) {
    if (!s_ok) return false;
    // interval 0 = disable, per the SH-2 sensor-config contract.
    return s_bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, on ? REPORT_SLOW_US : 0);
}

bool poll(Sample& out) {
    if (!s_ok) return false;

    // Retry the calibration-config until the part accepts it. Cheap (a few bytes of SHTP
    // every 2 s, and only until it succeeds) and it is what makes the mag converge at all.
    if (!s_calcfg_ok) {
        static uint32_t s_calcfg_try_ms = 0;
        const uint32_t now = millis();
        if (now - s_calcfg_try_ms >= 2000) {
            s_calcfg_try_ms = now;
            s_calcfg_ok = (sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_MAG) == SH2_OK);
            if (s_calcfg_ok) s_calcfg_became_ok = true;
        }
    }

    if (s_bno.wasReset()) {
        // The part needs ~90 ms after a reset before it accepts sh2_setSensorConfig, so
        // the immediate re-enable can be rejected. Don't rely on it: the watchdog below
        // re-issues if no attitude actually arrives. (wasReset() is a self-clearing
        // latch — once consumed there is no second chance from here.)
        enableReportsChecked();
        s_reset_count++;
        s_recovery_until = millis() + RECOVERY_MS;   // ignore identity garbage
    }

    // --- Self-heal watchdog -------------------------------------------------------
    // Without this, a rejected re-enable (or a wedged SHTP link) leaves the sensor
    // silent forever with nothing to recover it. Escalate: re-issue the reports, then
    // after a couple of seconds re-open the whole SH-2 session. Rate-limited so a truly
    // dead sensor doesn't hammer the I2C bus the rest of the system shares.
    {
        uint32_t now_ms = millis();
        bool starved = s_attitude_ever && (now_ms - s_last_attitude_ms > 250) &&
                       (now_ms >= s_recovery_until);
        if (starved && now_ms - s_last_heal_ms > 200) {
            s_last_heal_ms = now_ms;
            if (now_ms - s_last_attitude_ms > 2000) {
                // Reports alone aren't coming back — re-open the session entirely.
                s_reinit_count++;
                if (s_bno.begin_I2C(I2C_ADDR_BNO085, &Wire) ||
                    s_bno.begin_I2C(0x4B, &Wire)) {
                    enableReportsChecked();
                    s_recovery_until = now_ms + RECOVERY_MS;
                }
            } else {
                enableReportsChecked();
            }
        }
    }

    bool in_recovery = (millis() < s_recovery_until);

    // INT-gated read: when the INT pin is wired, only touch the SHTP bus while the
    // sensor asserts data-ready (H_INTN LOW). This avoids reading mid-packet — the
    // thing that desyncs SHTP at 400 Hz — and skips the empty polls entirely. When
    // INT is unwired (-1) we fall through and poll blind (old behaviour).
    // Safety net: if INT reads "no data" but we've heard nothing for >100 ms (flaky
    // or miswired INT), poll blind once so the IMU can never go permanently silent.
    if (s_int_pin >= 0 && digitalRead(s_int_pin) != LOW &&
        (millis() - s_last_event_ms) < 100) {
        return false;
    }

    bool updated = false;
    // BOUNDED drain: read at most a handful of events per call. An UNBOUNDED
    // `while(getSensorEvent)` can spin forever if events keep arriving or on an
    // SHTP/I2C error storm — the task then never yields, starves Core-1 IDLE, and
    // the Task Watchdog reboots the board (the root cause of the "freeze"/"0°"/
    // rN=1 symptoms). 8 = ~2 fresh reports/cycle at 500 Hz poll + headroom. The INT
    // gate above already prevents empty reads, so this only bounds a burst.
    for (int drain = 0; drain < 8 && s_bno.getSensorEvent(&s_val); ++drain) {
        updated = true;
        s_last_event_ms = millis();
        switch (s_val.sensorId) {
            case SH2_GAME_ROTATION_VECTOR:
                // During post-reset recovery, drop the sample and KEEP the last
                // good attitude in `out` (don't overwrite with identity/0°).
                if (in_recovery) break;
                out.qw = s_val.un.gameRotationVector.real;
                out.qx = s_val.un.gameRotationVector.i;
                out.qy = s_val.un.gameRotationVector.j;
                out.qz = s_val.un.gameRotationVector.k;
                quatToEuler(out.qw, out.qx, out.qy, out.qz,
                            out.roll, out.pitch, out.yaw);
#if BNO_SWAP_ROLL_PITCH
                { float t = out.roll; out.roll = out.pitch; out.pitch = t; }
#endif
                // Accel accuracy is taken from the rotation-vector status now that
                // the raw ACCELEROMETER report is disabled.
                out.acc_accuracy = s_val.status & 0x03;
                s_attitude_ever = true;
                s_last_attitude_ms = millis();
                break;
            case SH2_GRAVITY:
                out.grav_x = s_val.un.gravity.x;
                out.grav_y = s_val.un.gravity.y;
                out.grav_z = s_val.un.gravity.z;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
#if BNO_SWAP_ROLL_PITCH
                // Swap roll/pitch rates to match the swapped angle axes.
                out.gyro_x = s_val.un.gyroscope.y;
                out.gyro_y = s_val.un.gyroscope.x;
#else
                out.gyro_x = s_val.un.gyroscope.x;
                out.gyro_y = s_val.un.gyroscope.y;
#endif
                out.gyro_z = s_val.un.gyroscope.z;
                out.gyro_accuracy = s_val.status & 0x03;
                break;
            case SH2_MAGNETIC_FIELD_CALIBRATED:
                out.mag_x = s_val.un.magneticField.x;
                out.mag_y = s_val.un.magneticField.y;
                out.mag_z = s_val.un.magneticField.z;
                out.mag_accuracy = s_val.status & 0x03;
                break;
            default:
                break;
        }
    }
    return updated;
}

bool healthy() { return s_ok; }

bool attitudeValid() {
    if (!s_attitude_ever) return false;
    if (millis() < s_recovery_until) return false;      // mid-reset recovery
    return (millis() - s_last_attitude_ms) < 500;       // fresh fused sample
}

uint32_t resetCount()      { return s_reset_count; }
uint32_t enableFailCount() { return s_enable_fail; }
uint32_t reinitCount()     { return s_reinit_count; }

}  // namespace bno085
