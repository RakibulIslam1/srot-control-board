// =============================================================================
//  control/calibration — implementation
//
//  Fully implemented: GYRO, BARO_ZERO, LEVEL, MAG (min/max hard+soft iron),
//  ACCEL_6PT (min/max over 6 faces). Motor routines (MOTOR_DETECT/MOTOR_TEST)
//  sequence each thruster via the test-override hook; ESC_RANGE and JOYSTICK are
//  timed acknowledgements (ESC endpoint programming / GCS-side joystick cal).
// =============================================================================

#include "control/calibration.h"
#include <Preferences.h>
#include <math.h>
#include "config.h"
#include "state_types.h"
#include "control/mixer.h"

namespace calibration {

static const float GRAV = 9.80665f;

// --- persistence -------------------------------------------------------------
// Outcome of the last saveToNVS(), surfaced to the GCS. A silent failure here is the
// difference between "my calibration did not stick" and "my calibration did not stick and
// nothing told me", which is the state this vehicle was actually in.
static uint16_t    s_nvs_fail_n = 0;
static const char* s_nvs_fail_key = nullptr;
static uint32_t    s_nvs_saves_ok = 0;

void saveToNVS() {
    // Snapshot the cal state under the lock, then do the slow flash writes UNLOCKED
    // so cal-lock consumers aren't stalled for the flash-commit window. Call this
    // from Core 0 (deferred) — never from the flight loop.
    CalState c;
    {
        StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(10));
        if (!lk.ok()) return;
        c = g_state.cal;
    }
    // EVERY WRITE IS CHECKED. It was not, and that is how R14 (NVS partition too small)
    // stayed a mystery across several rounds: Preferences::putFloat() returns 0 on failure
    // and every return value here was discarded, so a full or unwritable partition looked
    // exactly like a successful save. A calibration you cannot trust to persist is worse
    // than one you know is not persisting.
    Preferences p;
    if (!p.begin(NVS_NS_CAL, false)) {
        s_nvs_fail_n++;
        s_nvs_fail_key = "<open>";
        return;
    }
    uint16_t fails = 0;
    const char* first_fail = nullptr;
    auto putF = [&](const char* k, float v) {
        if (p.putFloat(k, v) == 0) { fails++; if (!first_fail) first_fail = k; }
    };
    putF("gyro_ox", c.gyro_offset.x); putF("gyro_oy", c.gyro_offset.y); putF("gyro_oz", c.gyro_offset.z);
    putF("acc_ox", c.accel_offset.x); putF("acc_oy", c.accel_offset.y); putF("acc_oz", c.accel_offset.z);
    putF("acc_sx", c.accel_scale.x);  putF("acc_sy", c.accel_scale.y);  putF("acc_sz", c.accel_scale.z);
    putF("mag_ox", c.mag_offset.x);   putF("mag_oy", c.mag_offset.y);   putF("mag_oz", c.mag_offset.z);
    putF("mag_sx", c.mag_scale.x);    putF("mag_sy", c.mag_scale.y);    putF("mag_sz", c.mag_scale.z);
    putF("baro_z", c.baro_zero_mbar);
    putF("lvl_r", c.level_roll_off);  putF("lvl_p", c.level_pitch_off);
    char key[8];
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        snprintf(key, sizeof(key), "mdir%d", i);
        if (p.putChar(key, c.motor_dir[i]) == 0) { fails++; if (!first_fail) first_fail = "mdir"; }
    }
    p.end();

    // READ BACK. A write that "succeeded" into a partition with no room is the case that
    // fooled us before, so verify a representative sample actually survived rather than
    // trusting the return codes alone.
    if (fails == 0) {
        Preferences v;
        if (v.begin(NVS_NS_CAL, true)) {
            const bool ok =
                v.getFloat("mag_sx", NAN) == c.mag_scale.x &&
                v.getFloat("mag_ox", NAN) == c.mag_offset.x &&
                v.getFloat("lvl_r",  NAN) == c.level_roll_off;
            v.end();
            if (!ok) { fails++; first_fail = "<readback>"; }
        } else {
            fails++; first_fail = "<verify-open>";
        }
    }

    s_nvs_fail_n  = fails;
    s_nvs_fail_key = first_fail;
    if (fails == 0) s_nvs_saves_ok++;
}

uint16_t    nvsFailCount() { return s_nvs_fail_n; }
const char* nvsFailKey()   { return s_nvs_fail_key; }
uint32_t    nvsSaveOkCount() { return s_nvs_saves_ok; }

void loadFromNVS() {
    Preferences p;
    p.begin(NVS_NS_CAL, true);
    StateLock lk(g_state.mtx_cal);
    if (lk.ok()) {
        CalState& c = g_state.cal;
        c.gyro_offset = { p.getFloat("gyro_ox", 0), p.getFloat("gyro_oy", 0), p.getFloat("gyro_oz", 0) };
        c.accel_offset = { p.getFloat("acc_ox", 0), p.getFloat("acc_oy", 0), p.getFloat("acc_oz", 0) };
        c.accel_scale = { p.getFloat("acc_sx", 1), p.getFloat("acc_sy", 1), p.getFloat("acc_sz", 1) };
        c.mag_offset = { p.getFloat("mag_ox", 0), p.getFloat("mag_oy", 0), p.getFloat("mag_oz", 0) };
        c.mag_scale = { p.getFloat("mag_sx", 1), p.getFloat("mag_sy", 1), p.getFloat("mag_sz", 1) };
        c.baro_zero_mbar = p.getFloat("baro_z", 0);
        c.level_roll_off = p.getFloat("lvl_r", 0);
        c.level_pitch_off = p.getFloat("lvl_p", 0);
        char key[8];
        for (int i = 0; i < NUM_THRUSTERS; ++i) { snprintf(key, sizeof(key), "mdir%d", i); c.motor_dir[i] = p.getChar(key, 1); }
        c.loaded_from_nvs = true;
    }
    p.end();
}

// --- routine state -----------------------------------------------------------
static CalRoutine s_active = CalRoutine::NONE;
static uint32_t   s_start = 0;
static uint32_t   s_count = 0;
static double      s_ax = 0, s_ay = 0, s_az = 0;    // generic accumulators
static float       s_gyro_off0[3];                  // offsets at routine start
static float       s_min[3], s_max[3];              // mag / accel extents
static float       s_grav6[6][3];                   // accel 6-point samples
static uint8_t     s_face_mask = 0;
static uint8_t     s_last_step = 0xFF;
static int8_t      s_motor_idx = 0;
static int         s_md_phase = 0;      // motor-detect: 0=settle,1=thrust,2=detect
static uint32_t    s_md_t = 0;
static float       s_md_peak[3] = {0, 0, 0};

static void beginRoutine(CalRoutine r, uint32_t now) {
    s_active = r; s_start = now; s_count = 0;
    s_ax = s_ay = s_az = 0;
    for (int i = 0; i < 3; ++i) { s_min[i] = 1e9f; s_max[i] = -1e9f; }
    s_face_mask = 0; s_last_step = 0xFF; s_motor_idx = 0;
    s_md_phase = 0; s_md_t = now; s_md_peak[0] = s_md_peak[1] = s_md_peak[2] = 0;
    StateLock lk(g_state.mtx_cal);
    if (lk.ok()) {
        g_state.cal.progress = 0;
        s_gyro_off0[0] = g_state.cal.gyro_offset.x;
        s_gyro_off0[1] = g_state.cal.gyro_offset.y;
        s_gyro_off0[2] = g_state.cal.gyro_offset.z;
    }
}

static void setProgress(float pr) {
    StateLock lk(g_state.mtx_cal);
    if (lk.ok()) g_state.cal.progress = pr;
}

// Complete the active routine: record result, auto-persist to NVS, and bump the
// result sequence so the comms core can report success/failure to the GCS.
static void finish(CalResult result = CalResult::SUCCESS) {
    CalRoutine done = s_active;
    {
        StateLock lk(g_state.mtx_cal);
        if (lk.ok()) {
            g_state.cal.routine = CalRoutine::NONE;
            g_state.cal.progress = 1.0f;
            g_state.cal.result = result;
            g_state.cal.last_routine = done;
            g_state.cal.result_seq++;
            // Persist every successful calibration — but flag it for the COMMS core
            // to write (NVS flash from this Core-1 flight loop would stall it).
            if (result == CalResult::SUCCESS) g_state.cal.persist_pending = true;
        }
    }
    s_active = CalRoutine::NONE;
}

static void driveTestMotor(int8_t motor, float thr, uint32_t dur_ms) {
    StateLock lk(g_state.mtx_thrusters);
    if (lk.ok()) {
        g_state.thrusters.test_override = true;
        g_state.thrusters.test_motor = motor;
        g_state.thrusters.test_throttle = thr;
        g_state.thrusters.test_expire_ms = millis() + dur_ms;
    }
}

void update(float roll, float pitch, float yaw,
            float gx, float gy, float gz,
            float grav_x, float grav_y, float grav_z,
            float mx, float my, float mz,
            float pressure_mbar, uint32_t now) {
    // Detect a newly-requested routine. (NVS persistence is handled off-core by the
    // comms task via cal.persist_pending — no flash writes on this flight loop.)
    CalRoutine req;
    uint8_t step;
    {
        StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
        if (!lk.ok()) return;
        req = g_state.cal.routine;
        step = g_state.cal.step;
    }

    if (req == CalRoutine::NONE) { s_active = CalRoutine::NONE; return; }
    if (req != s_active) beginRoutine(req, now);

    uint32_t elapsed = now - s_start;

    switch (s_active) {
        case CalRoutine::GYRO: {
            // Accumulate raw gyro (published + prior offset) while still.
            s_ax += gx + s_gyro_off0[0];
            s_ay += gy + s_gyro_off0[1];
            s_az += gz + s_gyro_off0[2];
            s_count++;
            setProgress(constrain(elapsed / 1500.0f, 0.0f, 1.0f));
            if (elapsed >= 1500 && s_count > 0) {
                {   // scope the lock — finish() re-locks mtx_cal, must not nest
                    StateLock lk(g_state.mtx_cal);
                    if (lk.ok()) g_state.cal.gyro_offset = {
                        (float)(s_ax / s_count), (float)(s_ay / s_count), (float)(s_az / s_count) };
                }
                finish();
            }
            break;
        }

        case CalRoutine::BARO_ZERO: {
            s_ax += pressure_mbar; s_count++;
            setProgress(constrain(elapsed / 1000.0f, 0.0f, 1.0f));
            if (elapsed >= 1000 && s_count > 0) {
                {
                    StateLock lk(g_state.mtx_cal);
                    if (lk.ok()) g_state.cal.baro_zero_mbar = (float)(s_ax / s_count);
                }
                finish();
            }
            break;
        }

        case CalRoutine::LEVEL: {
            // Reconstruct raw attitude and average it as the mounting trim.
            float rr, pp;
            { StateLock lk(g_state.mtx_cal); rr = lk.ok() ? g_state.cal.level_roll_off : 0;
              pp = lk.ok() ? g_state.cal.level_pitch_off : 0; }
            s_ax += roll + rr; s_ay += pitch + pp; s_count++;
            setProgress(constrain(elapsed / 1000.0f, 0.0f, 1.0f));
            if (elapsed >= 1000 && s_count > 0) {
                {
                    StateLock lk(g_state.mtx_cal);
                    if (lk.ok()) { g_state.cal.level_roll_off = (float)(s_ax / s_count);
                                   g_state.cal.level_pitch_off = (float)(s_ay / s_count); }
                }
                finish();
            }
            break;
        }

        case CalRoutine::MAG: {
#if !BNO_USE_MAG
            // Magnetometer is ignored on this vehicle — compass cal is a no-op.
            (void)mx; (void)my; (void)mz;
            finish(CalResult::SUCCESS);
            break;
#else
            float v[3] = { mx, my, mz };
            for (int i = 0; i < 3; ++i) { s_min[i] = min(s_min[i], v[i]); s_max[i] = max(s_max[i], v[i]); }
            setProgress(constrain(elapsed / 15000.0f, 0.0f, 1.0f));   // 15 s rotate
            if (elapsed >= 15000) {
                float rng[3], avg = 0;
                for (int i = 0; i < 3; ++i) { rng[i] = (s_max[i] - s_min[i]) * 0.5f; avg += rng[i]; }
                avg /= 3.0f;
                {
                    StateLock lk(g_state.mtx_cal);
                    if (lk.ok()) {
                        g_state.cal.mag_offset = { (s_max[0] + s_min[0]) * 0.5f,
                                                   (s_max[1] + s_min[1]) * 0.5f,
                                                   (s_max[2] + s_min[2]) * 0.5f };
                        g_state.cal.mag_scale = { rng[0] > 0 ? avg / rng[0] : 1.0f,
                                                  rng[1] > 0 ? avg / rng[1] : 1.0f,
                                                  rng[2] > 0 ? avg / rng[2] : 1.0f };
                    }
                }
                finish();
            }
            break;
#endif  // BNO_USE_MAG
        }

        case CalRoutine::ACCEL_6PT: {
            // Abort if the GCS never drives the 6 positions (avoid hanging at N/6).
            if (elapsed > 120000) { finish(CalResult::FAIL); break; }
            // Capture one gravity sample each time the GCS advances the step (0..5).
            if (step != s_last_step && step < 6) {
                s_grav6[step][0] = grav_x; s_grav6[step][1] = grav_y; s_grav6[step][2] = grav_z;
                s_face_mask |= (1 << step);
                s_last_step = step;
            }
            setProgress((float)__builtin_popcount(s_face_mask) / 6.0f);
            if (s_face_mask == 0x3F) {   // all 6 faces captured
                float mn[3] = { 1e9f, 1e9f, 1e9f }, mx3[3] = { -1e9f, -1e9f, -1e9f };
                for (int f = 0; f < 6; ++f)
                    for (int a = 0; a < 3; ++a) { mn[a] = min(mn[a], s_grav6[f][a]); mx3[a] = max(mx3[a], s_grav6[f][a]); }
                {
                    StateLock lk(g_state.mtx_cal);
                    if (lk.ok()) {
                        for (int a = 0; a < 3; ++a) {
                            float off = (mx3[a] + mn[a]) * 0.5f;
                            float half = (mx3[a] - mn[a]) * 0.5f;
                            float sc = (half > 0.1f) ? (GRAV / half) : 1.0f;
                            if (a == 0) { g_state.cal.accel_offset.x = off; g_state.cal.accel_scale.x = sc; }
                            if (a == 1) { g_state.cal.accel_offset.y = off; g_state.cal.accel_scale.y = sc; }
                            if (a == 2) { g_state.cal.accel_offset.z = off; g_state.cal.accel_scale.z = sc; }
                        }
                    }
                }
                finish();
            }
            break;
        }

        case CalRoutine::MOTOR_DETECT: {
            // ArduSub-style per-motor detect (run in water, ARMED): settle until
            // gyro quiet, pulse the motor up, compare the measured gyro to the
            // frame's expected angular factor, and store MOTn_DIR (±1).
            float gmag = sqrtf(gx * gx + gy * gy + gz * gz);
            if (s_motor_idx >= NUM_THRUSTERS) { finish(); break; }
            if (s_md_phase == 0) {                 // SETTLE — hold neutral, wait quiet 500 ms
                driveTestMotor(-1, 0, 100);
                if (gmag > 0.15f) s_md_t = now;
                if (now - s_md_t >= 500) { s_md_phase = 1; s_md_t = now;
                                           s_md_peak[0] = s_md_peak[1] = s_md_peak[2] = 0; }
            } else if (s_md_phase == 1) {          // THRUST — pulse up, capture peak gyro
                driveTestMotor((int8_t)s_motor_idx, 0.30f, 200);
                if (fabsf(gx) > fabsf(s_md_peak[0])) s_md_peak[0] = gx;
                if (fabsf(gy) > fabsf(s_md_peak[1])) s_md_peak[1] = gy;
                if (fabsf(gz) > fabsf(s_md_peak[2])) s_md_peak[2] = gz;
                if (now - s_md_t >= 500) s_md_phase = 2;
            } else {                                // DETECT — sign vs expected
                float e[3]; mixer::motorAngular(s_motor_idx, e[0], e[1], e[2]);
                int dom = 0;
                for (int a = 1; a < 3; ++a) if (fabsf(e[a]) > fabsf(e[dom])) dom = a;
                int8_t dir = 1;
                if (fabsf(e[dom]) > 0.1f && fabsf(s_md_peak[dom]) > 0.05f)
                    dir = ((s_md_peak[dom] > 0) == (e[dom] > 0)) ? 1 : -1;
                { StateLock lk(g_state.mtx_cal); if (lk.ok()) g_state.cal.motor_dir[s_motor_idx] = dir; }
                s_motor_idx++; s_md_phase = 0; s_md_t = now;
                if (s_motor_idx >= NUM_THRUSTERS) { finish(); break; }
            }
            setProgress((float)s_motor_idx / NUM_THRUSTERS);
            break;
        }

        case CalRoutine::MOTOR_TEST: {
            // Simple sequence: each thruster at a gentle throttle for 1 s.
            const uint32_t PER = 1000;
            int idx = elapsed / PER;
            if (idx >= NUM_THRUSTERS) { finish(); break; }
            driveTestMotor((int8_t)idx, 0.15f, PER);
            setProgress((float)idx / NUM_THRUSTERS);
            break;
        }

        case CalRoutine::ESC_RANGE:
        case CalRoutine::JOYSTICK: {
            // ESC endpoint programming is done with a dedicated procedure; joystick
            // calibration is GCS-side. Acknowledge over ~1 s and complete.
            setProgress(constrain(elapsed / 1000.0f, 0.0f, 1.0f));
            if (elapsed >= 1000) finish();
            break;
        }

        default:
            finish();
            break;
    }
}

}  // namespace calibration
