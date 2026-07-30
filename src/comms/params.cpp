// =============================================================================
//  comms/params — implementation (runtime table + NVS via Preferences)
//
//  The table is built at init() so it can include the generated SERVO9..16
//  params. Each row carries a MAVLink `name` (≤16, what QGC sees) and a separate
//  `nvskey` (≤15, the Preferences limit) — most rows use the same string, but the
//  long "SERVOn_FUNCTION" names need a short NVS key.
// =============================================================================

#include "comms/params.h"
#include <Preferences.h>
#include <nvs_flash.h>     // nvs_flash_init/erase — partition-growth recovery, see init()
#include <math.h>          // NAN sentinel for the skip-if-unchanged compare
#include <string.h>        // memcmp
#include "config.h"
#include "state_types.h"   // g_state.cal — backing store for the CAL_* rows

Params g_params;

namespace params {

// Set by init() when PARAM_DEFAULTS_VER changed and every param was rewritten from its
// compiled default — surfaced to the GCS once the MAVLink link is up.
static bool s_defaults_were_forced = false;

// --- calibration-as-parameters ------------------------------------------------
// Sensor calibration lives in g_state.cal and persists to a SEPARATE NVS namespace
// (NVS_NS_CAL) written by calibration::saveToNVS(). It is exposed here as read/write
// parameters purely so a GCS can back it up and restore it — the mag/accel/level cal
// and the MOTOR_DETECT directions are otherwise unrecoverable after an NVS wipe, and
// redoing the mag cal means physically spinning the vehicle.
//
// These rows are a VIEW, not a copy: they never touch s_prefs and are never seeded
// from a DEF_*. Duplicating them into NVS_NS_PARAMS would create a second source of
// truth that goes stale the moment a calibration routine runs (and NVS is only 0x5000,
// already tight for the existing table).
enum : uint8_t {
    CALP_NONE = 0,
    CALP_GYRO_OX, CALP_GYRO_OY, CALP_GYRO_OZ,
    CALP_ACC_OX,  CALP_ACC_OY,  CALP_ACC_OZ,
    CALP_ACC_SX,  CALP_ACC_SY,  CALP_ACC_SZ,
    CALP_MAG_OX,  CALP_MAG_OY,  CALP_MAG_OZ,
    CALP_MAG_SX,  CALP_MAG_SY,  CALP_MAG_SZ,
    CALP_BARO_Z,  CALP_LVL_R,   CALP_LVL_P,
    CALP_MDIR1,   // .. CALP_MDIR1 + 7 for the 8 thrusters
    CALP_MDIR_LAST = CALP_MDIR1 + NUM_THRUSTERS - 1,
    CALP_COUNT
};

// Last successfully-read value per id. mtx_cal is contended by the 500 Hz flight loop,
// so a read CAN miss the lock — returning 0 there would make a param download report a
// bogus calibration (and, if the GCS then wrote that file back, destroy the real one).
// Serving the last known value instead keeps a miss harmless.
//
// SEEDED FROM THE TABLE DEFAULTS in buildTable(), not left at zero. Zero-initialising was
// a hole in exactly the scenario this exists to prevent: on the FIRST post-boot param
// download, a lock miss on a *scale* row (CAL_ACC_S*, CAL_MAG_S*, whose real default is
// 1.0) would report 0.0 — and a scale of zero, exported and later imported, silently
// destroys the accel/mag calibration. The `def` field on those rows now actually does
// something instead of being decorative.
static float s_cal_shadow[CALP_COUNT] = {0};

// Resolve a cal id to its float field. motor_dir is int8_t and handled separately.
static float* calField(uint8_t id, CalState& c) {
    switch (id) {
        case CALP_GYRO_OX: return &c.gyro_offset.x;
        case CALP_GYRO_OY: return &c.gyro_offset.y;
        case CALP_GYRO_OZ: return &c.gyro_offset.z;
        case CALP_ACC_OX:  return &c.accel_offset.x;
        case CALP_ACC_OY:  return &c.accel_offset.y;
        case CALP_ACC_OZ:  return &c.accel_offset.z;
        case CALP_ACC_SX:  return &c.accel_scale.x;
        case CALP_ACC_SY:  return &c.accel_scale.y;
        case CALP_ACC_SZ:  return &c.accel_scale.z;
        case CALP_MAG_OX:  return &c.mag_offset.x;
        case CALP_MAG_OY:  return &c.mag_offset.y;
        case CALP_MAG_OZ:  return &c.mag_offset.z;
        case CALP_MAG_SX:  return &c.mag_scale.x;
        case CALP_MAG_SY:  return &c.mag_scale.y;
        case CALP_MAG_SZ:  return &c.mag_scale.z;
        case CALP_BARO_Z:  return &c.baro_zero_mbar;
        case CALP_LVL_R:   return &c.level_roll_off;
        case CALP_LVL_P:   return &c.level_pitch_off;
        default:           return nullptr;
    }
}

static float calGet(uint8_t id) {
    if (id == CALP_NONE || id >= CALP_COUNT) return 0.0f;
    StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(5));
    if (!lk.ok()) return s_cal_shadow[id];          // lock miss -> last known good
    float v;
    if (id >= CALP_MDIR1 && id <= CALP_MDIR_LAST) {
        v = (float)g_state.cal.motor_dir[id - CALP_MDIR1];
    } else {
        float* f = calField(id, g_state.cal);
        v = f ? *f : 0.0f;
    }
    s_cal_shadow[id] = v;
    return v;
}

static bool calSet(uint8_t id, float value) {
    if (id == CALP_NONE || id >= CALP_COUNT) return false;
    StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(5));
    if (!lk.ok()) return false;                     // caller sees the write fail, not a lie
    if (id >= CALP_MDIR1 && id <= CALP_MDIR_LAST) {
        // Only ±1 is meaningful; the control loop treats 0 as +1 anyway.
        g_state.cal.motor_dir[id - CALP_MDIR1] = (value >= 0.0f) ? 1 : -1;
    } else {
        float* f = calField(id, g_state.cal);
        if (!f) return false;
        *f = value;
    }
    s_cal_shadow[id] = value;
    // Flushed to NVS_NS_CAL by mav_commands::update() on Core 0 — never a flash write
    // from here, which may be the MAVLink task mid-parse.
    g_state.cal.persist_pending = true;
    return true;
}

struct Desc {
    const char* name;    // MAVLink param_id (≤16)
    const char* nvskey;  // NVS key (≤15)
    float*      ptr;
    float       def;
    uint8_t     cal_id;  // 0 = ordinary param; else a CALP_* view onto g_state.cal
};

// Scalar params (name == nvskey; all ≤15 chars).
static const Desc SCALARS[] = {
    // Informational: which defaults schema this firmware was built with. Forced in init()
    // and read by nothing, so it is safe to import from a file. Lets an exported backup
    // record whether it predates a PARAM_DEFAULTS_VER bump.
    { "SYS_PARAM_VER", "SYS_PARAM_VER", &g_params.param_ver, (float)PARAM_DEFAULTS_VER },
    { "STUNT_SPIN_CNT", "STUNT_SPIN_CNT", &g_params.stunt_spin_cnt, DEF_STUNT_SPIN_CNT },
    { "STUNT_RATE",     "STUNT_RATE",     &g_params.stunt_rate,     DEF_STUNT_RATE     },
    { "HEADROOM_DEPTH", "HEADROOM_DEPTH", &g_params.headroom_depth, DEF_HEADROOM_DEPTH },
    { "RGB_BRIGHTNESS", "RGB_BRIGHTNESS", &g_params.rgb_brightness, (float)DEF_RGB_BRIGHTNESS },
    { "PM1_SRC",        "PM1_SRC",        &g_params.pm1_src,        1.0f },
    { "PM2_SRC",        "PM2_SRC",        &g_params.pm2_src,        2.0f },
    { "PM1_VMULT",      "PM1_VMULT",      &g_params.pm1_vmult,      DEF_PM1_VMULT },
    { "PM2_VMULT",      "PM2_VMULT",      &g_params.pm2_vmult,      DEF_PM2_VMULT },
    { "LEAK_EN",        "LEAK_EN",        &g_params.leak_en,        0.0f },
    // Attitude gains use the ArduSub-standard ATC_* names so QGC's Tuning page tunes
    // the REAL SROT controller (backing g_params fields unchanged).
    { "ATC_ANG_RLL_P",  "ATC_ANG_RLL_P",  &g_params.ang_rll_p,      DEF_ANG_RLL_P },
    { "ATC_ANG_PIT_P",  "ATC_ANG_PIT_P",  &g_params.ang_pit_p,      DEF_ANG_PIT_P },
    { "ATC_ANG_YAW_P",  "ATC_ANG_YAW_P",  &g_params.ang_yaw_p,      DEF_ANG_YAW_P },
    { "ATC_RAT_RLL_P",  "ATC_RAT_RLL_P",  &g_params.rat_rll_p,      DEF_RAT_RLL_P },
    { "ATC_RAT_RLL_I",  "ATC_RAT_RLL_I",  &g_params.rat_rll_i,      DEF_RAT_RLL_I },
    { "ATC_RAT_RLL_D",  "ATC_RAT_RLL_D",  &g_params.rat_rll_d,      DEF_RAT_RLL_D },
    { "ATC_RAT_RLL_IMAX", "atc_rll_imax", &g_params.rat_rll_imax,   0.5f },
    { "ATC_RAT_RLL_FF", "ATC_RAT_RLL_FF", &g_params.rat_rll_ff,     DEF_RAT_RLL_FF },
    { "ATC_RAT_PIT_P",  "ATC_RAT_PIT_P",  &g_params.rat_pit_p,      DEF_RAT_PIT_P },
    { "ATC_RAT_PIT_I",  "ATC_RAT_PIT_I",  &g_params.rat_pit_i,      DEF_RAT_PIT_I },
    { "ATC_RAT_PIT_D",  "ATC_RAT_PIT_D",  &g_params.rat_pit_d,      DEF_RAT_PIT_D },
    { "ATC_RAT_PIT_IMAX", "atc_pit_imax", &g_params.rat_pit_imax,   0.5f },
    { "ATC_RAT_PIT_FF", "ATC_RAT_PIT_FF", &g_params.rat_pit_ff,     DEF_RAT_PIT_FF },
    { "ATC_RAT_YAW_P",  "ATC_RAT_YAW_P",  &g_params.rat_yaw_p,      DEF_RAT_YAW_P },
    { "ATC_RAT_YAW_I",  "ATC_RAT_YAW_I",  &g_params.rat_yaw_i,      DEF_RAT_YAW_I },
    { "ATC_RAT_YAW_D",  "ATC_RAT_YAW_D",  &g_params.rat_yaw_d,      DEF_RAT_YAW_D },
    { "ATC_RAT_YAW_IMAX", "atc_yaw_imax", &g_params.rat_yaw_imax,   0.5f },
    { "ATC_RAT_YAW_FF", "ATC_RAT_YAW_FF", &g_params.rat_yaw_ff,     DEF_RAT_YAW_FF },
    { "DEPTH_P",        "DEPTH_P",        &g_params.depth_p,        DEF_DEPTH_P },
    { "DEPTH_I",        "DEPTH_I",        &g_params.depth_i,        DEF_DEPTH_I },
    { "DEPTH_D",        "DEPTH_D",        &g_params.depth_d,        DEF_DEPTH_D },
    // Model-based control (Phase 2a) — see ALGORITHMS.md §5.
    { "ATC_DRAG_RLL",   "ATC_DRAG_RLL",   &g_params.atc_drag_rll,   DEF_ATC_DRAG_RLL },
    { "ATC_DRAG_PIT",   "ATC_DRAG_PIT",   &g_params.atc_drag_pit,   DEF_ATC_DRAG_PIT },
    { "ATC_DRAG_YAW",   "ATC_DRAG_YAW",   &g_params.atc_drag_yaw,   DEF_ATC_DRAG_YAW },
    { "XC_YAW2RLL",     "XC_YAW2RLL",     &g_params.xc_yaw2rll,     DEF_XC_YAW2RLL },
    { "XC_YAW2PIT",     "XC_YAW2PIT",     &g_params.xc_yaw2pit,     DEF_XC_YAW2PIT },
    { "TRIM_EN",        "TRIM_EN",        &g_params.trim_en,        DEF_TRIM_EN },
    { "TRIM_LEAK",      "TRIM_LEAK",      &g_params.trim_leak,      DEF_TRIM_LEAK },
    { "TRIM_MAX",       "TRIM_MAX",       &g_params.trim_max,       DEF_TRIM_MAX },

    { "BUZZ_MASK",      "BUZZ_MASK",      &g_params.buzz_mask,      DEF_BUZZ_MASK },
    { "THR_BEEP_EN",    "THR_BEEP_EN",    &g_params.thr_beep_en,    DEF_THR_BEEP_EN },
    { "BUZZ_DUTY",      "BUZZ_DUTY",      &g_params.buzz_duty,      DEF_BUZZ_DUTY },

    // Autotune safety monitor
    { "ST_ANGLE_MAX",   "ST_ANGLE_MAX",   &g_params.st_angle_max,   DEF_ST_ANGLE_MAX },
    { "ST_RATE_MAX",    "ST_RATE_MAX",    &g_params.st_rate_max,    DEF_ST_RATE_MAX },
    { "ST_DEPTH_DELTA", "ST_DEPTH_DELT",  &g_params.st_depth_delta, DEF_ST_DEPTH_DELTA },
    { "ST_RPM_MAX",     "ST_RPM_MAX",     &g_params.st_rpm_max,     DEF_ST_RPM_MAX },
    // Motor closed-loop RPM control (MOTOR_TUNE writes these; pushed to the Pico)
    { "RPM_KP",         "RPM_KP",         &g_params.rpm_kp,         DEF_RPM_KP },
    { "RPM_KI",         "RPM_KI",         &g_params.rpm_ki,         DEF_RPM_KI },
    { "RPM_FF_A",       "RPM_FF_A",       &g_params.rpm_ff_a,       DEF_RPM_FF_A },
    { "RPM_IDLE",       "RPM_IDLE",       &g_params.rpm_idle,       DEF_RPM_IDLE },
    { "RPM_MAX",        "RPM_MAX",        &g_params.rpm_max,        DEF_RPM_MAX },
    { "RPM_FILT",       "RPM_FILT",       &g_params.rpm_filt,       DEF_RPM_FILT },
    { "RPM_SLEW",       "RPM_SLEW",       &g_params.rpm_slew,       DEF_RPM_SLEW },
    { "RPM_MIN_TGT",    "RPM_MIN_TGT",    &g_params.rpm_min_tgt,    DEF_RPM_MIN_TGT },
    { "RPM_LOOP",       "RPM_LOOP",       &g_params.rpm_loop,       DEF_RPM_LOOP },
    { "DSHOT_BIDIR",    "DSHOT_BIDIR",    &g_params.dshot_bidir,    DEF_DSHOT_BIDIR },
    { "MTUNE_EN",       "MTUNE_EN",       &g_params.mtune_en,       DEF_MTUNE_EN },
    // AUTO-mode movement
    { "MOVE_CRUISE_MAX","MOVE_CRUISE_MX", &g_params.move_cruise_max, DEF_MOVE_CRUISE_MAX },
    { "MOVE_ACCEL",     "MOVE_ACCEL",     &g_params.move_accel,     DEF_MOVE_ACCEL },
    { "MOVE_BRAKE_GAIN","MOVE_BRAKE_GN",  &g_params.move_brake_gain,DEF_MOVE_BRAKE_GAIN },
    { "MOVE_BRAKE_K",   "MOVE_BRAKE_K",   &g_params.move_brake_k,   DEF_MOVE_BRAKE_K },
    { "MOVE_DEPTH_RATE","MOVE_DEPTH_RT",  &g_params.move_depth_rate,DEF_MOVE_DEPTH_RATE },
    { "MOVE_YAW_RATE",  "MOVE_YAW_RATE",  &g_params.move_yaw_rate,  DEF_MOVE_YAW_RATE },
    // ArduSub-standard motor-reverse names so QGC's Motors page reads them (and no
    // "MOT_n_DIRECTION missing" popup). Value +1 normal / -1 reversed.
    { "MOT_1_DIRECTION", "MOT_1_DIRECTION", &g_params.mot_dir[0], 1.0f },
    { "MOT_2_DIRECTION", "MOT_2_DIRECTION", &g_params.mot_dir[1], 1.0f },
    { "MOT_3_DIRECTION", "MOT_3_DIRECTION", &g_params.mot_dir[2], 1.0f },
    { "MOT_4_DIRECTION", "MOT_4_DIRECTION", &g_params.mot_dir[3], 1.0f },
    { "MOT_5_DIRECTION", "MOT_5_DIRECTION", &g_params.mot_dir[4], 1.0f },
    { "MOT_6_DIRECTION", "MOT_6_DIRECTION", &g_params.mot_dir[5], 1.0f },
    { "MOT_7_DIRECTION", "MOT_7_DIRECTION", &g_params.mot_dir[6], 1.0f },
    { "MOT_8_DIRECTION", "MOT_8_DIRECTION", &g_params.mot_dir[7], 1.0f },
    { "BTN0_FUNCTION",  "BTN0_FUNCTION",  &g_params.btn_func[0],  3.0f  },
    { "BTN1_FUNCTION",  "BTN1_FUNCTION",  &g_params.btn_func[1],  4.0f  },
    { "BTN2_FUNCTION",  "BTN2_FUNCTION",  &g_params.btn_func[2],  6.0f  },
    { "BTN3_FUNCTION",  "BTN3_FUNCTION",  &g_params.btn_func[3],  7.0f  },
    { "BTN4_FUNCTION",  "BTN4_FUNCTION",  &g_params.btn_func[4],  5.0f  },
    { "BTN5_FUNCTION",  "BTN5_FUNCTION",  &g_params.btn_func[5],  32.0f },
    { "BTN6_FUNCTION",  "BTN6_FUNCTION",  &g_params.btn_func[6],  33.0f },
    { "BTN7_FUNCTION",  "BTN7_FUNCTION",  &g_params.btn_func[7],  53.0f },
    { "BTN8_FUNCTION",  "BTN8_FUNCTION",  &g_params.btn_func[8],  0.0f  },
    { "BTN9_FUNCTION",  "BTN9_FUNCTION",  &g_params.btn_func[9],  0.0f  },
    { "BTN10_FUNCTION", "BTN10_FUNCTION", &g_params.btn_func[10], 0.0f  },
    { "BTN11_FUNCTION", "BTN11_FUNCTION", &g_params.btn_func[11], 0.0f  },
    { "BTN12_FUNCTION", "BTN12_FUNCTION", &g_params.btn_func[12], 0.0f  },
    { "BTN13_FUNCTION", "BTN13_FUNCTION", &g_params.btn_func[13], 0.0f  },
    { "BTN14_FUNCTION", "BTN14_FUNCTION", &g_params.btn_func[14], 0.0f  },
    { "BTN15_FUNCTION", "BTN15_FUNCTION", &g_params.btn_func[15], 0.0f  },
    { "FRAME_CONFIG",   "FRAME_CONFIG",   &g_params.frame_config,   2.0f  },
    { "JS_GAIN_DEFAULT","JS_GAIN_DEFAULT",&g_params.js_gain_default,0.5f  },
    { "LIGHTS_STEP",    "LIGHTS_STEP",    &g_params.lights_step,    0.1f  },
    { "FS_GCS_ENABLE",  "FS_GCS_ENABLE",  &g_params.fs_gcs_enable,  1.0f  },
    { "FS_BAT_ENABLE",  "FS_BAT_ENABLE",  &g_params.fs_bat_enable,  1.0f  },
    { "FS_BAT_VOLTAGE", "FS_BAT_VOLTAGE", &g_params.fs_bat_voltage, 13.2f },
    { "PILOT_SPEED",    "PILOT_SPEED",    &g_params.pilot_speed,    DEF_PILOT_SPEED },
    { "PILOT_YAW_RATE", "PILOT_YAW_RATE", &g_params.pilot_yaw_rate, DEF_PILOT_YAW_RATE },
    { "PILOT_EXPO",     "PILOT_EXPO",     &g_params.pilot_expo,     DEF_PILOT_EXPO },
    { "ARMING_CHECK",   "ARMING_CHECK",   &g_params.arming_check,   1.0f  },
    { "ATUNE",          "ATUNE",          &g_params.atune,          0.0f  },
    { "ESPNOW_EN",      "ESPNOW_EN",      &g_params.espnow_en,      0.0f  },
    { "MOT_PWM_TYPE",   "MOT_PWM_TYPE",   &g_params.mot_pwm_type,   4.0f /*DShot150*/ },
    { "MOT_PWM_MIN",    "MOT_PWM_MIN",    &g_params.mot_pwm_min,    1100.0f },
    { "MOT_PWM_MAX",    "MOT_PWM_MAX",    &g_params.mot_pwm_max,    1900.0f },
    { "MOT_THST_EXPO",  "MOT_THST_EXPO",  &g_params.mot_thst_expo,  DEF_MOT_THST_EXPO },
    { "MOT_SPIN_MIN",   "MOT_SPIN_MIN",   &g_params.mot_spin_min,   DEF_MOT_SPIN_MIN },
    { "MOT_SPIN_ARM",   "MOT_SPIN_ARM",   &g_params.mot_spin_arm,   DEF_MOT_SPIN_ARM },
    { "THR_TRIM_EN",    "THR_TRIM_EN",    &g_params.thr_trim_en,    DEF_THR_TRIM_EN },
    { "THR_TRIM_TAU",   "THR_TRIM_TAU",   &g_params.thr_trim_tau,   DEF_THR_TRIM_TAU },
    { "THR_TRIM_MAX",   "THR_TRIM_MAX",   &g_params.thr_trim_max,   DEF_THR_TRIM_MAX },
    { "MOT_BAT_V_MIN",  "MOT_BAT_V_MIN",  &g_params.mot_bat_v_min,  DEF_MOT_BAT_V_MIN },
    { "MOT_BAT_V_MAX",  "MOT_BAT_V_MAX",  &g_params.mot_bat_v_max,  DEF_MOT_BAT_V_MAX },
    // One-shot magnetic yaw reference (control/yaw_ref). Default OFF: yaw stays relative
    // and the vehicle behaves exactly as before until you calibrate the mag and opt in.
    { "MAG_YAW_REF",    "MAG_YAW_REF",    &g_params.mag_yaw_ref,    DEF_MAG_YAW_REF },
    { "MAG_DECL",       "MAG_DECL",       &g_params.mag_decl,       DEF_MAG_DECL },
    { "MAG_ALIGN",      "MAG_ALIGN",      &g_params.mag_align,      0.0f },
    // Configurable GPIOs (defaults = the wired pinout; reboot to apply).
    // (PIN_THR1..8 removed — thrusters are on the Pico co-processor by default; the
    //  RMT fallback uses the fixed compile-time THRUSTER_PINS in config.h.)
    { "PIN_BUZZER",   "PIN_BUZZER",   &g_params.pin_buzzer,   (float)PIN_BUZZER },
    { "PIN_RGB",      "PIN_RGB",      &g_params.pin_rgb,      (float)PIN_WS2812B },
    { "PIN_LEAK",     "PIN_LEAK",     &g_params.pin_leak,     (float)PIN_LEAK },
    { "PIN_BATTVOLT", "PIN_BATTVOLT", &g_params.pin_battvolt, (float)PIN_BATT_VOLT },
    { "PIN_BATTCURR", "PIN_BATTCURR", &g_params.pin_battcurr, (float)PIN_BATT_CURR },
    { "PIN_SDCS",     "PIN_SDCS",     &g_params.pin_sdcs,     (float)PIN_SD_CS },
    { "PIN_LORACS",   "PIN_LORACS",   &g_params.pin_loracs,   (float)PIN_LORA_CS },
    { "PIN_LORADIO",  "PIN_LORADIO",  &g_params.pin_loradio,  (float)PIN_LORA_DIO0 },

    // --- Sensor calibration (VIEW onto g_state.cal — see the CALP_* block above) ---
    // ptr is null and def is unused for these: they read/write g_state.cal through
    // calGet/calSet and persist to NVS_NS_CAL, not NVS_NS_PARAMS. Exposed so a GCS can
    // export/import them; a defaults bump and PREFLIGHT_STORAGE-reset both leave them
    // alone. Editing these by hand is a good way to break your attitude solution.
    { "CAL_GYRO_OX", "", nullptr, 0.0f, CALP_GYRO_OX },
    { "CAL_GYRO_OY", "", nullptr, 0.0f, CALP_GYRO_OY },
    { "CAL_GYRO_OZ", "", nullptr, 0.0f, CALP_GYRO_OZ },
    { "CAL_ACC_OX",  "", nullptr, 0.0f, CALP_ACC_OX  },
    { "CAL_ACC_OY",  "", nullptr, 0.0f, CALP_ACC_OY  },
    { "CAL_ACC_OZ",  "", nullptr, 0.0f, CALP_ACC_OZ  },
    { "CAL_ACC_SX",  "", nullptr, 1.0f, CALP_ACC_SX  },
    { "CAL_ACC_SY",  "", nullptr, 1.0f, CALP_ACC_SY  },
    { "CAL_ACC_SZ",  "", nullptr, 1.0f, CALP_ACC_SZ  },
    { "CAL_MAG_OX",  "", nullptr, 0.0f, CALP_MAG_OX  },
    { "CAL_MAG_OY",  "", nullptr, 0.0f, CALP_MAG_OY  },
    { "CAL_MAG_OZ",  "", nullptr, 0.0f, CALP_MAG_OZ  },
    { "CAL_MAG_SX",  "", nullptr, 1.0f, CALP_MAG_SX  },
    { "CAL_MAG_SY",  "", nullptr, 1.0f, CALP_MAG_SY  },
    { "CAL_MAG_SZ",  "", nullptr, 1.0f, CALP_MAG_SZ  },
    { "CAL_BARO_Z",  "", nullptr, 0.0f, CALP_BARO_Z  },
    { "CAL_LVL_R",   "", nullptr, 0.0f, CALP_LVL_R   },
    { "CAL_LVL_P",   "", nullptr, 0.0f, CALP_LVL_P   },
    { "CAL_MDIR1",   "", nullptr, 1.0f, CALP_MDIR1 + 0 },
    { "CAL_MDIR2",   "", nullptr, 1.0f, CALP_MDIR1 + 1 },
    { "CAL_MDIR3",   "", nullptr, 1.0f, CALP_MDIR1 + 2 },
    { "CAL_MDIR4",   "", nullptr, 1.0f, CALP_MDIR1 + 3 },
    { "CAL_MDIR5",   "", nullptr, 1.0f, CALP_MDIR1 + 4 },
    { "CAL_MDIR6",   "", nullptr, 1.0f, CALP_MDIR1 + 5 },
    { "CAL_MDIR7",   "", nullptr, 1.0f, CALP_MDIR1 + 6 },
    { "CAL_MDIR8",   "", nullptr, 1.0f, CALP_MDIR1 + 7 },
};
static const uint16_t N_SCALARS = sizeof(SCALARS) / sizeof(SCALARS[0]);

// Servo params (16 channels × 5 fields: FUNCTION/MIN/MAX/TRIM/ROLE) generated at init().
static const int  N_SERVO_CH = 16;
static const int  N_SERVO = N_SERVO_CH * 5;

// (SROT-native: the QGC ArduSub-compat dummy params were removed — Bondor is the GCS.)
static Desc  s_table[N_SCALARS + N_SERVO];
static char  s_names[N_SERVO][17];
static char  s_keys[N_SERVO][16];
static uint8_t s_type[N_SCALARS + N_SERVO];   // MAV_PARAM_TYPE per row
static uint16_t s_n = 0;

// MAV_PARAM_TYPE values (avoid pulling the mavlink headers into this file).
static const uint8_t PT_INT32 = 6, PT_REAL32 = 9;

// Integer-like params get INT32 so QGC edits them as integers (like ArduSub).
// NOTE: currently unused — sendParam advertises REAL32 for every param (ArduPilot-
// style). Kept for potential future metadata. "SPIN_CNT" (not "SPIN") so the
// fractional MOT_SPIN_MIN/ARM aren't mis-tagged INT32 if this is ever wired.
static uint8_t classifyType(const char* n) {
    if (strstr(n, "FUNCTION") || strstr(n, "_SRC") || strstr(n, "_DIR") ||
        strstr(n, "_EN")      || strstr(n, "FRAME") || strstr(n, "MOT_PWM") ||
        strstr(n, "PIN")      || strstr(n, "ARMING")|| strstr(n, "ATUNE") ||
        strstr(n, "ESPNOW")   || strstr(n, "BRIGHT")|| strstr(n, "SPIN_CNT") ||
        strstr(n, "SERVO"))
        return PT_INT32;
    return PT_REAL32;
}

static Preferences s_prefs;

static void buildTable() {
    s_n = 0;
    for (uint16_t i = 0; i < N_SCALARS; ++i) s_table[s_n++] = SCALARS[i];
    // Seed the CAL_* shadow cache with each row's declared default, so a lock miss on the
    // very first read serves a plausible value (1.0 for a scale) rather than 0.0. See the
    // comment on s_cal_shadow. calGet() overwrites these with real values on first success.
    for (uint16_t i = 0; i < N_SCALARS; ++i) {
        uint8_t cid = SCALARS[i].cal_id;
        if (cid != CALP_NONE && cid < CALP_COUNT) s_cal_shadow[cid] = SCALARS[i].def;
    }
    // (servo rows appended below; types are classified after the table is built)

    int k = 0;   // index into s_names/s_keys
    for (int c = 0; c < N_SERVO_CH; ++c) {
        int svo = 1 + c;   // SERVO1..SERVO16 → PCA channel 0..15
        // FUNCTION — long name needs a short NVS key.
        snprintf(s_names[k], 17, "SERVO%d_FUNCTION", svo);
        snprintf(s_keys[k], 16, "svfn%d", svo);
        s_table[s_n++] = { s_names[k], s_keys[k], &g_params.servo_func[c], 0.0f }; k++;
        snprintf(s_names[k], 17, "SERVO%d_MIN", svo);  snprintf(s_keys[k], 16, "svmn%d", svo);
        s_table[s_n++] = { s_names[k], s_keys[k], &g_params.servo_min[c], 1100.0f }; k++;
        snprintf(s_names[k], 17, "SERVO%d_MAX", svo);  snprintf(s_keys[k], 16, "svmx%d", svo);
        s_table[s_n++] = { s_names[k], s_keys[k], &g_params.servo_max[c], 1900.0f }; k++;
        snprintf(s_names[k], 17, "SERVO%d_TRIM", svo); snprintf(s_keys[k], 16, "svtr%d", svo);
        s_table[s_n++] = { s_names[k], s_keys[k], &g_params.servo_trim[c], 1500.0f }; k++;
        // SERVOn_ROLE (SROT custom): 0 = off, 1 = PWM servo, 2 = MOSFET/switch.
        // Default keeps the old layout (ch1-8 servo, ch9-16 switch) but editable.
        snprintf(s_names[k], 17, "SERVO%d_ROLE", svo); snprintf(s_keys[k], 16, "svrl%d", svo);
        s_table[s_n++] = { s_names[k], s_keys[k], &g_params.servo_role[c], (c < 8) ? 1.0f : 2.0f }; k++;
    }

    for (uint16_t i = 0; i < s_n; ++i) s_type[i] = classifyType(s_table[i].name);
}

uint8_t paramType(uint16_t i) { return (i < s_n) ? s_type[i] : PT_REAL32; }

int pinOr(float value, int fallback, bool output) {
    int p = (int)value;
    if (p < 0 || p > 39) return fallback;       // out of range
    if (p >= 6 && p <= 11) return fallback;     // flash pins — never usable
    if (output && p >= 34) return fallback;     // 34-39 are INPUT-ONLY — dead as outputs
    return p;
}

// True when init() had to reformat NVS because it was unreadable — reported to the GCS.
static bool s_nvs_was_reformatted = false;

void init() {
    buildTable();

    // Make sure NVS is actually usable before opening a namespace on it.
    //
    // The NVS partition was enlarged (0x5000 -> 0xE000) because the old one physically
    // could not hold this parameter set — see partitions_hengla.csv. NVS grew INTO space
    // that previously held otadata and the start of the app, so on the first boot after
    // that change those pages contain non-blank, non-NVS data and nvs_flash_init() fails.
    // Without this, every Preferences::begin() would fail for ever and no parameter could
    // ever be saved again — the exact failure this partition change exists to fix.
    //
    // Also covers the ordinary NO_FREE_PAGES / NEW_VERSION_FOUND cases. A one-time
    // `pio run -t erase` avoids reaching this path at all; this is the safety net.
    // Erase ONLY for the two errors that mean "the contents are unusable" — not for any
    // failure. A partition-missing or out-of-memory error is not fixed by erasing, and
    // reacting to it by wiping would risk destroying a good calibration to no purpose.
    {
        esp_err_t e = nvs_flash_init();
        if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase();
            s_nvs_was_reformatted = (nvs_flash_init() == ESP_OK);
        }
    }

    s_prefs.begin(NVS_NS_PARAMS, false);

    // NVS outlives a firmware flash, and a stored value always beats its DEF_*. So a
    // build that changes a default has no effect until PARAM_DEFAULTS_VER is bumped:
    // on a mismatch we rewrite every parameter from its default and record the version.
    uint32_t stored_ver = s_prefs.getUInt(NVS_KEY_DEFAULTS_VER, 0);
    bool force_defaults = (stored_ver != (uint32_t)PARAM_DEFAULTS_VER);

    for (uint16_t i = 0; i < s_n; ++i) {
        // CAL_* rows are a view onto g_state.cal, loaded by calibration::loadFromNVS()
        // right after this. They have no NVS key here and MUST survive a defaults bump —
        // wiping a mag cal because a PID default changed would be indefensible.
        if (s_table[i].cal_id != CALP_NONE) continue;
        if (force_defaults) {
            *s_table[i].ptr = s_table[i].def;
            s_prefs.putFloat(s_table[i].nvskey, s_table[i].def);
        } else {
            *s_table[i].ptr = s_prefs.getFloat(s_table[i].nvskey, s_table[i].def);
        }
    }
    if (force_defaults) s_prefs.putUInt(NVS_KEY_DEFAULTS_VER, (uint32_t)PARAM_DEFAULTS_VER);
    s_prefs.end();

    s_defaults_were_forced = force_defaults;   // reported once the MAVLink link is up
    // ATUNE and MAG_ALIGN are momentary triggers, not persisted settings.
    g_params.atune = 0.0f;
    g_params.mag_align = 0.0f;
    // Always report THIS build's schema version, whatever a stored value or an imported
    // backup says — it describes the firmware, not the configuration.
    g_params.param_ver = (float)PARAM_DEFAULTS_VER;
}

bool defaultsWereReset() { return s_defaults_were_forced; }

bool nvsWasReformatted() { return s_nvs_was_reformatted; }

void resetAllToDefaults() {
    s_prefs.begin(NVS_NS_PARAMS, false);
    for (uint16_t i = 0; i < s_n; ++i) {
        // Calibration is deliberately NOT reset here — the caller
        // (MAV_CMD_PREFLIGHT_STORAGE param1=2) resets motor directions itself, and
        // discarding accel/mag/level cal would silently ground the vehicle.
        if (s_table[i].cal_id != CALP_NONE) continue;
        *s_table[i].ptr = s_table[i].def;
        s_prefs.putFloat(s_table[i].nvskey, s_table[i].def);
    }
    s_prefs.putUInt(NVS_KEY_DEFAULTS_VER, (uint32_t)PARAM_DEFAULTS_VER);
    s_prefs.end();
    g_params.atune = 0.0f;
}

uint16_t count() { return s_n; }

bool getByIndex(uint16_t i, char* name_out, float& value_out) {
    if (i >= s_n) return false;
    strncpy(name_out, s_table[i].name, 16);
    name_out[16] = '\0';
    value_out = (s_table[i].cal_id != CALP_NONE) ? calGet(s_table[i].cal_id)
                                                 : *s_table[i].ptr;
    return true;
}

int indexOf(const char* name) {
    for (uint16_t i = 0; i < s_n; ++i) {
        if (strncmp(name, s_table[i].name, 16) == 0) return (int)i;
    }
    return -1;
}

bool getByName(const char* name, float& value_out) {
    int i = indexOf(name);
    if (i < 0) return false;
    value_out = (s_table[i].cal_id != CALP_NONE) ? calGet(s_table[i].cal_id)
                                                 : *s_table[i].ptr;
    return true;
}

bool set(const char* name, float value, bool* persisted) {
    if (persisted) *persisted = false;
    int i = indexOf(name);
    if (i < 0) return false;
    // CAL_* rows write through to g_state.cal and are persisted to NVS_NS_CAL by the
    // comms task; they own no key in NVS_NS_PARAMS.
    if (s_table[i].cal_id != CALP_NONE) {
        bool ok = calSet(s_table[i].cal_id, value);
        if (persisted) *persisted = ok;   // cal rows persist via calibration::saveToNVS()
        return ok;
    }
    *s_table[i].ptr = value;
    s_prefs.begin(NVS_NS_PARAMS, false);
    // Already stored? Skip the write. Re-writing an identical value costs ~3 NVS entries of
    // garbage for no benefit, and a GCS that re-sends the whole table would churn the
    // partition needlessly. Persisted is still true — the value IS in flash.
    {
        float have = s_prefs.getFloat(s_table[i].nvskey, NAN);
        if (memcmp(&have, &value, sizeof(float)) == 0) {
            s_prefs.end();
            if (persisted) *persisted = true;
            return true;
        }
    }
    // Check the write, like saveAll() does. putFloat returns 0 on failure, and this NVS
    // namespace is only 0x5000 — a full or worn one would take the value in RAM, echo
    // PARAM_VALUE (so the GCS shows it accepted), then lose it on the next boot with
    // nothing said. Reported SEPARATELY from the return value: the value IS applied and
    // live for this session, so "accepted" is correct; the operator just needs to know it
    // will not survive a reboot.
    bool wrote = (s_prefs.putFloat(s_table[i].nvskey, value) != 0);
    s_prefs.end();
    if (persisted) *persisted = wrote;
    return true;
}

bool saveAll() {
    s_prefs.begin(NVS_NS_PARAMS, false);
    bool ok = true;
    for (uint16_t i = 0; i < s_n; ++i) {
        if (s_table[i].cal_id != CALP_NONE) continue;   // lives in NVS_NS_CAL
        const float want = *s_table[i].ptr;
        // Skip values that are already stored. This used to rewrite ALL ~193 parameters on
        // every save — and autotune, motor_tune and every GCS "Save to flash" all call it.
        // Each Preferences float is a BLOB costing ~3 NVS entries, so a full rewrite churned
        // ~579 entries of garbage per save and drove the partition into constant
        // garbage-collection. Writing only what changed makes a save nearly free in the
        // common case and is the difference between occasional GC and continuous thrash.
        // Bit-compare, not ==, so a NaN parameter still compares equal to itself and does
        // not get rewritten every single time.
        float have = s_prefs.getFloat(s_table[i].nvskey, NAN);
        if (memcmp(&have, &want, sizeof(float)) == 0) continue;
        // putFloat returns 0 on a failed write; without this check a full/worn NVS fails
        // silently and the user thinks their tuning was saved.
        if (s_prefs.putFloat(s_table[i].nvskey, want) == 0) ok = false;
    }
    s_prefs.end();
    return ok;
}

// Deferred save: the flight loop (autotune) requests; Core 0 services it so the
// blocking flash write never stalls the 500 Hz loop.
static volatile bool s_save_req = false;
void requestSaveAll() { s_save_req = true; }
void serviceSaveAll() { if (s_save_req) { s_save_req = false; saveAll(); } }

}  // namespace params
