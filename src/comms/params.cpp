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
#include "config.h"

Params g_params;

namespace params {

// Set by init() when PARAM_DEFAULTS_VER changed and every param was rewritten from its
// compiled default — surfaced to the GCS once the MAVLink link is up.
static bool s_defaults_were_forced = false;

struct Desc {
    const char* name;    // MAVLink param_id (≤16)
    const char* nvskey;  // NVS key (≤15)
    float*      ptr;
    float       def;
};

// Scalar params (name == nvskey; all ≤15 chars).
static const Desc SCALARS[] = {
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

void init() {
    buildTable();
    s_prefs.begin(NVS_NS_PARAMS, false);

    // NVS outlives a firmware flash, and a stored value always beats its DEF_*. So a
    // build that changes a default has no effect until PARAM_DEFAULTS_VER is bumped:
    // on a mismatch we rewrite every parameter from its default and record the version.
    uint32_t stored_ver = s_prefs.getUInt(NVS_KEY_DEFAULTS_VER, 0);
    bool force_defaults = (stored_ver != (uint32_t)PARAM_DEFAULTS_VER);

    for (uint16_t i = 0; i < s_n; ++i) {
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
    // ATUNE is a momentary trigger, not a persisted setting.
    g_params.atune = 0.0f;
}

bool defaultsWereReset() { return s_defaults_were_forced; }

void resetAllToDefaults() {
    s_prefs.begin(NVS_NS_PARAMS, false);
    for (uint16_t i = 0; i < s_n; ++i) {
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
    value_out = *s_table[i].ptr;
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
    value_out = *s_table[i].ptr;
    return true;
}

bool set(const char* name, float value) {
    int i = indexOf(name);
    if (i < 0) return false;
    *s_table[i].ptr = value;
    s_prefs.begin(NVS_NS_PARAMS, false);
    s_prefs.putFloat(s_table[i].nvskey, value);
    s_prefs.end();
    return true;
}

bool saveAll() {
    s_prefs.begin(NVS_NS_PARAMS, false);
    bool ok = true;
    for (uint16_t i = 0; i < s_n; ++i) {
        // NVS is only 0x5000 (5 pages) for ~191 entries plus the calibration namespace.
        // putFloat returns 0 on a failed write; without this check a full/worn NVS would
        // fail silently and the user would think their tuning was saved.
        if (s_prefs.putFloat(s_table[i].nvskey, *s_table[i].ptr) == 0) ok = false;
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
