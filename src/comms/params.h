#pragma once

// =============================================================================
//  comms/params — runtime parameter store (table + NVS), MAVLink-agnostic
//
//  Holds every live-editable tuning value as a float (MAV_PARAM_TYPE_REAL32).
//  P2's MAVLink layer wraps PARAM_REQUEST_LIST/READ/SET around this table; other
//  tasks just read the fields of g_params directly.
//
//  Concurrency: values are single 32-bit floats written only by the MAVLink task
//  and read by others — aligned 32-bit access is atomic on ESP32, so reads are
//  lock-free (a reader may see an old or new value, never a torn one).
// =============================================================================

#include <Arduino.h>

// All tunable parameters. Names in the descriptor table (params.cpp) mirror the
// MAVLink param_id and the NVS key (≤15 chars for Preferences).
struct Params {
    // INFORMATIONAL ONLY. Reflects the build's PARAM_DEFAULTS_VER; forced every boot and
    // never read back by anything, so writing it has no effect. It exists so an exported
    // parameter file records which defaults schema it was taken from — a file saved before
    // a bump restores onto a post-bump board fine, but you want to KNOW that is what you
    // are doing. init() sets it after loading, so it can't be shadowed by a stored value.
    float param_ver;
    // Stunt / pattern
    float stunt_spin_cnt;
    float stunt_rate;
    float headroom_depth;
    // Indicators
    float rgb_brightness;
    // Battery source + ADC calibration
    float pm1_src;   // PM1_SRC: 0 = off, 1 = local ADC (GPIO35), 2 = ESP-NOW aux
    float pm2_src;   // PM2_SRC: 0 = off, 1 = local ADC (GPIO36), 2 = ESP-NOW aux
    float pm1_vmult;
    float pm2_vmult;
    float leak_en;
    // Attitude — outer angle-loop P
    float ang_rll_p, ang_pit_p, ang_yaw_p;
    // Attitude — inner rate-loop PID (+ integrator limit ATC_RAT_*_IMAX + feedforward
    // ATC_RAT_*_FF for crisp small-command response).
    float rat_rll_p, rat_rll_i, rat_rll_d, rat_rll_imax, rat_rll_ff;
    float rat_pit_p, rat_pit_i, rat_pit_d, rat_pit_imax, rat_pit_ff;
    float rat_yaw_p, rat_yaw_i, rat_yaw_d, rat_yaw_imax, rat_yaw_ff;
    // Depth-hold PID
    float depth_p, depth_i, depth_d;
    // Model-based control (Phase 2a): angular drag feedforward, cross-coupling, CoB auto-trim.
    float atc_drag_rll, atc_drag_pit, atc_drag_yaw;
    float xc_yaw2rll, xc_yaw2pit;
    float trim_en, trim_leak, trim_max;
    float buzz_mask, thr_beep_en;   // indicator toggles
    float buzz_duty;                // BUZZ_DUTY (% drive duty; lower = less current/quieter)
    // Per-thruster direction (+1 normal / −1 reversed), ArduSub MOT_n_DIRECTION style.
    float mot_dir[8];
    // ArduSub-style joystick button function map (BTN0..15_FUNCTION).
    float btn_func[16];
    // Focused ArduSub-compatible params.
    float frame_config;     // FRAME_CONFIG (2 = vectored, informational)
    float js_gain_default;  // JS_GAIN_DEFAULT (0..1 pilot gain)
    float lights_step;      // LIGHTS_STEP (fraction per brighter/dimmer press)
    float fs_gcs_enable;    // FS_GCS_ENABLE (GCS-loss failsafe on/off)
    float fs_bat_enable;    // FS_BAT_ENABLE
    float fs_bat_voltage;   // FS_BAT_VOLTAGE (V, PM1 low-battery threshold)
    float pilot_speed;      // PILOT_SPEED (0..1 forward/lateral scale for fine moves)
    float pilot_yaw_rate;   // PILOT_YAW_RATE (deg/s yaw at full stick — low = precise)
    float pilot_expo;       // PILOT_EXPO (0..1 stick expo for fine centre resolution)
    float arming_check;     // ARMING_CHECK (0 = skip pre-arm checks, 1 = enforce)
    float atune;            // ATUNE (set 1 to start relay auto-tune; auto-resets)
    float espnow_en;        // ESPNOW_EN (0/1) — start the WiFi/ESP-NOW 2nd-board link
    // Autotune safety monitor — a tune that violates any of these auto-disarms.
    float st_angle_max;     // ST_ANGLE_MAX (deg)  tumbling guard
    float st_rate_max;      // ST_RATE_MAX (deg/s) spin-out guard
    float st_depth_delta;   // ST_DEPTH_DELTA (m)  depth-runaway guard
    float st_rpm_max;       // ST_RPM_MAX (rpm)    over-speed guard
    // Motor closed-loop RPM control (pushed to the Pico via the cfg frame).
    float rpm_kp, rpm_ki;   // RPM_KP / RPM_KI  PI gains
    float rpm_ff_a;         // RPM_FF_A         feedforward slope (norm-throttle per rpm)
    float rpm_idle;         // RPM_IDLE         dynamic-idle target rpm (0 = stop)
    float rpm_max;          // RPM_MAX          |rpm| mapped to full throttle
    float rpm_filt;         // RPM_FILT         measured-rpm IIR alpha (0..1)
    float rpm_slew;         // RPM_SLEW         max output-level change per cycle (0 = off)
    float rpm_min_tgt;      // RPM_MIN_TGT      |target| below this = stop at neutral (centre deadband)
    float rpm_loop;         // RPM_LOOP         0 = open-loop (feedforward) · 1 = closed-loop RPM (PI)
    float dshot_bidir;      // DSHOT_BIDIR      1 = bidirectional DShot (RPM telem) · 0 = normal DShot
    float mtune_en;         // MTUNE_EN         allow MOTOR_TUNE mode (spins motors in water)
    // AUTO-mode movement (Jetson-offloaded primitives; braking on the ESP32).
    float move_cruise_max;  // MOVE_CRUISE_MAX  cap on commanded translation speed (0..1)
    float move_accel;       // MOVE_ACCEL       speed ramp per second (smooth start/stop)
    float move_brake_gain;  // MOVE_BRAKE_GAIN  reverse-thrust fraction during braking
    float move_brake_k;     // MOVE_BRAKE_K     brake time (s) per unit cruise speed
    float move_depth_rate;  // MOVE_DEPTH_RATE  smooth dive rate (m/s target ramp — no splash)
    float move_yaw_rate;    // MOVE_YAW_RATE    default turn rate (deg/s)
    // ArduSub motor-config params QGC's Motors page reads (informational — our
    // output is DShot150 on dedicated RMT pins).
    float mot_pwm_type;     // MOT_PWM_TYPE (4 = DShot150)
    float mot_pwm_min;      // MOT_PWM_MIN (µs)
    float mot_pwm_max;      // MOT_PWM_MAX (µs)
    // Thrust shaping (ArduSub-standard) — the real micro-movement enablers.
    float mot_thst_expo;    // MOT_THST_EXPO (0=linear..1=quadratic thrust curve)
    float mot_spin_min;     // MOT_SPIN_MIN (0..1 min active output — crosses deadband)
    float mot_spin_arm;     // MOT_SPIN_ARM (0..1 armed idle output; 0 = off)
    // Slow RPM-based thrust normalisation (control/thrust_trim) — makes a timed command
    // travel the same distance regardless of battery state. NOT in the attitude path.
    float thr_trim_en;      // THR_TRIM_EN  0 = off (default; must be off with props in air)
    float thr_trim_tau;     // THR_TRIM_TAU seconds — learning time constant (slow on purpose)
    float thr_trim_max;     // THR_TRIM_MAX max +/- gain deviation (0.25 = +/-25%)
    // Thruster-battery voltage feedforward (open-loop thrust linearisation).
    float mot_bat_v_min;    // MOT_BAT_V_MIN clamp floor (V)
    float mot_bat_v_max;    // MOT_BAT_V_MAX pack full-charge voltage (V); 0 = disabled

    // One-shot magnetic yaw reference (control/yaw_ref). The mag is read ONCE at boot to
    // zero the yaw offset, then never again — attitude stays on the mag-free 6-axis fusion
    // so thruster currents cannot move it. Fixes the reference, not the drift.
    float mag_yaw_ref;      // MAG_YAW_REF  0 = off (yaw relative) · 1 = align once at boot
    float mag_decl;         // MAG_DECL     local magnetic declination (deg, E positive)
    float mag_align;        // MAG_ALIGN    momentary: set 1 to re-align (auto-clears)
    // Runtime-configurable GPIO assignments (peripheral pins). Changing any of
    // these needs a reboot to take effect (bus pins stay compile-time for safety).
    float pin_buzzer, pin_rgb, pin_leak, pin_battvolt, pin_battcurr;
    float pin_sdcs, pin_loracs, pin_loradio;
    // Servo-output config for all 16 PCA9685 channels (SERVO1..16 = PCA ch 0..15;
    // motors are on separate DShot pins, so every PCA channel is a free output).
    float servo_func[16];   // SERVOn_FUNCTION (ArduSub-facing; 0 = disabled)
    float servo_min[16];    // µs
    float servo_max[16];    // µs
    float servo_trim[16];   // µs (output when uncommanded)
    float servo_role[16];   // SERVOn_ROLE: 0 = off, 1 = PWM servo, 2 = MOSFET/switch
};

extern Params g_params;

namespace params {

// Load from NVS (ns "srot_prm"); any missing key falls back to its config.h
// default. Call once in setup() before tasks start.
//
// NVS survives a firmware flash and a stored value beats its DEF_*, so changing a
// default in config.h does nothing on a board that already saved that key. To make a
// build's defaults authoritative, bump PARAM_DEFAULTS_VER: init() then rewrites every
// parameter from its default (discarding user tuning; calibration is untouched).
void init();

// True if this boot's init() found a PARAM_DEFAULTS_VER mismatch and reset everything
// to the compiled defaults. Reported to the GCS once, so the reset is never silent.
bool defaultsWereReset();

// Restore every parameter to its config.h default and persist. Exposed via
// MAV_CMD_PREFLIGHT_STORAGE param1 = 2 (ArduPilot's "reset to defaults" convention).
void resetAllToDefaults();

// Table introspection for the MAVLink PARAM protocol.
uint16_t count();
// Fill name (17 bytes: 16 + NUL) and value for table index i. False if OOR.
bool getByIndex(uint16_t i, char* name_out, float& value_out);

// MAV_PARAM_TYPE for index i (INT32 for integer-like params, else REAL32) so QGC
// shows/edits them correctly like ArduSub. Returns REAL32 if OOR.
uint8_t paramType(uint16_t i);
// Look up by (≤16-char) name. Returns index or -1.
int  indexOf(const char* name);
bool getByName(const char* name, float& value_out);

// Set by name; updates the live field AND persists to NVS. False if unknown.
// Apply a value by name. Returns true if the parameter exists and was APPLIED (it is live
// immediately). `persisted`, if given, reports separately whether the NVS write succeeded —
// a full or worn namespace can accept the value in RAM and silently lose it on reboot, and
// the operator needs to be told that rather than discovering it after a power cycle.
bool set(const char* name, float value, bool* persisted = nullptr);

// Persist every parameter to NVS (used by PREFLIGHT_STORAGE). Blocking flash I/O —
// call only from a comms/Core-0 context, never the flight loop.
// Returns false if any write failed (NVS full/worn) so the caller can say so.
bool saveAll();

// Deferred save: requestSaveAll() sets a flag (safe from the flight loop);
// serviceSaveAll() performs the actual saveAll() if pending (call from Core 0).
void requestSaveAll();
void serviceSaveAll();

// Validate a GPIO param value: returns it if a usable ESP32 GPIO (0..39, not the
// flash pins 6..11), else the given fallback. Pass output=true for output-role pins
// to also reject the input-only pins 34..39. Use for runtime pin assignments.
int pinOr(float value, int fallback, bool output = false);

}  // namespace params
