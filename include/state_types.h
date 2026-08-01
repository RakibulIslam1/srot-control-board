#pragma once

// =============================================================================
//  SROT — SHARED STATE CONTRACT (state_types.h)
//
//  The dual-core tasks communicate ONLY through the global SystemState. Each of
//  the six sub-structs is guarded by its own FreeRTOS mutex so a producer on one
//  core and a consumer on the other never tear a multi-field update.
//
//  Locking rules:
//    - Take the relevant mutex, copy in/out quickly, give it back. Never block
//      on I/O or a second mutex while holding one (no nested locks → no deadlock).
//    - Real-time tasks (Core 1) must never wait indefinitely: use a short timeout
//      and skip the cycle if the lock is contended.
//  Helpers below (lockSensors()/giveSensors() ... or the scoped Lock guard)
//  wrap xSemaphoreTake/Give so call sites stay uniform.
// =============================================================================

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config.h"

// -----------------------------------------------------------------------------
// Enums
// -----------------------------------------------------------------------------

// Flight modes. Numeric values are mirrored into HEARTBEAT.custom_mode and MUST
// match ArduSub's custom_mode numbering so QGC/BlueOS's Sub plugin displays and
// commands modes correctly from its mode dropdown. The SROT-only modes (STUNT/
// PATTERN) use high values QGC ignores in its list; they are entered via the
// MAV_CMD_USER_* commands, not the dropdown.
enum class FlightMode : uint8_t {
    STABILIZE     = 0,   // ArduSub STABILIZE — cascaded angle→rate attitude hold
    ACRO          = 1,   // ArduSub ACRO — rate-only (inner loop) control
    DEPTH_HOLD    = 2,   // ArduSub ALT_HOLD — STABILIZE + barometric depth hold
    SURFACE       = 9,   // ArduSub SURFACE — ascend to surface (simple)
    MANUAL        = 19,  // ArduSub MANUAL — direct pilot passthrough
    MOTOR_DETECT  = 20,  // ArduSub MOTORDETECT — thruster direction/test
    AUTOTUNE      = 21,  // SROT — safety-monitored relay tune of all flight-loop PIDs
    MOTOR_TUNE    = 22,  // SROT — per-thruster RPM PID + idle tune (spins motors; in water)
    AUTO          = 23,  // SROT — Jetson-offloaded movement primitives (MAV_CMD_SROT_MOVE)
    STUNT         = 100, // SROT custom — N×360° spin on one axis
    PATTERN       = 101, // SROT custom — complex multi-step pattern
};

// Which axis a STUNT spins about.
enum class StuntAxis : uint8_t { NONE = 0, YAW = 1, PITCH = 2, ROLL = 3 };

// Steps of the complex PATTERN state machine.
enum class PatternStep : uint8_t {
    IDLE = 0,
    TURN_360,          // Step 1: complete a 360° turn
    HEADROOM,          // Step 2: ascend/descend to headroom clearance
    TASK,              // Step 3: perform the target maneuver
    RETURN,            // Step 4: return to previous depth + attitude
    DONE,
};

// Calibration routines (ArduSub-equivalent set).
enum class CalRoutine : uint8_t {
    NONE = 0,
    GYRO,              // zero gyro bias (hold still)
    ACCEL_6PT,         // 6-orientation accelerometer calibration
    MAG,               // compass / magnetometer (rotate)
    BARO_ZERO,         // surface depth-zero the Bar30
    ESC_RANGE,         // ESC throttle-range calibration
    MOTOR_DETECT,      // detect each thruster's spin direction
    MOTOR_TEST,        // per-motor micro-movement test
    LEVEL,             // level / AHRS trim
    JOYSTICK,          // joystick axis calibration
};

// Result of the last calibration routine (for GCS feedback).
enum class CalResult : uint8_t { NONE = 0, SUCCESS = 1, FAIL = 2 };

// PCA9685 per-channel role (mirrors PCA_* macros in config.h).
// NOTE: members are prefixed AUX_ because the ESP32 HAL #defines DISABLED/etc.
// as gpio macros, which would otherwise clobber unprefixed enumerators.
enum class PcaChannelRole : uint8_t { AUX_DISABLED = 0, AUX_SERVO = 1, AUX_SWITCH = 2 };

// -----------------------------------------------------------------------------
// A 3-axis vector helper (float)
// -----------------------------------------------------------------------------
struct Vec3f {
    float x = 0, y = 0, z = 0;
    Vec3f() = default;
    Vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

// -----------------------------------------------------------------------------
// SensorState — produced by Task_SensorRead (Core 1), consumed by control + comms
// -----------------------------------------------------------------------------
struct SensorState {
    // BNO085 fused orientation
    float    quat_w = 1, quat_x = 0, quat_y = 0, quat_z = 0;  // rotation vector
    float    roll = 0, pitch = 0, yaw = 0;                    // rad, from quaternion
    Vec3f    lin_accel;      // m/s^2, gravity removed
    Vec3f    gravity;        // m/s^2, gravity vector (for tilt/depth compensation)
    Vec3f    gyro;           // rad/s, calibrated angular velocity (inner PID loop)
    Vec3f    mag;            // uT, magnetometer (compass heading)
    uint8_t  cal_accel = 0, cal_gyro = 0, cal_mag = 0;        // BNO cal levels 0..3

    // Bar30 depth sensor
    float    depth_m = 0;        // m below surface (positive down)
    float    pressure_mbar = 0;  // absolute pressure
    float    water_temp_c = 0;

    // Analog / discrete
    float    pm1_voltage = 0;    // battery 1 (source per PM1_SRC)
    float    pm2_voltage = 0;    // battery 2 (source per PM2_SRC)
    float    curr_a = 0;         // battery current (A) from BATT_CURR ADC
    bool     pm1_present = false;// PM1 reporting (source active + data)
    bool     pm2_present = false;// PM2 reporting
    // Voltage of the THRUSTER battery, reported by the 2nd board over ESP-NOW. (The
    // local BATT_VOLT ADC is the SBC/electronics battery — a different pack.)
    float    aux_voltage = 0;
    // Freshness for the above. espnow_link returns aux_v even when the link is stale, so
    // without this a lost link would leave the last voltage in place forever — dangerous
    // for anything that scales motor output or trips a failsafe on it.
    uint32_t aux_stamp_ms = 0;   // millis() of the last FRESH ESP-NOW frame (0 = never)
    bool     aux_valid = false;  // link fresh right now
    bool     leak = false;       // leak detector
    bool     kill_switch = false;// thruster kill from 2nd board (display-only)

    // Validity / freshness
    bool     imu_valid = false;      // fresh attitude right now (false during a reset)
    bool     imu_ever_valid = false; // has EVER had a fix (false only at cold start)
    bool     baro_valid = false;
    uint32_t imu_stamp_ms = 0;
    uint32_t baro_stamp_ms = 0;
    uint32_t bno_reset_count = 0;   // BNO085 resets since boot (HW-health signal)
};

// -----------------------------------------------------------------------------
// ControlState — produced by Task_ControlLoop (Core 1), read by comms/UI
// -----------------------------------------------------------------------------
struct ControlState {
    bool        armed = false;
    FlightMode  mode = FlightMode::MANUAL;

    // Pilot setpoints (normalized -1..1, or rate/depth targets depending on mode)
    float       sp_roll = 0, sp_pitch = 0, sp_yaw = 0;
    float       sp_throttle = 0;     // vertical / heave
    float       sp_forward = 0;      // surge
    float       sp_lateral = 0;      // sway
    float       sp_depth_m = 0;      // depth target (DEPTH_HOLD)
    // Freshness for the pilot setpoints above. MANUAL_CONTROL was the ONLY external input
    // in this firmware without the value+stamp+valid triple every other one carries, and it
    // is the one where staleness is most dangerous: on a GCS-loss failsafe the last sticks
    // are stale BY DEFINITION, and nothing zeroed them — so the vehicle ascended while still
    // translating and yawing on whatever the last packet happened to say.
    // 0 = never received. millis() AT RECEIPT, taken on Core 0.
    uint32_t    sp_stamp_ms = 0;

    // PID outputs fed into the mixer (torque/force demands)
    float       out_roll = 0, out_pitch = 0, out_yaw = 0;
    float       out_throttle = 0, out_forward = 0, out_lateral = 0;

    // Stunt sub-state
    StuntAxis   stunt_axis = StuntAxis::NONE;
    float       stunt_target_deg = 0;   // total commanded rotation
    float       stunt_done_deg = 0;     // accumulated rotation
    float       stunt_progress = 0;     // 0..1 (also streamed as NAMED_VALUE_FLOAT)

    // Pattern sub-state
    PatternStep pattern_step = PatternStep::IDLE;
    float       pattern_return_depth = 0;
    float       pattern_return_yaw = 0;

    bool        autotune_active = false;
    uint32_t    loop_stamp_ms = 0;

    // AUTO-mode movement (MAV_CMD_SROT_MOVE). Command: set by Task_MAVLink (Core 0) on a new
    // SROT_MOVE; the control loop starts it when `mv_seq` changes. State: set by the control
    // loop, read by Task_MAVLink to send COMMAND_ACK progress/done + SROT_MOVE_STATE telemetry.
    uint8_t     mv_type = 0;         // movement::Type of the pending/active command
    float       mv_primary = 0;      // duration_s | degrees | depth_m | style-count
    float       mv_speed = 0;        // 0..1 (translation/arc) | yaw-rate deg/s (turn)
    float       mv_aux = 0;          // arc: signed yaw-rate; dive: descent speed
    uint8_t     mv_submode = 0;      // turn: 0 relative / 1 absolute
    float       mv_timeout = 0;
    uint32_t    mv_seq = 0;          // rising edge = new command to start
    uint8_t     mv_src_sys = 0, mv_src_comp = 0;  // for the COMMAND_ACK target
    uint8_t     mv_state = 0;        // active phase (movement state), 0 = idle
    float       mv_progress = 0;     // 0..1
    bool        mv_active = false;
    uint32_t    mv_done_seq = 0;     // set to mv_seq when that command finishes (→ ACCEPTED ACK)
};

// -----------------------------------------------------------------------------
// ThrusterState — produced by Task_ControlLoop, consumed by Task_DShot_RMT
// -----------------------------------------------------------------------------
struct ThrusterState {
    int16_t  throttle[NUM_THRUSTERS] = {0}; // DShot value: 0 or DSHOT_THROTTLE_MIN..MAX
    bool     armed = false;
    // Per-thruster RPM feedback from the Pico co-processor (THRUSTER_PICO backend);
    // 0 with the RMT backend. Signed (reverse = negative). status/fault per esc.
    int16_t  rpm[NUM_THRUSTERS] = {0};
    uint8_t  esc_status[NUM_THRUSTERS] = {0};
    uint8_t  esc_fault = 0;         // bit i = thruster i faulted (present but stalled)
    uint8_t  esc_present = 0;       // bit i = thruster i sending telemetry (connected)
    bool     link_ok = false;       // Pico link alive (fresh telemetry)
    // Normalized mixer demand (−1..1) per thruster — used to derive the signed target
    // RPM in closed-loop RPM mode (throttle[] is the open-loop DShot value).
    float    norm[NUM_THRUSTERS] = {0};
    // Motor-test / detect override: when active, the named motor is driven at
    // test_throttle (−1..1) until test_expire_ms, bypassing the mixer. Set by
    // MAV_CMD_DO_MOTOR_TEST / the calibration routines; consumed by the control
    // loop + DShot task.
    bool     test_override = false;
    int8_t   test_motor = -1;      // 0-based motor index, -1 = none
    float    test_throttle = 0;    // −1..1
    uint32_t test_expire_ms = 0;   // millis() deadline
};

// -----------------------------------------------------------------------------
// IndicatorState — consumed by Task_UI_Status (RGB, buzzer, OLED)
// -----------------------------------------------------------------------------
enum class RgbMode : uint8_t { OFF = 0, BOOT, DISARMED, ARMED, DEPTH_HOLD, STUNT, ERROR };
enum class BuzzTone : uint8_t {
    NONE = 0, ARM, DISARM, ALERT, ERROR, CALIB_STEP,
    STARTUP,    // Rick & Morty theme riff (power-up)
    PICO_LOST,  // Pico link dropped (down-sweep warble)
    PICO_OK,    // Pico link restored (up-chirp)
    GCS_LOST,   // GCS/telemetry link lost (two-tone warble)
    LEAK,       // water leak (urgent wave triple)
    CAL_START,  // calibration routine started (rising two-note)
    CAL_DONE    // calibration routine finished (little success arpeggio)
};

struct IndicatorState {
    RgbMode  rgb_mode = RgbMode::BOOT;
    uint8_t  rgb_brightness = DEF_RGB_BRIGHTNESS;
    BuzzTone buzz_request = BuzzTone::NONE;  // one-shot; UI task clears after playing
    uint8_t  oled_page = 0;
};

// -----------------------------------------------------------------------------
// AuxState — PCA9685 outputs, driven by Task_UI_Status over Bus1
// -----------------------------------------------------------------------------
struct AuxState {
    uint16_t servo_us[PCA9685_NUM_CH] = {0};   // for SERVO-role channels
    bool     switch_on[PCA9685_NUM_CH] = {false}; // for SWITCH-role channels (MOSFET/relay)
    bool     dirty = false;                    // set on change; UI task writes then clears
};

// -----------------------------------------------------------------------------
// CalState — calibration state machine, driven by Task_ControlLoop
// -----------------------------------------------------------------------------
struct CalState {
    CalRoutine routine = CalRoutine::NONE;
    uint8_t    step = 0;          // routine-specific step index
    float      progress = 0;      // 0..1
    bool       persist_pending = false; // request to write results to NVS
    CalResult  result = CalResult::NONE;      // last-completed routine result
    CalRoutine last_routine = CalRoutine::NONE; // which routine produced `result`
    uint8_t    result_seq = 0;                // bumps each completion (edge for comms)

    // Results (applied by Task_SensorRead). Scales default to 1, dirs to +1.
    Vec3f      accel_offset{0, 0, 0}, accel_scale{1, 1, 1};
    Vec3f      gyro_offset{0, 0, 0};
    Vec3f      mag_offset{0, 0, 0}, mag_scale{1, 1, 1};
    float      baro_zero_mbar = 0;
    float      level_roll_off = 0, level_pitch_off = 0; // AHRS mounting trim (rad)
    int8_t     motor_dir[NUM_THRUSTERS] = {1, 1, 1, 1, 1, 1, 1, 1}; // +1 / -1 spin dir
    bool       loaded_from_nvs = false;
};

// -----------------------------------------------------------------------------
// SystemState — the global, with one mutex per sub-struct
// -----------------------------------------------------------------------------
struct SystemState {
    SensorState    sensors;
    ControlState   control;
    ThrusterState  thrusters;
    IndicatorState indicators;
    AuxState       aux;
    CalState       cal;

    SemaphoreHandle_t mtx_sensors    = nullptr;
    SemaphoreHandle_t mtx_control    = nullptr;
    SemaphoreHandle_t mtx_thrusters  = nullptr;
    SemaphoreHandle_t mtx_indicators = nullptr;
    SemaphoreHandle_t mtx_aux        = nullptr;
    SemaphoreHandle_t mtx_cal        = nullptr;

    // Create all mutexes. Call once from setup() before starting tasks.
    void initMutexes() {
        mtx_sensors    = xSemaphoreCreateMutex();
        mtx_control    = xSemaphoreCreateMutex();
        mtx_thrusters  = xSemaphoreCreateMutex();
        mtx_indicators = xSemaphoreCreateMutex();
        mtx_aux        = xSemaphoreCreateMutex();
        mtx_cal        = xSemaphoreCreateMutex();
    }
};

// The single global instance (defined in main.cpp).
extern SystemState g_state;

// -----------------------------------------------------------------------------
// Scoped lock guard. Usage:
//     { StateLock lk(g_state.mtx_sensors); g_state.sensors.depth_m = d; }
// Real-time callers should check ok() and skip the cycle if the lock timed out.
// -----------------------------------------------------------------------------
class StateLock {
public:
    // Default timeout is BOUNDED (not portMAX_DELAY): a comms/RX handler must never
    // freeze forever waiting on a contended mutex. 20 ms is far longer than any real
    // hold (µs-scale) yet caps the pathological case. Real-time callers pass a shorter
    // explicit timeout; every caller checks ok() and skips on miss.
    explicit StateLock(SemaphoreHandle_t m, TickType_t timeout = pdMS_TO_TICKS(20))
        : _m(m) { _ok = (_m != nullptr) && (xSemaphoreTake(_m, timeout) == pdTRUE); }
    ~StateLock() { if (_ok) xSemaphoreGive(_m); }
    bool ok() const { return _ok; }
    // non-copyable
    StateLock(const StateLock&) = delete;
    StateLock& operator=(const StateLock&) = delete;
private:
    SemaphoreHandle_t _m;
    bool _ok = false;
};
