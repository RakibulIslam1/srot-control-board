// =============================================================================
//  Task_ControlLoop (Core 1) — flight controller core @ CONTROL_LOOP_HZ (500 Hz)
//  Arming enforcement, motor-test override, mode→demands cascade, Vectored-8 mixing
//  + thrust shaping. Snapshots shared state under short locks, computes on locals.
// =============================================================================

#include "tasks.h"
#include "config.h"
#include "state_types.h"
#include "control/mixer.h"
#include "control/arming.h"
#include "control/attitude_control.h"
#include "control/depth_control.h"
#include "control/feedforward.h"
#include "control/calibration.h"
#include "control/stunt.h"
#include "control/pattern.h"
#include "control/autotune.h"
#include "control/motor_tune.h"
#include "control/movement.h"
#include "control/safety_monitor.h"
#include "comms/ui_log.h"
#include "comms/params.h"
#include "comms/mav_commands.h"
#include "comms/mav_stream.h"   // queueStatusText: Core-1-safe disarm/failsafe notices

// Local snapshot of everything the loop needs from shared state.
struct LoopIn {
    bool       armed;
    FlightMode mode;
    float      sp_roll, sp_pitch, sp_yaw, sp_throttle, sp_forward, sp_lateral;
    // sensors
    float      roll, pitch, yaw, gx, gy, gz, depth;
    bool       depth_ok;     // Bar30 present AND reporting recently — see readInputs()
    bool       imu_ok;       // fused attitude fresh (drives the in-flight health warning)
    float      grav_x, grav_y, grav_z, mx, my, mz, pressure;
    bool       kill;
    bool       leak;
    float      pm1;          // SBC / electronics battery (local ADC)
    float      thr_volts;    // THRUSTER battery (2nd board via ESP-NOW); 0 = none/stale
    // stunt / pattern / autotune
    StuntAxis  stunt_axis;
    float      stunt_target_deg;
    float      pattern_headroom;
    bool       autotune;
    // motor test
    bool       test_override;
    int8_t     test_motor;
    float      test_throttle;
    uint32_t   test_expire;
    // motor directions
    int8_t     dir[NUM_THRUSTERS];
    // per-thruster RPM (Pico telemetry) — used by MOTOR_TUNE + the safety monitor
    int16_t    rpm[NUM_THRUSTERS];
    // AUTO-mode move command (from Task_MAVLink)
    uint8_t    mv_type; float mv_primary, mv_speed, mv_aux; uint8_t mv_submode; float mv_timeout;
    uint32_t   mv_seq;
};

static void readInputs(LoopIn& in) {
    {
        StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
        if (lk.ok()) {
            const ControlState& c = g_state.control;
            in.armed = c.armed;  in.mode = c.mode;
            in.sp_roll = c.sp_roll; in.sp_pitch = c.sp_pitch; in.sp_yaw = c.sp_yaw;
            in.sp_throttle = c.sp_throttle; in.sp_forward = c.sp_forward; in.sp_lateral = c.sp_lateral;
            in.stunt_axis = c.stunt_axis;
            in.stunt_target_deg = c.stunt_target_deg;
            in.pattern_headroom = c.pattern_return_depth;
            in.autotune = c.autotune_active;
            in.mv_type = c.mv_type; in.mv_primary = c.mv_primary; in.mv_speed = c.mv_speed;
            in.mv_aux = c.mv_aux; in.mv_submode = c.mv_submode; in.mv_timeout = c.mv_timeout;
            in.mv_seq = c.mv_seq;

            // Age out the pilot sticks. Applied HERE, once, so every mode downstream sees
            // decayed intent without needing to know about it — rather than adding a
            // freshness test to each of six mode branches and forgetting one.
            //
            // A stale stick is not a held stick: it means nobody is driving. Keeping the last
            // value is how a GCS-loss failsafe ended up ascending while still translating on
            // the final packet. Ramp rather than cliff — see MANUAL_FRESH_MS.
            const uint32_t sp_age = (c.sp_stamp_ms != 0) ? (millis() - c.sp_stamp_ms) : UINT32_MAX;
            float sp_auth = 1.0f;
            if (c.sp_stamp_ms == 0 || sp_age >= MANUAL_DECAY_MS)      sp_auth = 0.0f;
            else if (sp_age > MANUAL_FRESH_MS)
                sp_auth = 1.0f - (float)(sp_age - MANUAL_FRESH_MS) /
                                 (float)(MANUAL_DECAY_MS - MANUAL_FRESH_MS);
            if (sp_auth < 1.0f) {
                in.sp_roll *= sp_auth; in.sp_pitch *= sp_auth; in.sp_yaw *= sp_auth;
                in.sp_throttle *= sp_auth; in.sp_forward *= sp_auth; in.sp_lateral *= sp_auth;
            }
        }
    }
    {
        StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(2));
        if (lk.ok()) {
            const SensorState& s = g_state.sensors;
            in.roll = s.roll; in.pitch = s.pitch; in.yaw = s.yaw;
            in.gx = s.gyro.x; in.gy = s.gyro.y; in.gz = s.gyro.z;
            in.depth = s.depth_m; in.kill = s.kill_switch;
            // With no Bar30 fitted, depth_m is never written and sits at 0.0 forever —
            // which silently made the SURFACE failsafe a no-op (target 0 - measured 0 = no
            // thrust) and made the depth-runaway guard untrippable. Require both the
            // driver's health flag AND a recent sample, so a sensor that dies mid-dive is
            // caught too.
            in.depth_ok = s.baro_valid && (s.baro_stamp_ms != 0) &&
                          (millis() - s.baro_stamp_ms < DEPTH_STALE_MS);
            in.imu_ok   = s.imu_valid;
            in.grav_x = s.gravity.x; in.grav_y = s.gravity.y; in.grav_z = s.gravity.z;
            in.mx = s.mag.x; in.my = s.mag.y; in.mz = s.mag.z;
            in.pressure = s.pressure_mbar;
            in.leak = s.leak; in.pm1 = s.pm1_voltage;
            // Thruster pack: only trust it while the ESP-NOW link is genuinely fresh —
            // aux_voltage holds its last value on link loss.
            in.thr_volts = (s.aux_valid && s.aux_stamp_ms != 0 &&
                            (millis() - s.aux_stamp_ms < ESPNOW_STALE_MS))
                               ? s.aux_voltage : 0.0f;
        }
    }
    {
        StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
        if (lk.ok()) {
            in.test_override = g_state.thrusters.test_override;
            in.test_motor    = g_state.thrusters.test_motor;
            in.test_throttle = g_state.thrusters.test_throttle;
            in.test_expire   = g_state.thrusters.test_expire_ms;
            for (int i = 0; i < NUM_THRUSTERS; ++i) in.rpm[i] = g_state.thrusters.rpm[i];
        }
    }
    {
        // Effective direction = QGC-editable MOTn_DIR reverse toggle × any
        // direction detected by MOTOR_DETECT calibration.
        StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
        bool have_cal = lk.ok();
        for (int i = 0; i < NUM_THRUSTERS; ++i) {
            int8_t detected = have_cal ? g_state.cal.motor_dir[i] : 1;
            if (detected == 0) detected = 1;
            int8_t param_dir = (g_params.mot_dir[i] < 0) ? -1 : 1;
            in.dir[i] = detected * param_dir;
        }
    }
}

// Mode → body-axis demands (−1..1). Surge/sway (fwd/lat) are always pilot
// passthrough; roll/pitch/yaw/heave depend on the mode's controller.
// STUNT/PATTERN are handled by their own state machines in P6; until then they
// fall through to STABILIZE behaviour.
static void computeDemands(const LoopIn& in, float dt,
                           float& roll, float& pitch, float& yaw,
                           float& thr, float& fwd, float& lat) {
    roll = pitch = yaw = thr = fwd = lat = 0;
    // Forward/lateral: scale by PILOT_SPEED and shape with PILOT_EXPO so small stick
    // = small, precise translation (thrust itself is linearized in the mixer).
    {
        float pe = constrain(g_params.pilot_expo, 0.0f, 1.0f);
        float ps = (g_params.pilot_speed > 0.0f) ? g_params.pilot_speed : 1.0f;
        float f = in.sp_forward, l = in.sp_lateral;
        fwd = ((1.0f - pe) * f + pe * f * f * f) * ps;
        lat = ((1.0f - pe) * l + pe * l * l * l) * ps;
    }

    switch (in.mode) {
        case FlightMode::MANUAL:
            // No stabilization — direct passthrough of all pilot axes.
            roll  = in.sp_roll;
            pitch = in.sp_pitch;
            yaw   = in.sp_yaw;
            thr   = in.sp_throttle;
            break;

        case FlightMode::ACRO:
            attitude::acro(in.sp_roll, in.sp_pitch, in.sp_yaw,
                           in.gx, in.gy, in.gz, dt, roll, pitch, yaw);
            thr = in.sp_throttle;
            break;

        case FlightMode::DEPTH_HOLD: {
            attitude::stabilize(in.sp_roll, in.sp_pitch, in.sp_yaw,
                                in.roll, in.pitch, in.yaw, in.gx, in.gy, in.gz, dt,
                                roll, pitch, yaw);
            float tgt;
            thr = depth::update(in.sp_throttle, in.depth, dt, tgt);
            break;
        }

        case FlightMode::SURFACE: {
            // Hold attitude and drive depth target to the surface (0 m).
            //
            // TRANSLATION AND YAW ARE ZEROED. SURFACE is a failsafe destination — reached on
            // leak, low thruster battery, or GCS loss — and on the GCS-loss case the last
            // sticks are stale by definition. This used to inherit fwd/lat from the
            // unconditional block above and pass in.sp_yaw straight through, so the vehicle
            // ascended while continuing to translate and yaw on whatever the last packet
            // said, indefinitely. A vehicle that has lost its operator should not still be
            // driving somewhere. (JETSON_FEEDBACK.md §3.)
            //
            // Belt and braces with the staleness ramp in readInputs(): that handles a sender
            // that went quiet, this handles a sender that is still talking while the vehicle
            // is surfacing for an unrelated reason (a leak with the pilot still on the sticks).
            fwd = 0; lat = 0;
            attitude::stabilize(0, 0, 0,
                                in.roll, in.pitch, in.yaw, in.gx, in.gy, in.gz, dt,
                                roll, pitch, yaw);
            if (in.depth_ok) {
                depth::setTarget(0.0f);
                float tgt;
                thr = depth::update(0, in.depth, dt, tgt);
            } else {
                // No depth sensor: the PID would see target 0 - measured 0 = no error and
                // command NO vertical thrust, i.e. a leak/battery/GCS failsafe would leave
                // the sub sitting where it is. Ascend open-loop instead — surfacing blind
                // beats not surfacing. (+throttle = ascend.)
                thr = DEPTH_LOST_ASCENT;
            }
            break;
        }

        case FlightMode::AUTO: {
            // Jetson-offloaded movement: heading + smoothed depth hold, translation/turn from
            // the movement primitive (which brakes on-board). STYLE spins via the demand directly.
            bool running = false;
            movement::Demand md = movement::update(in.roll, in.pitch, in.yaw,
                                                   in.gx, in.gy, in.gz, in.depth, in.rpm, dt, running);
            if (md.spin) {
                roll = md.roll; pitch = md.pitch; yaw = md.yaw; thr = in.sp_throttle;
                fwd = 0; lat = 0;
            } else {
                // Explicit heading lock: a translate/hold leg locks the heading it started
                // with, and a finished TURN locks the heading it was commanded to reach. A
                // centred md.yaw then holds exactly that, so "forward 3 s" tracks straight
                // instead of relying on whatever the attitude loop last latched.
                if (md.yaw_lock_valid) attitude::holdYaw(md.yaw_lock);
                attitude::stabilize(0, 0, md.yaw, in.roll, in.pitch, in.yaw, in.gx, in.gy, in.gz, dt,
                                    roll, pitch, yaw);
                depth::setTarget(md.depth_target);
                float tgt; thr = depth::update(0, in.depth, dt, tgt);
                fwd = md.fwd; lat = md.lat;
            }
            break;
        }

        case FlightMode::STABILIZE:
        case FlightMode::STUNT:
        case FlightMode::PATTERN:
        case FlightMode::MOTOR_DETECT:
        default:
            attitude::stabilize(in.sp_roll, in.sp_pitch, in.sp_yaw,
                                in.roll, in.pitch, in.yaw, in.gx, in.gy, in.gz, dt,
                                roll, pitch, yaw);
            thr = in.sp_throttle;
            break;
    }
}

void Task_ControlLoop(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ);
    TickType_t last = xTaskGetTickCount();

    LoopIn in{};   // zero-init: a contended first-cycle lock must not feed garbage
    float norm[NUM_THRUSTERS] = {0};
    int16_t dshot[NUM_THRUSTERS] = {0};
    FlightMode prev_mode = FlightMode::MANUAL;
    bool prev_armed = false;         // disarmed->armed edge: reset controllers
    bool prev_autotune = false;
    bool prev_mtune = false;
    uint32_t prev_mv_seq = 0;        // AUTO move-command edge
    bool mv_was_active = false;
    float at_depth0 = 0;             // depth captured at autotune start (runaway guard)
    const float dt = CONTROL_LOOP_DT;
    uint32_t stk_cnt = 0;
    extern volatile uint32_t g_stack_hw[6];
    extern volatile bool g_mtune_active;   // tells the DShot task to send RAW during MOTOR_TUNE

    for (;;) {
        readInputs(in);
        if (++stk_cnt >= CONTROL_LOOP_HZ) { stk_cnt = 0; g_stack_hw[2] = uxTaskGetStackHighWaterMark(nullptr); }

        // Advance any active calibration routine (it may drive motors via the
        // test-override hook, honoured below).
        calibration::update(in.roll, in.pitch, in.yaw, in.gx, in.gy, in.gz,
                            in.grav_x, in.grav_y, in.grav_z, in.mx, in.my, in.mz,
                            in.pressure, millis());

        // On the DISARMED -> ARMED edge, reset the controllers. While disarmed the PID
        // integrators, the feedforward trim (s_trim_*) and the yaw-hold latch keep
        // accumulating against an attitude error nobody is correcting, so by the time you
        // arm they can hold a large demand — the sub would lurch on arm with the sticks
        // centred. Same canonical trio used on mode entry below.
        if (in.armed && !prev_armed) {
            attitude::reset();
            depth::reset(in.depth);
            feedforward::reset();
            // Also drop any stale motor-test override left over from a previous session,
            // so re-arming can never resume driving the last-tested motor.
            StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
            if (lk.ok()) {
                g_state.thrusters.test_override = false;
                g_state.thrusters.test_motor    = -1;
                g_state.thrusters.test_expire_ms = 0;
            }
            in.test_override = false;
        }
        // ARMED -> DISARMED: stop everything that would otherwise keep advancing while
        // disarmed and resume the moment you re-arm. None of these state machines had an
        // armed gate, so a tune/move interrupted by a disarm silently continued and then
        // drove the motors again on the next arm.
        if (!in.armed && prev_armed) {
            motor_tune::abort();
            autotune::abort();
            movement::abort();
            // STUNT and PATTERN had no abort(), so they kept their phase across a disarm
            // and resumed driving thrusters on the next arm with no new command — e.g. a
            // panic-disarm 200 deg into a 360 deg spin would finish the remaining 160 deg
            // the instant you re-armed. Disarming does not change the GCS mode selector,
            // so the mode-entry reset never re-fired to restart them cleanly.
            stunt::abort();
            pattern::abort();
            {   // drop any calibration routine (MOTOR_DETECT/MOTOR_TEST re-assert
                // test_override every cycle, which would defeat the arm-edge clear above)
                StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
                if (lk.ok()) g_state.cal.routine = CalRoutine::NONE;
            }
            {   // A stale stick from before the disarm must not become thrust on re-arm.
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    g_state.control.sp_forward = 0; g_state.control.sp_lateral  = 0;
                    g_state.control.sp_throttle = 0; g_state.control.sp_yaw     = 0;
                    g_state.control.sp_roll = 0;     g_state.control.sp_pitch   = 0;
                    g_state.control.autotune_active = false;
                }
            }
            in.sp_forward = 0; in.sp_lateral = 0; in.sp_throttle = 0; in.sp_yaw = 0;
        }
        prev_armed = in.armed;

        // Depth-dependent modes need a working Bar30. Without one, depth is a constant 0
        // and these modes don't just degrade — DEPTH_HOLD's integrator winds up against a
        // fixed 0 and AUTO's dive legs can never complete. Refuse them (with a reason) and
        // fall back to STABILIZE, which flies fine without depth. SURFACE is NOT blocked:
        // it has an open-loop ascent fallback above and is a failsafe destination.
        //
        // PATTERN is in this list. It was missing, and that only became harmful once PATTERN
        // came under the safety monitor: its HEADROOM step commands entry_depth + headroom
        // while depth reads a constant 0, so any headroom beyond ST_DEPTH_DELTA now reads as
        // a depth runaway and DISARMS. Refusing the mode says the same thing in the right
        // way — a message the pilot can act on instead of a disarm they have to diagnose.
        if (!in.depth_ok &&
            (in.mode == FlightMode::DEPTH_HOLD || in.mode == FlightMode::AUTO ||
             in.mode == FlightMode::PATTERN)) {
            static uint32_t s_last_warn = 0;
            if (millis() - s_last_warn > 3000) {
                s_last_warn = millis();
                mav_stream::queueStatusText(MAV_SEVERITY_ERROR,
                                            "No depth sensor - mode refused, using STABILIZE");
            }
            in.mode = FlightMode::STABILIZE;
            StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
            if (lk.ok()) g_state.control.mode = FlightMode::STABILIZE;
        }

        // MOTOR_DETECT finished (the routine cleared itself) -> return to STABILIZE so the
        // vehicle doesn't sit in a mode whose job is done. prev_mode is still MOTOR_DETECT
        // here, so the leave-mode cleanup below is a no-op and this is the only transition.
        if (in.mode == FlightMode::MOTOR_DETECT && prev_mode == FlightMode::MOTOR_DETECT) {
            bool detect_running = false;
            { StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
              if (lk.ok()) detect_running = (g_state.cal.routine == CalRoutine::MOTOR_DETECT); }
            if (!detect_running) {
                in.mode = FlightMode::STABILIZE;
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) g_state.control.mode = FlightMode::STABILIZE;
            }
        }

        // MOTOR_DETECT drives the thrusters, so it needs to be armed. Refuse otherwise
        // rather than sitting in a mode that silently does nothing.
        if (in.mode == FlightMode::MOTOR_DETECT && !in.armed) {
            static uint32_t s_last_md_warn = 0;
            if (millis() - s_last_md_warn > 3000) {
                s_last_md_warn = millis();
                mav_stream::queueStatusText(MAV_SEVERITY_WARNING,
                                            "Motor detect: arm first (needs water, free to rotate)");
            }
            in.mode = FlightMode::STABILIZE;
            StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
            if (lk.ok()) g_state.control.mode = FlightMode::STABILIZE;
        }

        // On mode entry, reset controllers and arm the stunt/pattern SMs.
        if (in.mode != prev_mode) {
            attitude::reset();
            depth::reset(in.depth);
            feedforward::reset();   // don't carry a learned trim across modes
            if (in.mode == FlightMode::STUNT) {
                stunt::start(in.stunt_axis, in.stunt_target_deg, g_params.stunt_rate);
            } else if (in.mode == FlightMode::PATTERN) {
                float hr = (in.pattern_headroom > 0) ? in.pattern_headroom : g_params.headroom_depth;
                pattern::start(hr, in.depth, in.yaw);
            } else if (in.mode == FlightMode::AUTO) {
                movement::enter(in.depth);   // latch depth setpoint, idle station-keep
            } else if (in.mode == FlightMode::MOTOR_DETECT) {
                // Actually START the detect routine. Selecting the mode used to do nothing
                // at all — FlightMode::MOTOR_DETECT falls through to the STABILIZE branch
                // and CalRoutine::MOTOR_DETECT was never assigned anywhere, so the button
                // was dead. calibration::update() calls beginRoutine() on the change, which
                // resets the routine's own per-motor state.
                StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    g_state.cal.routine  = CalRoutine::MOTOR_DETECT;
                    g_state.cal.step     = 0;
                    g_state.cal.progress = 0;
                    g_state.cal.result   = CalResult::NONE;
                }
                mav_stream::queueStatusText(MAV_SEVERITY_WARNING,
                    "Motor detect: thrusters will pulse - needs water, free to rotate");
            }
            // Leaving MOTOR_DETECT must stop it driving motors.
            if (prev_mode == FlightMode::MOTOR_DETECT && in.mode != FlightMode::MOTOR_DETECT) {
                StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
                if (lk.ok() && g_state.cal.routine == CalRoutine::MOTOR_DETECT)
                    g_state.cal.routine = CalRoutine::NONE;
            }
            // Leaving AUTO must END the running move, or it is stranded on IN_PROGRESS for
            // ever and the companion's action hangs. (JETSON_FEEDBACK.md §1 — their highest
            // severity item, and it fires exactly when the vehicle is already in trouble.)
            //
            // Displace AUTO mid-move and the mv_* publish at the bottom of this loop stops
            // running, so mv_active freezes true and mv_done_seq never advances — and
            // mav_stream's done-condition `(mv_done_seq == s_seq) || (s_seen_active &&
            // !mv_active)` then has BOTH terms permanently false. The un-started escape hatch
            // there needs !s_seen_active, so it cannot rescue a move that had already begun.
            // Nothing else writes those fields: COMMAND_ACK IN_PROGRESS at 3 Hz, for ever.
            //
            // cancel(), NOT abort(): abort() parks the command in PH_BRAKE, which only
            // advances inside movement::update() — and update() is only reached from the AUTO
            // branch we are leaving. It would swap one permanent non-terminal state for
            // another. See movement.h.
            //
            // This one hook covers BOTH ways AUTO is displaced. An operator/GCS mode change
            // arrives through readInputs(). A failsafe forces the mode further down this same
            // loop, after this block has run — so it is caught on the next cycle (2 ms later),
            // by which point computeDemands() has already stopped calling movement::update().
            if (prev_mode == FlightMode::AUTO && in.mode != FlightMode::AUTO) {
                movement::cancel();
            }
            prev_mode = in.mode;
        }

        // AUTO: start a new movement command on a rising mv_seq (from Task_MAVLink). A new
        // command preempts the current one.
        //
        // start() now carries the outgoing leg's travel axis and speed forward (see the
        // capture in movement::start), so a preempting STOP brakes the momentum it inherited
        // instead of coasting. For every other verb the acceleration ramp smooths the
        // transition as before.
        //
        // JETSON_COMMS.md claimed preemption did "a quick brake first". It never did — there
        // is no null-momentum phase between the two commands, just a direct hand-over. Rather
        // than insert a brake (which would add an unbounded stall to every mission sequence,
        // and the companion sequences legs back-to-back deliberately), the doc is corrected to
        // describe what actually happens. See Stage C3 of the L0 plan.
        if (in.mode == FlightMode::AUTO && in.mv_seq != prev_mv_seq) {
            movement::start((movement::Type)in.mv_type, in.mv_primary, in.mv_speed,
                            in.mv_submode, in.mv_aux, in.mv_timeout, in.yaw, in.depth);
            prev_mv_seq = in.mv_seq;
        }

        // Tune modes. AUTOTUNE = flight-loop relay tune (ATUNE param also triggers it).
        // MOTOR_TUNE = per-thruster RPM tune (gated by MTUNE_EN; spins motors in water).
        // Both require ARMED: previously the state machines advanced while disarmed, so a
        // tune "ran" against dead motors, consumed its phases, and then drove the motors on
        // the next arm. Requiring armed also makes DISARM the universal stop button.
        bool at_active = (in.autotune || in.mode == FlightMode::AUTOTUNE) && in.armed;
        bool mt_active = (in.mode == FlightMode::MOTOR_TUNE) && (g_params.mtune_en > 0.5f) && in.armed;
        g_mtune_active = mt_active;
        if (at_active && !prev_autotune) {
            autotune::start(in.depth_ok); at_depth0 = in.depth;
            // Put the vehicle IN AutoTune mode, whichever way the tune was triggered.
            // The ATUNE param (and MAV_CMD_USER_5) set autotune_active with no reference to
            // the flight mode, and the gate below is `in.autotune || mode == AUTOTUNE` — so
            // setting ATUNE while disarmed latched, and the next arm started a full-authority
            // relay tune in whatever mode the pilot happened to be in, with the GCS and OLED
            // still showing MANUAL (or STABILIZE, or...). Reflecting it in the mode makes
            // what the vehicle is actually doing visible everywhere the mode is displayed.
            if (in.mode != FlightMode::AUTOTUNE) {
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) g_state.control.mode = FlightMode::AUTOTUNE;
                in.mode = FlightMode::AUTOTUNE;
            }
            // Kept under 50 chars — MAVLink's STATUSTEXT.text field is char[50] and
            // anything past it is silently dropped, tail first.
            mav_stream::queueStatusText(MAV_SEVERITY_WARNING,
                                        "AUTOTUNE started - thrusters WILL drive");
        }
        // Leaving AUTOTUNE mode / clearing ATUNE must actually STOP the tuner. Previously
        // the loop just stopped calling update(), freezing the state machine mid-phase so
        // it resumed where it left off next time.
        if (!at_active && prev_autotune) {
            autotune::abort();
            mav_stream::queueStatusText(MAV_SEVERITY_INFO, "Autotune stopped");
        }
        if (mt_active && !prev_mtune) { motor_tune::start(); at_depth0 = in.depth;
                                        ui_log::set("MTune: motors will spin");
                                        mav_stream::queueStatusText(MAV_SEVERITY_WARNING,
                                            "Motor tune started - thrusters will spin"); }
        if (!mt_active && prev_mtune) {   // same freeze problem as autotune — really stop it
            motor_tune::abort();
            mav_stream::queueStatusText(MAV_SEVERITY_INFO, "Motor tune stopped");
        }
        // Explain the two ways these silently do nothing, instead of leaving the pilot guessing.
        if (in.mode == FlightMode::MOTOR_TUNE && !in.armed && !prev_mtune)
            ui_log::set("MTune: arm to start");
        if (in.mode == FlightMode::MOTOR_TUNE && g_params.mtune_en <= 0.5f && !prev_mtune)
            ui_log::set("MTune off: set MTUNE_EN=1");
        prev_autotune = at_active;
        prev_mtune = mt_active;

        // Safety monitor: while any AUTOMATIC maneuver is armed, a tumble/spin-out (and
        // RPM/NaN/depth-runaway) aborts it and DISARMS. The leak/battery/GCS
        // surface-failsafe below is separate.
        //
        // STUNT and PATTERN are included. They were not, which left the two modes that
        // drive full authority with no pilot in the loop as the ONLY automatic modes with
        // no guard at all — a PATTERN step that never converged (a current holding the
        // vehicle off depth, a stuck sensor) drove full heave indefinitely with nothing to
        // stop it. They now get the same coverage as a tune or a move.
        bool auto_armed  = (in.mode == FlightMode::AUTO)    && in.armed;
        bool stunt_armed = (in.mode == FlightMode::STUNT)   && in.armed;
        bool patt_armed  = (in.mode == FlightMode::PATTERN) && in.armed;
        if ((at_active || mt_active || auto_armed || stunt_armed || patt_armed) && in.armed) {
            const char* why = nullptr;
            // Depth reference for the runaway guard. A tune must not move depth at all, so it
            // is checked against where it started. AUTO and PATTERN *command* depth, so check
            // against the active SETPOINT instead: passing the current depth (as this did)
            // made the delta identically zero, i.e. AUTO had no depth protection whatsoever.
            // depth::target() is one cycle stale at 500 Hz, irrelevant next to ST_DEPTH_DELTA.
            float d0 = (at_active || mt_active) ? at_depth0 : depth::target();
            // Commanded rotations pass ST_ANGLE_MAX almost immediately and used to disarm
            // themselves mid-manoeuvre. Exempt ONLY the angle guard for those: a STYLE move,
            // and STUNT (whose whole purpose is a 360 deg rotation about one axis). The rate,
            // depth and RPM guards still apply, and those are what catch a genuine tumble.
            bool spinning = (auto_armed && movement::type() == movement::Type::STYLE) ||
                            stunt_armed;
            // STUNT owns no depth setpoint — it passes the pilot's throttle straight through
            // and never calls depth::setTarget(), so d0 is frozen at the mode-entry depth.
            // Checking against it would disarm mid-roll, possibly inverted, for a deliberate
            // pilot climb. Every other guard still applies to STUNT.
            bool depth_ref_valid = !stunt_armed;
            bool safe = safety_monitor::ok(in.roll, in.pitch, in.gx, in.gy, in.gz, in.depth, d0,
                                           mt_active ? in.rpm : nullptr, mt_active ? NUM_THRUSTERS : 0, &why,
                                           spinning, depth_ref_valid);
            if (!safe) {
                autotune::abort(); motor_tune::abort(); movement::abort();
                stunt::abort(); pattern::abort();
                char b[64]; snprintf(b, sizeof(b), "Disarmed: %s", why ? why : "safety abort");
                mav_stream::queueStatusText(MAV_SEVERITY_ERROR, b);   // -> GCS log + OLED
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) { g_state.control.armed = false; g_state.control.autotune_active = false; }
                in.armed = false;
                // Clear the LOCAL latches too. They were captured before this check, so the
                // tune branch further down still ran on an aborted tuner, saw "not running"
                // and queued "Disarmed: autotune finished" immediately after the safety
                // reason — reporting success for a run that was killed.
                at_active = false;
                mt_active = false;
                g_mtune_active = false;
                // And the EDGE latches, which were assigned above this block. Leaving them
                // set means next cycle sees (!at_active && prev_autotune) and fires the
                // ordinary stop-edge: a redundant abort plus an "Autotune stopped" INFO
                // queued one cycle behind the safety ERROR that actually explains it. The
                // tuners are already aborted here, so there is no edge left to handle.
                prev_autotune = false;
                prev_mtune = false;
            }
        }

        // Feed the thruster-pack voltage to the mixer's open-loop thrust linearisation.
        // 0 volts (no fresh ESP-NOW source) leaves the output uncompensated.
        mixer::setBatteryVoltage(in.thr_volts, dt);

        // --- Failsafes → controlled SURFACE ascent. ---
        // Declared OUTSIDE the mode guard so it can be cleared while already surfacing —
        // see the else branch at the bottom.
        static uint32_t s_bat_low_since = 0;
        // Was SURFACE entered BY A FAILSAFE, or did the operator ask for it?
        //
        // The auto-disarm below must only apply to the failsafe case. SURFACE is a normal,
        // directly selectable mode (JS_MODE_SURFACE / DO_SET_MODE 9), and a pilot who chooses
        // it to end a dive and hold at the top would otherwise be disarmed out from under
        // themselves after FS_SURFACE_HOLD_MS. Nothing in ControlState distinguishes the two,
        // so the distinction is latched here, where the failsafe actually forces the mode.
        static bool     s_fs_surface = false;
        static uint32_t s_at_surface_since = 0;
        if (in.mode != FlightMode::SURFACE) {
            // Left SURFACE (operator, or the failsafe cleared and something re-selected a
            // mode). Drop both latches: the next surfacing must observe its own full hold
            // window, not inherit a timestamp from a previous one.
            s_fs_surface = false;
            s_at_surface_since = 0;
            bool fs = false;
            const char* fs_why = nullptr;
            // Kept in lockstep with fs_why: every branch that assigns fs_why also assigns
            // this, so a later branch overwriting the reason cannot leave a stale flag and
            // print another failsafe's reason with a voltage stapled to it.
            bool fs_is_bat = false;
            if (g_params.leak_en > 0 && in.leak) { fs = true; fs_why = "leak"; fs_is_bat = false; }
            // Watch the THRUSTER battery, not the electronics one. This used to test
            // in.pm1 (the local ADC = SBC/electronics pack) against FS_BAT_VOLTAGE, which
            // is clearly a thruster-pack threshold — so it was guarding the wrong battery.
            // in.thr_volts is 0 when no fresh thruster-voltage source exists, and the
            // > 1.0 test then makes this inert rather than surfacing on a 0 V reading.
            // DEBOUNCED, unlike the others. A 4S LiPo driving eight T200s sags well over a
            // volt under load, so an instantaneous test would surface the vehicle mid-burst on
            // a pack that is actually fine. Require the voltage to stay low continuously for
            // FS_BAT_HOLD_MS; any sample above the threshold resets the timer. The 2nd board
            // already averages its ADC over 500 ms and sends at 4 Hz, so 3 s is ~12
            // consecutive low readings — far too long for a transient, still early enough that
            // a genuinely flat pack surfaces with reserve to spare.
            //
            // Note this ALSO makes the failsafe real for the first time: thr_volts was 0 until
            // the 2nd board's ESP-NOW sender existed, so the > 1.0 guard kept the whole branch
            // inert. 0 still means "no source", never "flat battery".
            //
            // The timer only runs while ARMED. The failsafe only ACTS when armed, so
            // counting idle bench time would let the 3 s window expire long before anyone
            // armed — and then surface on the very first armed cycle with zero debounce
            // actually observed under load. That is the same false trip this exists to
            // prevent, just relocated to the arm boundary.
            const float bat_v = in.thr_volts;
            if (in.armed && g_params.fs_bat_enable > 0 &&
                bat_v > 1.0f && bat_v < g_params.fs_bat_voltage) {
                uint32_t nb = millis();
                if (s_bat_low_since == 0) s_bat_low_since = nb;
                if (nb - s_bat_low_since >= FS_BAT_HOLD_MS) {
                    fs = true; fs_why = "low thruster battery"; fs_is_bat = true;
                }
            } else {
                s_bat_low_since = 0;
            }
            if (g_params.fs_gcs_enable  > 0 && in.armed && !mav_commands::gcsLinkOk()) {
                fs = true; fs_why = "GCS link lost"; fs_is_bat = false;
            }
            // Edge-trigger latch: this branch re-runs every loop while the fault holds,
            // so announce only when the reason changes, and re-arm once it clears.
            // (Comparing pointers is fine — every fs_why is a string literal.)
            static const char* s_fs_last = nullptr;
            if (fs && in.armed) {
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) g_state.control.mode = FlightMode::SURFACE;
                in.mode = FlightMode::SURFACE;
                s_fs_surface = true;     // this surfacing is the failsafe's, not the pilot's
                if (fs_why != s_fs_last) {           // was silent: it just started surfacing
                    s_fs_last = fs_why;
                    // 64, not more: queueStatusText copies into a 64-byte queue slot, so a
                    // larger local would imply headroom that does not exist. Worst case here
                    // is 49 chars.
                    char b[64];
                    // Quote the voltage on a battery trip. Without the number a spurious
                    // surface is impossible to diagnose after the fact — you cannot tell a
                    // flat pack from a load sag from a miscalibrated divider.
                    if (fs_is_bat)
                        snprintf(b, sizeof(b), "Failsafe: surfacing (%s %.1f V)", fs_why, bat_v);
                    else
                        snprintf(b, sizeof(b), "Failsafe: surfacing (%s)", fs_why ? fs_why : "?");
                    mav_stream::queueStatusText(MAV_SEVERITY_CRITICAL, b);
                }
            } else if (!fs) {
                s_fs_last = nullptr;
            }
        } else {
            // ALREADY SURFACING. Decide the end state, or there isn't one: a failsafe forces
            // SURFACE without disarming, so a permanent link loss left the vehicle armed and
            // station-keeping at 0 m indefinitely — thrusters live, nobody watching, until
            // the battery ran out. (JETSON_FEEDBACK.md §4.)
            //
            // Once at the surface and settled, disarm. Gated on a real depth reading: with no
            // Bar30 the ascent is open-loop (DEPTH_LOST_ASCENT) and "am I at the surface?" is
            // unanswerable, so we keep pushing up rather than disarm at an unknown depth and
            // sink. Requires the depth to stay shallow continuously, so a wave slapping the
            // sensor cannot disarm a vehicle that is still 2 m down.
            //
            // s_fs_surface gates the whole thing: a pilot who SELECTS SurfaceMode is not
            // failsafing and must keep their thrusters. Both latches are cleared on leaving
            // SURFACE, so every failsafe surfacing observes its own full hold window.
            if (s_fs_surface && in.armed && in.depth_ok && in.depth < FS_SURFACE_DEPTH_M) {
                uint32_t ns = millis();
                if (s_at_surface_since == 0) s_at_surface_since = ns;
                if (ns - s_at_surface_since >= FS_SURFACE_HOLD_MS) {
                    s_at_surface_since = 0;
                    StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                    if (lk.ok()) g_state.control.armed = false;
                    in.armed = false;
                    mav_stream::queueStatusText(MAV_SEVERITY_CRITICAL,
                                                "Failsafe: surfaced - disarmed");
                }
            } else {
                s_at_surface_since = 0;
            }
            // Already surfacing, so everything above is skipped and the debounce timer would
            // freeze at an already-expired value — leaving SURFACE with the pack still low
            // would then re-trip on the very next cycle with no fresh window observed. Clear
            // it so any resumed flight mode gets a full FS_BAT_HOLD_MS again.
            s_bat_low_since = 0;
        }

        // --- In-flight IMU health: WARN and degrade, never disarm. ---
        // Pre-arm checks gate the ARM transition (mav_commands::setArmed -> canArm); they
        // must not re-fire mid-dive. Re-running the full canArm() here used to disarm on a
        // transient sensor-mutex miss ("state busy"), on a 150 ms BNO085 reset-recovery
        // window, and even on "calibrating" — which made MOTOR_DETECT impossible, since it
        // sets cal.routine itself. For a submarine, cutting the thrusters mid-water is
        // worse than flying on the last known attitude: roll/pitch/yaw already hold their
        // last-good values, so we keep control and just say so.
        bool armed = in.armed;
        {
            static bool     s_imu_bad = false;      // debounced state
            static uint32_t s_bad_since = 0;
            uint32_t now_ms = millis();
            if (!in.imu_ok) {
                if (s_bad_since == 0) s_bad_since = now_ms;
                // 300 ms swallows the BNO085's 150 ms post-reset recovery entirely.
                if (!s_imu_bad && now_ms - s_bad_since > 300) {
                    s_imu_bad = true;
                    mav_stream::queueStatusText(MAV_SEVERITY_ERROR,
                        "IMU lost - holding last attitude, STILL ARMED");
                }
            } else {
                s_bad_since = 0;
                if (s_imu_bad) {
                    s_imu_bad = false;
                    mav_stream::queueStatusText(MAV_SEVERITY_INFO, "IMU recovered");
                }
            }
            // While the attitude is stale the gyro is stale too. Feeding a frozen non-zero
            // rate to the rate PIDs winds their integrators up against an error that can
            // never be corrected, so zero the rates instead of holding them.
            if (s_imu_bad) { in.gx = 0; in.gy = 0; in.gz = 0; }
        }

        // --- Motor test override takes precedence over the mixer. ---
        bool test_active = in.test_override && (millis() < in.test_expire);
        if (in.test_override && !test_active) {
            // Motor test keep-alive lapsed (normally: the operator released the button).
            // Clear the override so the motor stops immediately — but do NOT disarm.
            // ArduSub's verify_motor_test() disarms here; we deliberately don't, so one
            // arm session can cover several motors and arming stays the operator's call.
            // Safety is unaffected: the motor is commanded stop on this very cycle, and
            // every other auto-disarm path (arming checks, safety monitor, failsafes) is
            // untouched.
            { StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
              if (lk.ok()) { g_state.thrusters.test_override = false; g_state.thrusters.test_motor = -1; } }
            in.test_override = false;
            mav_stream::queueStatusText(MAV_SEVERITY_INFO, "Motor test ended");
        }

        if (test_active) {
            for (int i = 0; i < NUM_THRUSTERS; ++i) dshot[i] = DSHOT_DISARMED;
            if (in.test_motor >= 0 && in.test_motor < NUM_THRUSTERS) {
                // Apply the configured direction (MOT_n_DIRECTION x detected). This used
                // to force +1 "raw stimulus", which meant the Motor Test tab could never
                // show a direction change — the operator flipped the parameter, saw no
                // difference, and concluded the parameter was broken. Motor Test is the
                // tool people use to verify thruster orientation, so it must show what the
                // vehicle will actually do. (MOTOR_TUNE below keeps raw +1: it
                // characterises the motor itself, not the frame.)
                dshot[in.test_motor] = mixer::oneToDshot(in.test_throttle, in.dir[in.test_motor]);
            }
        } else if (mt_active) {
            // MOTOR_TUNE: drive ONE motor's raw throttle (the tuner picks it), rest stopped.
            int am = -1; float mtthr = 0;
            bool running = motor_tune::update(in.rpm, dt, millis(), am, mtthr);
            for (int i = 0; i < NUM_THRUSTERS; ++i) dshot[i] = DSHOT_DISARMED;
            if (armed && am >= 0 && am < NUM_THRUSTERS) dshot[am] = mixer::oneToDshot(mtthr, 1);
            StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
            if (lk.ok()) {
                g_state.control.stunt_progress = motor_tune::progress();
                if (!running) { g_state.control.armed = false;         // done → safe, disarmed
                                g_state.control.mode = FlightMode::STABILIZE;
                                mav_stream::queueStatusText(MAV_SEVERITY_INFO,
                                                            "Disarmed: motor tune finished"); }
            }
        } else {
            float roll, pitch, yaw, thr, fwd, lat;
            fwd = in.sp_forward; lat = in.sp_lateral;

            if (at_active) {
                bool running = autotune::update(in.roll, in.pitch, in.yaw,
                                                in.gx, in.gy, in.gz, in.depth,
                                                dt, millis(),
                                                roll, pitch, yaw, thr);
                fwd = 0; lat = 0;   // thr driven by the depth-tune phase
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    g_state.control.stunt_progress = autotune::progress();  // reuse for ATUNE %
                    if (!running) {
                        g_state.control.autotune_active = false;
                        g_params.atune = 0;   // clear the trigger param
                        mav_stream::queueStatusText(MAV_SEVERITY_INFO,
                                                    "Disarmed: autotune finished");
                        // Finished (or aborted): DISARM so the sub sits safely.
                        g_state.control.armed = false;
                        if (in.mode == FlightMode::AUTOTUNE) g_state.control.mode = FlightMode::STABILIZE;
                    }
                }
            } else if (in.mode == FlightMode::STUNT) {
                bool running = stunt::update(in.roll, in.pitch, in.gx, in.gy, in.gz, dt,
                                             roll, pitch, yaw);
                thr = in.sp_throttle;
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    g_state.control.stunt_progress = stunt::progress();
                    if (!running) g_state.control.mode = FlightMode::STABILIZE;
                }
            } else if (in.mode == FlightMode::PATTERN) {
                bool running = pattern::update(in.roll, in.pitch, in.yaw, in.gx, in.gy, in.gz,
                                               in.depth, dt, millis(),
                                               roll, pitch, yaw, thr);
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    g_state.control.pattern_step = pattern::step();
                    if (!running) g_state.control.mode = FlightMode::STABILIZE;
                }
            } else {
                computeDemands(in, dt, roll, pitch, yaw, thr, fwd, lat);
                // Model-based feedforward (drag + cross-coupling + CoB auto-trim). Learn
                // the trim only when the pilot sticks are near centre.
                bool learn = fabsf(in.sp_roll) < 0.1f && fabsf(in.sp_pitch) < 0.1f &&
                             fabsf(in.sp_yaw) < 0.1f && fabsf(in.sp_throttle) < 0.1f;
                feedforward::apply(roll, pitch, yaw, thr, in.gx, in.gy, in.gz, in.mode, learn);
            }

            mixer::mix(roll, pitch, yaw, thr, fwd, lat, norm);
            mixer::toDshot(norm, in.dir, armed, dshot);
        }

        // --- Publish thruster demand + loop timestamp. ---
        {
            StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
            if (lk.ok()) {
                for (int i = 0; i < NUM_THRUSTERS; ++i) {
                    g_state.thrusters.throttle[i] = dshot[i];
                    // Bake the configured motor direction (MOT_n_DIRECTION × auto-detect)
                    // into the published norm, so the Pico closed-loop RPM path — which
                    // derives its signed target from norm — reverses a thruster exactly
                    // like the raw DShot path (toDshot already applies in.dir[]). Setup-time
                    // MOT_n_DIRECTION now takes effect on the Pico backend too.
                    // Publish ZERO while disarmed. The controllers keep running (so they
                    // track attitude and are warm on arm), but their output must not leave
                    // this task: task_dshot_rmt converts norm[] into a target RPM and sends
                    // it to the Pico every cycle regardless of arm state. With a live norm[]
                    // the integrators' wound-up demand was handed straight to the Pico, and
                    // the instant you armed it became real thrust with no pilot input.
                    g_state.thrusters.norm[i]     = (!armed || test_active)
                                                        ? 0.0f
                                                        : (norm[i] * (float)in.dir[i]);
                }
                g_state.thrusters.armed = armed;
            }
        }
        {
            StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(1));
            if (lk.ok()) g_state.control.loop_stamp_ms = millis();
        }

        // Publish movement state for Task_MAVLink (ACK progress/complete + telemetry).
        //
        // UNCONDITIONAL, deliberately. This used to be gated on `in.mode == AUTO`, which meant
        // the one transition the ACK machine most needs to observe — a move ending because
        // AUTO was taken away — was the one it could never see. The falling edge below is what
        // latches mv_done_seq, and it has to be reachable from outside AUTO for the cancel()
        // above to mean anything. Outside AUTO the movement is idle, so this simply publishes
        // "not active" and the edge fires exactly once.
        {
            bool now_active = (movement::type() != movement::Type::NONE);
            StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(1));
            if (lk.ok()) {
                g_state.control.mv_active   = now_active;
                g_state.control.mv_state    = movement::phase();
                g_state.control.mv_progress = movement::progress();
                if (mv_was_active && !now_active) g_state.control.mv_done_seq = prev_mv_seq;  // completed
            }
            // Only advance the edge tracker when the write landed. On a lock miss the state
            // was not published, so clearing mv_was_active here would drop the completion.
            if (lk.ok()) mv_was_active = now_active;
        }

        vTaskDelayUntil(&last, period);
    }
}
