// =============================================================================
//  Task_DShot_RMT (Core 1) — thruster output @ TASK_DSHOT_HZ (500 Hz)
//  Emits g_state.thrusters.throttle[] each cycle. Backend selected by
//  THRUSTER_BACKEND (config.h):
//    THRUSTER_RMT  — on-board ESP32 RMT DShot (dshot_out), no RPM.
//    THRUSTER_PICO — UART link to the Pico 2 co-processor; publishes RPM back.
// =============================================================================

#include "tasks.h"
#include "config.h"
#include "state_types.h"
#if THRUSTER_BACKEND == THRUSTER_PICO
  #include "drivers/thruster_link.h"
  #include "thruster_link_proto.h"   // TL_ST_PRESENT
  #include "comms/ui_log.h"
  #include "comms/params.h"          // g_params.rpm_* for the config push
  #include "control/thrust_trim.h"   // slow RPM-based thrust normalisation
  #include "comms/mav_stream.h"      // queueStatusText — stall reports
  #include "comms/mavlink_bridge.h"  // MAV_SEVERITY_*
#else
  #include "drivers/dshot_out.h"
#endif

extern volatile uint32_t g_stack_hw[6];   // [5] = this task
extern volatile bool g_thr_beep_req;      // set by Task_Buzzer; one-shot ESC beacon chirp
volatile bool g_mtune_active = false;     // MOTOR_TUNE: force RAW throttle (control loop sets it)

void Task_DShot_RMT(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_DSHOT_HZ);
    TickType_t last = xTaskGetTickCount();

#if THRUSTER_BACKEND == THRUSTER_PICO
    thruster_link::begin();
#else
    dshot_out::begin();
#endif

    // These persist across cycles so a missed state-lock reuses the LAST command instead of
    // sending nothing — the Pico is request/response, so a skipped send = no telemetry that
    // cycle, and a run of lock misses would flap the link. Init to a safe stop/disarmed.
    int16_t out[NUM_THRUSTERS] = {0};
    float   nrm[NUM_THRUSTERS] = {0};
    bool    armed = false, testing = false;
    uint32_t stk_cnt = 0;
    int beep_frames = 0;      // when >0, tag outgoing frames with the beep flag
    bool prev_link = false;   // Pico link edge, for the gap-reason diagnostic
    uint32_t last_cycle_ms = 0;   // to catch a flight-core (Core 1) stall of THIS task
    uint32_t last_stall_ms = 0;   // when this task last stalled (to attribute a flap)
    uint32_t prev_uptime   = 0;   // last Pico uptime (detect a Pico reboot = jumps backward)
    uint32_t reboot_cnt    = 0;
    uint32_t cfg_last_ms   = 0;   // periodic RPM-config push (survives a Pico reset)

    for (;;) {
        if (++stk_cnt >= TASK_DSHOT_HZ) { stk_cnt = 0; g_stack_hw[5] = uxTaskGetStackHighWaterMark(nullptr); }

        // If THIS task didn't run for a while, the flight core (Core 1) was starved — that
        // stops the Pico poll and flaps the link. Log it so we can tell an ESP32-side stall
        // apart from a Pico-side reset. (Runs at 500 Hz, so a >120 ms gap is abnormal.)
#if THRUSTER_BACKEND == THRUSTER_PICO
        uint32_t nowc = millis();
        if (last_cycle_ms != 0 && (nowc - last_cycle_ms) > 120) last_stall_ms = nowc;
        last_cycle_ms = nowc;
#endif

        {
            StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
            if (lk.ok()) {   // refresh the cache; on a miss we keep the previous command
                for (int i = 0; i < NUM_THRUSTERS; ++i) { out[i] = g_state.thrusters.throttle[i]; nrm[i] = g_state.thrusters.norm[i]; }
                armed = g_state.thrusters.armed;
                testing = g_state.thrusters.test_override;
            }
        }

#if THRUSTER_BACKEND == THRUSTER_PICO
        // One-shot ESC beacon chirp: latch the request and tag ~6 consecutive frames so
        // the Pico can't miss it. Only sent once the link is up, and only while disarmed.
        if (g_thr_beep_req && thruster_link::linkOk()) { g_thr_beep_req = false; beep_frames = 6; }
        bool beep = false;
        if (beep_frames > 0 && !armed) { beep = true; beep_frames--; }
        else if (armed) beep_frames = 0;

        // Closed-loop RPM (Stage 2) unless disabled or a motor test is running (test =
        // raw DShot for predictable bench behaviour).
        bool rpm_mode = (THR_RPM_CLOSED_LOOP != 0) && !testing && !g_mtune_active;
        int16_t tgt[NUM_THRUSTERS] = {0};   // target rpm (rpm_mode) — kept for the stall message
        if (rpm_mode) {   // always send (cached command on a lock miss) — never starve the link
            // Scale by the RPM_MAX *param*, not the THR_MAX_RPM #define: the Pico
            // normalises its loop error by the same value (g_rpm_max, pushed in the cfg
            // frame), so two sources of truth could silently disagree. Using the param
            // also lets it be matched to the thruster/battery from Bondor without a
            // rebuild (a T200 tops out ~3600 rpm at 16 V, not the old 4000).
            float rpm_full = g_params.rpm_max > 1.0f ? g_params.rpm_max : (float)THR_MAX_RPM;
            for (int i = 0; i < NUM_THRUSTERS; ++i) {
                float r = nrm[i] * rpm_full;
                tgt[i] = (int16_t)constrain(r, -32000.0f, 32000.0f);
            }
            thruster_link::send(tgt, armed, false, true, beep);
        } else {
            thruster_link::send(out, armed, false, false, beep);
        }
        // Drain the Pico's telemetry reply and publish RPM/status back to state.
        int16_t rpm[NUM_THRUSTERS];
        uint8_t st[NUM_THRUSTERS];
        uint8_t fault = 0;
        bool fresh = thruster_link::poll(rpm, st, fault);
        // Pico-reset detector: its uptime jumping backward = it rebooted. Catches a reset even
        // when the Pico recovers fast enough that the link never drops. Definitive answer.
        if (fresh) {
            uint32_t up = thruster_link::picoUptime();
            if (prev_uptime > 3000 && up + 500 < prev_uptime) {
                char b[32]; snprintf(b, sizeof(b), "PICO REBOOT#%lu", (unsigned long)++reboot_cnt);
                ui_log::set(b);
            }
            prev_uptime = up;
        }
        {
            StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
            if (lk.ok()) {
                if (fresh) {
                    uint8_t present = 0;
                    for (int i = 0; i < NUM_THRUSTERS; ++i) {
                        g_state.thrusters.rpm[i] = rpm[i];
                        g_state.thrusters.esc_status[i] = st[i];
                        if (st[i] & TL_ST_PRESENT) present |= (1 << i);
                    }
                    g_state.thrusters.esc_fault = fault;
                    g_state.thrusters.esc_present = present;
                }
                g_state.thrusters.link_ok = thruster_link::linkOk();
            }
        }
        // Announce a stalled thruster HERE rather than in mav_stream: this is the only
        // place the commanded value and the measured rpm are in scope on the same cycle,
        // so the message can actually say what was asked for vs what happened. A stall
        // WARNS but never disarms — the pilot decides what to do about it.
        if (fresh) {
            static uint8_t s_fault_prev = 0;
            uint8_t rising = (uint8_t)(fault & ~s_fault_prev);
            s_fault_prev = fault;
            for (int i = 0; i < NUM_THRUSTERS; ++i) {
                if (!(rising & (1 << i))) continue;
                char b[64];
                if (rpm_mode) {
                    snprintf(b, sizeof(b), "Thruster %d STALLED: target=%d rpm, got %d",
                             i + 1, (int)tgt[i], (int)rpm[i]);
                } else {
                    snprintf(b, sizeof(b), "Thruster %d STALLED: dshot=%d rpm=%d",
                             i + 1, (int)out[i], (int)rpm[i]);
                }
                mav_stream::queueStatusText(MAV_SEVERITY_ERROR, b);
            }
        }
        // --- Slow thrust normalisation (~10 Hz) ---------------------------------------
        // Learns a bounded per-thruster gain so a given command produces the same thrust
        // as the battery drains. Deliberately OUTSIDE the attitude path and ~100x slower
        // than it: RPM as a fast setpoint is what made the vehicle oscillate.
        {
            static uint32_t trim_last_ms = 0;
            static bool     trim_prev_armed = false;
            uint32_t nowt = millis();
            if (trim_last_ms == 0) trim_last_ms = nowt;
            if (armed && !trim_prev_armed) thrust_trim::reset();   // clean slate each dive
            trim_prev_armed = armed;

            if (nowt - trim_last_ms >= 100) {
                float dt_s = (nowt - trim_last_ms) * 0.001f;
                trim_last_ms = nowt;
                // Only learn while armed and actually driving; never while a motor
                // test/tune is stimulating the thrusters, and never without telemetry.
                bool allow = armed && !testing && !g_mtune_active && thruster_link::linkOk() &&
                             (g_params.dshot_bidir >= 0.5f);
                uint8_t present = 0;
                int16_t rpm_now[NUM_THRUSTERS] = {0};
                { StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
                  if (lk.ok()) {
                      present = g_state.thrusters.esc_present;
                      for (int i = 0; i < NUM_THRUSTERS; ++i) rpm_now[i] = g_state.thrusters.rpm[i];
                  } else {
                      allow = false;   // couldn't read a consistent snapshot this tick
                  } }
                thrust_trim::update(nrm, rpm_now, present, dt_s, allow);
            }
        }

        // Push the Pico's RPM closed-loop config ~1 Hz so it always has current gains and
        // re-applies them after a Pico reset (MOTOR_TUNE writes g_params.rpm_*).
        if (millis() - cfg_last_ms >= 1000) {
            cfg_last_ms = millis();
            thruster_link::sendConfig(g_params.rpm_kp, g_params.rpm_ki, g_params.rpm_ff_a,
                                      g_params.rpm_idle, g_params.rpm_max,
                                      g_params.rpm_filt, g_params.rpm_slew,
                                      (uint8_t)(g_params.rpm_loop >= 0.5f ? 1 : 0),
                                      (uint8_t)(g_params.dshot_bidir >= 0.5f ? 1 : 0),
                                      g_params.mot_spin_min, g_params.rpm_min_tgt);
        }

        // On the link-drop edge: resync (flush stale RX) + log ONE self-classifying line naming
        // the guilty side. With telemetry now streamed, this should essentially never fire.
        bool link_now = thruster_link::linkOk();
        if (prev_link && !link_now) {
            thruster_link::resyncRx();
            const char* why = (last_stall_ms != 0 && (millis() - last_stall_ms) < 600) ? "ESP32-STALL"
                            : thruster_link::bytesRecent() ? "LINK-NOISE" : "PICO-SILENT";
            char b[32]; snprintf(b, sizeof(b), "Flap#%lu %s", (unsigned long)thruster_link::flapCount(), why);
            ui_log::set(b);
        }
        prev_link = link_now;
#else
        dshot_out::write(out);   // cached command on a lock miss
#endif

        vTaskDelayUntil(&last, period);
    }
}
