// =============================================================================
//  Task_Buzzer (Core 0) — owns the passive buzzer entirely, @ TASK_BUZZER_HZ.
//
//  Runs at 200 Hz (5 ms) so tone lengths stay uniform — on the old 30 Hz UI task
//  the durations quantized to the jittery loop period ("sometimes long, sometimes
//  short"). Because play() and update() both run here, there is no cross-task race.
//
//  It self-detects events from shared state (startup, arm/disarm, Pico link, GCS
//  link, leak) and plays the matching tone, each gated by the BUZZ_MASK param. It
//  also consumes the one-shot IndicatorState::buzz_request (calibration steps).
//  Separately it requests the Pixhawk-style ESC beacon beep (THR_BEEP_EN).
// =============================================================================

#include "tasks.h"
#include "config.h"
#include "state_types.h"
#include "drivers/buzzer.h"
#include "comms/params.h"
#include "comms/mav_commands.h"
#include "comms/ui_log.h"

// One-shot ESC beacon-beep request (Core 0 sets, the DShot task on Core 1 sends it to
// the Pico as TL_FLAG_BEEP and clears it). A missed/extra beep is harmless.
volatile bool g_thr_beep_req = false;

static inline bool maskOn(uint8_t bit) { return ((uint8_t)g_params.buzz_mask) & bit; }

void Task_Buzzer(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_BUZZER_HZ);
    TickType_t last = xTaskGetTickCount();

    buzzer::begin();

    // Edge-tracking state.
    bool     prev_armed   = false;
    bool     prev_link    = false;   // Pico link
    bool     pico_ever_up = false;
    bool     gcs_ever_up  = false;
    uint32_t gcs_lost_since = 0;   // debounce: ignore brief link blips (esp. over LoRa)
    bool     gcs_lost_fired = false;
    bool     prev_leak    = false;
    bool     prev_cal     = false;   // calibration active
    bool     started      = false;
    uint32_t leak_last_ms = 0;

    auto playIf = [](BuzzTone t, uint8_t bit) { if (maskOn(bit)) buzzer::play(t); };

    for (;;) {
        uint32_t now = millis();

        // Power-up melody (once) — held ~900 ms so it isn't drawing buzzer current during
        // the OLED init (a concurrent brown-out was blanking the display). Also fires the
        // ESC beacon chirp.
        if (!started && now > 900) {
            started = true;
            playIf(BuzzTone::STARTUP, BUZZ_BIT_STARTUP);
            if (g_params.thr_beep_en > 0.5f) g_thr_beep_req = true;
        }

        // --- Snapshot the few flags we react to (short locks). ---
        // CRITICAL: default to the PREVIOUS value, not false. mtx_thrusters is written by the
        // 500 Hz control loop, so this 2 ms lock occasionally times out — defaulting to false
        // then created a phantom link_ok/armed edge → a spurious PICO_LOST/PICO_OK (or ARM/
        // DISARM) beep at random intervals. Holding the last value on a miss = no false edge.
        bool armed = prev_armed;
        {
            StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
            if (lk.ok()) armed = g_state.control.armed;
        }
        bool link_ok = prev_link;
        {
            StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
            if (lk.ok()) link_ok = g_state.thrusters.link_ok;
        }
        bool leak = prev_leak;
        {
            StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(2));
            if (lk.ok()) leak = g_state.sensors.leak;
        }
        bool cal = prev_cal;
        {
            StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(2));
            if (lk.ok()) cal = (g_state.cal.routine != CalRoutine::NONE);
        }
        bool gcs = mav_commands::gcsLinkOk();

        // --- Arm / disarm edges. ---
        if (armed != prev_armed) {
            playIf(armed ? BuzzTone::ARM : BuzzTone::DISARM,
                   armed ? BUZZ_BIT_ARM : BUZZ_BIT_DISARM);
            ui_log::set(armed ? "Armed" : "Disarmed");   // name it (incl. auto-disarm)
            if (armed && g_params.thr_beep_en > 0.5f) g_thr_beep_req = true;  // "connected" chirp
            prev_armed = armed;
        }

        // --- Pico link lost / restored (ignore the very first connect at boot). ---
        if (link_ok != prev_link) {
            // NOTE: don't log "restored" — it would overwrite the DShot task's "Flap#n <REASON>"
            // / "PICO REBOOT#n" line before it can be read. Just sound the restore chirp.
            if (link_ok) { if (pico_ever_up) playIf(BuzzTone::PICO_OK, BUZZ_BIT_PICO_LINK);
                           pico_ever_up = true; }
            // (The DShot task logs the drop REASON — "Pico gap: SILENT/NOISE" — so don't
            //  overwrite it here; just sound the alert.)
            else if (pico_ever_up) playIf(BuzzTone::PICO_LOST, BUZZ_BIT_PICO_LINK);
            prev_link = link_ok;
        }

        // --- GCS link lost (only after it was ever up; debounced ~1.5 s so a brief LoRa
        //     blip doesn't beep/log spuriously). ---
        if (gcs) {
            gcs_ever_up = true;
            gcs_lost_since = 0;
            gcs_lost_fired = false;
        } else if (gcs_ever_up) {
            if (gcs_lost_since == 0) gcs_lost_since = now;
            else if (!gcs_lost_fired && now - gcs_lost_since > 1500) {
                gcs_lost_fired = true;
                playIf(BuzzTone::GCS_LOST, BUZZ_BIT_GCS_LOST);
                ui_log::set("GCS link LOST");
            }
        }

        // --- Leak: alert on rising edge, then re-alert every 3 s while active. ---
        if (leak && (!prev_leak || now - leak_last_ms > 3000)) {
            playIf(BuzzTone::LEAK, BUZZ_BIT_LEAK);
            ui_log::set("!! LEAK DETECTED !!");
            leak_last_ms = now;
        }
        prev_leak = leak;

        // --- Calibration start / finish. ---
        if (cal != prev_cal) {
            playIf(cal ? BuzzTone::CAL_START : BuzzTone::CAL_DONE, BUZZ_BIT_CALIB);
            ui_log::set(cal ? "Calibration started" : "Calibration done");
            prev_cal = cal;
        }

        // --- Explicit one-shot request (calibration steps, etc.). ---
        {
            StateLock lk(g_state.mtx_indicators, pdMS_TO_TICKS(2));
            if (lk.ok() && g_state.indicators.buzz_request != BuzzTone::NONE) {
                if (maskOn(BUZZ_BIT_CALIB)) buzzer::play(g_state.indicators.buzz_request);
                g_state.indicators.buzz_request = BuzzTone::NONE;
            }
        }

        buzzer::update(now);
        vTaskDelayUntil(&last, period);
    }
}
