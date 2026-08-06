// =============================================================================
//  Task_MAVLink (Core 0) — MAVLink telemetry + command handling
//  P2: SROT-identity telemetry out; params/commands/manual-control in.
// =============================================================================

#include "tasks.h"
#include <math.h>
#include <nvs_flash.h>   // nvs_get_stats — NVS free-entry report at boot
#include "config.h"
#include "state_types.h"
#include "comms/mavlink_bridge.h"
#include "comms/mav_stream.h"
#include "comms/mav_commands.h"
#include "comms/params.h"       // defaultsWereReset() / nvsWasReformatted() boot notices
#include "drivers/bno085.h"     // calConfigOk() — is dynamic cal actually running?
#include "drivers/analog_mon.h" // voltMultLooksStale() — PM1_VMULT units-change warning

// Reset reason captured in main.cpp setup().
extern const char* g_reset_reason;

void Task_MAVLink(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_MAVLINK_HZ);
    TickType_t last = xTaskGetTickCount();

    // Emit a HEARTBEAT immediately so QGC detects the vehicle right after the
    // USB-DTR reset, then announce the SROT identity + the reset reason (this is
    // the key diagnostic: a "RST: TASK_WDT/BROWNOUT/PANIC" arriving on every
    // reconnect proves the board is rebooting, and says why).
    mav_stream::sendHeartbeatNow();
    mav_stream::sendBootIdentity();
    mav_stream::sendAutopilotVersion();
    {
        char rst[40];
        snprintf(rst, sizeof(rst), "RST: %s", g_reset_reason);
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING, rst);
    }
    // Never let a defaults reset be silent — it discards the user's saved tuning.
    if (params::defaultsWereReset()) {
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                                   "Params reset to build defaults");
    }
    // Louder than a defaults reset: this also wipes the sensor CALIBRATION, which lives in
    // the same NVS. Expected exactly once, after the partition was enlarged. Any other time
    // it means the flash storage itself failed.
    if (params::nvsWasReformatted()) {
        mav_stream::sendStatusText(MAV_SEVERITY_CRITICAL,
                                   "NVS reformatted - params AND calibration lost, re-import");
    }
    // PM1_VMULT changed units (volts-per-count -> divider ratio) when the battery reading
    // moved to the calibrated ADC path. A backup written before that change carries the old
    // value; applying it as a ratio reports ~0 V on a healthy pack. The value is APPLIED
    // either way (see analog_mon::read) — this is advice, not a rejection.
    if (analog_mon::voltMultLooksStale(g_params.pm1_vmult)) {
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                                   "PM1_VMULT looks stale (it is a divider ratio, ~11)");
    }
    // PM2_VMULT was rewritten from the old volts-per-count units to the new trim. Announced
    // because it silently changes a number the operator may have set by hand.
    if (params::pm2VmultMigrated()) {
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                                   "PM2_VMULT migrated to 1.0 (it is now an ESP-NOW trim)");
    }
    // The PM1 rework moved which physical PIN the pack voltage is read from. Announce it:
    // a silent pin change is indistinguishable from a wiring fault if the reading looks odd.
    if (params::pm1PinsMigrated()) {
        char b[64];
        snprintf(b, sizeof(b), "PM1 migrated: volt=GPIO%d curr=GPIO%d mult=%.1f",
                 (int)g_params.pin_battvolt, (int)g_params.pin_battcurr,
                 (double)g_params.pm1_vmult);
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING, b);
    }
    // MAG_YAW_REF was switched on for a board that stored the old 0. Announce it, because
    // it changes what `yaw` MEANS -- absolute compass heading instead of relative-to-boot --
    // so a heading noted from an earlier run is no longer the same number.
    if (params::magYawRefMigrated()) {
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                                   "MAG_YAW_REF on: yaw is now ABSOLUTE, not boot-relative");
    }
    // If sh2_setCalConfig() failed the sensor is not running dynamic calibration at all, so
    // mag accuracy can never climb and no amount of figure-eights will help. That has to be
    // distinguishable from "you have not calibrated yet".
    // Deliberately NOT reported here any more. The first sh2_setCalConfig() is normally
    // rejected (the part needs ~90 ms after reset), so this fired on every healthy boot and
    // told the operator their mag would never converge when it was about to. poll() retries
    // it and announces the success; a genuine permanent failure shows up as the absence of
    // that message plus MAGACC never leaving 0.
    // Where did the calibration come from? "Is my cal actually loaded" should not require a
    // param download to answer -- and after a power-cycle complaint it is the first question.
    {
        bool from_nvs = false;
        { StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(10));
          if (lk.ok()) from_nvs = g_state.cal.loaded_from_nvs; }
        char b[64];
        nvs_stats_t st;
        if (nvs_get_stats(nullptr, &st) == ESP_OK) {
            snprintf(b, sizeof(b), "CAL %s | NVS %u/%u entries free",
                     from_nvs ? "loaded from NVS" : "DEFAULTS (not calibrated)",
                     (unsigned)st.free_entries, (unsigned)st.total_entries);
        } else {
            snprintf(b, sizeof(b), "CAL %s | NVS stats unavailable",
                     from_nvs ? "loaded from NVS" : "DEFAULTS (not calibrated)");
        }
        mav_stream::sendStatusText(from_nvs ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING, b);

        // Report the MAG calibration specifically, with its numbers. "is my compass
        // calibration actually saved" has been the operator's question for several rounds and
        // there was no way to answer it without a param download -- so a save that worked and
        // a save that silently did nothing looked identical from the console. The magnitude
        // of the offset vector is enough to tell "calibrated" from "all zeros" at a glance.
        float ox = 0, oy = 0, oz = 0, sx = 1;
        { StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(10));
          if (lk.ok()) { ox = g_state.cal.mag_offset.x; oy = g_state.cal.mag_offset.y;
                         oz = g_state.cal.mag_offset.z; sx = g_state.cal.mag_scale.x; } }
        const float omag = sqrtf(ox * ox + oy * oy + oz * oz);
        char mb[64];
        if (omag > 1e-6f) snprintf(mb, sizeof(mb), "MAG cal PRESENT |off|=%.1fuT sx=%.2f", omag, sx);
        else              snprintf(mb, sizeof(mb), "MAG cal MISSING - heading stays relative");
        mav_stream::sendStatusText(omag > 1e-6f ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING, mb);
    }
    // Same for PM1_VMULT — the migration R21 gave PM2 and forgot to give PM1.
    if (params::pm1VmultMigrated()) {
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                                   "PM1_VMULT migrated to 11.28 (divider ratio)");
    }
    // The thruster-pack voltage exists ONLY over ESP-NOW. With the link off the OLED shows
    // OFF, Battery 2 stays empty, and both the mixer's voltage linearisation and the
    // low-thruster-battery failsafe are inert — all of which look like faults if you do not
    // know the link was never switched on. Say so once at boot instead.
    if (!params::espnowWanted() && (int)g_params.pm2_src != 0) {
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                                   "ESPNOW_EN=-1 - no thruster voltage (set it to 0)");
    }
    // PM2_SRC = 1 meant "local ADC" in the header comment and NOTHING in the code: there is no
    // second voltage divider on this board, so the branch never existed and the value fell
    // through to "off" in silence. A board in the vehicle was found set to 1 on 2026-08-02 —
    // ESP-NOW was up and delivering 15.4 V the whole time and every consumer read zero.
    // It is now treated as ESP-NOW (the only source PM2 can physically have), and said out loud.
    if ((int)g_params.pm2_src == 1) {
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                                   "PM2_SRC=1 has no 2nd ADC - using ESP-NOW; set it to 2");
    }

    for (;;) {
        // Drain all pending inbound messages.
        mavlink_message_t msg;
        while (mav::poll(msg)) {
            mav_commands::handle(msg);
        }

        uint32_t now = millis();
        mav_commands::update(now);
        mav_stream::update(now);

        vTaskDelayUntil(&last, period);
    }
}
