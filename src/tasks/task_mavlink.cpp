// =============================================================================
//  Task_MAVLink (Core 0) — MAVLink telemetry + command handling
//  P2: SROT-identity telemetry out; params/commands/manual-control in.
// =============================================================================

#include "tasks.h"
#include "config.h"
#include "state_types.h"
#include "comms/mavlink_bridge.h"
#include "comms/mav_stream.h"
#include "comms/mav_commands.h"
#include "comms/params.h"       // defaultsWereReset() / nvsWasReformatted() boot notices
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
    // value; applying it as a ratio would report ~0 V on a healthy pack. read() substitutes
    // the default, and this says so rather than leaving a plausible-looking wrong number.
    if (analog_mon::voltMultLooksStale(g_params.pm1_vmult)) {
        mav_stream::sendStatusText(MAV_SEVERITY_WARNING,
                                   "PM1_VMULT is stale (now a divider ratio) - recalibrate");
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
