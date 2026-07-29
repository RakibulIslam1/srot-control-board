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
#include "comms/params.h"       // defaultsWereReset() boot notice

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
