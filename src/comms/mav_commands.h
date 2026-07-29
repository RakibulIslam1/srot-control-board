#pragma once

// =============================================================================
//  comms/mav_commands — inbound MAVLink dispatch (params, commands, control in)
// =============================================================================

#include <Arduino.h>
#include "comms/mavlink_bridge.h"

namespace mav_commands {

// Dispatch one received message (params, commands, SET_MODE, MANUAL_CONTROL,
// GCS heartbeat tracking).
void handle(const mavlink_message_t& msg);

// Live pilot gain (0.1..1.0), stepped by the JS_GAIN_* button functions. Runtime only —
// JS_GAIN_DEFAULT stays the power-on value. Reported to the GCS as NAMED_VALUE_FLOAT
// "GAIN" so a joystick UI can show what the buttons are doing.
float pilotGain();

// Periodic housekeeping: drains the PARAM_REQUEST_LIST stream and updates the
// GCS-link failsafe timer. Call from Task_MAVLink with millis().
void update(uint32_t now_ms);

// True while a GCS heartbeat has been seen within GCS_FAILSAFE_MS.
bool gcsLinkOk();

// Mark GCS activity now (any inbound packet counts as a live link). Used by the
// LoRa uplink path so the link stays "up" while commands flow, not only heartbeats.
void feedGcs();

// True while a full parameter-list download is streaming (telemetry throttles
// itself so QGC's download completes quickly and its setup tabs latch).
bool paramDownloadActive();

// Param-download progress for the OLED: n = index sent, total = count (0/0 idle).
void paramDownloadProgress(uint16_t& n, uint16_t& total);

}  // namespace mav_commands
