#pragma once

// =============================================================================
//  comms/mav_stream — periodic MAVLink telemetry senders + firmware identity
// =============================================================================

#include <Arduino.h>

namespace mav_stream {

// Send the boot identity: a STATUSTEXT banner "Hengla vx.y.z ready", which a GCS
// parses into its firmware-version field. Call once after the link is up.
void sendBootIdentity();

// Send AUTOPILOT_VERSION (SROT board vendor id + Hengla product id, and
// flight_custom_version = "HENGLA"). Sent in response to a capabilities request.
void sendAutopilotVersion();

// Rate-scheduled telemetry. Call frequently from Task_MAVLink with millis().
void update(uint32_t now_ms);

// MAV_CMD_SET_MESSAGE_INTERVAL (511) / GET_MESSAGE_INTERVAL (510) backing.
//
// `interval_us`: >0 sets the interval (clamped to a sane band), 0 restores this stream's
// compiled default, <0 disables the stream. Returns false when `msgid` is not a stream this
// firmware emits, so the caller can answer DENIED rather than silently accepting.
//
// This exists because the fixed rates were the hard ceiling on every companion-side loop —
// ATTITUDE at 10 Hz could be neither raised for a control loop nor lowered for a slow link.
bool    setMessageInterval(uint32_t msgid, int32_t interval_us);
int32_t getMessageInterval(uint32_t msgid);   // µs; -1 disabled, 0 unknown/default

// Force one HEARTBEAT now (used on mode/arm change for snappy GCS feedback).
void sendHeartbeatNow();

// Send a STATUSTEXT (severity = MAV_SEVERITY_*). Used for arming feedback and
// calibration prompts/results.
// MUST be called from Task_MAVLink (Core 0) — it writes UART0 directly.
void sendStatusText(uint8_t severity, const char* text);

// Queue a STATUSTEXT from ANY task/core. Only copies the string under a short
// spinlock; Task_MAVLink emits it on its next update(). Use this from the Core-1
// control loop so real-time code never touches UART0 or blocks on the tx mutex.
// Silently drops if the small queue is full (these are human-facing notices).
void queueStatusText(uint8_t severity, const char* text);

}  // namespace mav_stream
