#pragma once

// =============================================================================
//  comms/mav_stream — periodic MAVLink telemetry senders + SROT identity
// =============================================================================

#include <Arduino.h>

namespace mav_stream {

// Send the SROT boot identity: a STATUSTEXT banner "SROT Vx.y.z" that QGC parses
// into the firmware-version field. Call once after the link is up.
void sendBootIdentity();

// Send AUTOPILOT_VERSION (SROT vendor/product + flight_custom_version "SROT").
// Sent in response to a capabilities request.
void sendAutopilotVersion();

// Rate-scheduled telemetry. Call frequently from Task_MAVLink with millis().
void update(uint32_t now_ms);

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
