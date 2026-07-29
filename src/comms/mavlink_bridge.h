#pragma once

// =============================================================================
//  comms/mavlink_bridge — low-level MAVLink v2 transport over UART0
//
//  Includes the ardupilotmega dialect (which supersets common/minimal) so every
//  comms module shares the generated types. We use pack + to_send_buffer (not
//  the _send convenience functions), so no comm_send_ch hook is needed.
// =============================================================================

#include <Arduino.h>

// Suppress the packed-member address warnings the generated headers emit under
// -Wall on this toolchain (GCC 8.4 has no -Wno-address-of-packed-member flag).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <ardupilotmega/mavlink.h>
#pragma GCC diagnostic pop

namespace mav {

// Serialize and write a packed message to the link. NON-BLOCKING: if the UART
// TX buffer can't hold the whole message it is dropped and false is returned
// (never blocks the caller). Telemetry ignores the result and re-sends next
// interval; the param stream must NOT advance its index on a false return.
bool tx(const mavlink_message_t& msg);

// Reliable send for messages QGC must not lose (COMMAND_ACK, PARAM_VALUE,
// STATUSTEXT): waits (yielding, feeding the watchdog) up to `timeout_ms` for TX
// buffer space, then writes. Bounded — returns false and gives up if the buffer
// never frees (so it can never hang the task). Default 20 ms.
bool txReliable(const mavlink_message_t& msg, uint32_t timeout_ms = 20);

// Non-blocking parse: drains available UART bytes; returns true and fills `out`
// when a complete message has been decoded (call again for more).
bool poll(mavlink_message_t& out);

// --- LoRa downlink tap -------------------------------------------------------
// When enabled (a LoRa GCS is connected), tx()/txReliable() also copy selected
// low-rate control-plane messages (PARAM_VALUE, COMMAND_ACK, STATUSTEXT,
// AUTOPILOT_VERSION) into a ring buffer so Task_LoRa_SD can relay them over LoRa
// (Bondor then receives real params/ACKs/prompts over the radio link). No effect
// on USB/UDP links (tap stays off). Thread-safe.
void loraTapEnable(bool on);
// Pop the next queued frame into `buf` (up to `cap`); returns its length, or 0 if
// the queue is empty. Call from Task_LoRa_SD.
uint16_t loraTapPop(uint8_t* buf, uint16_t cap);

}  // namespace mav
