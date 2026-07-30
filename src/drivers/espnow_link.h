#pragma once

// =============================================================================
//  drivers/espnow_link — ESP-NOW receive-only link from the 2nd board
//
//  The 2nd board (src/second_board, env:second-board) broadcasts thruster-kill state
//  and the THRUSTER-pack voltage; this board receives them (WiFi STA, no
//  association) and sends nothing back.
//
//  The wire format lives in shared/espnow_proto.h so both boards compile the SAME
//  definition — it used to be a hand-copied comment here, which is how two boards
//  end up disagreeing about a format with no version field.
// =============================================================================

#include <Arduino.h>
#include "espnow_proto.h"      // shared with src/second_board

namespace espnow_link {

using FromSecond = espnow_from_second_t;
constexpr uint8_t MAGIC = ESPNOW_MSG_MAGIC;

// Bring up WiFi STA + ESP-NOW on ESPNOW_CHANNEL and register the RX callback.
// Returns false if ESP-NOW init fails. No-op if ESPNOW_ENABLE is 0.
bool begin();

// Latest values. `kill`/`aux_v` reflect the most recent packet; `fresh` is false
// once the link has been silent for ESPNOW_STALE_MS (then kill is forced false).
void poll(bool& kill, float& aux_v, bool& fresh);

}  // namespace espnow_link
