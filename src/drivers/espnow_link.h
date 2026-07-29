#pragma once

// =============================================================================
//  drivers/espnow_link — ESP-NOW receive-only link from the 2nd board (P11)
//
//  The 2nd board broadcasts thruster-kill state + an aux voltage; this board
//  receives them (WiFi STA, no association). Main sends nothing back.
//
//  Packet contract (the 2nd board must send exactly this, little-endian):
//     struct EspNowFromSecond { uint8_t magic=0x53; uint8_t thruster_kill;
//                               float aux_voltage; }   // 6 bytes
// =============================================================================

#include <Arduino.h>

namespace espnow_link {

#pragma pack(push, 1)
struct FromSecond {
    uint8_t magic;          // 0x53 ('S')
    uint8_t thruster_kill;  // 0/1
    float   aux_voltage;    // volts
};
#pragma pack(pop)

constexpr uint8_t MAGIC = 0x53;

// Bring up WiFi STA + ESP-NOW on ESPNOW_CHANNEL and register the RX callback.
// Returns false if ESP-NOW init fails. No-op if ESPNOW_ENABLE is 0.
bool begin();

// Latest values. `kill`/`aux_v` reflect the most recent packet; `fresh` is false
// once the link has been silent for ESPNOW_STALE_MS (then kill is forced false).
void poll(bool& kill, float& aux_v, bool& fresh);

}  // namespace espnow_link
