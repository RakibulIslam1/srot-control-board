#pragma once

// =============================================================================
//  comms/flare_order — an ordered sequence sent to the vehicle during a run
//
//  SAUVC Task 4 tells the TEAM the flare order after Navigation; the team has to get it
//  to the vehicle without surfacing. The ground station already relays operator MAVLink
//  over LoRa into mav_commands::handle(), so the order rides that path as ONE
//  COMMAND_LONG and needs no new radio profile, frame or message.
//
//  Wire: MAV_CMD_SROT_FLARE_ORDER (COMMAND_LONG)
//    param1..param4  colour code per position, 1..9; 0 ends the sequence
//    param7          nonce, 0..65535, chosen by the sender (echoed so the host can tell
//                    one transmission from a repeat of an older one)
//  The colour-code table is the host's (srot_protocol.FLARE_COLOURS); the board only
//  stores digits, so a new colour needs no reflash.
//
//  Latched in RAM ONLY, never NVS. A persisted order would survive a reboot and stand in,
//  confidently, for the one the judges read out at the next attempt. Absence must read as
//  absence: until an order arrives nothing named FLARE_* is published.
//
//  Reported on NAMED_VALUE_FLOAT: FLARE_ORD (packed digits, position 1 most significant,
//  so red-blue-yellow with codes 1,2,3 is 123), FLARE_NON (nonce), FLARE_AGE (seconds since
//  the latch). The host compares FLARE_AGE with its own mission start, so a practice order
//  still in RAM from an earlier attempt is refused without any "clear" command.
// =============================================================================

#include <cstdint>
#include <cmath>

#define MAV_CMD_SROT_FLARE_ORDER 31020

namespace flare_order {

constexpr int MAX_SLOTS = 4;

// Pure: validate params and pack them into decimal digits. Returns 0 for anything that is
// not a clean sequence: a fractional or out-of-range code, a gap (a code after a 0), a
// repeated colour (an operator typo, never a real order), no codes at all, or a nonce that
// is not an integer in 0..65535. Non-finite values are refused before dispatch.
inline uint32_t pack(const float p[7]) {
    uint32_t digits = 0;
    bool ended = false;
    bool seen[10] = {false};
    int n = 0;
    for (int i = 0; i < MAX_SLOTS; ++i) {
        const float v = p[i];
        if (v != std::floor(v) || v < 0.0f || v > 9.0f) return 0;
        const int c = (int)v;
        if (c == 0) { ended = true; continue; }
        if (ended || seen[c]) return 0;
        seen[c] = true;
        digits = digits * 10 + (uint32_t)c;
        ++n;
    }
    if (n == 0) return 0;
    if (p[6] != std::floor(p[6]) || p[6] < 0.0f || p[6] > 65535.0f) return 0;
    return digits;
}

// Store a validated order. Safe from any task: the LoRa task and Task_MAVLink both dispatch.
void latch(uint32_t digits, uint16_t nonce, uint32_t now_ms);

// Latest order, or false if none has arrived since boot.
bool get(uint32_t& digits, uint16_t& nonce, uint32_t& stamp_ms);

}  // namespace flare_order
