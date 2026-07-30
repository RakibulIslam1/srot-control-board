#pragma once
// =============================================================================
//  ESP-NOW link: 2nd board (thruster control board)  →  SROT control board
//
//  ONE definition compiled by BOTH sides — src/second_board (sender) and
//  src/drivers/espnow_link (receiver). It used to be hand-copied into a comment on
//  the receiver, which is exactly how two boards end up disagreeing about a wire
//  format that has no version field.
//
//  Broadcast, unacknowledged, ~4 Hz. The receiver registers a plain RX callback
//  with no peer, so the sender broadcasts to FF:FF:FF:FF:FF:FF. Both ends must be
//  on ESPNOW_CHANNEL with WiFi power-save OFF.
//
//  Why it exists: the thruster pack and the electronics pack are DIFFERENT
//  batteries. The SROT board's own ADC measures the electronics/SBC pack, so the
//  thruster-pack voltage can only come from the board that is physically wired to
//  it. Two features on the control board consume it:
//    * mixer::setBatteryVoltage() — thrust linearisation, so a timed move travels
//      the same distance on a full or a flat pack (needs MOT_BAT_V_MAX set).
//    * the low-thruster-battery failsafe (FS_BAT_*).
//  Both are inert until these packets arrive.
//
//  CHANGING THIS STRUCT means reflashing BOTH boards. There is no version byte and
//  no CRC — a size mismatch is rejected by the receiver's length check, but a
//  same-size field reshuffle would be silently misread. Add fields at the END and
//  bump nothing else, or accept the dual reflash.
// =============================================================================

#include <stdint.h>

// 'S' — sanity byte, so a stray ESP-NOW frame from unrelated hardware on the same
// channel cannot be mistaken for telemetry.
#define ESPNOW_MSG_MAGIC 0x53

// Send/expect interval. The receiver treats data older than ESPNOW_STALE_MS (2000)
// as absent, so this must stay comfortably faster than that; 4 Hz gives 8x margin.
// The control board low-passes the voltage at ~0.5 Hz anyway, so a faster rate buys
// nothing and only adds airtime.
#define ESPNOW_TX_HZ 4

#pragma pack(push, 1)
typedef struct {
    uint8_t magic;          // ESPNOW_MSG_MAGIC
    uint8_t thruster_kill;  // 1 = thruster power is CUT, 0 = live (see the note below)
    float   aux_voltage;    // THRUSTER pack voltage, volts
} espnow_from_second_t;     // 6 bytes
#pragma pack(pop)

// thruster_kill polarity: 1 means "thrusters are killed / power removed". The
// control board treats this as INDICATION ONLY (OLED, the KILL telemetry value and a
// LoRa flag) — it does not gate thrust — so an inverted sense misreports but cannot
// cause or prevent a stop. Verify it on the bench before trusting the indicator.

#ifdef __cplusplus
static_assert(sizeof(espnow_from_second_t) == 6, "espnow packet must stay 6 bytes");
#endif
