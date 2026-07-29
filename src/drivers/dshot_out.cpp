// =============================================================================
//  drivers/dshot_out — implementation (legacy RMT, DShot150)
//
//  Timing @ clk_div=1 (80 MHz APB → 12.5 ns/tick), DShot150 bit = 6.67 µs:
//    bit "1": 5.00 µs high (400 ticks) + 1.67 µs low (133)
//    bit "0": 2.50 µs high (200 ticks) + 4.17 µs low (333)
//  16-bit frame MSB-first: [11-bit throttle][1 telemetry][4-bit CRC].
// =============================================================================

#include "drivers/dshot_out.h"
#include "driver/rmt.h"
#include "comms/params.h"

namespace dshot_out {

static const uint16_t T1H = 400, T1L = 133;
static const uint16_t T0H = 200, T0L = 333;

// Fixed compile-time thruster pins (RMT fallback backend only).
static const uint8_t  s_pins[NUM_THRUSTERS] = THRUSTER_PINS;
static rmt_item32_t   s_items[NUM_THRUSTERS][16];
static int16_t        s_last[NUM_THRUSTERS] = {0};   // last value → skip rebuild if unchanged
static bool           s_ok = false;

bool begin() {
    s_ok = true;
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX((gpio_num_t)s_pins[i], (rmt_channel_t)i);
        cfg.clk_div = 1;   // 12.5 ns/tick
        if (rmt_config(&cfg) != ESP_OK) { s_ok = false; continue; }
        if (rmt_driver_install(cfg.channel, 0, 0) != ESP_OK) { s_ok = false; }
    }
    return s_ok;
}

// Build the 16 RMT items for one channel from a throttle value (0..2047).
static void buildFrame(rmt_item32_t* items, uint16_t value) {
    if (value > 2047) value = 2047;
    uint16_t frame = (uint16_t)(value << 1);          // telemetry bit = 0
    uint16_t crc = (frame ^ (frame >> 4) ^ (frame >> 8)) & 0x0F;
    uint16_t packet = (uint16_t)((frame << 4) | crc); // 16 bits

    for (int b = 0; b < 16; ++b) {
        bool one = (packet >> (15 - b)) & 0x1;
        items[b].level0 = 1;
        items[b].duration0 = one ? T1H : T0H;
        items[b].level1 = 0;
        items[b].duration1 = one ? T1L : T0L;
    }
}

void write(const int16_t throttle[NUM_THRUSTERS]) {
    if (!s_ok) return;
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        int16_t v = throttle[i];
        if (v < 0) v = 0;
        // Rebuild the 16-item frame + CRC only when the value changed; the ESC needs
        // a continuous signal, so we always re-emit the (cached) items.
        if (v != s_last[i]) { buildFrame(s_items[i], (uint16_t)v); s_last[i] = v; }
        rmt_write_items((rmt_channel_t)i, s_items[i], 16, false /*non-blocking*/);
    }
}

}  // namespace dshot_out
