#pragma once

// =============================================================================
//  drivers/dshot_out — 8-channel DShot150 on the legacy ESP32 RMT peripheral
//
//  One RMT channel (0..7) per thruster on THRUSTER_PINS[]. All 8 channels are
//  consumed here, which is why the WS2812B is bit-banged elsewhere.
//  Implemented on driver/rmt.h (arduino-esp32 2.0.17 / IDF 4.4).
// =============================================================================

#include <Arduino.h>
#include "config.h"

namespace dshot_out {

// Configure and install all 8 RMT channels. Returns false on any channel error.
bool begin();

// Emit one DShot frame per channel from the given throttle values
// (0 = disarmed, else 48..2047). Non-blocking.
void write(const int16_t throttle[NUM_THRUSTERS]);

}  // namespace dshot_out
