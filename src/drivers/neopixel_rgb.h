#pragma once

// =============================================================================
//  drivers/neopixel_rgb — single WS2812B status LED, TRUE bit-bang (no RMT)
//
//  All 8 RMT channels are used by DShot, and Adafruit_NeoPixel drives the ESP32
//  via RMT — so it cannot be used here. This is a cycle-timed bit-bang for one
//  LED (24 bits, ~30 µs with interrupts briefly masked), which is safe at the
//  low UI refresh rate.
// =============================================================================

#include <Arduino.h>
#include "state_types.h"

namespace rgb {

void begin();

// Pick a colour/animation from the mode and push it at the given brightness.
void update(RgbMode mode, uint8_t brightness, uint32_t now);

}  // namespace rgb
