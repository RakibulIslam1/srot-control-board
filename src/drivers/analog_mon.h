#pragma once

// =============================================================================
//  drivers/analog_mon — battery voltage + current + leak (ADC1, input-only)
//
//  New pinout: one battery-voltage ADC (BATT_VOLT), one current ADC (BATT_CURR),
//  and a digital leak input (LEAK). PM2 (2nd battery) comes from the 2nd board
//  over ESP-NOW, not a local ADC.
// =============================================================================

#include <Arduino.h>

namespace analog_mon {

struct Sample {
    float volt = 0;   // battery voltage (V)
    float curr = 0;   // battery current (A)
    bool  leak = false;
};

void begin();

// Read battery voltage (× volt_mult), current (× curr_mult), and — when
// leak_enable — the leak pin. All averaged over several samples.
void read(Sample& out, float volt_mult, float curr_mult, bool leak_enable);

}  // namespace analog_mon
