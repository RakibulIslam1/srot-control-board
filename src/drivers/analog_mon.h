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

// Read battery voltage, current (× curr_mult) and — when leak_enable — the leak pin, each
// averaged over several samples.
//
// `volt_mult` (PM1_VMULT) is the resistor-divider RATIO: battery volts per volt at the pin
// (e.g. ~11 for a 16.8 V pack into a 3.3 V input). It used to be volts-per-ADC-count (~0.009);
// the reading now comes from analogReadMilliVolts(), which applies the chip's factory ADC
// calibration and removes the 11 dB attenuation non-linearity that no single multiplier could
// compensate. A value below ~0.5 is treated as a stale pre-change setting and replaced by the
// default — see voltMultLooksStale().
void read(Sample& out, float volt_mult, float curr_mult, bool leak_enable);

// True when PM1_VMULT looks like a pre-calibration-change value (volts-per-count rather than
// a divider ratio). The caller warns the operator; read() substitutes the default so the
// displayed voltage is merely uncalibrated instead of wrong by ~1000x.
bool voltMultLooksStale(float volt_mult);

}  // namespace analog_mon
