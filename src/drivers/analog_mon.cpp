// =============================================================================
//  drivers/analog_mon — implementation
// =============================================================================

#include "drivers/analog_mon.h"
#include "config.h"
#include "comms/params.h"

namespace analog_mon {

// 4× oversample: battery/leak don't need 16, and each analogRead (~100 µs) busy-waits
// on the highest-priority Core-1 sensor task — 4 keeps the noise floor low while
// cutting that blocking window 4× so it can't starve the 500 Hz control loop.
static const uint8_t OVERSAMPLE = 4;
static uint8_t s_volt_pin, s_curr_pin, s_leak_pin;

void begin() {
    s_volt_pin = (uint8_t)params::pinOr(g_params.pin_battvolt, PIN_BATT_VOLT);
    s_curr_pin = (uint8_t)params::pinOr(g_params.pin_battcurr, PIN_BATT_CURR);
    s_leak_pin = (uint8_t)params::pinOr(g_params.pin_leak, PIN_LEAK);
    analogSetPinAttenuation(s_volt_pin, ADC_11db);
    analogSetPinAttenuation(s_curr_pin, ADC_11db);
    pinMode(s_leak_pin, INPUT);   // input-only, no internal pull — external bias needed
}

// Average CALIBRATED millivolts at the pin.
//
// analogReadMilliVolts() applies this chip's factory eFuse ADC calibration curve; raw
// analogRead() counts do not. At 11 dB attenuation the ESP32's ADC is markedly non-linear —
// worst above ~2.5 V and below ~0.15 V — so no single multiplier can fit it, and the error
// after a battery divider is easily several hundred millivolts. Scaling calibrated millivolts
// by the divider ratio fixes the cause instead of tuning around it.
static float readAvgMv(uint8_t pin) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < OVERSAMPLE; ++i) sum += analogReadMilliVolts(pin);
    return (float)sum / OVERSAMPLE;
}

// PM1_VMULT changed meaning with the switch above: it is now the resistor-divider RATIO
// (battery volts per volt at the pin, e.g. ~11 for a 16.8 V pack into a 3.3 V input), not
// volts-per-ADC-count (~0.009). An imported .params file written before that change carries
// the old value, and silently applying ~0.009 as a ratio would report a flat battery on a
// full pack. Anything below this is therefore treated as "not yet configured": fall back to
// the default and say so, rather than display a number that is wrong by three orders of
// magnitude.
static const float MIN_PLAUSIBLE_RATIO = 0.5f;

bool voltMultLooksStale(float volt_mult) { return !(volt_mult >= MIN_PLAUSIBLE_RATIO); }

void read(Sample& out, float volt_mult, float curr_mult, bool leak_enable) {
    const float ratio = voltMultLooksStale(volt_mult) ? DEF_PM1_VMULT : volt_mult;
    out.volt = (readAvgMv(s_volt_pin) / 1000.0f) * ratio;
    // Current is left on raw counts: its multiplier is sensor-specific and uncalibrated
    // either way, and CURR is display-only (no failsafe consumes it).
    uint32_t csum = 0;
    for (uint8_t i = 0; i < OVERSAMPLE; ++i) csum += analogRead(s_curr_pin);
    out.curr = ((float)csum / OVERSAMPLE) * curr_mult;
    out.leak = leak_enable && (digitalRead(s_leak_pin) == (LEAK_ACTIVE_HIGH ? HIGH : LOW));
}

}  // namespace analog_mon
