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

static float readAvg(uint8_t pin) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < OVERSAMPLE; ++i) sum += analogRead(pin);
    return (float)sum / OVERSAMPLE;
}

void read(Sample& out, float volt_mult, float curr_mult, bool leak_enable) {
    out.volt = readAvg(s_volt_pin) * volt_mult;
    out.curr = readAvg(s_curr_pin) * curr_mult;
    out.leak = leak_enable && (digitalRead(s_leak_pin) == (LEAK_ACTIVE_HIGH ? HIGH : LOW));
}

}  // namespace analog_mon
