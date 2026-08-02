#pragma once

// =============================================================================
//  drivers/bar30 — BlueRobotics Bar30 (MS5837-30BA) depth/pressure on I2C Bus0
//  Uses the VENDORED MS5837 driver in lib/MS5837 (see its README — upstream's reset()
//  skipped the PROM-reload delay and the PROM CRC, so coefficients were boot-time luck).
// =============================================================================

#include <Arduino.h>

namespace bar30 {

struct Sample {
    float pressure_mbar = 0;
    float temp_c = 0;
    float depth_m = 0;    // positive down, relative to the surface reference
};

// Init the MS5837 in 30BA (Bar30) mode on Wire. Returns false if not connected.
bool begin();

// Read one sample. `surface_mbar` is the pressure reference for zero depth
// (from calibration / CalState.baro_zero_mbar); pass 1013.25 if uncalibrated.
// Returns false on read error.
bool read(Sample& out, float surface_mbar);

// LIVE health, not a boot-time constant: false when begin() failed (including a PROM CRC
// mismatch) and after consecutive read failures or implausible samples. This is what backs
// `SensorState.baro_valid`, which gates DEPTH_HOLD / AUTO / PATTERN — so it going false
// REFUSES those modes. That is deliberate: fabricated depth on a loop that has never run
// closed is the worse failure.
bool healthy();

// Raw calibration PROM word i (0..6), unscaled, for diagnostics. Log these before concluding
// a sensor is bad — it distinguishes a genuinely corrupt PROM from a CRC implementation bug.
uint16_t promWord(uint8_t i);

}  // namespace bar30
