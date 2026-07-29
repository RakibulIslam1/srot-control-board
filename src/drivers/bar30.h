#pragma once

// =============================================================================
//  drivers/bar30 — BlueRobotics Bar30 (MS5837-30BA) depth/pressure on I2C Bus0
//  Uses robtillaart/MS5837.
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

bool healthy();

}  // namespace bar30
