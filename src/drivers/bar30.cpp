// =============================================================================
//  drivers/bar30 — implementation (robtillaart/MS5837)
// =============================================================================

#include "drivers/bar30.h"
#include <Wire.h>
#include <MS5837.h>
#include "config.h"

namespace bar30 {

static MS5837 s_ms(&Wire);
static bool s_ok = false;

bool begin() {
    // MS5837_TYPE_30 = Bar30 (30 bar) variant.
    s_ok = s_ms.begin(MS5837_TYPE_30);
    if (s_ok) {
        // Fresh water ~0.99802; salt water ~1.029 (kg/L). Adjust for the pool.
        s_ms.setDensity(0.99802f);
    }
    return s_ok;
}

bool read(Sample& out, float surface_mbar) {
    if (!s_ok) return false;
    // read() triggers a conversion + reads pressure/temperature.
    if (s_ms.read() != 0) {
        return false;
    }
    out.pressure_mbar = s_ms.getPressure();
    out.temp_c        = s_ms.getTemperature();
    // getDepth(airPressure) zeroes at the given surface reference pressure.
    out.depth_m       = s_ms.getDepth(surface_mbar);
    return true;
}

bool healthy() { return s_ok; }

}  // namespace bar30
