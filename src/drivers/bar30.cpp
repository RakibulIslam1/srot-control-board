// =============================================================================
//  drivers/bar30 — implementation (MS5837, VENDORED in lib/MS5837)
//
//  The driver is vendored, not a lib_deps entry, because upstream's reset() never
//  waited for the sensor's PROM reload and never checked the PROM CRC. See
//  lib/MS5837/README — that bug is the whole reason this file grew a health model.
// =============================================================================

#include "drivers/bar30.h"
#include <Wire.h>
#include <MS5837.h>
#include <math.h>
#include "config.h"

namespace bar30 {

static MS5837 s_ms(&Wire);
static bool s_ok = false;

// Consecutive failed reads. `healthy()` used to be a boot-time constant — s_ok was set once
// in begin() and never cleared — which made `s.baro_valid` decorative: a sensor that stopped
// answering mid-flight still reported itself healthy for ever, and the depth guards that
// consume that flag never fired.
static uint8_t  s_fail_n = 0;
static uint32_t s_last_retry_ms = 0;

// A sensor that has failed this many reads in a row is not a sensor any more.
static const uint8_t  FAIL_UNHEALTHY = 3;
// After this many, try to bring it back — an I2C glitch or a brown-out should not need a
// reboot to recover from. Bounded by RETRY_GAP_MS so a genuinely absent sensor cannot turn
// the sensor task into a busy re-probe loop (same shape as lora_mission::poll()'s re-probe).
static const uint8_t  FAIL_RETRY     = 10;
static const uint32_t RETRY_GAP_MS   = 2000;

// Plausibility bands. DELIBERATELY WIDE: these gate `baro_valid`, and `baro_valid` gates
// DEPTH_HOLD / AUTO / PATTERN — so a band that is too tight does not degrade the reading,
// it GROUNDS THE VEHICLE. Do not narrow these without a venue survey.
//
// They are a backstop, not the primary defence. The -160 C case fails the temperature band,
// but the +10 C case is perfectly plausible and only the PROM CRC in begin() catches it.
static const float TEMP_MIN_C   = -5.0f;
static const float TEMP_MAX_C   = 60.0f;
static const float PRESS_MIN_MB = 300.0f;      // ~9000 m altitude, far below any pool
static const float PRESS_MAX_MB = 40000.0f;    // ~300 m depth, past the Bar30's range

static bool tryBegin() {
    // MS5837_TYPE_30 = Bar30 (30 bar) variant.
    // begin() -> reset() now enforces the post-RESET PROM delay AND validates the PROM CRC,
    // so a corrupt coefficient set fails here instead of being flown on.
    if (!s_ms.begin(MS5837_TYPE_30)) return false;
    // Fresh water ~0.99802; salt water ~1.029 (kg/L). Adjust for the pool.
    s_ms.setDensity(0.99802f);
    return true;
}

bool begin() {
    // Retry, because the failure this guards against is a RACE and a race can be lost twice.
    // Three attempts with a gap: if the PROM read is being disturbed by something transient
    // (bus contention at boot, a slow power rail), the next attempt usually wins. If the PROM
    // is genuinely corrupt or the sensor is absent, all three fail and we report absent —
    // which is the honest answer and the one that keeps DEPTH_HOLD refused.
    s_ok = false;
    for (uint8_t attempt = 0; attempt < 3 && !s_ok; ++attempt) {
        if (attempt) delay(20);
        s_ok = tryBegin();
    }
    s_fail_n = 0;
    s_last_retry_ms = millis();
    return s_ok;
}

bool read(Sample& out, float surface_mbar) {
    const uint32_t now = millis();

    if (!s_ok) {
        // Bounded recovery attempt for a sensor that failed at boot or dropped off later.
        if (now - s_last_retry_ms >= RETRY_GAP_MS) {
            s_last_retry_ms = now;
            if (tryBegin()) { s_ok = true; s_fail_n = 0; }
        }
        if (!s_ok) return false;
    }

    // read() triggers a conversion + reads pressure/temperature.
    // Library default OSR (bits=8). A higher OSR was tried on the bench and made the read
    // FAIL outright -- no SCALED_PRESSURE2 at all -- so it is not the knob it looked like.
    // The 608 -> 744 mbar excursion that prompted it was the mag-report disable colliding
    // with a Bar30 conversion on the shared I2C0 bus, not conversion-time margin; that code
    // is backed out (see task_sensor_read) and pressure is stable at the default again.
    if (s_ms.read() != 0) {
        if (s_fail_n < 255) ++s_fail_n;
        if (s_fail_n >= FAIL_RETRY && now - s_last_retry_ms >= RETRY_GAP_MS) {
            s_last_retry_ms = now;
            s_ok = tryBegin();
            if (s_ok) s_fail_n = 0;
        }
        return false;
    }

    const float press = s_ms.getPressure();
    const float temp  = s_ms.getTemperature();
    // getDepth(airPressure) zeroes at the given surface reference pressure.
    const float depth = s_ms.getDepth(surface_mbar);

    // Reject the whole sample if ANY field is implausible — they share one dT, so a bad
    // temperature means the pressure and depth from the same conversion are equally suspect.
    // Publishing depth while suppressing only the temperature would hide the corruption in
    // the one signal that actually steers the vehicle.
    if (!isfinite(press) || !isfinite(temp) || !isfinite(depth) ||
        temp  < TEMP_MIN_C   || temp  > TEMP_MAX_C ||
        press < PRESS_MIN_MB || press > PRESS_MAX_MB) {
        if (s_fail_n < 255) ++s_fail_n;
        return false;
    }

    out.pressure_mbar = press;
    out.temp_c        = temp;
    out.depth_m       = depth;
    s_fail_n = 0;
    return true;
}

bool healthy() { return s_ok && s_fail_n < FAIL_UNHEALTHY; }

uint16_t promWord(uint8_t i) { return s_ms.promRaw(i); }

}  // namespace bar30
