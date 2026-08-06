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

// --- JITTER GATE -------------------------------------------------------------------
//
// A PER-SAMPLE BAND IS STRUCTURALLY BLIND TO VARIANCE, and that blindness had teeth.
//
// duburi_ws measured this board on 2026-08-02 (BENCH_FINDINGS_FROM_DUBURI_WS_2026-08-02.md):
// a failing Bar30 produced 317..874 mbar and 6..30 C from sample to sample, on a stationary
// bench. EVERY ONE of those readings sits inside the bands above -- they are deliberately
// wide so a judgement call cannot ground the vehicle -- so SCALED_PRESSURE2 and WTEMP kept
// streaming, the SYS_STATUS health bit stayed SET, and DEPTH_HOLD/AUTO stayed available
// while the derived depth wandered six metres.
//
// That is also the whole of the arming spin-up: phantom depth -> DEPTH_ERR -3..-6.7 m ->
// DEPTH_OUT saturated at -1.0 -> the mixer's -1 throttle column -> all four verticals at
// full, horizontals at idle. Exactly the symptom, with nothing left over.
//
// Peak-to-peak over a short window catches what a band cannot. Real depth cannot change by
// this much this fast on a vehicle that is not moving, and even a genuine fast descent is
// bounded far below it.
//
// The threshold is DELIBERATELY LOOSE (~15 mbar p2p ~= 15 cm of water). Wave action, a
// thruster wash over the port and a real descent all produce real variance; this is aimed at
// the 500+ mbar garbage that is unambiguously a broken sensor, not at tight sensor QA.
//
// The board is the right place for this even though duburi_ws added a host-side check too:
// they can only refuse to ARM, the board can refuse AUTO.
static const uint8_t JITTER_N          = 8;      // samples in the window (~0.4 s at 20 Hz)
static const float   JITTER_MAX_MBAR   = 15.0f;  // peak-to-peak above this = not a sensor
static float   s_hist[JITTER_N] = {0};
static uint8_t s_hist_n = 0, s_hist_i = 0;
static bool    s_jitter_bad = false;

static void jitterReset() { s_hist_n = 0; s_hist_i = 0; s_jitter_bad = false; }

// Feed one accepted pressure sample; returns true when the window looks like noise.
static bool jitterCheck(float press) {
    s_hist[s_hist_i] = press;
    s_hist_i = (uint8_t)((s_hist_i + 1) % JITTER_N);
    if (s_hist_n < JITTER_N) { ++s_hist_n; return false; }   // not enough history yet
    float lo = s_hist[0], hi = s_hist[0];
    for (uint8_t i = 1; i < JITTER_N; ++i) {
        if (s_hist[i] < lo) lo = s_hist[i];
        if (s_hist[i] > hi) hi = s_hist[i];
    }
    s_jitter_bad = (hi - lo) > JITTER_MAX_MBAR;
    return s_jitter_bad;
}

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
    jitterReset();
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

    // The sample is individually plausible. Is the STREAM?
    if (jitterCheck(press)) {
        // Do NOT touch s_fail_n: this is not a failed read, it is a sensor whose readings
        // cannot be trusted as a set. healthy() consults s_jitter_bad separately, so depth
        // withdraws from the stack (SCALED_PRESSURE2/WTEMP suppressed, DEPTH_HOLD/AUTO
        // refused) without the read-failure recovery path fighting it.
        return false;
    }

    out.pressure_mbar = press;
    out.temp_c        = temp;
    out.depth_m       = depth;
    s_fail_n = 0;
    return true;
}

bool healthy() { return s_ok && s_fail_n < FAIL_UNHEALTHY && !s_jitter_bad; }

bool jittery() { return s_jitter_bad; }

float jitterP2P() {
    if (s_hist_n == 0) return 0.0f;
    float lo = s_hist[0], hi = s_hist[0];
    for (uint8_t i = 1; i < s_hist_n; ++i) {
        if (s_hist[i] < lo) lo = s_hist[i];
        if (s_hist[i] > hi) hi = s_hist[i];
    }
    return hi - lo;
}

uint16_t promWord(uint8_t i) { return s_ms.promRaw(i); }

}  // namespace bar30
