// =============================================================================
//  drivers/neopixel_rgb — implementation (cycle-timed bit-bang WS2812B)
// =============================================================================

#include "drivers/neopixel_rgb.h"
#include "config.h"
#include "comms/params.h"

namespace rgb {

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_t0h, s_t1h, s_t0l, s_t1l;   // cycle counts
static uint8_t  s_pin = PIN_WS2812B;
static uint32_t s_pinmask = 0;
static bool s_ready = false;

static inline uint32_t ccount() { return xthal_get_ccount(); }
static inline void waitCycles(uint32_t start, uint32_t n) {
    while ((ccount() - start) < n) { /* spin */ }
}

void begin() {
    // The bit-bang writes GPIO.out_w1ts/w1tc, which only cover GPIO0..31 — so a
    // configured pin must be <32, else fall back to the default.
    int p = params::pinOr(g_params.pin_rgb, PIN_WS2812B, true);
    s_pin = (p < 32) ? (uint8_t)p : PIN_WS2812B;
    s_pinmask = (1UL << s_pin);
    pinMode(s_pin, OUTPUT);
    digitalWrite(s_pin, LOW);   // stays dark
    uint32_t mhz = getCpuFrequencyMhz();
    // WS2812B 800 kHz: bit = 1.25 µs; '0' high 0.4 µs, '1' high 0.8 µs.
    s_t0h = (uint32_t)(0.40f * mhz);
    s_t1h = (uint32_t)(0.80f * mhz);
    s_t0l = (uint32_t)(0.85f * mhz);
    s_t1l = (uint32_t)(0.45f * mhz);
    s_ready = true;
}

// Push one GRB pixel. Interrupts masked for the ~30 µs shift.
static void show(uint8_t r, uint8_t g, uint8_t b) {
    if (!s_ready) return;
    uint8_t bytes[3] = { g, r, b };   // WS2812B order = GRB
    const uint32_t pinmask = s_pinmask;

    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < 3; ++i) {
        uint8_t val = bytes[i];
        for (int bit = 7; bit >= 0; --bit) {
            uint32_t start = ccount();
            GPIO.out_w1ts = pinmask;                 // high
            if (val & (1 << bit)) {
                waitCycles(start, s_t1h);
                start = ccount();
                GPIO.out_w1tc = pinmask;             // low
                waitCycles(start, s_t1l);
            } else {
                waitCycles(start, s_t0h);
                start = ccount();
                GPIO.out_w1tc = pinmask;
                waitCycles(start, s_t0l);
            }
        }
    }
    portEXIT_CRITICAL(&s_mux);
    // Latch: hold low >50 µs (caller's cadence guarantees this).
}

static void scale(uint8_t& r, uint8_t& g, uint8_t& b, uint8_t bright) {
    r = (uint16_t)r * bright / 255;
    g = (uint16_t)g * bright / 255;
    b = (uint16_t)b * bright / 255;
}

void update(RgbMode mode, uint8_t brightness, uint32_t now) {
    uint8_t r = 0, g = 0, b = 0;
    bool blink_slow = ((now / 500) & 1);
    bool blink_fast = ((now / 150) & 1);

    switch (mode) {
        case RgbMode::OFF:        r = g = b = 0; break;
        case RgbMode::BOOT:       b = blink_slow ? 255 : 40; break;         // blue pulse
        case RgbMode::DISARMED:   g = 60; break;                            // dim green
        case RgbMode::ARMED:      g = blink_slow ? 255 : 90; break;         // green breathe
        case RgbMode::DEPTH_HOLD: g = 80; b = 200; break;                   // cyan
        case RgbMode::STUNT:      r = 180; b = 200; break;                  // purple
        case RgbMode::ERROR:      r = blink_fast ? 255 : 0; break;          // red blink
    }
    scale(r, g, b, brightness);
    show(r, g, b);
}

}  // namespace rgb
