// =============================================================================
//  drivers/buzzer — implementation (LEDC square-wave tone)
// =============================================================================

#include "drivers/buzzer.h"
#include "config.h"
#include "comms/params.h"

namespace buzzer {

static const uint8_t LEDC_CH = 0;

struct Step { uint16_t freq; uint16_t dur_ms; };   // freq 0 = silent gap
static const int MAX_STEPS = 24;                   // room for a short melody

// Note frequencies (upper octaves — a piezo is loudest ~2.5-3.2 kHz, so the melody
// sits high to stay audible). Hz.
enum {
    N_C6=1047, N_D6=1175, N_E6=1319, N_F6=1397, N_G6=1568, N_A6=1760, N_B6=1976,
    N_C7=2093, N_D7=2349, N_E7=2637, N_F7=2794, N_G7=3136, N_A7=3520, N_AS6=1865
};

static Step     s_seq[MAX_STEPS];
static int      s_len = 0;
static int      s_idx = -1;         // -1 = idle
static uint32_t s_step_start = 0;
static uint8_t  s_duty8 = 76;       // 8-bit drive duty (set from BUZZ_DUTY per play())

void begin() {
    ledcSetup(LEDC_CH, 2000, 8);
    ledcAttachPin(params::pinOr(g_params.pin_buzzer, PIN_BUZZER, true), LEDC_CH);
    ledcWrite(LEDC_CH, 0);          // idle LOW
}

// A small passive buzzer is LOUDEST near its ~2.7 kHz resonance; driving it at
// 500-2000 Hz (as before) is much quieter. Keep tones in the loud 2.5-3.2 kHz band —
// still distinguishable by pattern/pitch. (A bare 3.3 V GPIO is inherently limited;
// for more volume add a transistor driver or use an active buzzer.)
static void loadPattern(BuzzTone t) {
    s_len = 0;
    auto add = [&](uint16_t f, uint16_t d) { if (s_len < MAX_STEPS) s_seq[s_len++] = { f, d }; };
    switch (t) {
        case BuzzTone::ARM:        add(2500, 90); add(3100, 140); break;         // rising
        case BuzzTone::DISARM:     add(3100, 90); add(2500, 140); break;         // falling
        case BuzzTone::ALERT:      add(2900, 90); add(0, 60); add(2900, 90); break;
        case BuzzTone::ERROR:      add(2700, 450); break;                        // long
        case BuzzTone::CALIB_STEP: add(3000, 70); break;

        // Power-up: a compact Rick & Morty theme riff, transcribed into the audible
        // upper octaves (recognizable phrase, not a full arrangement).
        case BuzzTone::STARTUP:
            add(N_G6,120); add(N_C7,120); add(N_E7,120); add(N_G7,180);
            add(N_E7,120); add(N_C7,120); add(N_D7,120); add(N_E7,220);
            add(0,60);
            add(N_F7,140); add(N_E7,140); add(N_D7,140); add(N_C7,260);
            break;

        // "Ocean-feel" event tones: gentle stepped sweeps/warbles in the loud band —
        // organic-feeling but clearly audible (a square-wave piezo can't do true low
        // ocean rumble, so we sweep pitch instead).
        case BuzzTone::PICO_LOST:                                                // down-sweep
            add(3100,90); add(2800,90); add(2500,90); add(2200,140); break;
        case BuzzTone::PICO_OK:                                                  // up-chirp
            add(2400,80); add(2800,80); add(3200,120); break;
        case BuzzTone::GCS_LOST:                                                 // two-tone warble
            add(2600,110); add(2200,110); add(2600,110); add(2200,150); break;
        case BuzzTone::LEAK:                                                     // urgent wave triple
            add(3200,110); add(2600,70); add(3200,110); add(2600,70);
            add(3200,110); add(2600,70); add(3200,180); break;

        case BuzzTone::CAL_START:                                                // rising two-note
            add(N_C7,110); add(N_G7,160); break;
        case BuzzTone::CAL_DONE:                                                 // success arpeggio
            add(N_C7,90); add(N_E7,90); add(N_G7,90); add(N_C7,180); break;

        case BuzzTone::NONE:
        default: break;
    }
}

static void applyStep() {
    if (s_idx < 0 || s_idx >= s_len) { ledcWrite(LEDC_CH, 0); return; }
    uint16_t f = s_seq[s_idx].freq;
    if (f == 0) { ledcWrite(LEDC_CH, 0); return; }   // silent gap
    // Drive at the configured (low) duty rather than ledcWriteTone's fixed 50%: less average
    // current → the buzzer can't brown the 3.3 V rail into a rst:poweron reboot.
    ledcSetup(LEDC_CH, f, 8);
    ledcWrite(LEDC_CH, s_duty8);
}

void play(BuzzTone t) {
    if (t == BuzzTone::NONE) return;
    loadPattern(t);
    if (s_len == 0) return;
    float pct = g_params.buzz_duty;
    if (pct < 1)  pct = 1;
    if (pct > 100) pct = 100;
    s_duty8 = (uint8_t)(pct * 255.0f / 100.0f + 0.5f);   // 8-bit duty from BUZZ_DUTY %
    s_idx = 0;
    s_step_start = millis();
    applyStep();
}

void update(uint32_t now) {
    if (s_idx < 0) return;
    if (now - s_step_start >= s_seq[s_idx].dur_ms) {
        s_idx++;
        s_step_start = now;
        if (s_idx >= s_len) {
            s_idx = -1;
            ledcWrite(LEDC_CH, 0);   // done — idle LOW
        } else {
            applyStep();
        }
    }
}

}  // namespace buzzer
