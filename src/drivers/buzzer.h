#pragma once

// =============================================================================
//  drivers/buzzer — passive buzzer via LEDC tone generation
//
//  A passive buzzer has no internal oscillator, so it must be driven with a
//  square-wave frequency (the "tone" feature). We use LEDC. GPIO12 is a
//  strapping pin, so the output idles LOW when no tone is playing.
// =============================================================================

#include <Arduino.h>
#include "state_types.h"

namespace buzzer {

void begin();

// Start a (non-blocking) tone pattern. Advance it with update().
void play(BuzzTone tone);

// Advance the active pattern; silences the output when finished.
void update(uint32_t now);

}  // namespace buzzer
