// =============================================================================
//  comms/flare_order — RAM latch (see flare_order.h)
// =============================================================================

#include "comms/flare_order.h"
#include <Arduino.h>

namespace flare_order {

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool     s_have = false;
static uint32_t s_digits = 0;
static uint16_t s_nonce = 0;
static uint32_t s_stamp_ms = 0;

void latch(uint32_t digits, uint16_t nonce, uint32_t now_ms) {
    portENTER_CRITICAL(&s_mux);
    s_digits = digits;
    s_nonce = nonce;
    s_stamp_ms = now_ms;
    s_have = true;
    portEXIT_CRITICAL(&s_mux);
}

bool get(uint32_t& digits, uint16_t& nonce, uint32_t& stamp_ms) {
    portENTER_CRITICAL(&s_mux);
    const bool have = s_have;
    digits = s_digits;
    nonce = s_nonce;
    stamp_ms = s_stamp_ms;
    portEXIT_CRITICAL(&s_mux);
    return have;
}

}  // namespace flare_order
