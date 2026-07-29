// =============================================================================
//  ui_log — implementation (static buffer guarded by a portMUX spinlock)
// =============================================================================

#include "comms/ui_log.h"
#include <Arduino.h>
#include <string.h>

namespace ui_log {

static const size_t CAP = 48;
static char s_msg[CAP] = "";
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void set(const char* msg) {
    if (!msg) return;
    portENTER_CRITICAL(&s_mux);
    strncpy(s_msg, msg, CAP - 1);
    s_msg[CAP - 1] = '\0';
    portEXIT_CRITICAL(&s_mux);
}

void get(char* out, size_t n) {
    if (!out || n == 0) return;
    portENTER_CRITICAL(&s_mux);
    strncpy(out, s_msg, n - 1);
    out[n - 1] = '\0';
    portEXIT_CRITICAL(&s_mux);
}

}  // namespace ui_log
