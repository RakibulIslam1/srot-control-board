#pragma once
// =============================================================================
//  ui_log — the latest one-line status message for the OLED ticker.
//
//  A single shared string that any task can set (mirrored from every STATUSTEXT,
//  plus buzzer beep-reasons and the boot reset reason). Task_UI_Status scrolls it
//  under the mode tab. Writes/reads are short critical sections — a torn read is
//  cosmetic at worst.
// =============================================================================

#include <stddef.h>

namespace ui_log {

void set(const char* msg);            // replace the current message (truncates to fit)
void get(char* out, size_t n);        // copy the latest message into out[n]

}  // namespace ui_log
