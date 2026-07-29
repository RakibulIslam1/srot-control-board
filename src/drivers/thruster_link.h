#pragma once
// =============================================================================
//  drivers/thruster_link — ESP32 side of the Pico 2 thruster co-processor link
//
//  Sends the 8 DShot values the control loop already computes to the Pico over
//  UART2 (request), and parses the Pico's telemetry reply (per-thruster RPM +
//  status). Framing/CRC per shared/thruster_link_proto.h. Non-blocking: send is
//  a fixed-size UART write; telemetry is drained each cycle.
// =============================================================================

#include <Arduino.h>
#include "config.h"

namespace thruster_link {

// Open UART2 to the Pico + the e-stop line. Call once from the link task.
void begin();

// Send one command frame. rpm_mode=false: cmd[] = DShot values (0 | 48..2047, open
// loop). rpm_mode=true: cmd[] = signed target RPM (the Pico closes the per-motor loop).
// beep=true requests a one-shot ESC beacon chirp (Pico plays it only while disarmed).
void send(const int16_t cmd[NUM_THRUSTERS], bool armed, bool estop, bool rpm_mode,
          bool beep = false);

// Push the Pico's RPM control config (resent periodically so it survives a Pico reset).
// loop_mode:   0 = open-loop (feedforward), 1 = closed-loop RPM (PI).
// dshot_bidir: 1 = bidirectional DShot (RPM telemetry), 0 = normal DShot (any ESC, no RPM).
void sendConfig(float kp, float ki, float ff_a, float idle_rpm, float max_rpm,
                float filt, float slew, uint8_t loop_mode, uint8_t dshot_bidir,
                float spin_min, float min_target_rpm);

// Drain + parse any telemetry frames. Fills rpm[8]/status[8]/fault; returns true
// if a fresh, CRC-valid frame arrived this call (updates the link-alive clock).
bool poll(int16_t rpm[NUM_THRUSTERS], uint8_t status[NUM_THRUSTERS], uint8_t& fault);

// True while telemetry has been fresh within PICO_LINK_TIMEOUT_MS.
bool linkOk();

// Link-drop diagnostics: bytesRecent() true = the Pico is still sending (so a drop = NOISE/CRC);
// false = SILENT (Pico stopped/reset). resyncRx() flushes stale RX + resets framing after a gap.
// flapCount() increments and returns the running link-drop count.
bool     bytesRecent();
void     resyncRx();
uint32_t flapCount();
uint32_t picoUptime();   // last Pico uptime_ms (detect a Pico reset = value jumps backward)

}  // namespace thruster_link
