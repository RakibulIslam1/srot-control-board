#pragma once

// =============================================================================
//  SROT — FreeRTOS task entry points
//
//  Core 0 (comms/display/fs):  Task_MAVLink, Task_LoRa_SD, Task_UI_Status
//  Core 1 (real-time flight):  Task_SensorRead, Task_ControlLoop, Task_DShot_RMT
//
//  All task bodies are stubs in Phase 0 — they create cleanly, pin to the right
//  core, run at their configured rate, and touch g_state through its mutexes so
//  the locking contract is exercised from day one. Logic is filled per plan.md.
// =============================================================================

#include <Arduino.h>

void Task_MAVLink(void* pv);      // Core 0 — MAVLink telemetry/params/commands
void Task_LoRa_SD(void* pv);      // Core 0 — LoRa mission RX + SD logging init
void Task_UI_Status(void* pv);    // Core 0 — OLED, WS2812B, PCA9685 aux
void Task_Buzzer(void* pv);       // Core 0 — buzzer melodies/alerts @ 200 Hz
void Task_SensorRead(void* pv);   // Core 1 — BNO085 + Bar30 sampling
void Task_ControlLoop(void* pv);  // Core 1 — PID cascade, mixer, stunt/cal SM
void Task_DShot_RMT(void* pv);    // Core 1 — 8-channel DShot150 output
