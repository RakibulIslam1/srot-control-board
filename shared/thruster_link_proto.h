#pragma once
// =============================================================================
//  thruster_link_proto — ESP32 <-> RP2350 (Pico 2) co-processor wire protocol
//
//  ONE definition shared by BOTH firmwares (ESP32 `src/` and `pico_thruster/`)
//  so the command/telemetry frame format can never drift. Fixed-length,
//  little-endian, header + CRC16 framed over UART @ 2 Mbaud, request/response
//  at the 500 Hz control rate:
//     ESP32 --Command--> Pico    (8 thruster setpoints + flags)
//     Pico  --Telemetry-> ESP32  (8 RPMs + per-ESC status + fault mask)
//
//  Framing is self-synchronizing: scan for the 2-byte magic, validate CRC16,
//  drop and resync on any garbage. `__attribute__((packed))` guarantees the
//  same layout on both the Xtensa (ESP32) and Cortex-M33 (RP2350) toolchains.
// =============================================================================

#include <stdint.h>

#define TL_NUM_THRUSTERS   8

#define TL_MAGIC_CMD_0     0xAA
#define TL_MAGIC_CMD_1     0x55
#define TL_MAGIC_TLM_0     0xAA
#define TL_MAGIC_TLM_1     0x56
#define TL_MAGIC_CFG_0     0xAA
#define TL_MAGIC_CFG_1     0x57

// Command flags
#define TL_FLAG_ARMED      0x01   // thrusters live (else Pico sends DShot 0 = stop)
#define TL_FLAG_MODE_RPM   0x02   // 0 = raw DShot value in cmd[]; 1 = signed target RPM (Stage 2)
#define TL_FLAG_ESTOP      0x04   // hard stop request from the ESP32
#define TL_FLAG_BEEP       0x08   // one-shot: play a beacon chirp on the ESCs (disarmed only)

// Per-ESC status byte (mirrors bidir-DShot extended-telemetry flags where available)
#define TL_ST_RPM_VALID    0x01   // a fresh valid eRPM was decoded this window
#define TL_ST_PRESENT      0x02   // ESC seen recently (any valid telemetry) => connected.
                                  // Clear = no telemetry at all: ESC absent, OR an ESC
                                  // without bidirectional DShot enabled ("not detected").
#define TL_ST_WARNING      0x40
#define TL_ST_ALERT        0x80

// --- ESP32 -> Pico : thruster command -----------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t  magic0;                        // TL_MAGIC_CMD_0
    uint8_t  magic1;                        // TL_MAGIC_CMD_1
    uint8_t  seq;                           // rolling sequence (wraps)
    uint8_t  flags;                         // TL_FLAG_*
    int16_t  cmd[TL_NUM_THRUSTERS];         // raw: DShot 3D value 0|48..2047; rpm: signed RPM
    uint16_t crc;                           // CRC16 over all preceding bytes
} tl_cmd_t;

// --- Pico -> ESP32 : thruster telemetry ---------------------------------------
typedef struct __attribute__((packed)) {
    uint8_t  magic0;                        // TL_MAGIC_TLM_0
    uint8_t  magic1;                        // TL_MAGIC_TLM_1
    uint8_t  seq;                           // echoes the last command seq acted on
    uint8_t  fault_mask;                    // bit i set = thruster i faulted
    int16_t  rpm[TL_NUM_THRUSTERS];         // signed mechanical RPM (eRPM / pole_pairs)
    uint8_t  status[TL_NUM_THRUSTERS];      // TL_ST_* per thruster
    uint32_t uptime_ms;                     // Pico millis() — ESP32 detects a Pico reset if this
                                            // jumps backward (definitive "did the Pico reboot?")
    uint16_t crc;                           // CRC16 over all preceding bytes
} tl_tlm_t;

// --- ESP32 -> Pico : RPM closed-loop config (sent on boot + on change) --------
// Lets the ESP32 (and MOTOR_TUNE) set the Pico's per-motor RPM control gains without
// reflashing. Applied when a CRC-valid cfg frame arrives.
typedef struct __attribute__((packed)) {
    uint8_t  magic0;                        // TL_MAGIC_CFG_0
    uint8_t  magic1;                        // TL_MAGIC_CFG_1
    float    kp, ki;                        // PI gains
    float    ff_a;                          // feedforward slope (norm-throttle per rpm)
    float    idle_rpm;                      // dynamic-idle target (0 = stop when centred)
    float    max_rpm;                       // |rpm| that maps to full throttle
    float    filt;                          // measured-rpm IIR alpha (0..1)
    float    slew;                          // max output-level change per cycle (0 = off)
    uint8_t  loop_mode;                     // 0 = open-loop (feedforward), 1 = closed-loop RPM (PI)
    uint8_t  dshot_bidir;                   // 1 = bidirectional DShot (RPM telemetry), 0 = normal DShot
    // Low-end shaping — was hard-coded on the Pico, so retuning meant a rebuild.
    float    spin_min;                      // min |output level| once a target is commanded (MOT_SPIN_MIN)
    float    min_target_rpm;                // |target| below this = stop at neutral (RPM_MIN_TGT)
    uint16_t crc;                           // CRC16 over all preceding bytes
} tl_cfg_t;

// CRC16-CCITT (0x1021, init 0xFFFF) — small, no table, identical on both MCUs.
static inline uint16_t tl_crc16(const uint8_t* d, uint32_t n) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < n; ++i) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

// CRC covers every byte except the trailing 2-byte crc field itself.
static inline uint16_t tl_cmd_crc(const tl_cmd_t* c) {
    return tl_crc16((const uint8_t*)c, sizeof(tl_cmd_t) - sizeof(uint16_t));
}
static inline uint16_t tl_tlm_crc(const tl_tlm_t* t) {
    return tl_crc16((const uint8_t*)t, sizeof(tl_tlm_t) - sizeof(uint16_t));
}
static inline uint16_t tl_cfg_crc(const tl_cfg_t* c) {
    return tl_crc16((const uint8_t*)c, sizeof(tl_cfg_t) - sizeof(uint16_t));
}
