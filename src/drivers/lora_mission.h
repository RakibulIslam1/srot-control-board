#pragma once

// =============================================================================
//  drivers/lora_mission — SX127x LoRa mission-upload receiver (VSPI)
//
//  Receives waypoints as small chunked packets, assembles them into a mission
//  buffer, and ACKs each chunk. Shares VSPI with the SD card (CS arbitration).
//
//  Packet layout (14 bytes): [seq][total][x:f32][y:f32][z:f32]
//  ACK (2 bytes): [0xA5][seq]
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include "lora_telem_proto.h"

namespace lora_mission {

constexpr uint8_t  MAX_WAYPOINTS = 32;

struct Waypoint { float x, y, z; };

bool begin(SPIClass* spi);

// Poll for incoming chunks; call frequently. Assembles + ACKs.
void poll();

// Transmit one black-box telemetry frame (blocking ~tens of ms; call rate-limited).
// No-op if the radio isn't up. Returns to RX after sending.
void sendTelemetry(const LoraTelem& t);

// Transmit a raw byte frame over LoRa (used to relay whole MAVLink frames downlink —
// PARAM_VALUE/ACK/STATUSTEXT — so config works over the radio). Blocking; no-op if down.
void sendRaw(const uint8_t* buf, uint16_t len);

// Register a handler for received MAVLink-uplink packets (raw v2 frame bytes, first
// byte 0xFD). The control board feeds these into its MAVLink parser + command dispatch.
void setUplinkHandler(void (*cb)(const uint8_t* buf, int len));

bool     missionReady();
uint16_t waypointCount();
bool     getWaypoint(uint16_t i, Waypoint& wp);
bool     healthy();

}  // namespace lora_mission
