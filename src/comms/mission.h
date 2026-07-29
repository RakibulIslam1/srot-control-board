#pragma once

// =============================================================================
//  comms/mission — MAVLink mission upload/download (ArduSub-style)
//
//  Implements the standard MISSION_* handshake so QGC/BlueOS can upload and
//  download a waypoint mission over MAVLink. LoRa mission upload is an extra,
//  parallel path (drivers/lora_mission). Only MAV_MISSION_TYPE_MISSION is stored.
// =============================================================================

#include <Arduino.h>
#include "comms/mavlink_bridge.h"

namespace mission {

constexpr uint16_t MAX_ITEMS = 64;

// Dispatch a mission-related message. Returns true if it was a mission message.
bool handle(const mavlink_message_t& msg);

// Number of stored mission items.
uint16_t count();
bool getItem(uint16_t seq, mavlink_mission_item_int_t& out);

}  // namespace mission
