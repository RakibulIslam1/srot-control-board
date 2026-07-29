// =============================================================================
//  comms/mavlink_bridge — implementation
// =============================================================================

#include "comms/mavlink_bridge.h"
#include "config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace mav {

// UART0 is now written by TWO Core-0 tasks (Task_MAVLink streaming + Task_LoRa_SD
// dispatching LoRa-uplink command replies). Serialise the write so frames can't
// interleave. Function-local static init is thread-safe (compiler guard). Buffers
// are local (not static) so two callers never share the encode scratch.
static SemaphoreHandle_t txMutex() {
    static SemaphoreHandle_t m = xSemaphoreCreateMutex();
    return m;
}

// --- LoRa downlink tap: length-prefixed byte ring (8 KB) ---------------------
static const size_t   LORA_RING_SZ = 16384;   // holds a full 189-param burst + margin
static uint8_t        s_ring[LORA_RING_SZ];
static size_t         s_head = 0, s_tail = 0;   // head = write, tail = read
static bool           s_lora_tap = false;
static SemaphoreHandle_t ringMutex() {
    static SemaphoreHandle_t m = xSemaphoreCreateMutex();
    return m;
}
static inline size_t ringFree() {
    return (s_tail + LORA_RING_SZ - s_head - 1) % LORA_RING_SZ;
}
static void ringPush(const uint8_t* buf, uint16_t len) {
    xSemaphoreTake(ringMutex(), portMAX_DELAY);
    if ((size_t)(len + 2) <= ringFree()) {   // drop whole frame if it won't fit
        s_ring[s_head] = (uint8_t)(len & 0xff);        s_head = (s_head + 1) % LORA_RING_SZ;
        s_ring[s_head] = (uint8_t)((len >> 8) & 0xff); s_head = (s_head + 1) % LORA_RING_SZ;
        for (uint16_t i = 0; i < len; ++i) { s_ring[s_head] = buf[i]; s_head = (s_head + 1) % LORA_RING_SZ; }
    }
    xSemaphoreGive(ringMutex());
}

// Copy whitelisted control-plane messages into the LoRa ring when the tap is on.
static void tapMaybe(const mavlink_message_t& msg, const uint8_t* buf, uint16_t len) {
    if (!s_lora_tap) return;
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_PARAM_VALUE:
        case MAVLINK_MSG_ID_COMMAND_ACK:
        case MAVLINK_MSG_ID_STATUSTEXT:
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            ringPush(buf, len);
            break;
        default: break;
    }
}

void loraTapEnable(bool on) { s_lora_tap = on; }

uint16_t loraTapPop(uint8_t* buf, uint16_t cap) {
    uint16_t len = 0;
    xSemaphoreTake(ringMutex(), portMAX_DELAY);
    if (s_head != s_tail) {
        uint16_t lo = s_ring[s_tail]; s_tail = (s_tail + 1) % LORA_RING_SZ;
        uint16_t hi = s_ring[s_tail]; s_tail = (s_tail + 1) % LORA_RING_SZ;
        len = (uint16_t)(lo | (hi << 8));
        for (uint16_t i = 0; i < len; ++i) {
            uint8_t b = s_ring[s_tail]; s_tail = (s_tail + 1) % LORA_RING_SZ;
            if (i < cap) buf[i] = b;
        }
        if (len > cap) len = 0;   // shouldn't happen; frame consumed either way
    }
    xSemaphoreGive(ringMutex());
    return len;
}

bool tx(const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tapMaybe(msg, buf, len);
    xSemaphoreTake(txMutex(), portMAX_DELAY);
    // Only write if the whole message fits in the free TX buffer — otherwise the
    // Arduino write() would block until space frees up, stalling the task. Drop instead.
    bool ok = ((int)MAVLINK_SERIAL.availableForWrite() >= (int)len);
    if (ok) MAVLINK_SERIAL.write(buf, len);
    xSemaphoreGive(txMutex());
    return ok;
}

bool txReliable(const mavlink_message_t& msg, uint32_t timeout_ms) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    tapMaybe(msg, buf, len);
    uint32_t start = millis();
    xSemaphoreTake(txMutex(), portMAX_DELAY);
    while ((int)MAVLINK_SERIAL.availableForWrite() < (int)len) {
        if (millis() - start >= timeout_ms) { xSemaphoreGive(txMutex()); return false; }
        vTaskDelay(1);                                       // yield + feed the WDT
    }
    MAVLINK_SERIAL.write(buf, len);
    xSemaphoreGive(txMutex());
    return true;
}

bool poll(mavlink_message_t& out) {
    mavlink_status_t status;
    while (MAVLINK_SERIAL.available()) {
        uint8_t c = (uint8_t)MAVLINK_SERIAL.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &out, &status)) {
            return true;
        }
    }
    return false;
}

}  // namespace mav
