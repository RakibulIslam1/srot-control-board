// =============================================================================
//  drivers/espnow_link — implementation (receive-only)
//  Follows the WiFi-STA + fixed-channel ESP-NOW pattern from
//  example/2nd_board_firmware/main.cpp.
// =============================================================================

#include "drivers/espnow_link.h"
#include "config.h"

#if ESPNOW_ENABLE
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

namespace espnow_link {

static volatile bool     s_kill = false;
static volatile float    s_aux = 0;
static volatile uint32_t s_last_ms = 0;
static volatile bool     s_got = false;

// arduino-esp32 2.0.17 RX callback signature.
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
    (void)mac;
    if (len < (int)sizeof(FromSecond)) return;
    FromSecond p;
    memcpy(&p, data, sizeof(p));
    if (p.magic != MAGIC) return;
    s_kill = (p.thruster_kill != 0);
    s_aux = p.aux_voltage;
    s_last_ms = millis();
    s_got = true;
}

bool begin() {
    WiFi.mode(WIFI_STA);
    // Pin to the 2nd board's channel (native call — the Arduino API can't force it).
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_ps(WIFI_PS_NONE);            // no power-save → reliable RX
    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_recv_cb(onRecv);
    return true;
}

void poll(bool& kill, float& aux_v, bool& fresh) {
    bool f = s_got && (millis() - s_last_ms < ESPNOW_STALE_MS);
    fresh = f;
    aux_v = s_aux;
    kill = f ? s_kill : false;   // link lost → don't assert kill (display-only)
}

}  // namespace espnow_link

#else  // ESPNOW_ENABLE == 0

namespace espnow_link {
bool begin() { return false; }
void poll(bool& kill, float& aux_v, bool& fresh) { kill = false; aux_v = 0; fresh = false; }
}  // namespace espnow_link

#endif
