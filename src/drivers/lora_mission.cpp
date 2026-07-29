// =============================================================================
//  drivers/lora_mission — implementation (sandeepmistry/LoRa)
// =============================================================================

#include "drivers/lora_mission.h"
#include <LoRa.h>
#include "config.h"
#include "comms/params.h"
#include "comms/ui_log.h"

namespace lora_mission {

// Radio band/config comes from shared/lora_telem_proto.h (LORA_FREQ_HZ/BW/SF/CR) so
// the control board and the ground station always match.
static const uint8_t REG_VERSION  = 0x42;    // SX127x version register → must read 0x12
static const uint32_t RETRY_MS    = 2000;    // re-probe cadence while the module is down

static Waypoint  s_wp[MAX_WAYPOINTS];
static bool      s_have[MAX_WAYPOINTS];
static uint8_t   s_total = 0;
static bool      s_ready = false;
static bool      s_ok = false;
static SPIClass* s_spi = nullptr;            // stashed so poll() can retry begin()
static uint32_t  s_last_try = 0;
static void (*s_uplink_cb)(const uint8_t*, int) = nullptr;   // MAVLink uplink over LoRa

// Raw SX127x register read (the LoRa lib keeps readRegister private). Read = addr with
// MSB clear, then clock a dummy byte. Used only for the version-register diagnostic.
static uint8_t readRegRaw(int cs, uint8_t addr) {
    pinMode(cs, OUTPUT);
    s_spi->beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    s_spi->transfer(addr & 0x7F);
    uint8_t v = s_spi->transfer(0x00);
    digitalWrite(cs, HIGH);
    s_spi->endTransaction();
    return v;
}

// One detection attempt: (re)attach SPI + pins and check the version register. On
// success the radio is put into continuous RX. Non-blocking, cheap.
static bool tryBegin() {
    if (!s_spi) return false;
    int cs = params::pinOr(g_params.pin_loracs, PIN_LORA_CS, true);
    LoRa.setSPI(*s_spi);
    // LoRa RST is wired to EN (PIN_LORA_RST = -1 → the driver won't toggle a GPIO), so a
    // stuck module can't be soft-reset without rewiring — the retry covers late/loose power.
    LoRa.setPins(cs, PIN_LORA_RST, params::pinOr(g_params.pin_loradio, PIN_LORA_DIO0));
    if (LoRa.begin(LORA_FREQ_HZ) == 1) {
        LoRa.setSignalBandwidth(LORA_BW);     // must match the ground station
        LoRa.setSpreadingFactor(LORA_SF);
        LoRa.setCodingRate4(LORA_CR);
        LoRa.receive();                       // enter continuous RX
        return true;
    }
    // begin() failed → report the raw version byte so the fault is diagnosable on the OLED:
    //   0x12 = module answers (begin ran early → the retry will catch it)
    //   0x00 = MISO reads low / not driven (MISO / GND / power)
    //   0xFF = MISO floating / no power / MISO not connected
    uint8_t ver = readRegRaw(cs, REG_VERSION);
    char b[24]; snprintf(b, sizeof(b), "LoRa? v=0x%02X", ver);
    ui_log::set(b);
    return false;
}

bool begin(SPIClass* spi) {
    s_spi = spi;
    s_last_try = millis();
    s_ok = tryBegin();
    if (s_ok) ui_log::set("LoRa OK");
    return s_ok;
}

void setUplinkHandler(void (*cb)(const uint8_t*, int)) { s_uplink_cb = cb; }

void sendRaw(const uint8_t* buf, uint16_t len) {
    if (!s_ok || len == 0) return;
    LoRa.beginPacket();
    LoRa.write(buf, len);
    LoRa.endPacket();          // blocks until TX done; leaves the radio in standby.
}

void sendTelemetry(const LoraTelem& t) {
    if (!s_ok) return;
    LoRa.beginPacket();
    LoRa.write((const uint8_t*)&t, sizeof(t));
    LoRa.endPacket();          // blocks until TX done; leaves the radio in standby.
    // NOTE: do NOT call LoRa.receive() (continuous) here — poll() uses parsePacket()
    // which drives one-shot RX_SINGLE; mixing the two leaves the radio not-listening.
    // The caller opens an RX window (repeated poll()) right after this to catch the
    // ground station's uplink reply (TDM slot).
}

static void sendAck(uint8_t seq) {
    LoRa.beginPacket();
    LoRa.write(0xA5);
    LoRa.write(seq);
    LoRa.endPacket();
    LoRa.receive();   // back to RX
}

void poll() {
    if (!s_ok) {
        // Module down: re-probe every RETRY_MS so a late/loose/reconnected radio comes up
        // without a reboot (→ OLED "L" turns solid). Time-gated: never stalls the task loop.
        uint32_t now = millis();
        if (now - s_last_try >= RETRY_MS) {
            s_last_try = now;
            if (tryBegin()) { s_ok = true; ui_log::set("LoRa OK"); }
        }
        return;
    }
    int sz = LoRa.parsePacket();
    if (sz <= 0) return;

    uint8_t buf[64];
    int n = 0;
    for (; n < sz && n < (int)sizeof(buf) && LoRa.available(); ++n) buf[n] = (uint8_t)LoRa.read();
    if (n <= 0) return;

    // Classify by the first byte: 0xFD = MAVLink v2 uplink (commands / joystick from the
    // ground station) → hand to the uplink handler; 0xC5 = our own telemetry class (ignore);
    // otherwise a 14-byte mission chunk.
    if (buf[0] == 0xFD) { if (s_uplink_cb) s_uplink_cb(buf, n); return; }
    if (buf[0] == LORA_TELEM_MAGIC0) return;
    if (n < 14) return;

    uint8_t seq = buf[0];
    uint8_t total = buf[1];
    if (seq >= MAX_WAYPOINTS || total == 0 || total > MAX_WAYPOINTS) return;

    Waypoint wp;
    memcpy(&wp.x, &buf[2], 4);
    memcpy(&wp.y, &buf[6], 4);
    memcpy(&wp.z, &buf[10], 4);
    s_wp[seq] = wp;
    s_have[seq] = true;
    s_total = total;

    sendAck(seq);

    // Mission complete when every chunk 0..total-1 has arrived.
    bool all = true;
    for (uint8_t i = 0; i < total; ++i) if (!s_have[i]) { all = false; break; }
    s_ready = all;
}

bool missionReady() { return s_ready; }
uint16_t waypointCount() { return s_ready ? s_total : 0; }

bool getWaypoint(uint16_t i, Waypoint& wp) {
    if (i >= s_total || !s_have[i]) return false;
    wp = s_wp[i];
    return true;
}

bool healthy() { return s_ok; }

}  // namespace lora_mission
