// =============================================================================
//  drivers/thruster_link — implementation (ESP32 UART2 <-> Pico 2)
// =============================================================================

#include "drivers/thruster_link.h"
#include "thruster_link_proto.h"

namespace thruster_link {

static HardwareSerial& s_uart = Serial2;   // UART2 (remapped to PIN_PICO_TX/RX)
static uint8_t         s_seq = 0;
static uint32_t        s_last_tlm_ms  = 0;
static uint32_t        s_last_byte_ms = 0;   // last time ANY byte arrived (gap diagnosis)
static uint32_t        s_crc_errs     = 0;   // CRC failures (bytes arriving but corrupt)
static uint16_t        s_rx_idx       = 0;   // RX framing state (file scope so resyncRx can reset)
static uint32_t        s_flap_count   = 0;   // link-drop events (diagnostic)
static uint32_t        s_pico_uptime  = 0;   // last Pico uptime_ms from telemetry

void begin() {
    pinMode(PIN_PICO_ESTOP, OUTPUT);
    digitalWrite(PIN_PICO_ESTOP, LOW);     // LOW = run permit (HIGH asserts stop)
    // At 2 Mbaud a 256 B RX buffer fills in ~1.3 ms, so any brief poll() delay (e.g. a
    // BNO reset spike on Core 1) overflowed it → dropped frames → the Pico "link flap".
    // 1024 B ≈ 5 ms of headroom; larger TX buffer so send() never drops a command.
    s_uart.setRxBufferSize(1024);
    s_uart.setTxBufferSize(512);
    s_uart.begin(PICO_LINK_BAUD, SERIAL_8N1, PIN_PICO_RX, PIN_PICO_TX);
}

void send(const int16_t cmd[NUM_THRUSTERS], bool armed, bool estop, bool rpm_mode,
          bool beep) {
    digitalWrite(PIN_PICO_ESTOP, estop ? HIGH : LOW);

    tl_cmd_t c;
    c.magic0 = TL_MAGIC_CMD_0;
    c.magic1 = TL_MAGIC_CMD_1;
    c.seq    = s_seq++;
    c.flags  = (armed ? TL_FLAG_ARMED : 0) | (estop ? TL_FLAG_ESTOP : 0)
             | (rpm_mode ? TL_FLAG_MODE_RPM : 0) | (beep ? TL_FLAG_BEEP : 0);
    for (int i = 0; i < TL_NUM_THRUSTERS; ++i) c.cmd[i] = cmd[i];
    c.crc = tl_cmd_crc(&c);

    // Non-blocking: only write if the whole frame fits, else drop (retry next cycle).
    if (s_uart.availableForWrite() >= (int)sizeof(c))
        s_uart.write((const uint8_t*)&c, sizeof(c));
}

void sendConfig(float kp, float ki, float ff_a, float idle_rpm, float max_rpm,
                float filt, float slew, uint8_t loop_mode, uint8_t dshot_bidir,
                float spin_min, float min_target_rpm) {
    tl_cfg_t c;
    c.magic0 = TL_MAGIC_CFG_0; c.magic1 = TL_MAGIC_CFG_1;
    c.kp = kp; c.ki = ki; c.ff_a = ff_a; c.idle_rpm = idle_rpm;
    c.max_rpm = max_rpm; c.filt = filt; c.slew = slew;
    c.loop_mode = loop_mode;
    c.dshot_bidir = dshot_bidir;
    c.spin_min = spin_min;
    c.min_target_rpm = min_target_rpm;
    c.crc = tl_cfg_crc(&c);
    if (s_uart.availableForWrite() >= (int)sizeof(c))
        s_uart.write((const uint8_t*)&c, sizeof(c));
}

bool poll(int16_t rpm[NUM_THRUSTERS], uint8_t status[NUM_THRUSTERS], uint8_t& fault) {
    static uint8_t  buf[sizeof(tl_tlm_t)];
    bool got = false;

    while (s_uart.available()) {
        uint8_t b = (uint8_t)s_uart.read();
        s_last_byte_ms = millis();                        // any byte = the Pico is alive
        if (s_rx_idx == 0)      { if (b == TL_MAGIC_TLM_0) buf[s_rx_idx++] = b; }
        else if (s_rx_idx == 1) { if (b == TL_MAGIC_TLM_1) buf[s_rx_idx++] = b; else s_rx_idx = 0; }
        else {
            buf[s_rx_idx++] = b;
            if (s_rx_idx >= sizeof(tl_tlm_t)) {
                s_rx_idx = 0;
                tl_tlm_t t;
                memcpy(&t, buf, sizeof(t));
                if (t.crc != tl_tlm_crc(&t)) { s_crc_errs++; continue; }   // corrupt -> resync
                for (int i = 0; i < TL_NUM_THRUSTERS; ++i) { rpm[i] = t.rpm[i]; status[i] = t.status[i]; }
                fault = t.fault_mask;
                s_pico_uptime = t.uptime_ms;
                s_last_tlm_ms = millis();
                got = true;
            }
        }
    }
    return got;
}

bool linkOk() { return s_last_tlm_ms != 0 && (millis() - s_last_tlm_ms) < PICO_LINK_TIMEOUT_MS; }

// "SILENT" = no bytes at all (Pico stopped/reset) vs "NOISE" = bytes arriving but CRC-failing.
bool bytesRecent() { return (millis() - s_last_byte_ms) <= 150; }

// Flush stale/partial RX and reset framing so re-lock after a gap is clean (no buffered junk).
void resyncRx() { while (s_uart.available()) s_uart.read(); s_rx_idx = 0; }

uint32_t flapCount() { return ++s_flap_count; }   // call once per drop; returns new count
uint32_t picoUptime() { return s_pico_uptime; }   // last Pico uptime_ms reported

}  // namespace thruster_link
