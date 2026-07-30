// =============================================================================
//  2ND BOARD — thruster control board  (env:second-board)
//
//  A second ESP32 that lives with the THRUSTER battery. Two jobs:
//
//    1. Rotary power switch. An AS5600 magnetic encoder reads a knob; two angular
//       ON windows drive a MOSFET that makes or breaks thruster power. Hysteresis
//       stops it chattering on the window edges.
//
//    2. Report the THRUSTER-pack voltage to the SROT control board over ESP-NOW.
//       This is the half that was missing. The control board's own ADC measures the
//       ELECTRONICS/SBC pack — a different battery — so nothing on that board could
//       see the thruster pack, and two features that already exist there sat inert:
//         * mixer::setBatteryVoltage() thrust linearisation, which is what makes a
//           timed move travel the same distance on a full or a flat pack;
//         * the low-thruster-battery failsafe (FS_BAT_*).
//       Both come alive as soon as these packets arrive.
//
//  Wire format: shared/espnow_proto.h, compiled by both boards.
//
//  NO DISPLAY. The OLED moved to the SROT board, so the SH1106 driver, the second
//  I2C bus and the whole render path are gone — this board is headless and its only
//  human output is the serial debug line.
//
//  Also removed: the ESP-NOW *receive* of a C3 Mini's service voltage. Its only
//  consumer was the display, and the control board already measures the electronics
//  pack itself. Re-adding it would mean a third field in the packet, i.e. a wire
//  format change and reflashing both boards — so it is deliberately not carried.
//
//  On the control board, set ESPNOW_EN = 1 (it defaults to 0) or these packets are
//  ignored, and set MOT_BAT_V_MAX to the pack's full-charge voltage or the
//  feedforward stays disabled even with good data arriving.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>          // native set_channel / set_ps — the Arduino API cannot
#include "espnow_proto.h"

// --- Build options -----------------------------------------------------------
#define ENABLE_DEBUG            1
// The MOSFET drives thruster power. If your wiring is inverted (HIGH = cut), flip
// this: the packet's thruster_kill must mean "power is CUT" whichever way the FET goes.
#define SECOND_KILL_ACTIVE_LOW  0
// Must match the control board's ESPNOW_CHANNEL.
#define ESPNOW_CHANNEL          1

// --- Pins --------------------------------------------------------------------
static const int SWITCH_PIN    = 16;   // MOSFET gate: HIGH = thruster power ON
static const int PM1_VOLT_PIN  = 35;   // THRUSTER battery divider (input-only pin)
static const int AS5600_SDA    = 21;
static const int AS5600_SCL    = 22;

// --- AS5600 ------------------------------------------------------------------
static const int AS5600_ADDR    = 0x36;
static const int RAW_ANGLE_REG  = 0x0C;
static const int HYSTERESIS_RAW = 34;   // ~3 deg of 4096 counts

// Two ON windows (degrees). Outside both, thruster power is off.
static float on_angle_deg  = 240.0f, off_angle_deg  = 330.0f;
static float on_angle_deg2 =  60.0f, off_angle_deg2 = 120.0f;
static int   on_angle_raw, off_angle_raw, on_angle_raw2, off_angle_raw2;
static bool  actual_output_state = false;

// --- Voltage -----------------------------------------------------------------
// Calibrated volts per ADC count for this board's divider. Same constant the control
// board uses for its own pack (PM1_VMULT) — measure yours before trusting it.
static const float PM1_VOLT_MULT = 0.009088f;
static long  pm1_v_sum = 0;
static int   volt_samples = 0;
static float local_thruster_voltage = 0.0f;

// --- Timers ------------------------------------------------------------------
static uint32_t last_sample_time = 0, last_update_time = 0;
static uint32_t last_tx_time = 0, last_debug_time = 0;
static const uint32_t DEBUG_INTERVAL_MS = 250;
static const uint32_t TX_INTERVAL_MS    = 1000 / ESPNOW_TX_HZ;

// --- ESP-NOW -----------------------------------------------------------------
static uint8_t  BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool     espnow_up = false;
static uint32_t tx_ok = 0, tx_fail = 0;

static void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
    (void)mac;
    if (status == ESP_NOW_SEND_SUCCESS) tx_ok++; else tx_fail++;
}

static bool espnowBegin() {
    WiFi.mode(WIFI_STA);
    // Force the channel natively — the Arduino WiFi API will not pin it without an AP
    // association, and ESP-NOW peers must be on the same channel to hear each other.
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    // Power-save parks the radio between beacons and drops broadcast frames.
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (esp_now_init() != ESP_OK) return false;
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    // Broadcast needs an explicit peer entry even though it is not a real device.
    if (esp_now_add_peer(&peer) != ESP_OK) return false;
    return true;
}

static void espnowSend() {
    if (!espnow_up) return;
    espnow_from_second_t pkt;
    pkt.magic = ESPNOW_MSG_MAGIC;
    // "kill" = thruster power is CUT. actual_output_state is the MOSFET gate level, so
    // kill is its logical inverse (or the same, on inverted wiring).
#if SECOND_KILL_ACTIVE_LOW
    pkt.thruster_kill = actual_output_state ? 1 : 0;
#else
    pkt.thruster_kill = actual_output_state ? 0 : 1;
#endif
    pkt.aux_voltage = local_thruster_voltage;
    esp_now_send(BROADCAST, (const uint8_t*)&pkt, sizeof(pkt));
}

// --- AS5600 helpers ----------------------------------------------------------
static bool isAngleInOnZone(int current, int on_target, int off_target) {
    if (on_target < off_target) return (current >= on_target && current < off_target);
    return (current >= on_target || current < off_target);   // window wraps through 0
}

static int angularDifference(int a, int b) {
    int diff = abs(a - b);
    if (diff > 2048) diff = 4096 - diff;    // shortest way round
    return diff;
}

static int readRawAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(RAW_ANGLE_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() == 2) return (Wire.read() << 8) | Wire.read();
    return -1;   // no encoder / bus fault
}

// =============================================================================
void setup() {
#if ENABLE_DEBUG
    Serial.begin(115200);
    delay(100);
#endif

    // Default to thrusters OFF until the encoder says otherwise — a boot with an
    // unreadable encoder must not energise them.
    pinMode(SWITCH_PIN, OUTPUT);
    digitalWrite(SWITCH_PIN, LOW);

    Wire.begin(AS5600_SDA, AS5600_SCL);
    Wire.setClock(400000);

    espnow_up = espnowBegin();

#if ENABLE_DEBUG
    Serial.println();
    Serial.println("=== 2nd board (thruster control) ===");
    Serial.print("MAC: ");       Serial.println(WiFi.macAddress());
    Serial.print("ESP-NOW: ");   Serial.println(espnow_up ? "up" : "FAILED");
    Serial.print("Broadcasting thruster voltage on channel ");
    Serial.print(ESPNOW_CHANNEL); Serial.print(" at "); Serial.print(ESPNOW_TX_HZ);
    Serial.println(" Hz");
    Serial.println("Control board needs ESPNOW_EN=1 to listen.");
    Serial.println("====================================");
#endif

    on_angle_raw   = (int)(on_angle_deg   * 4096.0f / 360.0f);
    off_angle_raw  = (int)(off_angle_deg  * 4096.0f / 360.0f);
    on_angle_raw2  = (int)(on_angle_deg2  * 4096.0f / 360.0f);
    off_angle_raw2 = (int)(off_angle_deg2 * 4096.0f / 360.0f);

    // Adopt the knob's current position immediately, so power state matches reality
    // on a brown-out recovery instead of waiting for the knob to be moved.
    int initial = readRawAngle();
    if (initial != -1) {
        actual_output_state = isAngleInOnZone(initial, on_angle_raw,  off_angle_raw) ||
                              isAngleInOnZone(initial, on_angle_raw2, off_angle_raw2);
        digitalWrite(SWITCH_PIN, actual_output_state ? HIGH : LOW);
    }
}

void loop() {
    const uint32_t now = millis();

    // 1. Switching — run every pass so the MOSFET follows the knob with no lag.
    int   raw = readRawAngle();
    float angle_deg = 0;
    if (raw != -1) {
        angle_deg = raw * (360.0f / 4096.0f);

        bool in_on = isAngleInOnZone(raw, on_angle_raw,  off_angle_raw) ||
                     isAngleInOnZone(raw, on_angle_raw2, off_angle_raw2);

        // Only act once clear of EVERY window edge, so a knob resting on a boundary
        // cannot buzz the MOSFET.
        int min_dist = min(min(angularDifference(raw, on_angle_raw),
                               angularDifference(raw, off_angle_raw)),
                           min(angularDifference(raw, on_angle_raw2),
                               angularDifference(raw, off_angle_raw2)));
        bool past_deadband = (min_dist > HYSTERESIS_RAW);

        if (!actual_output_state && in_on && past_deadband) {
            actual_output_state = true;
            digitalWrite(SWITCH_PIN, HIGH);
        } else if (actual_output_state && !in_on && past_deadband) {
            actual_output_state = false;
            digitalWrite(SWITCH_PIN, LOW);
        }
    }

    // 2. Thruster-pack voltage: oversample at 20 Hz, average every 500 ms. The ESP32
    //    ADC is noisy enough that a single read wanders by tenths of a volt.
    if (now - last_sample_time >= 50) {
        last_sample_time = now;
        pm1_v_sum += analogRead(PM1_VOLT_PIN);
        volt_samples++;
    }
    if (now - last_update_time >= 500) {
        last_update_time = now;
        if (volt_samples > 0) {
            local_thruster_voltage = ((float)pm1_v_sum / volt_samples) * PM1_VOLT_MULT;
        }
        pm1_v_sum = 0;
        volt_samples = 0;
    }

    // 3. Broadcast to the control board.
    if (now - last_tx_time >= TX_INTERVAL_MS) {
        last_tx_time = now;
        espnowSend();
    }

    // 4. Serial debug. tx_ok/tx_fail are here on purpose: without a display this is
    //    the only way to tell "not sending" from "sending, nobody listening".
#if ENABLE_DEBUG
    if (now - last_debug_time >= DEBUG_INTERVAL_MS) {
        last_debug_time = now;
        Serial.print("THR ");   Serial.print(local_thruster_voltage, 2);
        Serial.print(" V | knob "); Serial.print((int)angle_deg);
        Serial.print(" deg | power ");
        Serial.print(actual_output_state ? "ON " : "OFF");
        Serial.print(" | tx ");  Serial.print(tx_ok);
        Serial.print(" fail ");  Serial.print(tx_fail);
        if (raw == -1) Serial.print(" | AS5600 NOT READ");
        Serial.println();
    }
#endif
}
