#pragma once

// =============================================================================
//  drivers/oled — SH1106 128x64 status display on Bus1
// =============================================================================

#include <Arduino.h>
#include <Wire.h>

namespace oled {

struct View {
    bool    armed;
    uint8_t mode;
    float   depth;
    float   pm1, pm2;
    float   roll, pitch, yaw;   // rad
    bool    cal_active;
    float   cal_progress;
    bool    leak;
    bool    kill;   // thruster kill (from 2nd board) — display only
    float   aux_v;  // aux voltage reported by the 2nd board (ESP-NOW)
    bool    imu_ever_valid; // false only at cold start → show "--"; else hold value
    uint32_t bno_resets; // BNO085 reset count (hardware-health indicator)
    float   curr;        // battery current (A)
    uint16_t param_n, param_total; // param-download progress (0/0 = idle)
    bool    pm1_present, pm2_present; // power modules reporting (else show "--")
    bool    baro_ok;     // Bar30 giving fresh readings (else "NC" in the depth box)
    bool    pca_ok;      // PCA9685 payload-servo extender ACKs on I2C
    // Connectivity (network indicators)
    bool    pico_ok;     // Pico thruster-link alive
    bool    lora_ok;     // LoRa radio present
    bool    gcs_ok;      // GCS/telemetry link alive
    uint8_t esc_present; // bit i = thruster i sending telemetry
};

bool begin(TwoWire* wire);
void keepAlive();        // non-destructive: re-send DC-DC + DISPLAY-ON (no clear/off/delay)
void splash(float progress); // animated "Booting SROT" screen; progress 0..1 = boot bar
void render(const View& v);
bool healthy();

}  // namespace oled
