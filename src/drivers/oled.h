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
    // pm1 = electronics/SBC pack (local ADC). pm2 = THRUSTER pack, which arrives from the
    // 2nd board over ESP-NOW when PM2_SRC = 2 (the default) — so the top-left box IS the
    // thruster-voltage readout. It shows "--" whenever that link is stale, which is the
    // correct behaviour: a held last value that looks live would be worse.
    // (There used to be a separate `aux_v` field here for the same number. Nothing ever
    //  drew it, and its presence made it look as though the thruster voltage had no display
    //  path at all. Removed rather than left as a decoy.)
    float   pm1, pm2;
    float   roll, pitch, yaw;   // rad
    bool    cal_active;
    float   cal_progress;
    bool    leak;
    bool    kill;   // thruster kill (from 2nd board) — display only
    bool    imu_ever_valid; // false only at cold start → show "--"; else hold value
    uint32_t bno_resets; // BNO085 reset count (hardware-health indicator)
    float   curr;        // battery current (A)
    uint16_t param_n, param_total; // param-download progress (0/0 = idle)
    bool    pm1_present, pm2_present; // power modules reporting (else show "--")
    // PM2's source is switched OFF (PM2_SRC = 0, or PM2_SRC = 2 with ESPNOW_EN = 0), as
    // opposed to selected-but-not-reporting. Both used to render as "--", so a link that had
    // never been enabled was indistinguishable from one that was enabled and failing — and
    // the fix for each is the opposite of the other. "OFF" means "you turned this off".
    bool    pm2_src_off;
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
