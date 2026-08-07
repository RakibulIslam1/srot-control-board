// =============================================================================
//  drivers/oled — implementation (Adafruit SH110X)
// =============================================================================

#include "drivers/oled.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <string.h>
#include "config.h"
#include "state_types.h"     // FlightMode — modeName() switches on the enum, not raw numbers
#include "comms/ui_log.h"

extern const char* g_reset_reason;   // captured in main.cpp

namespace oled {

static Adafruit_SH1106G s_disp(128, 64, &Wire, -1);
static bool      s_ok = false;
static TwoWire*  s_wire = nullptr;
static bool      s_constructed = false;

// Short labels so the mode tab fits the narrower left column.
// Mirrors FlightMode in state_types.h. Cases are written against the ENUM, not raw numbers,
// so adding a mode cannot silently fall through to "?" again -- which is exactly what
// happened to AUTO (23): it was missing here while Bondor mapped it correctly, so the board
// screen showed "?" for a mode the vehicle was genuinely in.
static const char* modeName(uint8_t m) {
    switch ((FlightMode)m) {
        case FlightMode::STABILIZE:    return "STAB";
        case FlightMode::ACRO:         return "ACRO";
        case FlightMode::DEPTH_HOLD:   return "DEPTH";
        case FlightMode::SURFACE:      return "SURF";
        case FlightMode::MANUAL:       return "MANUAL";
        case FlightMode::MOTOR_DETECT: return "MOTDET";
        case FlightMode::AUTOTUNE:     return "ATUNE";
        case FlightMode::MOTOR_TUNE:   return "MTUNE";
        case FlightMode::AUTO:         return "AUTO";
        case FlightMode::STUNT:        return "STUNT";
        case FlightMode::PATTERN:      return "PATRN";
    }
    return "?";   // no default: a new enumerator now warns at compile time instead
}

bool begin(TwoWire* wire) {
    s_wire = wire;
    // Construct the driver object ONCE. Reconstructing would null the framebuffer
    // pointer and re-malloc on every re-begin (leak); keepAlive() avoids begin() entirely.
    if (!s_constructed) { s_disp = Adafruit_SH1106G(128, 64, wire, -1); s_constructed = true; }
    // Retry: a single failed init (reset timing / power settling) must not blank forever.
    for (int i = 0; i < 3; ++i) {
        s_ok = s_disp.begin(I2C_ADDR_OLED, true);
        if (s_ok) break;
        delay(40);
    }
    if (s_ok) s_disp.setRotation(2);   // 180° — mounting orientation
    return s_ok;
}

// Non-destructive keep-alive: re-enable the DC-DC charge pump and turn the panel ON.
// This does NOT touch the framebuffer, DISPLAYOFF, delay, or the Adafruit splash — so it
// costs ZERO visible blackout (unlike begin(), which off→delay(100)→logo→on). If a brown-out
// glitch (e.g. the buzzer's current dip) dropped the charge pump, this restores it silently.
void keepAlive() {
    if (!s_ok) return;
    s_disp.oled_command(SH110X_DCDC); s_disp.oled_command(0x8B);   // DC-DC on
    s_disp.oled_command(SH110X_DISPLAYON);                         // 0xAF
}

// Animated boot splash: centred "SROT" (smaller), a fill bar growing L→R, version below.
void splash(float progress) {
    if (!s_ok) return;
    if (progress < 0) progress = 0; if (progress > 1) progress = 1;
    s_disp.clearDisplay();
    s_disp.setTextColor(SH110X_WHITE);
    s_disp.setTextSize(2);
    s_disp.setCursor(40, 8);                 // "SROT" (4×12=48px) centred on 128
    s_disp.print("SROT");
    s_disp.drawRoundRect(24, 30, 80, 10, 3, SH110X_WHITE);      // boot-bar outline
    int fw = (int)(progress * 80.0f + 0.5f);
    if (fw > 0) s_disp.fillRoundRect(24, 30, fw, 10, 3, SH110X_WHITE);  // grows L→R
    s_disp.setTextSize(1);
    // The BOARD name ("SROT") is the big text above; this line names the FIRMWARE.
    // "Hengla v0.2.0" = 13 chars x 6px = 78px, so x = (128-78)/2 = 25.
    s_disp.setCursor(25, 50);
    s_disp.print(HENGLA_FW_VERSION_STR);
    s_disp.display();
}

// Draw one network-signal cell: a letter + a bar. Present = solid; absent = blink.
static void netCell(int x, char c, bool present, bool blink_on) {
    s_disp.setCursor(x, 15);
    s_disp.write(c);
    if (present || blink_on) s_disp.fillRoundRect(x, 25, 8, 4, 1, SH110X_WHITE);
}

void render(const View& v) {
    if (!s_ok) return;
    const float R2D = 57.2957795f;
    const bool  blink_on = ((millis() / 400) & 1) != 0;   // ~1.25 Hz blink
    s_disp.clearDisplay();
    s_disp.setTextSize(1);
    s_disp.setTextColor(SH110X_WHITE);

    // ---- Top row: PM2 box (left) · SROT box (centre) · PM1 box (right) ----
    s_disp.drawRoundRect(0, 0, 30, 11, 2, SH110X_WHITE);
    s_disp.setCursor(2, 2);
    // "OFF" = the source is disabled, "--" = enabled but no fresh data. Distinct on purpose:
    // "--" alone could not tell "ESPNOW_EN is 0" from "the 2nd board is not transmitting".
    if (v.pm2_present)      s_disp.printf("%.1f", v.pm2);
    else if (v.pm2_src_off) s_disp.print("OFF");
    else                    s_disp.print("--");
    s_disp.drawRoundRect(49, 0, 30, 11, 3, SH110X_WHITE);   // centred on x64
    s_disp.setCursor(52, 2);  s_disp.print("SROT");
    s_disp.drawRoundRect(98, 0, 30, 11, 2, SH110X_WHITE);
    s_disp.setCursor(101, 2);
    if (v.pm1_present) s_disp.printf("%.1f", v.pm1); else s_disp.print("--");

    // ---- Row 1: depth box | cal/servo cell | arm circle | network box ----
    // Depth box: bucket (fills bottom-up) + value; "NC" (blink) if the Bar30 is dead/missing.
    s_disp.drawRoundRect(0, 12, 42, 18, 2, SH110X_WHITE);
    if (!v.baro_ok) {
        if (blink_on) { s_disp.setCursor(14, 17); s_disp.print("NC"); }   // no depth sensor
    } else {
        s_disp.drawRoundRect(3, 14, 8, 14, 1, SH110X_WHITE);              // bucket outline
        float df = v.depth / (float)OLED_DEPTH_FULL;
        if (df < 0) df = 0; if (df > 1) df = 1;
        int fh = (int)(df * 12.0f + 0.5f);
        if (fh > 0) s_disp.fillRect(5, 27 - fh, 4, fh, SH110X_WHITE);
        s_disp.setCursor(14, 17); s_disp.printf("%.1f", v.depth);
    }

    // Payload cell (snug, ~one T/L/G cell wide): 'P' + bar; while calibrating → 'C' + progress bar.
    s_disp.drawRoundRect(44, 12, 15, 18, 2, SH110X_WHITE);
    if (v.cal_active) {
        s_disp.setCursor(46, 15); s_disp.print('C');
        int w = (int)(v.cal_progress * 8.0f + 0.5f);
        if (w > 0) s_disp.fillRect(46, 25, w, 4, SH110X_WHITE);   // % as a fill bar
    } else {
        netCell(48, 'P', v.pca_ok, blink_on);   // letter + bar under — same look as T/L/G
    }

    // Arm state: a wide, fully-rounded pill (height matches the T/L/G box) — filled = armed.
    if (v.armed) { s_disp.fillRoundRect(60, 12, 32, 18, 9, SH110X_WHITE); s_disp.setTextColor(SH110X_BLACK); }
    else           s_disp.drawRoundRect(60, 12, 32, 18, 9, SH110X_WHITE);
    s_disp.setCursor(v.armed ? 74 : 75, 17); s_disp.print(v.armed ? 'A' : 'D');
    s_disp.setTextColor(SH110X_WHITE);

    // Network box (compact): T = thruster/Pico link, L = LoRa, G = GCS. Solid=up, blink=down.
    s_disp.drawRoundRect(94, 12, 34, 18, 2, SH110X_WHITE);
    netCell(96,  'T', v.pico_ok, blink_on);
    netCell(107, 'L', v.lora_ok, blink_on);
    netCell(118, 'G', v.gcs_ok,  blink_on);

    // ---- Row 2: mode box (left) · merged angle box (right) ----
    s_disp.drawRoundRect(0, 32, 58, 11, 2, SH110X_WHITE);
    s_disp.setCursor(3, 34); s_disp.print(modeName(v.mode));   // cal% now lives in row 1

    // Merged roll/pitch + yaw box.
    s_disp.drawRoundRect(60, 32, 68, 20, 2, SH110X_WHITE);
    s_disp.setCursor(61, 34);   // fixed-width fields → R/P never shift their start position
    if (!v.imu_ever_valid) s_disp.print("R  -- P  --");
    else                   s_disp.printf("R%+4.0f P%+4.0f", v.roll * R2D, v.pitch * R2D);
    {   // yaw centred in the box, with a space after Y (1 px up: y44)
        char yb[8];
        if (!v.imu_ever_valid) strcpy(yb, "Y --");
        else {
            float yd = v.yaw * R2D;
            while (yd < 0)       yd += 360.0f;
            while (yd >= 360.0f) yd -= 360.0f;
            snprintf(yb, sizeof(yb), "Y %.0f", yd);
        }
        int yx = 94 - (int)strlen(yb) * 3;   // box centre x=94
        s_disp.setCursor(yx, 44); s_disp.print(yb);
    }
    // Log ticker under the mode tab (x0-58 window). Leak/kill take priority.
    if (v.leak) { if (blink_on) { s_disp.setCursor(2, 45); s_disp.print("LEAK!"); } }
    else if (v.kill) { if (blink_on) { s_disp.setCursor(2, 45); s_disp.print("KILL"); } }
    else {
        char msg[48]; ui_log::get(msg, sizeof(msg));
        int len = (int)strlen(msg);
        const int win = 9;                       // 9 chars fit x2..56
        if (len <= win) { s_disp.setCursor(2, 45); s_disp.print(msg); }
        else {
            int total = len + 3;                 // 3-space gap between repeats
            int off = (millis() / 250) % total;  // scroll right→left
            char wb[win + 1];
            for (int k = 0; k < win; ++k) { int idx = (off + k) % total; wb[k] = (idx < len) ? msg[idx] : ' '; }
            wb[win] = '\0';
            s_disp.setCursor(2, 45); s_disp.print(wb);
        }
    }

    // ---- Bottom row: 8 thruster circles ----
    const int R = 5, SP = 16;   // start 3 px from the left: first left edge = 3
    for (int i = 0; i < NUM_THRUSTERS; ++i) {
        int cx = 3 + R + i * SP, cy = 58;
        bool present = (v.esc_present >> i) & 1;
        if (present) {
            s_disp.fillCircle(cx, cy, 5, SH110X_WHITE);
            s_disp.setTextColor(SH110X_BLACK);          // digit reads on the white fill
            s_disp.setCursor(cx - 2, cy - 3); s_disp.print(i + 1);
            s_disp.setTextColor(SH110X_WHITE);
        } else if (blink_on) {                          // absent → outline + blink
            s_disp.drawCircle(cx, cy, 5, SH110X_WHITE);
            s_disp.setCursor(cx - 2, cy - 3); s_disp.print(i + 1);
        }
    }

    s_disp.display();
}

bool healthy() { return s_ok; }

}  // namespace oled
