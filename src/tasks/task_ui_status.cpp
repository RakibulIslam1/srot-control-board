// =============================================================================
//  Task_UI_Status (Core 0) — PCA9685 aux + OLED + WS2812B RGB + buzzer
//  Owns Bus1 (Wire1) peripherals. Derives indicator state from control/sensors.
// =============================================================================

#include "tasks.h"
#include "config.h"
#include "state_types.h"
#include "comms/params.h"
#include "comms/mav_commands.h"
#include "drivers/pca9685_aux.h"
#include "drivers/neopixel_rgb.h"
#include "drivers/oled.h"
#include "drivers/lora_mission.h"

// Second I2C bus, created in main.cpp.
extern TwoWire Wire1Bus;
extern volatile uint32_t g_stack_hw[6];   // [3] = this task (deepest GFX call chain)

// Choose an RGB mode from the current vehicle state.
static RgbMode pickRgb(bool armed, uint8_t mode, bool err, bool cal_or_tune, uint32_t now) {
    if (now < 2000)  return RgbMode::BOOT;
    if (err)         return RgbMode::ERROR;
    if (cal_or_tune) return RgbMode::STUNT;      // magenta = busy
    if (!armed)      return RgbMode::DISARMED;
    if (mode == 3)   return RgbMode::DEPTH_HOLD;
    if (mode == 5 || mode == 6) return RgbMode::STUNT;
    return RgbMode::ARMED;
}

void Task_UI_Status(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_UI_HZ);
    TickType_t last = xTaskGetTickCount();

    pca9685_aux::begin(&Wire1Bus);
    rgb::begin();
    oled::begin(&Wire1Bus);
    // Animated "Booting SROT" splash — the bar grows over ~1.4 s.
    for (uint32_t t0 = millis(), el = 0; el < 1400; el = millis() - t0) {
        oled::splash(el / 1400.0f);
        vTaskDelay(pdMS_TO_TICKS(40));
    }

    uint16_t servo_us[PCA9685_NUM_CH];
    bool     switch_on[PCA9685_NUM_CH];
    uint8_t  role[PCA9685_NUM_CH];
    uint32_t oled_last = 0;
    uint32_t pca_last = 0;
    uint32_t oled_heal = 0;   // periodic re-init to self-heal a display glitch
    uint32_t pca_probe_last = 0;
    bool     pca_present = false;   // PCA9685 payload-servo extender ACKs on I2C

    // Persistent View: a missed lock HOLDS the last values instead of blanking
    // (fixes the intermittent "--"/0 flicker from lock contention with the 200 Hz
    // sensor task). Only fields whose lock is acquired get overwritten.
    static oled::View v{};

    for (;;) {
        uint32_t now = millis();

        // --- Snapshot control + sensors for the display/indicators. ---
        bool cal_active = false, autotune = false;
        {
            StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(15));
            if (lk.ok()) {
                v.armed = g_state.control.armed;
                v.mode  = (uint8_t)g_state.control.mode;
                autotune = g_state.control.autotune_active;
            }
        }
        {
            StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(15));
            if (lk.ok()) {
                v.depth = g_state.sensors.depth_m;
                v.pm1 = g_state.sensors.pm1_voltage; v.pm2 = g_state.sensors.pm2_voltage;
                v.pm1_present = g_state.sensors.pm1_present; v.pm2_present = g_state.sensors.pm2_present;
                v.curr = g_state.sensors.curr_a;
                v.roll = g_state.sensors.roll; v.pitch = g_state.sensors.pitch; v.yaw = g_state.sensors.yaw;
                v.leak = g_state.sensors.leak;
                v.kill = g_state.sensors.kill_switch;
                // (No separate aux_v: the thruster-pack voltage reaches the OLED as pm2,
                //  via PM2_SRC = 2. See the note in oled.h.)
                // A source that is switched off must not look like one that is failing.
                // PM2_SRC = 2 (ESP-NOW) still needs ESPNOW_EN = 1, or no packet can arrive.
                {
                    const int src2 = (int)g_params.pm2_src;
                    v.pm2_src_off = (src2 == 0) || (src2 == 2 && g_params.espnow_en <= 0.5f);
                }
                v.imu_ever_valid = g_state.sensors.imu_ever_valid;
                v.bno_resets = g_state.sensors.bno_reset_count;
                // Bar30 health = fresh reading (its healthy() is only a boot flag, so a
                // runtime disconnect would otherwise just freeze the depth number). Read at
                // 20 Hz, so >2 s stale = disconnected/faulty.
                uint32_t bs = g_state.sensors.baro_stamp_ms;
                v.baro_ok = (bs != 0) && (now - bs < 2000);
            }
        }
        {
            StateLock lk(g_state.mtx_cal, pdMS_TO_TICKS(15));
            if (lk.ok()) { cal_active = (g_state.cal.routine != CalRoutine::NONE);
                           v.cal_progress = g_state.cal.progress; }
        }
        v.cal_active = cal_active;
        mav_commands::paramDownloadProgress(v.param_n, v.param_total);

        // --- Connectivity indicators (Pico / LoRa / GCS) + thruster presence. ---
        {
            StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(5));
            if (lk.ok()) { v.pico_ok = g_state.thrusters.link_ok;
                           // Link down → show all thrusters absent (they blink → clear cue).
                           v.esc_present = v.pico_ok ? g_state.thrusters.esc_present : 0; }
        }
        v.lora_ok = lora_mission::healthy();
        v.gcs_ok  = mav_commands::gcsLinkOk();

        // Probe the PCA9685 payload-servo extender ~1 Hz (its begin() never verifies the
        // chip). An ACK on Wire1 = connected. Same task owns Wire1, so no bus contention.
        if (now - pca_probe_last >= 1000) {
            pca_probe_last = now;
            Wire1Bus.beginTransmission(I2C_ADDR_PCA9685);
            pca_present = (Wire1Bus.endTransmission() == 0);
        }
        v.pca_ok = pca_present;

        // (Buzzer is now owned by the dedicated Task_Buzzer @ 200 Hz — see task_buzzer.cpp.)

        // --- RGB. ---
        RgbMode rm = pickRgb(v.armed, v.mode, v.leak, cal_active || autotune, now);
        rgb::update(rm, (uint8_t)g_params.rgb_brightness, now);

        // --- PCA9685 aux: apply SERVOn_FUNCTION/trim/min-max, then write on
        //     change or at ~5 Hz (so param edits take effect). ---
        bool changed = false;
        {
            StateLock lk(g_state.mtx_aux, pdMS_TO_TICKS(2));
            if (lk.ok()) {
                for (int i = 0; i < PCA9685_NUM_CH; ++i) {
                    servo_us[i]  = g_state.aux.servo_us[i];
                    switch_on[i] = g_state.aux.switch_on[i];
                }
                if (g_state.aux.dirty) { changed = true; g_state.aux.dirty = false; }
            }
        }
        // Per-channel role (SERVOn_ROLE): 0 = off, 1 = PWM servo, 2 = MOSFET/switch.
        // For a servo channel, output the commanded µs (or trim when uncommanded),
        // clamped to [min,max]. MOSFET channels are driven ON/OFF via switch_on and
        // ignore servo_us; disabled channels are left off.
        for (int c = 0; c < PCA9685_NUM_CH; ++c) {
            int r = (int)g_params.servo_role[c];
            role[c] = (r == 1 || r == 2) ? (uint8_t)r : (uint8_t)PCA_DISABLED;
            if (role[c] == PCA_SERVO) {
                uint16_t us = servo_us[c] ? servo_us[c] : (uint16_t)g_params.servo_trim[c];
                uint16_t lo = (uint16_t)g_params.servo_min[c];
                uint16_t hi = (uint16_t)g_params.servo_max[c];
                servo_us[c] = constrain(us, lo, hi);
            } else {
                servo_us[c] = 0;
            }
        }
        if (changed || now - pca_last >= 200) {
            pca_last = now;
            pca9685_aux::writeState(role, servo_us, switch_on);
        }

        // --- OLED recovery + render at ~10 Hz. ---
        // Dead panel (failed init at reset) → full begin() retry ≤1/s. Alive → a
        // NON-destructive keep-alive every 1.5 s (re-enables charge pump + DISPLAY-ON with
        // no blackout) so a brown-out/buzzer glitch self-heals without the periodic black
        // flash the old full re-init caused.
        if (!oled::healthy()) {
            if (now - oled_heal >= 1000) { oled_heal = now; oled::begin(&Wire1Bus); }
        } else if (now - oled_heal >= 1500) {
            oled_heal = now; oled::keepAlive();
        }
        if (now - oled_last >= 100) {
            oled_last = now;
            oled::render(v);
            g_stack_hw[3] = uxTaskGetStackHighWaterMark(nullptr);   // ~10 Hz probe
        }

        vTaskDelayUntil(&last, period);
    }
}
