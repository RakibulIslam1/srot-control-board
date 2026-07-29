// =============================================================================
//  Task_LoRa_SD (Core 0) — LoRa mission receiver + SD binary logger
//  Both share VSPI (SCK/MISO/MOSI) with separate CS lines.
// =============================================================================

#include "tasks.h"
#include "config.h"
#include "state_types.h"
#include <SPI.h>
#include "comms/params.h"
#include "comms/mavlink_bridge.h"
#include "comms/mav_commands.h"
#include "drivers/lora_mission.h"
#include "drivers/sd_log.h"
#include "drivers/espnow_link.h"

extern volatile uint32_t g_stack_hw[6];   // [4] = this task

// MAVLink received over LoRa (uplink from the ground station): parse on a dedicated
// channel and dispatch through the SAME handler as UART0 — so arm/mode/autotune/
// motor-tune/SROT_MOVE/MANUAL_CONTROL all work wirelessly. (Replies go out UART0.)
static volatile uint16_t s_ul_rx = 0;         // diag: uplink MAVLink msgs handled over LoRa
static volatile uint32_t s_last_uplink_ms = 0; // last LoRa uplink packet (→ LoRa GCS active)
static void onLoraUplink(const uint8_t* buf, int len) {
    s_last_uplink_ms = millis();
    mav_commands::feedGcs();   // ANY uplink packet keeps the vehicle's GCS link alive
    mavlink_message_t msg;
    mavlink_status_t st;
    for (int i = 0; i < len; ++i)
        if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &st)) { mav_commands::handle(msg); s_ul_rx++; }
}

void Task_LoRa_SD(void* pv) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_LORA_SD_HZ);
    TickType_t last = xTaskGetTickCount();

    // Shared VSPI bus for LoRa + SD.
    static SPIClass vspi(VSPI);
    vspi.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

    lora_mission::begin(&vspi);
    lora_mission::setUplinkHandler(onLoraUplink);
    sd_log::begin(&vspi);

    // ESP-NOW / WiFi is started at runtime only when ESPNOW_EN=1 (keeps WiFi off
    // the MAVLink core until the user opts in).
    bool espnow_started = false;

    uint32_t log_last = 0;
    uint32_t stk_cnt = 0;
    uint32_t tlm_last = 0;      // LoRa telemetry downlink pacing
    uint16_t tlm_seq = 0;

    for (;;) {
        if (++stk_cnt >= TASK_LORA_SD_HZ) { stk_cnt = 0; g_stack_hw[4] = uxTaskGetStackHighWaterMark(nullptr); }
        uint32_t now = millis();

        // --- LoRa: receive + assemble mission chunks. ---
        lora_mission::poll();

        // --- LoRa: black-box telemetry downlink (~8 Hz) — this is also the TDM beacon;
        //     the ground station replies with one uplink packet right after each frame,
        //     so the two radios never transmit at once. ---
        if (now - tlm_last >= 120) {
            tlm_last = now;
            LoraTelem t = {};
            t.magic0 = LORA_TELEM_MAGIC0;
            t.magic1 = LORA_TELEM_MAGIC1;
            t.seq = tlm_seq++;
            {
                StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    const SensorState& x = g_state.sensors;
                    t.roll_cd  = (int16_t)(x.roll  * 5729.578f);   // rad → centideg (180/π*100)
                    t.pitch_cd = (int16_t)(x.pitch * 5729.578f);
                    t.yaw_cd   = (int16_t)(x.yaw   * 5729.578f);
                    t.depth_cm = (int16_t)(x.depth_m * 100.0f);
                    t.batt_mv  = (uint16_t)(x.pm1_voltage * 1000.0f);
                    t.curr_ca  = (int16_t)(x.curr_a * 100.0f);
                    t.wtemp_c  = (int8_t)x.water_temp_c;
                    if (x.leak)        t.flags |= LT_FLAG_LEAK;
                    if (x.kill_switch) t.flags |= LT_FLAG_KILL;
                }
            }
            {
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    t.mode = (uint8_t)g_state.control.mode;
                    if (g_state.control.armed) t.flags |= LT_FLAG_ARMED;
                }
            }
            {
                StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    for (int i = 0; i < 8 && i < NUM_THRUSTERS; ++i) t.rpm[i] = g_state.thrusters.rpm[i];
                    if (g_state.thrusters.link_ok) t.flags |= LT_FLAG_THR_LINK;
                }
            }
            t.ul_rx = s_ul_rx;
            t.crc = lora_telem_crc(&t);
            lora_mission::sendTelemetry(t);

            // TDM receive window: right after beaconing, listen hard (~70 ms) for the
            // ground station's uplink reply. parsePacket() drives one-shot RX, so poll
            // it tightly here — otherwise the reply lands while we're not listening and
            // commands never arrive (attitude/downlink still work; uplink doesn't).
            uint32_t win = millis();
            while (millis() - win < 70) {
                lora_mission::poll();
                vTaskDelay(1);
            }

            // Config-plane downlink: relay a couple of queued control-plane MAVLink frames
            // (PARAM_VALUE / ACK / STATUSTEXT) over LoRa so Bondor's config tabs work. Only
            // populated while a LoRa GCS is active (tap enabled below).
            for (int k = 0; k < 2; ++k) {
                uint8_t fbuf[MAVLINK_MAX_PACKET_LEN];
                uint16_t flen = mav::loraTapPop(fbuf, sizeof(fbuf));
                if (!flen) break;
                lora_mission::sendRaw(fbuf, flen);
            }
        }

        // Enable the control-plane tap only while a LoRa GCS is present (uplink < 3 s ago),
        // so we don't queue/relay params when the vehicle is on USB/UDP.
        mav::loraTapEnable((now - s_last_uplink_ms) < 3000);

        // --- ESP-NOW: start on demand, then read thruster-kill + aux voltage. ---
        if (!espnow_started && g_params.espnow_en > 0.5f) {
            espnow_started = espnow_link::begin();
        }
        if (espnow_started) {
            bool kill, fresh; float aux_v;
            espnow_link::poll(kill, aux_v, fresh);
            StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(2));
            if (lk.ok()) {
                g_state.sensors.kill_switch = kill;
                // Publish freshness, not just the value. This is the THRUSTER battery and
                // it feeds the low-battery failsafe (and, later, motor voltage
                // compensation) — consuming a silently-held stale reading would be unsafe.
                if (fresh) {
                    g_state.sensors.aux_voltage  = aux_v;
                    g_state.sensors.aux_stamp_ms = millis();
                }
                g_state.sensors.aux_valid = fresh;
            }
        }

        // --- SD: append a flight-log record at ~20 Hz. ---
        if (sd_log::healthy() && now - log_last >= 50) {
            log_last = now;
            sd_log::Record rec{};
            rec.t_ms = now;
            {
                StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    rec.qw = g_state.sensors.quat_w; rec.qx = g_state.sensors.quat_x;
                    rec.qy = g_state.sensors.quat_y; rec.qz = g_state.sensors.quat_z;
                    rec.gx = g_state.sensors.gyro.x; rec.gy = g_state.sensors.gyro.y; rec.gz = g_state.sensors.gyro.z;
                    rec.depth = g_state.sensors.depth_m;
                    rec.pm1 = g_state.sensors.pm1_voltage; rec.pm2 = g_state.sensors.pm2_voltage;
                }
            }
            {
                StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    for (int i = 0; i < 8; ++i) rec.thr[i] = g_state.thrusters.throttle[i];
                    rec.armed = g_state.thrusters.armed ? 1 : 0;
                }
            }
            {
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(2));
                if (lk.ok()) rec.mode = (uint8_t)g_state.control.mode;
            }
            sd_log::logRecord(rec);
        }

        vTaskDelayUntil(&last, period);
    }
}
