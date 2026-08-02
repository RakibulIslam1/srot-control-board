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
#include "comms/mav_stream.h"      // queueStatusText — thruster-pack link notices
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
    uint32_t espnow_try_ms = 0;   // backoff timer for espnow_link::begin() retries
    uint32_t espnow_fail_n = 0;   // consecutive failures (throttles the STATUSTEXT)

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
                    // Thruster pack, relayed from the 2nd board over ESP-NOW. Send 0 rather
                    // than a stale number when the link is not fresh, so the ground station
                    // can show "no data" instead of a value frozen at whatever arrived last.
                    t.aux_mv   = x.pm2_present ? (uint16_t)(x.pm2_voltage * 1000.0f) : 0;
                    t.curr_ca  = (int16_t)(x.curr_a * 100.0f);
                    // INT8_MIN (-128) = "no data". The bare cast wrapped a corrupt -160 C to
                    // +96 C -- it manufactured a plausible-looking number out of a fault,
                    // which is the worst possible way to fail. The wire layout is unchanged,
                    // so an un-reflashed ground station shows -128: degraded, not wrong.
                    {
                        const bool fresh = x.baro_valid && (x.baro_stamp_ms != 0) &&
                                           (millis() - x.baro_stamp_ms < DEPTH_STALE_MS);
                        const float wt = x.water_temp_c;
                        t.wtemp_c = (fresh && wt > -127.0f && wt < 127.0f)
                                        ? (int8_t)wt : (int8_t)INT8_MIN;
                    }
                    if (x.leak)        t.flags |= LT_FLAG_LEAK;
                    if (x.kill_switch) t.flags |= LT_FLAG_KILL;
                }
            }
            // LAST-GOOD CACHE for the two locks whose fields the ground station displays as
            // vehicle STATE. `LoraTelem t = {}` plus `if (lk.ok())` with no `else` meant a
            // contended lock shipped a frame asserting mode = 0 (STABILIZE) and DISARMED --
            // with a VALID CRC, so nothing downstream could reject it, and the ground
            // station's modeIsKnown() guard cannot help because 0 is a legal mode. That is a
            // one-frame lie about the arm state of a 20 kg vehicle.
            //
            // Same shape as the s_rpm_last block in mav_stream::snapshot(), which exists for
            // exactly this class of bug. Timeouts raised 2 -> 5 ms here as well; mtx_sensors
            // stays at 2 ms because task_sensor_read documents its duty-cycle sensitivity.
            static uint8_t s_mode_last = 0;
            static uint8_t s_flags_last = 0;          // ARMED bit only
            static int16_t s_rpm_last[8] = {0};
            static uint8_t s_thrflag_last = 0;
            static bool    s_have_control = false;
            {
                StateLock lk(g_state.mtx_control, pdMS_TO_TICKS(5));
                if (lk.ok()) {
                    s_mode_last  = (uint8_t)g_state.control.mode;
                    s_flags_last = g_state.control.armed ? LT_FLAG_ARMED : 0;
                    s_have_control = true;
                }
            }
            // If the control lock has NEVER been taken since boot there is no honest value to
            // send. Guard the SEND only -- NOT with `continue`, which would also skip the SD
            // log, the ESP-NOW poll, the TDM receive window and the config-plane relay below.
            // At ~8 Hz one skipped frame costs nothing; inventing "STABILIZE, disarmed" costs
            // the operator's trust in the arm indicator.
            t.mode   = s_mode_last;
            t.flags |= s_flags_last;
            {
                StateLock lk(g_state.mtx_thrusters, pdMS_TO_TICKS(5));
                if (lk.ok()) {
                    for (int i = 0; i < 8 && i < NUM_THRUSTERS; ++i)
                        s_rpm_last[i] = g_state.thrusters.rpm[i];
                    s_thrflag_last = g_state.thrusters.link_ok ? LT_FLAG_THR_LINK : 0;
                }
            }
            for (int i = 0; i < 8 && i < NUM_THRUSTERS; ++i) t.rpm[i] = s_rpm_last[i];
            t.flags |= s_thrflag_last;
            t.ul_rx = s_ul_rx;
            t.crc = lora_telem_crc(&t);
            if (s_have_control) lora_mission::sendTelemetry(t);

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
        // begin() was retried every loop (20 Hz) and its failure never reported, so a radio
        // that could not initialise produced exactly the same symptom as one with nothing
        // transmitting to it: a permanent "--". Back off to 2 s and say so on the first
        // failure and every ~30 s after, so the OLED "--" always has an explanation.
        if (!espnow_started && params::espnowWanted() && (now - espnow_try_ms >= 2000)) {
            espnow_try_ms = now;
            espnow_started = espnow_link::begin();
            if (espnow_started) {
                mav_stream::queueStatusText(MAV_SEVERITY_INFO, "ESP-NOW started");
                espnow_fail_n = 0;
            } else if (espnow_fail_n++ % 15 == 0) {
                mav_stream::queueStatusText(MAV_SEVERITY_ERROR,
                                           "ESP-NOW init failed - retrying");
            }
        }
        if (espnow_started) {
            bool kill, fresh; float aux_v;
            espnow_link::poll(kill, aux_v, fresh);
            {
                StateLock lk(g_state.mtx_sensors, pdMS_TO_TICKS(2));
                if (lk.ok()) {
                    g_state.sensors.kill_switch = kill;
                    // Publish freshness, not just the value. This is the THRUSTER battery and
                    // it feeds the low-battery failsafe and the mixer's voltage compensation —
                    // consuming a silently-held stale reading would be unsafe.
                    if (fresh) {
                        // PM2_VMULT trims the 2nd board's reported volts (1.0 = take it as
                        // sent). Applied HERE, at the single ingest point, so the display,
                        // PM2, the low-thruster-battery failsafe, the mixer's voltage
                        // linearisation and the LoRa aux_mv field can never disagree about
                        // what the pack voltage is.
                        //
                        // Until this line existed PM2_VMULT was read by NOTHING — the
                        // parameter was in the table, echoed on set and stored in NVS, so it
                        // looked live while changing it could not affect any reading. That is
                        // the "I change it and see no effect" half of the report.
                        const float trim = (g_params.pm2_vmult > 0.0f) ? g_params.pm2_vmult : 1.0f;
                        g_state.sensors.aux_voltage  = aux_v * trim;
                        g_state.sensors.aux_stamp_ms = millis();
                    }
                    g_state.sensors.aux_valid = fresh;
                }
            }
            // Announce the link edges. Losing this link degrades SILENTLY and safely: the
            // mixer's voltage compensation stops and the low-battery failsafe goes inert
            // again, so nothing misbehaves — but the pilot has no way to know the
            // compensation they tuned around has switched itself off. Same edge-triggered
            // pattern as the IMU lost/recovered and Pico link-flap notices.
            static bool s_thr_link_prev = false;
            if (fresh != s_thr_link_prev) {
                s_thr_link_prev = fresh;
                char b[64];
                if (fresh) snprintf(b, sizeof(b), "Thruster pack link up (%.1f V)", aux_v);
                else       snprintf(b, sizeof(b), "Thruster pack link LOST - voltage comp off");
                mav_stream::queueStatusText(fresh ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING, b);
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
