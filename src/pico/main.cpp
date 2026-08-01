// =============================================================================
//  pico_thruster — RP2350 (Pico 2) 8-channel bidirectional-DShot + RPM co-proc
//
//  Stage 1 (this file): a pure DShot + telemetry BRIDGE.
//    - core0: drive 8 bidirectional-DShot ESCs (PIO) and read back eRPM
//    - core1: UART link to the ESP32 (Serial1 @ 1 Mbaud) — parse commands, reply
//             with telemetry, using the shared `thruster_link_proto.h` framing.
//  The ESP32 computes the 8 DShot 3D values (its normal mixer output) and sends
//  them; this Pico just outputs them and returns RPM. Stage 2 will add a per-
//  motor closed-loop RPM controller here.
//
//  Failsafes (layered):
//    1. UART link timeout -> all thrusters to STOP within LINK_TIMEOUT_MS.
//    2. Hardware watchdog (rp2040.wdt) -> chip reset if core0 stalls.
//    3. Intrinsic ESC timeout (bidir DShot ESC stops if sendThrottle < 500 Hz).
//    + optional hardware e-stop line from the ESP32.
//
//  NOTE: this compiles against `bastian2001/pico-bidir-dshot`. The telemetry
//  enum names below (ERPM / NO_PACKET / CHECKSUM_ERROR) follow that library's
//  documented API; if a pinned version differs, adjust the two enum references.
//  (The ESP32 side of the link builds and is verified separately; this Pico
//  firmware must be built/flashed on the Pico 2 by the user.)
// =============================================================================

#include <Arduino.h>
#include <PIO_DShot.h>
#include "hardware/pio.h"
#include "pico/mutex.h"
#include "thruster_link_proto.h"

// ---- board config (edit to match your wiring / motors) ----------------------
// ESC1..8 signal → Pico GP6..GP13 (4 on PIO0, 4 on PIO1).
static const uint8_t  DSHOT_PIN[TL_NUM_THRUSTERS] = { 6, 7, 8, 9,  10, 11, 12, 13 };
static const uint32_t DSHOT_SPEED_KBAUD = 300;   // DShot300 — margin for sub wiring
static const uint32_t LINK_BAUD         = 1000000;   // MUST match ESP32 PICO_LINK_BAUD (1 Mbaud)
static const int      PIN_ESTOP         = 15;    // input from ESP32: HIGH = hard stop
static const uint32_t LINK_TIMEOUT_MS   = 150;   // no valid frame this long -> stop
// motor poles/2. VERIFIED CORRECT for the Blue Robotics T200: Blue Robotics state the
// T200 has 14 poles ("runs at about 3800RPM at full throttle. Electrical RPM is thus
// 3800*7=22600"). Mechanical RPM = eRPM / THR_POLE_PAIRS — only change this for a
// different motor, and count the bell magnets to confirm.
static const uint8_t  THR_POLE_PAIRS    = 7;

// Onboard LED (no power LED on the Pico 2) — power + status indicator.
static const int      PIN_STATUS_LED    = LED_BUILTIN;

// ---- closed-loop RPM (Stage 2) — best control: feedforward + PI + dynamic idle + slew ----
// Tunable at runtime via the tl_cfg_t frame (MOTOR_TUNE writes them on the ESP32 side).
static float          g_rpm_max   = 4000.0f;  // |target| that maps to full throttle
static float          g_rpm_kp    = 0.60f;    // PI proportional (per normalized error)
static float          g_rpm_ki    = 0.02f;    // PI integral step per cycle
static float          g_rpm_ff_a  = 0.00025f; // feedforward: norm-throttle per rpm (~1/max)
static float          g_rpm_idle  = 0.0f;     // dynamic-idle target rpm (0 = stop when centred)
static float          g_rpm_filt  = 0.30f;    // measured-RPM IIR alpha (0..1, higher=faster)
static float          g_rpm_slew  = 0.0f;     // max output-level change per cycle (0 = off)
static uint8_t        g_rpm_loop  = 1;        // 0 = open-loop (feedforward) · 1 = closed-loop RPM (PI)
// Centre deadband and break-away floor. Both were compile-time constants (150 rpm and
// 0.10), which together threw away the bottom ~10 % of the thrust range and needed a
// rebuild to retune. They now arrive in the cfg frame from RPM_MIN_TGT and MOT_SPIN_MIN.
// These initialisers are only the pre-first-cfg fallback.
static float g_rpm_min_target = 30.0f;    // below this |target| -> idle/stop
static float g_rpm_spin_min   = 0.02f;    // min |level| once a target is commanded
static const int16_t  RPM_FAULT_RPM     = 100;     // "not spinning" threshold for a fault
static const uint32_t RPM_FAULT_MS      = 300;     // sustained no-spin under command -> fault
static const uint32_t PRESENT_TIMEOUT_MS = 500;    // no telemetry this long -> "not detected"
// Fixed output/telemetry cadence. This period is ALSO the settle time the ESC gets to
// answer, because bidir telemetry is read at the top of the NEXT cycle (see loop()).
// DShot300 needs ~200 us for frame + reply, and the library wants sends > 500 Hz.
static const uint32_t OUT_PERIOD_US = 500;         // 2 kHz

// ---- DShot instances (created + used only on core0) -------------------------
// Two output modes, user-selectable at runtime (DSHOT_BIDIR param):
//   bidir (default) → BidirDShotX1 per channel, inverted DShot + RPM telemetry (RPM
//                     loop, detection, constant-distance). Needs a bidir-capable ESC.
//   normal          → DShotX4 (4 channels/PIO), standard DShot, NO telemetry — runs any
//                     DShot ESC (e.g. stock BLHeli_S) so you can spin/bench them.
static BidirDShotX1* s_esc[TL_NUM_THRUSTERS] = {0};   // used in bidir mode
static DShotX4*      s_esc4[2]               = {0};   // used in normal mode (pio0 GP6-9, pio1 GP10-13)
static bool          s_bidir        = true;           // current output mode (core0)
static volatile uint8_t s_desired_bidir = 1;          // requested mode (set by core1 from cfg)

// ---- per-motor closed-loop state (core0 only) -------------------------------
static float    s_integ[TL_NUM_THRUSTERS]    = {0};   // PI integrator (normalized throttle)
static float    s_prev_lvl[TL_NUM_THRUSTERS] = {0};   // last output level (for the slew limit)
static float    s_filt_rpm[TL_NUM_THRUSTERS] = {0};   // filtered signed RPM
static uint32_t s_nospin_ms[TL_NUM_THRUSTERS] = {0};  // start of a suspected no-spin
static uint32_t s_last_tlm_ms[TL_NUM_THRUSTERS] = {0}; // last valid telemetry -> "present"
static int      s_prev_dir[TL_NUM_THRUSTERS] = {1};   // direction of the frame we last SENT
// Telemetry-type tallies since the last debug print — makes a bench failure self-diagnosing:
// none counting up = nothing coming back (wiring/power/mode); crc = signal integrity;
// edt but no erpm = ESC answering with Extended DShot Telemetry only.
static uint16_t s_tc_erpm[TL_NUM_THRUSTERS] = {0};
static uint16_t s_tc_edt[TL_NUM_THRUSTERS]  = {0};
static uint16_t s_tc_crc[TL_NUM_THRUSTERS]  = {0};
static uint16_t s_tc_none[TL_NUM_THRUSTERS] = {0};

// ESC beacon beep (Pixhawk-style chirp). DShot command 1..5 = BLHeli beacon tones;
// sendRaw11Bit() sets the telemetry bit as commands require. Only while disarmed.
static const uint16_t BEACON_CMD = 2;   // DShot beacon tone #2 (BLHeli)
static uint32_t s_beep_end_ms = 0;
static bool     s_beep_prev   = false;

// ---- cross-core shared state (guarded by an auto-initialised mutex) ---------
auto_init_mutex(s_mtx);
static volatile uint32_t s_last_cmd_ms = 0;
struct Shared {
    int16_t cmd[TL_NUM_THRUSTERS];      // raw DShot 3D value (0 | 48..2047) from ESP32
    uint8_t flags;
    uint8_t seq;
    int16_t rpm[TL_NUM_THRUSTERS];      // signed mechanical RPM back to ESP32
    uint8_t status[TL_NUM_THRUSTERS];
    uint8_t fault_mask;
};
static Shared s_sh = {};

// Map our protocol raw DShot value (0 stop | 48..2047 in 3D bands) -> the
// library's sendThrottle() range (0 stop | 1..2000, it internally adds 47).
static inline uint16_t rawToThrottle(int16_t d) {
    if (d <= 0) return 0;                 // stop
    if (d < 48)   d = 48;
    if (d > 2047) d = 2047;
    int v = (int)d - 47;                  // library re-adds 47 -> original DShot cmd
    if (v < 1)    v = 1;
    if (v > 2000) v = 2000;
    return (uint16_t)v;
}

// Bidirectional DShot reports an UNSIGNED eRPM magnitude, so the sign has to come from
// the direction we are driving. Flipping it the instant the command reverses is wrong:
// the prop has inertia and is still turning the old way, so the reported speed would jump
// by 2x in one cycle (e.g. +900 -> -900), which a controller sees as a huge fake error.
// Only adopt the new sign once the wheel has actually slowed through zero.
static const int32_t RPM_SIGN_FLIP_MAX = 60;      // below this the prop may have reversed
static int8_t s_rpm_sign[TL_NUM_THRUSTERS] = {1}; // last reported sign, per motor

static inline int16_t signedRpm(int idx, int dir, uint32_t erpm) {
    int32_t mag = (int32_t)(erpm / THR_POLE_PAIRS);
    if (mag > 32000) mag = 32000;
    int8_t want = (dir < 0) ? -1 : 1;
    if (mag < RPM_SIGN_FLIP_MAX) s_rpm_sign[idx] = want;   // slow enough to have flipped
    return (s_rpm_sign[idx] < 0) ? (int16_t)(-mag) : (int16_t)mag;
}

// A signed throttle level (-1..1) -> raw DShot 3D value (0 | 48..2047), 1048 = neutral.
// BOTH bands run low->high WITHIN themselves: forward 1049 (min) .. 2047 (max), reverse
// 48 (min) .. 1047 (max) — the reverse band is NOT mirrored about the 1048 neutral gap.
// Mapping reverse backwards makes a hair of negative demand ~full reverse from standstill:
// the ESC stalls, the loop backs off, and it slams again -> wobble, never a real spin.
static inline int16_t levelToDshotRaw(float lv) {
    if (lv >  1.0f) lv =  1.0f;
    if (lv < -1.0f) lv = -1.0f;
    if (lv > -0.003f && lv < 0.003f) return 1048;             // neutral / stop
    if (lv > 0.0f) return (int16_t)(1049.0f + lv * (2047.0f - 1049.0f));   // fwd: 1049..2047
    return (int16_t)(48.0f - lv * (1047.0f - 48.0f));                      // rev: 48..1047 (lv<0)
}

// (Re)create the DShot drivers for the given mode. Tears down the current set first
// (both classes free their PIO/pin on delete). Fast (<1 ms); called at boot and when
// the user switches DSHOT_BIDIR (only while disarmed).
static void initDrivers(bool bidir) {
    for (int i = 0; i < TL_NUM_THRUSTERS; i++) if (s_esc[i])  { delete s_esc[i];  s_esc[i]  = nullptr; }
    for (int g = 0; g < 2; g++)               if (s_esc4[g]) { delete s_esc4[g]; s_esc4[g] = nullptr; }
    if (bidir) {
        for (int i = 0; i < 4; i++) s_esc[i]   = new BidirDShotX1(DSHOT_PIN[i],   DSHOT_SPEED_KBAUD, pio0);
        for (int i = 0; i < 4; i++) s_esc[4+i] = new BidirDShotX1(DSHOT_PIN[4+i], DSHOT_SPEED_KBAUD, pio1);
    } else {
        s_esc4[0] = new DShotX4(DSHOT_PIN[0], 4, DSHOT_SPEED_KBAUD, pio0);   // GP6-9
        s_esc4[1] = new DShotX4(DSHOT_PIN[4], 4, DSHOT_SPEED_KBAUD, pio1);   // GP10-13
    }
    s_bidir = bidir;
}

// ================= core0: DShot output + eRPM telemetry ======================
void setup() {
    Serial.begin(115200);                          // USB debug
    pinMode(PIN_ESTOP, INPUT_PULLDOWN);            // floats LOW = run; ESP32 HIGH = stop
    initDrivers(true);                             // bidir by default (unchanged behaviour)
    rp2040.wdt_begin(250);                          // 250 ms hardware watchdog (margin vs a
                                                    // loop() spike spuriously resetting the Pico
                                                    // → a "SILENT" link gap. Still << the ESP32's
                                                    // 500 ms link timeout.)
}

void loop() {
    // Run the output + telemetry + control loop at a FIXED cadence. This used to free-run,
    // which (a) gave the ESC no time to answer before we read telemetry and (b) left
    // g_rpm_ki ("integral step per cycle") without a defined meaning. The period doubles as
    // the ESC's reply window, since telemetry is read at the top of the next cycle.
    static uint32_t next_out_us = 0;
    if ((int32_t)(micros() - next_out_us) < 0) { rp2040.wdt_reset(); return; }
    next_out_us = micros() + OUT_PERIOD_US;

    // Snapshot the latest command.
    int16_t cmd[TL_NUM_THRUSTERS];
    uint8_t flags;
    mutex_enter_blocking(&s_mtx);
    memcpy(cmd, s_sh.cmd, sizeof(cmd));
    flags = s_sh.flags;
    mutex_exit(&s_mtx);

    uint32_t now = millis();
    bool link_ok  = (now - s_last_cmd_ms) < LINK_TIMEOUT_MS && s_last_cmd_ms != 0;
    bool estop    = (digitalRead(PIN_ESTOP) == HIGH) || (flags & TL_FLAG_ESTOP);
    bool armed    = (flags & TL_FLAG_ARMED) && link_ok && !estop;
    bool rpm_mode = (flags & TL_FLAG_MODE_RPM);

    // Latch a beacon-beep request (rising edge, disarmed only) into a short window.
    bool beep_flag = (flags & TL_FLAG_BEEP);
    if (beep_flag && !s_beep_prev && !armed) s_beep_end_ms = now + 140;
    s_beep_prev = beep_flag;
    bool beeping = (now < s_beep_end_ms) && !armed;

    // Apply a DSHOT_BIDIR mode switch only while DISARMED (recreates the PIO drivers —
    // never mid-flight). The user picks the mode in Bondor; it arrives via the cfg frame.
    if (!armed && (bool)s_desired_bidir != s_bidir) initDrivers((bool)s_desired_bidir);

    int16_t rpm[TL_NUM_THRUSTERS];
    uint8_t status[TL_NUM_THRUSTERS];
    uint8_t fault = 0;

    // ---- decide the raw DShot 3D value per channel (mode-independent) ----
    int16_t rv[TL_NUM_THRUSTERS];       // 3D DShot raw: 0 stop | 48..2047 | -1 = beacon
    int     dr[TL_NUM_THRUSTERS];       // commanded direction (telemetry sign)
    for (int i = 0; i < TL_NUM_THRUSTERS; i++) {
        if (beeping) { rv[i] = -1; dr[i] = 1; s_nospin_ms[i] = 0; continue; }
        int16_t rawval;
        int     dir;
        if (!armed) {                                   // disarm/link-loss/estop -> stop
            rawval = 0; dir = 1; s_integ[i] = 0;
        } else if (rpm_mode) {                          // Stage 2: FF + PI + dynamic idle + slew
            int16_t target = cmd[i];                    // signed target RPM
            bool centred = (fabsf((float)target) < g_rpm_min_target);
            if (centred && g_rpm_idle >= 1.0f) target = (int16_t)g_rpm_idle;   // dynamic idle (fwd)
            if (centred && g_rpm_idle < 1.0f) {
                rawval = 1048; dir = 1; s_integ[i] = 0; s_prev_lvl[i] = 0;     // stop at neutral
            } else {
                // Loop mode is the USER's choice (RPM_LOOP param), not an auto-fallback:
                //   g_rpm_loop==1 → closed-loop RPM (FF + PI); needs bidir-DShot telemetry.
                //   Anti-windup: integrate only on a fresh measurement, so a missing sample
                //   can't wind up to full. g_rpm_loop==0 → open-loop feedforward (any ESC).
                bool has_tlm = (s_last_tlm_ms[i] != 0) && (now - s_last_tlm_ms[i] < PRESENT_TIMEOUT_MS);
                float lvl = g_rpm_ff_a * (float)target;                       // feedforward (both modes)
                if (g_rpm_loop) {
                    float err = ((float)target - s_filt_rpm[i]) / g_rpm_max;  // normalized error
                    if (has_tlm) {                                            // integrate only on a real measurement
                        s_integ[i] += g_rpm_ki * err;
                        if (s_integ[i] >  1.0f) s_integ[i] =  1.0f;
                        if (s_integ[i] < -1.0f) s_integ[i] = -1.0f;
                    }
                    lvl += g_rpm_kp * err + s_integ[i];                       // + PI
                } else {
                    s_integ[i] = 0;                                           // open-loop: no integrator
                }
                lvl = constrain(lvl, -1.0f, 1.0f);
                if (g_rpm_slew > 0.0f) {                                       // slew limit
                    float d = lvl - s_prev_lvl[i];
                    if (d >  g_rpm_slew) lvl = s_prev_lvl[i] + g_rpm_slew;
                    if (d < -g_rpm_slew) lvl = s_prev_lvl[i] - g_rpm_slew;
                }
                // Break-away floor. Unlike the raw path (mixer::oneToDshot applies
                // MOT_SPIN_MIN), closed-loop RPM has no minimum, so a small target asks for
                // a level far below what it takes to actually start a loaded thruster: the
                // motor never turns, no RPM comes back, and the integrator winds up and then
                // lurches. Push any non-centred demand to at least g_rpm_spin_min, keeping
                // the sign, so the motor always breaks away and the loop then trims it
                // down. Keep this just above the ESC's own break-away (Bluejay's Minimum
                // Startup Power) — a large floor here throws away the usable low end.
                if (g_rpm_spin_min > 0.0f &&
                    lvl > -g_rpm_spin_min && lvl < g_rpm_spin_min)
                    lvl = (target < 0) ? -g_rpm_spin_min : g_rpm_spin_min;

                s_prev_lvl[i] = lvl;
                rawval = levelToDshotRaw(lvl);
                // Sign the telemetry by the direction we are ACTUALLY driving (lvl), not
                // by the target. When the PI drives lvl negative to brake a positive
                // target, the ESC really does run in reverse — labelling that measurement
                // positive made the error grow instead of shrink, i.e. a runaway.
                dir = (lvl < 0.0f) ? -1 : 1;
            }
        } else {                                        // Stage 1: raw DShot passthrough
            rawval = cmd[i];
            dir = (rawval >= 48 && rawval <= 1047) ? -1 : 1;
        }
        rv[i] = rawval; dr[i] = dir;
    }

    // ---- output + telemetry, per mode ----
    if (s_bidir) {
        for (int i = 0; i < TL_NUM_THRUSTERS; i++) {
            status[i] = 0;

            // ---- 1. READ the answer to the frame we sent LAST cycle ----------------
            // This MUST come before this cycle's send: the ESC only begins replying ~30 us
            // after the frame ends, so reading straight after a send always yields
            // NO_PACKET and the ESC would never be seen as present.
            // "ESC is answering" and "we got an RPM" are different things — any valid
            // frame type proves the ESC is alive (Bluejay interleaves Extended DShot
            // Telemetry frames), but only ERPM updates the measured speed.
            uint32_t tval = 0;
            switch (s_esc[i]->getTelemetryPacket(&tval)) {
            case BidirDshotTelemetryType::ERPM: {
                int16_t meas = signedRpm(i, s_prev_dir[i], tval);   // sign from the frame it answers
                s_filt_rpm[i] += g_rpm_filt * ((float)meas - s_filt_rpm[i]);
                status[i] |= TL_ST_RPM_VALID;
                s_last_tlm_ms[i] = now;
                s_tc_erpm[i]++;
                break;
            }
            case BidirDshotTelemetryType::VOLTAGE:
            case BidirDshotTelemetryType::CURRENT:
            case BidirDshotTelemetryType::TEMPERATURE:
            case BidirDshotTelemetryType::STATUS:
            case BidirDshotTelemetryType::STRESS:
            case BidirDshotTelemetryType::DEBUG_FRAME_1:
            case BidirDshotTelemetryType::DEBUG_FRAME_2:
                s_last_tlm_ms[i] = now;                 // not RPM, but the ESC IS answering
                s_tc_edt[i]++;
                break;
            case BidirDshotTelemetryType::CHECKSUM_ERROR:
                s_tc_crc[i]++;                          // answered but corrupt — don't trust it
                break;
            default:                                    // NO_PACKET / OTHER_VALUE
                s_tc_none[i]++;
                break;
            }
            rpm[i] = (int16_t)s_filt_rpm[i];

            // ---- 2. SEND this cycle's frame ---------------------------------------
            if (rv[i] == -1) {                          // beacon chirp
                s_esc[i]->sendRaw11Bit(BEACON_CMD);
                s_prev_dir[i] = 1;
                continue;
            }
            s_esc[i]->sendThrottle(rawToThrottle(rv[i]));
            s_prev_dir[i] = dr[i];

            bool present = (s_last_tlm_ms[i] != 0) && (now - s_last_tlm_ms[i] < PRESENT_TIMEOUT_MS);
            if (present) status[i] |= TL_ST_PRESENT;
            else {
                // s_filt_rpm only moves when an eRPM frame arrives, so a silent ESC would
                // otherwise keep reporting its last value forever ("rpm=182" for a motor we
                // have heard nothing from). Unknown must read as 0, not as stale data.
                s_filt_rpm[i] = 0.0f;
                rpm[i] = 0;
            }
            // fault: PRESENT but commanded to spin and effectively stopped.
            int16_t meas_mag = (int16_t)fabsf(s_filt_rpm[i]);
            // "Commanded" must mean REAL thrust, not merely off-neutral: while
            // stabilising, the mixer hands out tiny corrections that are below the ESC's
            // break-away, and calling those a stall produced constant false alarms.
            // Require a margin either side of the 1048 neutral gap.
            const int16_t STALL_MARGIN = 60;    // ~6% of a band
            bool commanded = armed && present && (rv[i] != 0) &&
                             (rv[i] > 1048 + STALL_MARGIN || rv[i] < 1048 - STALL_MARGIN);
            if (commanded && meas_mag < RPM_FAULT_RPM) {
                if (s_nospin_ms[i] == 0) s_nospin_ms[i] = now;
                else if (now - s_nospin_ms[i] > RPM_FAULT_MS) { fault |= (1 << i); status[i] |= TL_ST_ALERT; }
            } else {
                s_nospin_ms[i] = 0;
            }
        }
    } else {
        // Normal DShot: 4 channels per PIO, NO telemetry (present stays clear → "not
        // detected" is expected here; the motor still runs). raw12 = data<<1 | telem-bit.
        for (int g = 0; g < 2; g++) {
            uint16_t raw12[4];
            for (int j = 0; j < 4; j++) {
                int16_t r = rv[g * 4 + j];
                if (r == -1) raw12[j] = (uint16_t)((BEACON_CMD << 1) | 1);    // beacon cmd (telem bit set)
                else {
                    uint16_t dv = (r <= 0) ? 0 : (uint16_t)(r < 48 ? 48 : (r > 2047 ? 2047 : r));
                    raw12[j] = (uint16_t)(dv << 1);                           // throttle (telem bit 0)
                }
            }
            if (s_esc4[g]) s_esc4[g]->sendRaw12Bit(raw12);
        }
        for (int i = 0; i < TL_NUM_THRUSTERS; i++) { rpm[i] = 0; status[i] = 0; s_nospin_ms[i] = 0; }
    }

    // Publish telemetry for core1 to send.
    mutex_enter_blocking(&s_mtx);
    memcpy(s_sh.rpm, rpm, sizeof(rpm));
    memcpy(s_sh.status, status, sizeof(status));
    s_sh.fault_mask = fault;
    mutex_exit(&s_mtx);

    // --- USB debug (~2 Hz): open the Pico's USB serial @115200. Reading it:
    //     bidir=0        -> normal DShot, NO telemetry exists; "not detected" is expected.
    //                       Set the DSHOT_BIDIR param to 1 (applied while disarmed).
    //     e=<n>          -> eRPM frames decoded. Non-zero = everything works.
    //     d=<n>          -> Extended DShot Telemetry frames (ESC alive, just not RPM).
    //     c=<n>          -> replies arriving but failing checksum: signal integrity.
    //     n=<n> only     -> nothing coming back at all: wiring / common ground / ESC power.
    //     pres=1 rpm=0 with no motor attached is the correct healthy bench result. ---
    static uint32_t dbg_ms = 0;
    if (now - dbg_ms >= 500) {
        dbg_ms = now;
        Serial.printf("link=%d armed=%d bidir=%d rpm_mode=%d loop=%d | ",
                      link_ok, armed, s_bidir, rpm_mode, g_rpm_loop);
        for (int i = 0; i < TL_NUM_THRUSTERS; i++) {
            // cmd = what the ESP32 asked for (raw DShot in test/raw mode, target RPM in RPM
            // mode); out = the DShot value actually clocked out (-1 = beacon chirp).
            // A steady healthy `out` while the motor only judders means the firmware is
            // doing its job and the fault is in the ESC settings or motor wiring.
            Serial.printf("M%d[cmd=%d out=%d rpm=%d pres=%d e=%u d=%u c=%u n=%u] ",
                          i + 1, cmd[i], rv[i], rpm[i],
                          (status[i] & TL_ST_PRESENT) ? 1 : 0,
                          s_tc_erpm[i], s_tc_edt[i], s_tc_crc[i], s_tc_none[i]);
            s_tc_erpm[i] = s_tc_edt[i] = s_tc_crc[i] = s_tc_none[i] = 0;
        }
        Serial.println();
    }

    rp2040.wdt_reset();
}

// Onboard-LED status (the Pico 2 has no power LED). Patterns:
//   solid ON      = powered, link up, ARMED
//   heartbeat     = powered, link up, disarmed  (brief blink ~1 Hz)
//   slow blink    = powered but NO link from the ESP32 (0.5 Hz)
//   fast blink    = a thruster fault is latched (5 Hz)
static void updateStatusLed() {
    uint8_t flags, fault;
    mutex_enter_blocking(&s_mtx);
    flags = s_sh.flags; fault = s_sh.fault_mask;
    mutex_exit(&s_mtx);
    uint32_t now = millis();
    bool link_ok = (now - s_last_cmd_ms) < LINK_TIMEOUT_MS && s_last_cmd_ms != 0;
    bool armed   = (flags & TL_FLAG_ARMED);

    bool on;
    if (fault)          on = (now % 200) < 100;               // fast blink 5 Hz
    else if (!link_ok)  on = (now % 2000) < 1000;             // slow blink 0.5 Hz
    else if (armed)     on = true;                            // solid
    else                on = (now % 1000) < 80;               // heartbeat
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

// ================= core1: UART link to the ESP32 =============================
void setup1() {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);             // ON at boot = powered
    // The earlephilhower default Serial1 FIFO is only 32 B — at 1 Mbaud that overflows in
    // ~160 us, so any loop1() delay (mutex wait / blocking reply write) dropped commands →
    // the Pico stopped replying → ESP32 saw a telemetry gap ("link restored" flap). 1024 B
    // RX + TX gives plenty of headroom. Must be set BEFORE begin().
    Serial1.setFIFOSize(1024);
    Serial1.begin(LINK_BAUD);                       // GP0 = TX, GP1 = RX
}

void loop1() {
    static const uint16_t BUFCAP = sizeof(tl_cmd_t) > sizeof(tl_cfg_t) ? sizeof(tl_cmd_t)
                                                                       : sizeof(tl_cfg_t);
    static uint8_t  buf[BUFCAP];
    static uint16_t idx = 0;
    static uint16_t expect = 0;      // frame length once the type is known
    static bool     is_cfg = false;  // cmd (0x55) vs cfg (0x57) frame
    static uint32_t last_tlm_ms = 0;

    updateStatusLed();

    // --- Read commands + config (no reply here). Both frames share magic0=0xAA; the 2nd
    //     magic byte selects type + length (cmd 0x55, cfg 0x57). ---
    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        if (idx == 0)      { if (b == TL_MAGIC_CMD_0) buf[idx++] = b; }   // 0xAA (both types)
        else if (idx == 1) {
            if      (b == TL_MAGIC_CMD_1) { buf[idx++] = b; expect = sizeof(tl_cmd_t); is_cfg = false; }
            else if (b == TL_MAGIC_CFG_1) { buf[idx++] = b; expect = sizeof(tl_cfg_t); is_cfg = true;  }
            else idx = 0;
        } else {
            buf[idx++] = b;
            if (idx >= expect) {
                idx = 0;
                if (!is_cfg) {
                    tl_cmd_t c; memcpy(&c, buf, sizeof(c));
                    if (c.crc != tl_cmd_crc(&c)) continue;   // corrupt -> drop, resync
                    mutex_enter_blocking(&s_mtx);
                    memcpy(s_sh.cmd, c.cmd, sizeof(c.cmd));
                    s_sh.flags = c.flags;
                    s_sh.seq   = c.seq;
                    mutex_exit(&s_mtx);
                    s_last_cmd_ms = millis();
                } else {
                    tl_cfg_t cfg; memcpy(&cfg, buf, sizeof(cfg));
                    if (cfg.crc != tl_cfg_crc(&cfg)) continue;
                    g_rpm_kp = cfg.kp; g_rpm_ki = cfg.ki; g_rpm_ff_a = cfg.ff_a;
                    g_rpm_idle = cfg.idle_rpm; g_rpm_filt = cfg.filt; g_rpm_slew = cfg.slew;
                    g_rpm_loop = cfg.loop_mode;
                    s_desired_bidir = cfg.dshot_bidir;   // core0 applies it (disarmed only)
                    if (cfg.max_rpm > 1.0f) g_rpm_max = cfg.max_rpm;
                    // Low-end shaping (MOT_SPIN_MIN / RPM_MIN_TGT). Clamp: a bad value
                    // here would either make the thrusters creep at centre or swallow the
                    // whole low end, so keep both inside sane bounds.
                    if (cfg.spin_min >= 0.0f && cfg.spin_min <= 0.5f)
                        g_rpm_spin_min = cfg.spin_min;
                    if (cfg.min_target_rpm >= 0.0f && cfg.min_target_rpm <= 1000.0f)
                        g_rpm_min_target = cfg.min_target_rpm;
                }
            }
        }
    }

    // --- Stream telemetry on a timer (~200 Hz), DECOUPLED from commands ---
    // A brief ESP32 hiccup (skipped command / late poll) no longer stops telemetry, so the
    // link can't flap on an ESP32-side stall. Only a real Pico stop can end the stream.
    uint32_t now = millis();
    if (now - last_tlm_ms >= 5) {
        last_tlm_ms = now;
        tl_tlm_t t;
        t.magic0 = TL_MAGIC_TLM_0;
        t.magic1 = TL_MAGIC_TLM_1;
        mutex_enter_blocking(&s_mtx);
        t.seq        = s_sh.seq;                       // last accepted command's seq
        memcpy(t.rpm, s_sh.rpm, sizeof(t.rpm));
        memcpy(t.status, s_sh.status, sizeof(t.status));
        t.fault_mask = s_sh.fault_mask;
        mutex_exit(&s_mtx);
        t.uptime_ms  = now;                            // Pico uptime (reset detector)
        t.crc = tl_tlm_crc(&t);
        Serial1.write((const uint8_t*)&t, sizeof(t));
    }
}
