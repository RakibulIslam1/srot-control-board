#pragma once

// =============================================================================
//  HENGLA — HARDWARE & TUNING CONFIGURATION
//  MAVLink-native AUV/ROV flight-control firmware for the SROT control board.
//  ESP32 DevKit V1 + RP2350 thruster co-processor | dual-core FreeRTOS | 500 Hz.
//
//  This is the SINGLE source of truth for pin assignments, bus parameters,
//  FreeRTOS task allocation, and default tuning constants.
//  Nothing in the firmware should hardcode a pin or bus — it lives here.
// =============================================================================

// -----------------------------------------------------------------------------
// SECTION 0 — IDENTITY
//
//  The BOARD is "SROT". The FIRMWARE is "Hengla". Both names are load-bearing and
//  they are not interchangeable: SROT names the hardware (and the NVS namespaces,
//  the repos, and MAV_CMD_SROT_MOVE, all of which stay as they are), while Hengla
//  names this software.
//
//  Hengla is NOT ArduSub and does not pretend to be. The heartbeat reports
//  MAV_AUTOPILOT_GENERIC on a MAV_TYPE_SUBMARINE frame, so a GCS sees an honest
//  generic autopilot. The consequence is deliberate and accepted: QGroundControl's
//  and BlueOS's ArduPilot-gated setup pages stay empty, because they key off an
//  ArduPilot autopilot id. Bondor is the ground station. See docs/BLUEOS.md.
//
//  (Historical note: an APM_COMPAT_* block used to spoof "ArduSub V4.1.0" here to
//  unlock those QGC pages. It was already dead code — nothing referenced it — and
//  has been removed rather than left to mislead.)
//
//  Parameter IDs still follow the ArduPilot naming convention (ATC_*, MOT_*, FS_*).
//  That is descriptive, widely understood, and keeps existing .params backups
//  valid; it is not a claim of lineage. See PARAMETERS.md.
// -----------------------------------------------------------------------------
#define HENGLA_FW_NAME            "Hengla"
#define HENGLA_FW_VERSION_MAJOR   0
#define HENGLA_FW_VERSION_MINOR   2
#define HENGLA_FW_VERSION_PATCH   0
#define HENGLA_FW_VERSION_STR     "Hengla v0.2.0"
#define HENGLA_BUILD_DATE         __DATE__ " " __TIME__

// AUTOPILOT_VERSION identity fields.
#define HENGLA_MAV_VENDOR_ID      0x5352      // 'SR' — the SROT board
#define HENGLA_MAV_PRODUCT_ID     0x4847      // 'HG' — Hengla firmware
#define HENGLA_MAV_CUSTOM_VER     "HENGLA"    // ASCII → flight_custom_version[8] (max 8)

// packed 32-bit software version for AUTOPILOT_VERSION.flight_sw_version
#define HENGLA_FW_VERSION_PACKED  ( (uint32_t)HENGLA_FW_VERSION_MAJOR << 24 | \
                                    (uint32_t)HENGLA_FW_VERSION_MINOR << 16 | \
                                    (uint32_t)HENGLA_FW_VERSION_PATCH <<  8 )

// -----------------------------------------------------------------------------
// SECTION 1 — MAVLINK LINK IDENTITY
// -----------------------------------------------------------------------------
#define MAV_SYSTEM_ID           1           // this vehicle's system id
#define MAV_COMPONENT_ID        1           // MAV_COMP_ID_AUTOPILOT1
#define MAVLINK_SERIAL          Serial      // UART0 (TX0/RX0) to companion computer / BlueOS
#define MAVLINK_BAUD            115200

// -----------------------------------------------------------------------------
// SECTION 2 — HARDWARE PINS  (strictly mapped to avoid flash/boot conflicts)
//
//  ! STRAPPING-PIN WARNINGS (matches the actual assignments below) !
//    GPIO2  (WS2812B RGB) : strap must be LOW/floating at boot. WS2812B DIN idles LOW &
//                           is high-Z, so run-time drive is fine; never light at boot.
//    GPIO4  (BNO_INT)     : normal (non-strapping) GPIO — clean choice for the INT input.
//    GPIO0  : FREE (BOOT strap). Expansion pin in the default PICO backend; only a DShot
//             output (THR1) in the RMT fallback — keep hi-Z/HIGH at boot then.
//    GPIO12 (MTDI)        : MTDI selects flash voltage — MUST idle LOW at boot. FREE in the
//             PICO backend; a DShot output (THR2) only in RMT fallback (forced LOW at boot).
//    GPIO15 (MTDO)        : strapping pin; FREE in PICO backend, DShot output (THR5) in RMT.
//    GPIO34/35/36/39      : INPUT-ONLY (LoRa DIO0 / leak / batt-V / batt-I). Never outputs.
// -----------------------------------------------------------------------------

// --- Serial / MAVLink (UART0) ---
#define PIN_UART0_TX            1           // → companion computer / BlueOS
#define PIN_UART0_RX            3           // ← companion computer / BlueOS

// --- I2C Bus 0 : Primary navigation sensors (Wire, 400 kHz, serviced on Core 1) ---
#define PIN_I2C0_SDA            21          // BNO085 IMU + Bar30 depth
#define PIN_I2C0_SCL            22
// 400 kHz. Needed for the 400 Hz IMU (rotation + gyro) — at 200 kHz the dual-report
// SHTP reads saturate the bus. Reliability at this speed comes from the wired INT
// pin (INT-gated reads, see bno085.cpp) + the trimmed report set. If resets return
// (`rN` climbs), suspect wiring and drop back to 200000 (IMU then caps ~200-250 Hz).
#define I2C0_FREQ              400000

// --- I2C Bus 1 : Display & aux peripherals (Wire1, 100 kHz, serviced on Core 0) ---
#define PIN_I2C1_SDA            25          // OLED + PCA9685 servo/MOSFET expander
#define PIN_I2C1_SCL            26
#define I2C1_FREQ             100000

// --- SPI bus (VSPI) : LoRa radio + future SD card ---
#define PIN_SPI_SCK            18
#define PIN_SPI_MISO           19
#define PIN_SPI_MOSI           23
#define PIN_LORA_CS             5           // LoRa pulls high; GPIO5 high at boot OK
#define PIN_LORA_DIO0          34           // input-only — IRQ
#define PIN_LORA_RST           -1           // wired to ESP32 EN pin (no GPIO); LoRa reset = board reset
#define PIN_SD_CS              33            // SD chip select

// --- 8x DShot thruster outputs — RMT FALLBACK ONLY -------------------------------
// In the default THRUSTER_PICO backend the ESP32 drives NO thruster pins (all 8 ESCs
// hang off the Pico 2, GP6..13), so GPIO 0/12/13/14/15 are free for expansion sensors.
// These pins are used only if THRUSTER_BACKEND == THRUSTER_RMT. THR1 sits on GPIO0
// (was GPIO4, moved so BNO_INT can own GPIO4). GPIO0/12/15 are strapping pins — the ESC
// signal lines are hi-Z at boot so run-time DShot is fine.
#define PIN_THR1                0
#define PIN_THR2               12
#define PIN_THR3               13
#define PIN_THR4               14
#define PIN_THR5               15
#define PIN_THR6               16
#define PIN_THR7               17
#define PIN_THR8               27

#define NUM_THRUSTERS           8
// Aggregate array for iteration (index 0 == Thruster 1). Compile-time DEFAULT;
// runtime pins come from g_params.pin_thr[] (dshot_out fills its array at begin).
#define THRUSTER_PINS { PIN_THR1, PIN_THR2, PIN_THR3, PIN_THR4, \
                        PIN_THR5, PIN_THR6, PIN_THR7, PIN_THR8 }

// --- Status indicators & audio ---
#define PIN_WS2812B             2           // addressable RGB LED; <32 for bit-bang. GPIO2 strap: DIN is high-Z so run boot is fine.
#define PIN_BUZZER             32           // passive buzzer (LEDC tone)
#define WS2812B_COUNT           1

// --- Analog inputs (input-only ADC1) ---
#define PIN_LEAK               35           // digital leak input (LEAK_EN; needs external bias)
#define PIN_BATT_VOLT          36           // battery voltage ADC (PM1 when PM1_SRC=1)
#define PIN_BATT_CURR          39           // battery current ADC
#define LEAK_ACTIVE_HIGH        1           // 1: HIGH = leak detected
#define BATT_CURR_MULT          0.0f        // A per raw LSB (set once you have a current sensor)

// NOTE: the thruster kill switch is NOT a local input — its state arrives from
// the 2nd board (future ESP-NOW link) and is display-only (OLED). It never
// gates arming.

// -----------------------------------------------------------------------------
// SECTION 3 — BUS DEVICE ADDRESSES (no collisions across the two buses)
//   Bus0 (Wire) : BNO085 0x4A/0x4B, MS5837 0x76
//   Bus1 (Wire1): SH1106 0x3C, PCA9685 0x40
// -----------------------------------------------------------------------------
#define I2C_ADDR_BNO085         0x4A        // 0x4B if SA0/ADR strapped high
// BNO085 RESET is not wired (-1). INT (H_INTN) is on GPIO4: the Adafruit I2C API
// ignores INT (SPI-only), so the driver does its OWN INT-gated polling — it reads
// only when GPIO4 is LOW (packet ready), which is what makes 400 Hz over I2C
// reliable. GPIO4 is a normal (non-strapping) GPIO, so it never interferes with boot.
#define PIN_BNO_RESET           -1
#define PIN_BNO_INT              4
// Swap the BNO085 roll and pitch axes (and their gyro rates) to match how the
// sensor is physically mounted on this board.
#define BNO_SWAP_ROLL_PITCH     1
// Enable the BNO085's calibrated-magnetic-field REPORT. This does NOT put the mag
// into the attitude fusion — attitude is always the 6-axis GAME rotation vector, so
// thruster/metal fields can never move roll/pitch/yaw. It only makes the raw field
// vector available to (a) the MAG calibration routine and (b) control/yaw_ref, which
// reads it ONCE at boot to give yaw an absolute earth reference (MAG_YAW_REF).
// With this 0, yaw is purely RELATIVE: it holds short-term and drifts slowly, and the
// pilot steers heading. Costs one extra 50 Hz report on the INT-gated SHTP link.
#define BNO_USE_MAG             1
#define I2C_ADDR_BAR30          0x76        // MS5837
#define I2C_ADDR_OLED           0x3C        // SH1106
#define OLED_DEPTH_FULL         10.0f       // depth (m) that fills the OLED depth bucket bar
#define I2C_ADDR_PCA9685        0x40

// -----------------------------------------------------------------------------
// SECTION 4 — DSHOT
// 8 thrusters on RMT ch 0..7. DShot150 chosen for signal integrity over long
// tether-side wiring; bump to 300/600 later if latency demands it.
// Throttle range: 0 = disarmed/idle, 48..2047 = active (per DShot spec).
// -----------------------------------------------------------------------------
#define DSHOT_MODE_150          1           // informational; DShotRMT selects DSHOT150
#define DSHOT_THROTTLE_MIN      48
#define DSHOT_THROTTLE_MAX      2047
#define DSHOT_DISARMED          0

// --- Thruster backend --------------------------------------------------------
// THRUSTER_RMT  = on-board ESP32 RMT DShot on PIN_THR1..8 (standalone, no RPM).
// THRUSTER_PICO = offload to a Pico 2 co-processor over UART2: the ESP32 sends the
//   8 DShot values it already computes and gets per-thruster RPM back (bidir DShot
//   on the Pico). Frees all 8 RMT channels; see shared/thruster_link_proto.h.
#define THRUSTER_RMT            0
#define THRUSTER_PICO          1
#define THRUSTER_BACKEND       THRUSTER_PICO

// ESP32 <-> Pico 2 link (UART2; pins are freed former-DShot GPIOs, non-strapping).
#define PIN_PICO_TX            17           // ESP32 UART2 TX -> Pico GP1 (Serial1 RX)
#define PIN_PICO_RX            16           // ESP32 UART2 RX <- Pico GP0 (Serial1 TX)
#define PIN_PICO_ESTOP         27           // ESP32 -> Pico GP15: HIGH = hard-stop
#define PICO_LINK_BAUD         1000000      // 1 Mbaud (was 2M): more signal-integrity margin on
                                            // the 2-board wired link; still <20% utilised. MUST
                                            // match the Pico's LINK_BAUD. Re-flash BOTH on change.
#define PICO_LINK_TIMEOUT_MS   500          // no telemetry this long -> link flagged down.
// 500 (was 200) rides through a brief Pico blip (e.g. a 5 V dip that resets the Pico) so it
// doesn't flap the status/log. Safety is unaffected: the Pico self-stops its thrusters on its
// own 150 ms link timeout — this only governs the ESP32's status/failsafe view.

// Thruster command mode. THIS IS ONLY THE BOOT DEFAULT for the RPM_LOOP parameter — the
// live switch is `RPM_LOOP`, editable from the GCS, and both the ESP32's command mode and
// the Pico's loop enable now read that one param. (They used to disagree: this #define
// gated the ESP32 while RPM_LOOP gated the Pico, so setting RPM_LOOP=1 silently did
// nothing on this side.)
//   0 = raw DShot (stabilisation drives throttle directly)
//   1 = send signed target RPM and let the Pico close a per-motor RPM loop
//
// **0 is the recommended default.** Putting an RPM setpoint loop inside the attitude path
// made the vehicle oscillate: a 1 degree disturbance produced spin-up / stop / spin-up
// cycling. That is not a tuning problem, it is the architecture — thruster dynamics are a
// slow nonlinear lag (Yoerger/Cooke/Slotine 1990) and shaft-speed control is known to
// induce thrust oscillation in dynamic conditions (Smogeli/Sorensen 2009). No mainstream
// flight or ROV firmware closes a thrust loop on RPM; they all use bidirectional-DShot RPM
// for the notch filter, telemetry and health only.
//
// Voltage-independent thrust — the actual reason RPM was wanted — is delivered OUTSIDE the
// fast path instead: a slow per-thruster RPM trim (control/thrust_trim) plus battery
// feedforward. See docs/T200_PROFILE.md.
#define THR_RPM_CLOSED_LOOP    0
#define THR_MAX_RPM            4000         // signed target RPM at full |norm| (per your thruster)

// Pixhawk-style ESC beacon beep: a short chirp through the thrusters at power-up and
// on the first arm ("connected"). BLHeli only plays its 5 fixed beacon tones over
// DShot commands — this is a chirp, not a melody (melodies play on the buzzer).
#define THR_BEEP_EN            1            // 1 = enable ESC beacon beeps (param DEF_THR_BEEP_EN)

// --- Buzzer per-situation enable bitmask (param BUZZ_MASK) --------------------
// Each bit gates one situation; BUZZ_MASK=0 silences everything, 255 = all on.
#define BUZZ_BIT_STARTUP        0x01        // bit0: power-up melody (Rick & Morty riff)
#define BUZZ_BIT_ARM            0x02        // bit1: arm rising chirp
#define BUZZ_BIT_DISARM         0x04        // bit2: disarm falling chirp
#define BUZZ_BIT_PICO_LINK      0x08        // bit3: Pico link lost / restored
#define BUZZ_BIT_GCS_LOST       0x10        // bit4: GCS/telemetry link lost
#define BUZZ_BIT_LEAK           0x20        // bit5: leak detected
#define BUZZ_BIT_FAULT          0x40        // bit6: thruster fault
#define BUZZ_BIT_CALIB          0x80        // bit7: calibration step ticks

// -----------------------------------------------------------------------------
// SECTION 5 — PCA9685 AUX EXPANDER (Bus1)
// 16 channels. Each channel has a role:
//   PCA_DISABLED : unused
//   PCA_SERVO    : PWM servo output (SERVO_MIN_US..SERVO_MAX_US)
//   PCA_SWITCH   : digital HIGH/LOW to drive a payload MOSFET or relay ON/OFF
// Driven by Task_UI_Status (Core 0) from AuxState. Controllable from QGC via
// MAV_CMD_DO_SET_SERVO (servo) and MAV_CMD_DO_SET_RELAY (switch).
// Edit PCA_CH_ROLE below to match how your expander is wired.
// -----------------------------------------------------------------------------
#define PCA9685_NUM_CH          16
#define PCA9685_PWM_FREQ        50          // Hz — 50 for standard servos
#define SERVO_MIN_US            1000
#define SERVO_MID_US            1500
#define SERVO_MAX_US            2000
#define SWITCH_ACTIVE_HIGH      1           // 1: switch_on → channel full-ON (HIGH)

// Role codes (mirror PcaChannelRole in state_types.h — kept as ints for the macro array)
#define PCA_DISABLED            0
#define PCA_SERVO               1
#define PCA_SWITCH              2

// Joystick-button → PCA channel mapping (ArduSub-style relays/lights).
//   relay N (1-based) → SWITCH channel PCA_RELAY_BASE_CH + (N-1)  [MOSFET/relay]
//   lights1/2 (dimmable) → SERVO channels below
#define PCA_RELAY_BASE_CH       8    // first switch/MOSFET channel
#define PCA_LIGHTS1_CH          6    // servo-role channel for lights 1 (payload light)
#define PCA_LIGHTS2_CH          7    // servo-role channel for lights 2
#define PCA_PAYLOAD_SERVO_CH    0    // servo-role channel for payload-release servo

// Default channel role map. Example: 0-7 servos, 8-15 payload switches.
#define PCA_CH_ROLE_INIT { PCA_SERVO,  PCA_SERVO,  PCA_SERVO,  PCA_SERVO,  \
                           PCA_SERVO,  PCA_SERVO,  PCA_SERVO,  PCA_SERVO,  \
                           PCA_SWITCH, PCA_SWITCH, PCA_SWITCH, PCA_SWITCH, \
                           PCA_SWITCH, PCA_SWITCH, PCA_SWITCH, PCA_SWITCH }

// -----------------------------------------------------------------------------
// SECTION 6 — FREERTOS TASK ALLOCATION
//   Core 0 (comms / display / filesystem, low priority)
//   Core 1 (deterministic real-time flight loop, high priority)
// Rates: control loop 200 Hz; sensor read 200 Hz; DShot output 200 Hz.
// -----------------------------------------------------------------------------
#define CORE_COMMS              0
#define CORE_FLIGHT             1

// --- Core 0 tasks ---
#define TASK_MAVLINK_CORE       CORE_COMMS
#define TASK_MAVLINK_PRIO       2
#define TASK_MAVLINK_STACK      8192        // builds mavlink_message_t locals + deep lib frames
#define TASK_MAVLINK_HZ         100         // drain RX promptly for QGC bursts

#define TASK_LORA_SD_CORE       CORE_COMMS
#define TASK_LORA_SD_PRIO       1
#define TASK_LORA_SD_STACK      6144
#define TASK_LORA_SD_HZ         20

#define TASK_UI_CORE            CORE_COMMS
#define TASK_UI_PRIO            1
#define TASK_UI_STACK           6144        // headroom for the dashboard render + float printf
#define TASK_UI_HZ              30

// Dedicated buzzer task: 200 Hz gives 5 ms tone resolution so beep lengths stay
// uniform (the UI task at 30 Hz jittered them). It owns the buzzer entirely.
#define TASK_BUZZER_CORE        CORE_COMMS
#define TASK_BUZZER_PRIO        1
#define TASK_BUZZER_STACK       2560
#define TASK_BUZZER_HZ          200

// --- Core 1 tasks ---
#define TASK_SENSOR_CORE        CORE_FLIGHT
#define TASK_SENSOR_PRIO        6           // above control/dshot so a transient overrun can't pile up
#define TASK_SENSOR_STACK       8192        // deep Adafruit/sh2 stack + VLA I2C buffer
// 500 Hz task (2 ms, tick-exact) polling the BNO's 400 Hz rotation/gyro reports —
// oversamples cleanly with INT-gating so no read is wasted. 400 Hz isn't a whole-ms
// tick, so the task runs at 500 Hz; the sensor's 400 Hz fused output is the ceiling.
#define TASK_SENSOR_HZ          500

#define TASK_CONTROL_CORE       CORE_FLIGHT
#define TASK_CONTROL_PRIO       5
#define TASK_CONTROL_STACK      6144
// 500 Hz (2 ms, tick-exact — the fastest whole-ms rate on the 1 kHz FreeRTOS tick;
// "400 Hz" would run at 2 ms anyway but with a mismatched dt). CONTROL_LOOP_DT
// auto-derives, so PID integ/derivative stay correct.
#define CONTROL_LOOP_HZ         500
#define CONTROL_LOOP_DT         (1.0f / (float)CONTROL_LOOP_HZ)

// Derivative low-pass cutoff (Hz) for every PID (control/pid.h). A raw difference at
// 500 Hz multiplies gyro noise by 500, which is why D could not be raised during tuning:
// the term was mostly noise and buzzed the motors long before it damped anything. 20 Hz is
// well above sub-scale rigid-body dynamics (a few Hz) and well below the noise floor.
// Lower it if D still sounds harsh; 0 disables the filter (old raw behaviour).
// Compile-time on purpose — the parameter table is deliberately left untouched, so
// changing this needs a rebuild.
#define PID_D_FILT_HZ           20.0f

#define TASK_DSHOT_CORE         CORE_FLIGHT
#define TASK_DSHOT_PRIO         5
#define TASK_DSHOT_STACK        4096
#define TASK_DSHOT_HZ           500         // match the 500 Hz control loop

// -----------------------------------------------------------------------------
// SECTION 7 — DEFAULT TUNING CONSTANTS (NVS fallbacks; live-editable via QGC)
// The authoritative runtime values come from NVS (Preferences); these are the
// defaults used when NVS is empty. See PARAMETERS.md for the parameter table.
// -----------------------------------------------------------------------------
// Stunt / pattern
#define DEF_STUNT_SPIN_CNT      1.0f        // number of 360° rotations
#define DEF_STUNT_RATE          90.0f       // deg/s rotation speed
#define DEF_HEADROOM_DEPTH      0.5f        // m clearance offset for complex pattern

// Indicators
#define DEF_RGB_BRIGHTNESS      64          // 0..255 WS2812B brightness

// Battery ADC calibration (V per raw LSB) — from 2nd_board_firmware reference
// PM1_VMULT is the resistor-divider RATIO: battery volts per volt measured at the pin.
// (It used to be volts-per-ADC-count, ~0.009. The reading now comes from
// analogReadMilliVolts(), which applies this chip's factory ADC calibration and removes the
// 11 dB attenuation non-linearity that no single linear multiplier could compensate — the
// reason the reported pack voltage was wrong. A value below ~0.5 is reported over STATUSTEXT
// as probably-stale, but it is APPLIED — silently substituting the default made the parameter
// impossible to calibrate, because nothing on screen responded to a value being typed.)
//
// 11.28 reproduces the old full-scale (0.009088 V/count x 4095 counts = 37.2 V) against a
// 3.3 V input, so it is the closest equivalent starting point — but it is a STARTING POINT.
// Calibrate: PM1_VMULT_new = PM1_VMULT_now x (multimeter reading / reported reading).
#define DEF_PM1_VMULT           11.28f
// PM2 arrives pre-scaled (in volts) from the 2nd board over ESP-NOW, so this is a TRIM, not a
// divider ratio: 1.0 = report exactly what the 2nd board sends. Use it to correct the thruster
// pack reading from the GCS without reflashing the 2nd board —
//   PM2_VMULT_new = PM2_VMULT_now x (multimeter reading / reported reading).
// It was previously left at the old volts-per-ADC-count value and read by nothing; a board
// carrying that stored value is migrated to 1.0 once at boot (see params::init).
#define DEF_PM2_VMULT           1.0f
// Below this, a stored PM2_VMULT cannot be a display trim (it would report ~0 V on any pack)
// and is certainly the pre-change volts-per-count value. Migrated, not silently ignored.
#define PM2_VMULT_LEGACY_MAX    0.05f

// Cascaded angle→rate PID starting gains (per axis; tuned later — see ARCHITECTURE.md §4)
#define DEF_ANG_RLL_P           4.5f
#define DEF_ANG_PIT_P           4.5f
#define DEF_ANG_YAW_P           4.5f
#define DEF_RAT_RLL_P           0.135f
#define DEF_RAT_RLL_I           0.090f
#define DEF_RAT_RLL_D           0.0036f
#define DEF_RAT_PIT_P           0.135f
#define DEF_RAT_PIT_I           0.090f
#define DEF_RAT_PIT_D           0.0036f
#define DEF_RAT_YAW_P           0.180f
#define DEF_RAT_YAW_I           0.018f
#define DEF_RAT_YAW_D           0.0f

// Rate feedforward (per axis) — 0 by default; raise for crisper small commands.
#define DEF_RAT_RLL_FF          0.0f
#define DEF_RAT_PIT_FF          0.0f
#define DEF_RAT_YAW_FF          0.0f

// Thrust shaping (precision low-speed control).
#define DEF_MOT_THST_EXPO       0.65f       // inverse-quadratic thrust curve
// Min active output. 0.10 was a quadcopter-ish value that fought the ESC: with Bluejay's
// Minimum Startup Power at 1060 the ESC breaks away at ~1.1 % of the band, but a 0.10
// floor forced DShot 1173 for a 1 % command — measured at 941 rpm on a T200, i.e. ~26 %
// of full speed from the smallest possible input, with nothing usable below it. Let the
// ESC's own startup power handle break-away and keep the firmware floor just above zero.
#define DEF_MOT_SPIN_MIN        0.02f       // min active output (crosses prop deadband)
#define DEF_MOT_SPIN_ARM        0.0f        // armed idle (0 = props stopped at neutral)

// Slow RPM-based thrust normalisation (control/thrust_trim). This is how "20 % for 5 s
// travels the same distance on a full or flat pack" is achieved — NOT by putting RPM in
// the attitude loop (that oscillates; see THR_RPM_CLOSED_LOOP above).
// Default OFF: a prop in AIR has almost no load and spins far faster for the same duty,
// so learning on the bench would drive every gain to its clamp. Turn on in water.
#define DEF_THR_TRIM_EN         0.0f        // 0 = off, 1 = learn
#define DEF_THR_TRIM_TAU        4.0f        // s — slow on purpose (~100x slower than attitude)
#define DEF_THR_TRIM_MAX        0.25f       // max +/-25% gain deviation

// Battery-voltage feedforward for the THRUSTER pack (ArduPilot-style thrust
// linearisation). throttle commands volts, so scaling duty by 1/V keeps duty*V — and
// therefore thrust — constant as the pack sags. This is open-loop: it closes no loop, adds
// no lag, and cannot oscillate. Inert unless a FRESH thruster voltage exists (ESP-NOW),
// so a vehicle without the 2nd board behaves exactly as if it were disabled.
// Set MOT_BAT_V_MAX to the pack's full-charge voltage (4S LiPo = 16.8) and V_MIN to the
// lowest you will fly it at; 0 disables the compensation entirely.
#define DEF_MOT_BAT_V_MIN       13.2f       // V — clamp floor (also stops the gain blowing up)
#define DEF_MOT_BAT_V_MAX       0.0f        // V — 0 = compensation OFF until you set the pack

// One-shot magnetic yaw reference (control/yaw_ref). Attitude stays on the mag-FREE
// 6-axis GAME rotation vector; the magnetometer is read ONCE at boot (disarmed, still) to
// work out where north is, and never consulted again — so thruster currents can never move
// the attitude solution, but heading is absolute from the first second.
// This fixes the yaw REFERENCE, not the drift: the 6-axis yaw still creeps ~0.5-3 deg/min,
// so absolute heading degrades over a long dive. That is the accepted trade for immunity.
// Default OFF: needs a mag calibration done IN THE HULL with electronics powered first, and
// the tilt-compensation axis signs verified against a real compass (see docs).
#define DEF_MAG_YAW_REF         0.0f        // 0 = yaw relative (as before) · 1 = align at boot
#define DEF_MAG_DECL            0.4f        // deg, E positive (~0.4 E for Bangladesh)

// Pilot command shaping (micro-movements).
#define DEF_PILOT_YAW_RATE      45.0f       // deg/s at full yaw stick (lower = finer)
#define DEF_PILOT_EXPO          0.30f       // stick expo for fine centre resolution
#define DEF_PILOT_SPEED         1.0f        // forward/lateral scale (1 = full)

// Model-based control (Phase 2a) — all default OFF/neutral (behaviour unchanged until
// tuned in the pool). See ALGORITHMS.md §5.
#define DEF_ATC_DRAG_RLL        0.0f        // angular drag feedforward K (torque per rad²/s²)
#define DEF_ATC_DRAG_PIT        0.0f
#define DEF_ATC_DRAG_YAW        0.0f
#define DEF_XC_YAW2RLL          0.0f        // cross-coupling: yaw-rate → counter-roll torque
#define DEF_XC_YAW2PIT          0.0f        // cross-coupling: yaw-rate → counter-pitch torque
#define DEF_TRIM_EN             0.0f        // CoB auto-trim on/off
#define DEF_TRIM_LEAK           0.002f      // per-cycle bleed of the rate I-term into trim
#define DEF_TRIM_MAX            0.30f       // trim clamp (fraction of full torque)

// Indicators
#define DEF_BUZZ_MASK           255.0f      // per-situation buzzer enables (see BUZZ_BIT_*)
#define DEF_THR_BEEP_EN         1.0f        // ESC beacon beeps on startup/connect (0=off)
#define DEF_BUZZ_DUTY           30.0f       // buzzer drive duty % (lower = less current draw)

// Autotune safety monitor (auto-disarm on a dangerous state during a tune)
#define DEF_ST_ANGLE_MAX        70.0f       // deg — tumbling guard
#define DEF_ST_RATE_MAX         360.0f      // deg/s — spin-out guard
#define DEF_ST_DEPTH_DELTA      2.0f        // m — depth-runaway guard
#define DEF_ST_RPM_MAX          4500.0f     // rpm — over-speed guard. Must sit ABOVE the
                                            // reachable maximum (RPM_MAX default 3600 for a
                                            // T200) or a tune trips its own guard.

// Motor closed-loop RPM control (defaults; MOTOR_TUNE overwrites, cfg frame pushes to Pico)
#define DEF_RPM_KP              0.60f
#define DEF_RPM_KI              0.02f
#define DEF_RPM_FF_A            0.000278f   // norm-throttle per rpm (≈ 1/RPM_MAX); 0 disables FF
#define DEF_RPM_IDLE            0.0f        // dynamic-idle target rpm (0 = stop when centred)
// Blue Robotics T200 tops out near 3600 rpm at 16 V (3075 @ 12 V, 3800 @ 20 V). Asking
// for 4000 made full stick chase an RPM the motor can never reach, so the loop sat
// saturated. Keep DEF_RPM_FF_A ≈ 1/DEF_RPM_MAX when changing this.
#define DEF_RPM_MAX             3600.0f
#define DEF_RPM_FILT            0.30f
#define DEF_RPM_SLEW            0.0f        // max level change/cycle (0 = no slew limit)
// Centre deadband for the RPM loop: |target| below this stops the motor at 3D neutral.
// Was hard-coded at 150 rpm on the Pico, which at RPM_MAX 3600 threw away the first
// 4.2 % of stick travel. 30 rpm keeps a genuine "stopped at centre" without eating the
// usable low end. Pushed to the Pico in the cfg frame.
#define DEF_RPM_MIN_TGT         30.0f       // rpm
// The LIVE open-vs-closed-loop switch, and the operator's call to make. Both the ESP32's
// command mode (task_dshot_rmt) and the Pico's loop enable (pushed in the cfg frame) read
// this one param, so setting it to 1 genuinely switches the whole path. Its boot default is
// THR_RPM_CLOSED_LOOP above.
// The PI's sign bugs are fixed, but the architecture is still wrong for stabilisation —
// prefer THR_TRIM_EN for repeatable thrust. See the note at THR_RPM_CLOSED_LOOP.
#define DEF_RPM_LOOP            ((float)THR_RPM_CLOSED_LOOP)   // 0 = open-loop · 1 = closed-loop RPM (PI)
#define DEF_DSHOT_BIDIR         1.0f        // 1 = bidirectional DShot (RPM telem) · 0 = normal DShot
#define DEF_MTUNE_EN            0.0f        // MOTOR_TUNE disabled until explicitly enabled

// AUTO-mode movement (Jetson-offloaded, RPM-braked)
#define DEF_MOVE_CRUISE_MAX     0.80f       // cap on commanded translation speed
#define DEF_MOVE_ACCEL          2.5f        // speed ramp /s (0→full in ~0.4 s)
#define DEF_MOVE_BRAKE_GAIN     0.55f       // reverse-thrust fraction while braking
#define DEF_MOVE_BRAKE_K        0.60f       // brake seconds per unit cruise speed
#define DEF_MOVE_DEPTH_RATE     0.20f       // m/s smooth dive-target ramp (no splash)
#define DEF_MOVE_YAW_RATE       45.0f       // default turn rate (deg/s)

// Depth-hold PID starting gains
#define DEF_DEPTH_P             3.0f
#define DEF_DEPTH_I             0.5f
#define DEF_DEPTH_D             0.0f

// Depth sensor (Bar30/MS5837) health. Samples are published ~20 Hz; if none arrives for
// this long the depth reading is treated as INVALID, which blocks DEPTH_HOLD/AUTO and
// switches the SURFACE failsafe to a fixed open-loop ascent (see task_control_loop).
#define DEPTH_STALE_MS          1500
// Open-loop ascent used by the SURFACE failsafe when depth is unavailable — enough to
// surface the vehicle without a depth loop, gentle enough not to breach hard.
#define DEPTH_LOST_ASCENT       0.35f       // 0..1 heave command

// Low-thruster-battery failsafe hold time. The voltage must stay below FS_BAT_VOLTAGE
// continuously for this long before surfacing; any sample above it resets the timer.
// A 4S LiPo driving eight T200s sags well over a volt under load, so an instantaneous
// test would surface the vehicle mid-burst on a perfectly healthy pack. The 2nd board
// already averages its ADC over 500 ms and sends at 4 Hz, so 3 s is ~12 consecutive low
// readings — far too long for a transient, still early enough that a genuinely flat pack
// surfaces with reserve. Leak and GCS-loss are NOT debounced: neither is analogue.
#define FS_BAT_HOLD_MS          3000

// Failsafe
#define GCS_FAILSAFE_MS         5000        // no GCS HEARTBEAT for this long → failsafe
                                            // (5 s tolerates a few missed beats over lossy LoRa;
                                            //  USB/UDP heartbeat at 1 Hz reliably, so still safe)

// -----------------------------------------------------------------------------
// SECTION 8 — NVS NAMESPACES (Preferences)
// -----------------------------------------------------------------------------
#define NVS_NS_PARAMS           "srot_prm"  // MAVLink parameters
#define NVS_NS_CAL              "srot_cal"  // sensor calibration offsets/scales

// Parameter DEFAULTS schema version. NVS survives a firmware flash (the upload only
// rewrites the app partition), and a stored value always beats its DEF_* default — so
// editing a DEF_* in this file does NOTHING on a board that already has that key saved.
// BUMP THIS to declare "this build's defaults win": on boot, params::init() sees the
// mismatch, rewrites EVERY parameter from its DEF_*, and stores the new version.
//
// WARNING: a bump DISCARDS all user-set parameter values (tuning, pin assignments,
// servo config). Sensor calibration is NOT affected — it lives in NVS_NS_CAL.
//
// BEFORE YOU BUMP THIS: export the parameters from Bondor (Parameters -> Export). Four
// bumps have already silently thrown away a pool tuning session. A bump is the ONLY
// reason to lose params on a reflash — a plain upload preserves NVS.
//
// ADDING a parameter does NOT need a bump: a key that isn't in NVS yet falls back to its
// DEF_*, so new params get their defaults while existing ones keep their stored values.
// Only bump when an EXISTING default must override what boards already have saved.
//
// Bumped to 2: T200 thruster defaults (RPM_MAX/RPM_FF_A) + clearing a stale stored
// RPM_LOOP=0 that MOTOR_TUNE's saveAll() had frozen in.
// Bumped to 3: low-end resolution — MOT_SPIN_MIN 0.10 -> 0.02 and the new RPM_MIN_TGT,
// so a 1% command is a crawl instead of ~26% of full speed.
// Bumped to 4: RPM removed from the attitude path (RPM_LOOP default 0) and replaced by
// the slow thrust normalisation + battery feedforward (THR_TRIM_*, MOT_BAT_V_*).
// NOT bumped for the CAL_*/MAG_* params: they are additions, so existing tuning survives.
#define PARAM_DEFAULTS_VER      4
#define NVS_KEY_DEFAULTS_VER    "p_defver"
// Marker for the one-shot PM2_VMULT units migration (volts-per-count -> aux trim). A marker,
// not a value test: re-testing the value on every boot would overwrite a small trim the
// operator set deliberately, which is the failure this round removed from PM1_VMULT.
#define NVS_KEY_PM2_MIGRATED    "p_pm2mig"

// -----------------------------------------------------------------------------
// SECTION 8b — ESP-NOW LINK (P11, receive-only from the 2nd board)
// The 2nd board sends thruster-kill state + an aux voltage; the main board
// displays them (OLED) and streams kill via telemetry. WiFi runs in STA mode on
// Core 0 only. The channel MUST match the 2nd board (its firmware uses ch 1).
// We use ADC1 only, so WiFi does not disturb the battery/leak reads.
// -----------------------------------------------------------------------------
// ESP-NOW is now COMPILED IN (so it can be toggled at runtime via the ESPNOW_EN
// param) but WiFi is only STARTED when ESPNOW_EN=1. Off by default → WiFi never
// starts → no MAVLink contention. Enable ESPNOW_EN once the 2nd board is ready.
// (Setting PMx_SRC=2 only *selects* ESP-NOW as the voltage source; ESPNOW_EN is
// what actually brings the link up.)
#define ESPNOW_ENABLE           1
#define ESPNOW_CHANNEL          1
#define ESPNOW_STALE_MS         2000        // no packet for this long → kill = false

// -----------------------------------------------------------------------------
// SECTION 9 — DEBUG
// NOTE: UART0 is the MAVLink link. Do NOT print human text to Serial in normal
// operation or it corrupts the MAVLink stream. SROT_DEBUG is for bench bring-up
// only, when no GCS is attached.
// -----------------------------------------------------------------------------
#define SROT_DEBUG              0           // 1 = plain-text debug on UART0 (no MAVLink)
