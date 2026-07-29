# SROT — Parameter Reference (complete)

Every parameter in the live table, documented. `params::count()` = **189**:
**109** scalar params + **80** servo params (16 channels × 5).

SROT is now **SROT-native** — it no longer disguises itself as ArduSub, and the old
QGC-compatibility dummy params have been **removed**. The ground station is **Bondor**
(`bondor/`), which presents these under friendly SROT groupings via its own metadata layer;
every param below is real and changes firmware behaviour.

Storage: every value persists to NVS flash and survives reboot. `PIN_*` changes need a
reboot. Wire type is REAL32 for all params.

Legend: **[R]** real · **[i]** informational/inert (defined, not read by the control loop).

---

## Two things people ask about

**PM vs BATT.** `PM1_*`/`PM2_*` are the **real** dual-battery params (→ MAVLink
`BATTERY_STATUS` id 0 and id 1). The `BATT_*` group is **QGC dummies** — inert. Set the
`PM*` ones; ignore `BATT_*`.

**Servo vs MOSFET.** Each of the 16 PCA9685 aux channels is configured by
**`SERVOn_ROLE`**: `0` = off, `1` = **PWM servo** (driven by `DO_SET_SERVO`, clamped by
`SERVOn_MIN/MAX`, idles at `SERVOn_TRIM`), `2` = **MOSFET/relay** (driven by
`DO_SET_RELAY` / joystick relay buttons). QGC's Servo Outputs *page* only shows outputs
9-16 (it reserves 1-8 for the Sub frame's motors), but **all 16 are controllable** via
the Parameters list + `DO_SET_SERVO`/`DO_SET_RELAY`.

---

## A. Real SROT parameters — the ones that do something

### Attitude control — cascade PID (`ATC_*` names; Bondor's Tuning page edits these live)
Outer angle loop: `error × ATC_ANG_*_P → desired rate`. Inner rate loop:
`PID(desired_rate, gyro) + FF·desired_rate → torque`.

| Param | Unit | Default | What it does |
|---|---|---|---|
| `ATC_ANG_RLL_P` **[R]** | 1/s | 4.5 | Roll angle→rate P. Higher = stiffer level-hold. |
| `ATC_ANG_PIT_P` **[R]** | 1/s | 4.5 | Pitch angle→rate P. |
| `ATC_ANG_YAW_P` **[R]** | 1/s | 4.5 | Yaw **heading-hold** P (used when yaw stick centred). |
| `ATC_RAT_RLL_P/I/D` **[R]** | — | 0.135 / 0.090 / 0.0036 | Roll rate PID (torque ∈ ±1). |
| `ATC_RAT_RLL_IMAX` **[R]** | — | 0.5 | Roll integrator clamp (anti-windup). |
| `ATC_RAT_RLL_FF` **[R]** | — | 0.0 | Roll rate feedforward — crisper small commands. |
| `ATC_RAT_PIT_P/I/D` **[R]** | — | 0.135 / 0.090 / 0.0036 | Pitch rate PID. |
| `ATC_RAT_PIT_IMAX` / `_FF` **[R]** | — | 0.5 / 0.0 | Pitch integrator clamp / feedforward. |
| `ATC_RAT_YAW_P/I/D` **[R]** | — | 0.180 / 0.018 / 0.0 | Yaw rate PID (usually higher P, lower I than roll/pitch). |
| `ATC_RAT_YAW_IMAX` / `_FF` **[R]** | — | 0.5 / 0.0 | Yaw integrator clamp / feedforward. |

### Model-based control (Phase 2a — all default 0/OFF until pool-tuned; see ALGORITHMS.md §5)
| Param | Default | What it does |
|---|---|---|
| `ATC_DRAG_RLL/PIT/YAW` | 0 | Angular drag feedforward `K·ω·|ω|` — linearizes rotational drag so the rate PID sees a nicer plant |
| `XC_YAW2RLL` / `XC_YAW2PIT` | 0 | Cross-coupling: proactively counter the roll/pitch a fast yaw induces |
| `TRIM_EN` | 0 | Enable CoB auto-trim (bleed the steady I-term into a learned trim) |
| `TRIM_LEAK` | 0.002 | Per-cycle bleed rate of the I-term into trim |
| `TRIM_MAX` | 0.30 | Trim clamp (fraction of full torque) |

### Autotune & motor tuning (modes AUTOTUNE=21, MOTOR_TUNE=22)
| Param | Default | What it does |
|---|---|---|
| `ATUNE` | 0 | Set 1 to start the flight-loop relay autotune (auto-resets). Or select AUTOTUNE mode. |
| `ST_ANGLE_MAX` | 70 | Safety: tumbling angle (deg) that auto-disarms during a tune. |
| `ST_RATE_MAX` | 360 | Safety: spin-out body rate (deg/s) that auto-disarms. |
| `ST_DEPTH_DELTA` | 2.0 | Safety: depth runaway (m from tune start) that auto-disarms. |
| `ST_RPM_MAX` | 4500 | Safety: over-speed (rpm) that auto-disarms (MOTOR_TUNE). |
| `MTUNE_EN` | 0 | Must be 1 to allow MOTOR_TUNE (spins thrusters — run in water). |
| `RPM_KP` / `RPM_KI` | 0.6 / 0.02 | Pico per-motor RPM PI gains (MOTOR_TUNE writes them). |
| `RPM_FF_A` | 0.00025 | Feedforward slope (norm-throttle per rpm ≈ 1/RPM_MAX). |
| `RPM_IDLE` | 0 | Dynamic-idle target rpm (min rotation; 0 = stop when centred). |
| `RPM_MAX` | 4000 | \|rpm\| that maps to full throttle. |
| `RPM_FILT` | 0.30 | Measured-rpm IIR alpha (0..1). |
| `RPM_SLEW` | 0 | Max output-level change per Pico cycle (0 = no slew limit). |
| `RPM_LOOP` | 1 | **Control loop (your choice): 1 = closed-loop RPM (PI) · 0 = open-loop (feedforward).** Closed needs bidir-DShot telemetry (Bluejay); open works with any ESC. Pushed live to the Pico. |
| `DSHOT_BIDIR` | 1 | **DShot output mode: 1 = bidirectional (inverted, RPM telemetry → detection + closed loop) · 0 = normal DShot (any DShot ESC, e.g. stock BLHeli_S, spins but no RPM).** Pushed to the Pico; a change re-creates the PIO drivers **while disarmed**. Use 0 to bench a non-bidir ESC; 1 for the real setup. |

### AUTO movement (mode AUTO=23; Jetson-offloaded moves — see ALGORITHMS.md §11, JETSON_COMMS.md)
Timer-based (duration × speed); no distance sensor/estimate. Constant distance per time comes from
the **Pico RPM closed loop** (`THR_RPM_CLOSED_LOOP=1`, now enabled) — **precondition:** run
**MOTOR_TUNE in water** once to fit `RPM_KP/KI/FF_A/IDLE`, else the loop may over/undershoot.
| Param | Default | What it does |
|---|---|---|
| `MOVE_CRUISE_MAX` | 0.80 | Max normalized cruise speed (clamps the command's speed). |
| `MOVE_ACCEL` | 2.5 | Translation speed ramp (/s) — smooth start, no lurch. |
| `MOVE_BRAKE_GAIN` | 0.55 | Reverse-thrust fraction during the on-board brake. |
| `MOVE_BRAKE_K` | 0.60 | Brake duration = K·cruise-speed (s) → stops with no drift. |
| `MOVE_DEPTH_RATE` | 0.20 | Smooth dive/ascend rate (m/s) — no splash. |
| `MOVE_YAW_RATE` | 45 | Default turn rate (deg/s) when the command's rate = 0. |

### Depth hold (real depth loop)
| Param | Default | What it does |
|---|---|---|
| `DEPTH_P` **[R]** | 3.0 | Depth-error P (drives vertical thrust). |
| `DEPTH_I` **[R]** | 0.5 | Depth integrator (holds against buoyancy trim). |
| `DEPTH_D` **[R]** | 0.0 | Depth derivative. |

### Thrust shaping & pilot feel (precision low-speed control)
| Param | Unit | Default | What it does |
|---|---|---|---|
| `MOT_THST_EXPO` **[R]** | 0..1 | 0.65 | Thrust-curve linearizer (0 = linear, 1 = fully quadratic). Makes small commands give small predictable thrust. |
| `MOT_SPIN_MIN` **[R]** | 0..1 | 0.10 | Minimum active output — the smallest command crosses the prop deadband. Raise until a tiny nudge just moves the sub. |
| `MOT_SPIN_ARM` **[R]** | 0..1 | 0.0 | Armed idle. **0 = thrusters stopped at neutral** (fixed: previously crept forward). >0 = light forward idle. |
| `PILOT_YAW_RATE` **[R]** | deg/s | 45 | Max yaw rate at full stick. **Lower (e.g. 10) for fine yaw.** Centre-stick then holds heading. |
| `PILOT_EXPO` **[R]** | 0..1 | 0.30 | Stick expo (fine resolution near centre) for yaw + forward/lateral. |
| `PILOT_SPEED` **[R]** | 0..1 | 1.0 | Forward/lateral scale. Lower for delicate translation. |
| `JS_GAIN_DEFAULT` **[R]** | 0..1 | 0.5 | Pilot gain applied to manual-control inputs. |

### Battery (dual) & leak
| Param | Default | What it does |
|---|---|---|
| `PM1_SRC` **[R]** | 1 | Battery-1 source: 0 = off, 1 = local ADC (GPIO36), 2 = ESP-NOW aux. → BATTERY_STATUS id 0. |
| `PM2_SRC` **[R]** | 2 | Battery-2 source (same codes). → BATTERY_STATUS id 1. |
| `PM1_VMULT` **[R]** | 0.009088 | Volts per ADC LSB for PM1 (calibration). |
| `PM2_VMULT` **[i]** | 0.009088 | **Not read** — PM2 is ESP-NOW aux, already scaled upstream. |
| `LEAK_EN` **[R]** | 0 | Enable leak-sensor read + leak failsafe (real; not the compat `FS_LEAK_ENABLE`/`LEAK1_*`). |

### Failsafe & arming (real ones)
| Param | Default | What it does |
|---|---|---|
| `FS_GCS_ENABLE` **[R]** | 1 | GCS-loss failsafe → controlled SURFACE ascent. |
| `FS_BAT_ENABLE` **[R]** | 1 | Low-battery failsafe enable (PM1). |
| `FS_BAT_VOLTAGE` **[R]** | 13.2 | PM1 low-battery threshold (V). |
| `ARMING_CHECK` **[R]** | 1 | 0 = skip pre-arm checks, 1 = enforce. |
| `ATUNE` **[R]** | 0 | Momentary trigger: set 1 → relay auto-tune of all PID loops (auto-resets to 0; NVS save deferred to the comms core). |
| `ESPNOW_EN` **[R]** | 0 | Start the WiFi/ESP-NOW 2nd-board link (thruster-kill + aux voltage). |

### Motor reverse (Bondor Motors page reverse toggle)
| Param | Default | What it does |
|---|---|---|
| `MOT_1_DIRECTION` … `MOT_8_DIRECTION` **[R]** | 1 | Per-thruster spin direction (+1 / −1) — set at setup so all thrusters push correctly for a given command (ArduSub/Pixhawk style). Final dir = motor-detect result × this. **Works on the Pico backend in both raw and closed-loop RPM modes.** |

### Joystick button map (`BTNn_FUNCTION`)
Edit these in Bondor's **Joystick** tab, which also shows every button's live state — press a
button and watch which cell lights up to learn its index.

| Param | Default | What it does |
|---|---|---|
| `BTN0_FUNCTION` … `BTN15_FUNCTION` **[R]** | 3,4,6,7,5,32,33,53, then 0×8 | Button → action on the press edge. Only buttons 0–15 are decoded. |

Function IDs (ArduSub `AP_JSButton` numbering). **These are exactly the ones the firmware
implements** — anything else is accepted by PARAM_SET and then silently ignored:

| ID | Action | | ID | Action |
|---|---|---|---|---|
| 0 | None | | 32 | Lights brighter |
| 2 | Arm toggle | | 33 | Lights dimmer |
| 3 | Arm | | 41 | Gain toggle (low/high) |
| 4 | Disarm | | 42 | Gain up |
| 5 | Mode: Manual | | 43 | Gain down |
| 6 | Mode: Stabilize | | 51/52/53 | Relay 1 on/off/toggle |
| 7 | Mode: Depth Hold | | 54/55/56 | Relay 2 on/off/toggle |
| 9 | Mode: Surface | | | |
| 10 | Mode: Auto | | | |
| 12 | Mode: Acro | | | |

**Gain is runtime-only.** The gain buttons step the live pilot gain between 10 % and 100 %;
`JS_GAIN_DEFAULT` remains the power-on value and nothing is written to NVS, so the vehicle
always boots at a known gain (same as ArduSub). The live value is streamed to the GCS as
`NAMED_VALUE_FLOAT "GAIN"` and shown in the Joystick tab.

### Stunt / pattern
| Param | Default | What it does |
|---|---|---|
| `STUNT_SPIN_CNT` **[R]** | 1 | Default number of 360° rotations for a stunt command. |
| `STUNT_RATE` **[R]** | 90 | Stunt/pattern rotation rate (deg/s). |
| `HEADROOM_DEPTH` **[R]** | 0.5 | Depth clearance (m) used by the complex pattern. |

### Indicators
| Param | Default | What it does |
|---|---|---|
| `RGB_BRIGHTNESS` **[R]** | 64 | WS2812B brightness 0..255. |
| `LIGHTS_STEP` **[R]** | 0.1 | Payload-light brightness step per brighter/dimmer press. |
| `BUZZ_MASK` | 255 | Per-situation buzzer enable bitmask. Bits: 1=startup(R&M), 2=arm, 4=disarm, 8=Pico link, 16=GCS lost, 32=leak, 64=fault, 128=calib. 0 = silent. |
| `THR_BEEP_EN` | 1 | ESC beacon chirp (through the thrusters) on startup + connect. 0 = off. |

### Runtime GPIO assignments (reboot to apply; validated by `pinOr`)
Output pins reject the input-only pins 34-39; all reject flash pins 6-11.

| Param | Default | Pin role |
|---|---|---|
| `PIN_BUZZER` **[R]** | 32 | Passive buzzer via 2N2222 (LEDC tone). |
| `PIN_RGB` **[R]** | 2 | WS2812B data (bit-bang; must be <32). |
| `PIN_LEAK` **[R]** | 35 | Leak input. |
| `PIN_BATTVOLT` / `PIN_BATTCURR` **[R]** | 36 / 39 | Battery voltage / current ADC (input-only). |
| `PIN_SDCS` **[R]** | 33 | SD card chip-select. |
| `PIN_LORACS` / `PIN_LORADIO` **[R]** | 5 / 34 | LoRa CS / DIO0. |

### Servo / aux — 16 PCA9685 channels (SERVO1..16 = PCA ch 0..15)
Five params per channel `n`:
| Param | Default | What it does |
|---|---|---|
| `SERVOn_ROLE` **[R]** | 1 (ch 1-8) / 2 (ch 9-16) | **0 = off, 1 = PWM servo, 2 = MOSFET/switch.** The authoritative role selector. |
| `SERVOn_MIN` / `SERVOn_MAX` **[R]** | 1100 / 1900 µs | Servo output clamp. |
| `SERVOn_TRIM` **[R]** | 1500 µs | Servo output when uncommanded. |
| `SERVOn_FUNCTION` **[i]** | 0 | ArduSub-facing; **not read by SROT** (the role is `SERVOn_ROLE`). QGC Servo-page cosmetic. |

---

## B. Informational / not-yet-wired (defined but not read)
Documented so they aren't mistaken for tuning knobs:
- `PM2_VMULT` — PM2 is ESP-NOW aux (pre-scaled); unused.
- `FRAME_CONFIG` (=2) — informational; the mixer matrix is fixed vectored-6DOF.
- `MOT_PWM_TYPE` / `MOT_PWM_MIN` / `MOT_PWM_MAX` — informational; output is fixed DShot150.
- `SERVOn_FUNCTION` (all 16) — ArduSub-facing; role comes from `SERVOn_ROLE`.

---

## C. Removed: QGC-compatibility dummies
The ~87 ArduSub-plugin dummy params (`INS_*`, `COMPASS_*`, `AHRS_ORIENTATION`, `BATT_*`,
`FS_PRESS/TEMP/LEAK/EKF/CRASH/PILOT_*`, `LEAK1_*`, `FLTMODE*`, `RC*_OPTION`, `PSC_*`,
`WPNAV_*`, `LOIT_*`) were **removed** when SROT went ArduSub-independent — they were inert
(never read by firmware) and only existed to satisfy QGroundControl. The real equivalents
live above: battery = `PM1_*`/`PM2_*`; failsafes = `FS_GCS_ENABLE`/`FS_BAT_*`; leak = `LEAK_EN`
+ `PIN_LEAK`; depth = `DEPTH_*`; button map = `BTN*_FUNCTION`; modes are selected directly
(`DO_SET_MODE`) from Bondor.

*Authoritative source: `src/comms/params.cpp`. If this doc disagrees with the code, the code wins.*
