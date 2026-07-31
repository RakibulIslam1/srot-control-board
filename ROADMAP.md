# SROT — Roadmap, Research & Progress

Where the project is going (and why), the sourced research behind each decision, and the
chronological progress log. Architecture: `ARCHITECTURE.md`; hardware: `HARDWARE.md`;
parameters: `PARAMETERS.md`.

## Status (2026-07-31)
Core firmware complete + four audit rounds deep. QGC/BlueOS ArduSub compatibility working
(Motor Test, sensor cal, live Tuning). **Phase 1** precision control shipped. **Phase 2b**
(Pico 2 thruster + RPM co-processor) Stages 1–2 built — awaiting bench bring-up. Three boards
build from one project (`pio run` / `-e pico` / `-e second-board`). **Next:** Phase 2a
(model-based control), then Phase 2c (BlueOS/Jetson autonomy).

> **This build changes the LoRa air format.** Both LoRa boards — the vehicle and the ground
> station — must be reflashed together, or the link stays silent. See the progress-log entry
> below and `AUDIT.md` R17/R20.

---

## The big questions, answered

### Can a focused ESP32 system beat Pixhawk/ArduSub?
**Yes — in a narrow niche, no — in general.** For a small tethered sub doing precise
low-speed **attitude + depth + short-horizon hold**, a purpose-built controller can
match or beat stock ArduSub through specialization: lower latency, a thrust model
tuned to *your* thrusters, a 500 Hz flight loop on an isolated core, and an optional
RPM layer. Where Pixhawk stays ahead: **EKF3 sensor-fusion robustness**, sensor
breadth (DVL/USBL/sonar drivers), mature failsafes, and the whole ground-station /
logging ecosystem. Strategy: specialize where we win, don't try to reimplement EKF3.

### Micro-movements (2°/s yaw, target-angle, fine translation) — what actually enables them
Ranked by leverage (from research):
1. **Thruster deadband compensation** — props do nothing until they cross a dead zone.
2. **Thrust linearization** — thrust ∝ RPM², so equal command steps ≠ equal thrust
   (worst near zero). Invert the curve so command→thrust is linear.
3. **Low-latency, low-noise rate estimate** — our BNO085 calibrated gyro @400 Hz feeds
   the rate loop directly (good).
4. **Low end-to-end latency** — our 500 Hz isolated flight loop (good).

**Phase 1 (built) delivers all four in software, no new hardware** — see below.

### "Why not RPM-based control?" — the honest take
RPM feedback is a **refinement, not the micro-movement lever**. Its famous benefit
(the gyro RPM-notch filter) is a *vibration* tool — and vibration is largely absent
underwater (damped props, low RPM). ArduSub gets precise low-speed control with **no
RPM feedback**, purely via thrust linearization + deadband. RPM adds per-thruster
consistency (voltage/load rejection), health, and thrust self-calibration — worth
doing later, but it is not what unlocks 2°/s yaw.

### "Why can't the ESP32 get RPM itself?"
Bidirectional DShot **is** one pin (send frame, then receive eRPM ~30 µs later). The
blocker is the ESP32 **RMT peripheral**: each channel is TX *or* RX. Proven libraries
use **2 channels per motor** (one TX, one RX on the same GPIO) → 8 RMT channels ÷ 2 =
**4 motors max**. Reaching 8 would need **1 channel/motor with a custom TX↔RX flip
every frame + ISR GCR/CRC decode** — theoretically possible but **unproven and risky**
on ESP32, and still just a refinement. Practical options for RPM on all 8:
- **ESP32-only:** ESC **telemetry wire** (BLHeli_32/AM32 serial pad) shared on one
  spare UART, round-robin polled → ~62 Hz per motor. Enough for health/thrust-trim.
- **RP2040 co-processor:** its PIO does 8-channel bidirectional DShot + eRPM decode
  cleanly, streaming RPM to the ESP32 over SPI at full rate. The clean premium path.

### Sensor fusion for a GPS-denied sub — what's possible, what helps
- **Observable with IMU + baro:** attitude ✅, depth ✅. **Horizontal position ❌** —
  it drifts in seconds (accel double-integration). No algorithm fixes this without a
  velocity/position sensor.
- **You already own the fix:** the **MTF-01P** (optical flow + laser rangefinder) gives
  horizontal velocity (`flow_rate × altitude`, minus gyro) and altitude-above-bottom.
  With it, a reduced-order estimator can produce horizontal velocity + bounded
  position for station-keeping.
- **Feasible on ESP32:** keep BNO085 attitude; add a **1-D depth Kalman** (baro ⊕
  vertical accel) and a **small horizontal-velocity filter** (flow ⊕ accel). A full
  EKF3 is overkill; this fits easily on the spare core.
- **Honest caveat:** underwater optical flow is **experimental** — the laser ToF hates
  turbidity/backscatter and the flow needs a textured, lit bottom (thruster-stirred
  silt breaks it). Cheap to try; don't bet station-keeping on it alone.
- **Sensor adds ranked for precise hold:** ① **magnetometer** away from thrusters
  (cheap, fixes yaw/heading drift — directly helps precise yaw) → ② **echo-sounder /
  Ping sonar** (robust altitude where the optical laser fails) → ③ **DVL** (Water
  Linked A50, ~$4k — the real answer, bottom-referenced velocity, works in turbid
  water) → ④ USBL (topside tracking, not tight hold).

### Custom tools vs QGC
Keep **QGC** for standard telemetry/mission/setup (rebuilding it isn't worth it). Add
a **small custom dashboard** (web page over the ESP32's WiFi, or a light desktop tool
over MAVLink) for SROT-specific features: per-thruster RPM/health, thrust-curve
tuning, and a fine micro-movement joystick. Supplement QGC, don't replace it.

---

## Phases

### Phase 1 — Precision low-speed control ✅ (built + hardened, no new hardware)
> Hardened by a full-codebase audit (2026-07-25): fixed uncommanded thrust on arm,
> MANUAL roll/pitch, flash-writes-on-the-flight-loop, yaw-hold engagement, NaN guards,
> and comms lock/stream issues. See the progress log below and `ARCHITECTURE.md`.

- **Thrust linearization** (`MOT_THST_EXPO`) + **deadband/spin-min** (`MOT_SPIN_MIN`,
  `MOT_SPIN_ARM`) in the mixer → small commands produce small, predictable thrust.
- **Rate feedforward** (`ATC_RAT_*_FF`) → crisper small-command response.
- **Micro-movement command modes:** fine yaw-rate (`PILOT_YAW_RATE` deg/s + `PILOT_EXPO`),
  **yaw heading-hold** (centre stick locks heading, nudge to slew), and
  scaled/expo'd forward-lateral (`PILOT_SPEED` + `PILOT_EXPO`).
- All ArduSub-standard names where possible → QGC's Tuning/Motors pages edit them.

### Phase 2a — Hydrodynamic model-based control ✅ (built, no new hardware)
`control/feedforward.cpp`, a demand post-processing stage before the mixer (all gains
default 0/off → unchanged until tuned; math in `ALGORITHMS.md §5`):
- **Angular drag feedforward** `ATC_DRAG_*·ω·|ω|` — linearizes rotational drag (real
  measured feedforward; we have the gyro).
- **Cross-coupling** `XC_YAW2RLL/PIT·gz` — proactively counter yaw-induced roll/pitch.
- **CoB auto-trim** (`TRIM_EN`) — bleed the steady rate/depth I-term into a learned trim
  (I-term headroom / no windup — *not* energy saving). Needs pool-tuning.
- Deferred to Phase 3: linear-velocity feedforward / velocity hold (needs a velocity sensor);
  full identified 6×6 (needs system-ID).

### Phase 2b — RPM feedback via RP2350 (Pico 2) co-processor  🔨 Stage 1 built
- **Chosen:** Pico 2 co-processor, hybrid role (Pico owns DShot I/O + per-motor RPM
  loop + telemetry; ESP32 keeps allocation). UART link (header+CRC), `bastian2001/
  pico-bidir-dshot` (8-ch bidir DShot on RP2350 PIO).
- **Stage 1 (built):** link + 8-ch DShot + eRPM telemetry bridge; ESP32 backend flag
  `THRUSTER_BACKEND=PICO`, RPM → `ESC_STATUS`. See `src/pico/main.cpp`, `shared/
  thruster_link_proto.h`, `src/drivers/thruster_link.*`. Needs bench bring-up.
- **Stage 2 (built):** per-motor closed-loop RPM PI on the Pico (voltage-independent
  thrust, `THR_RPM_CLOSED_LOOP`), thruster fault detection → `STATUSTEXT` + `ESC_STATUS`.
  Needs bench-tuning of RPM_KP/KI + `THR_MAX_RPM`. Thrust estimation `k_T·RPM²` deferred
  until Phase 2a consumes it. Wiring: **`HARDWARE.md`**. Both boards build from one
  project (`pio run` / `pio run -e pico`).
- (ESP32-only alternative — ESC-telemetry-wire round-robin ~62 Hz/motor — not taken.)

### Phase 3 — GPS-denied fusion & position-hold (uses the MTF-01P; recommend a magnetometer)
- 1-D depth Kalman (baro ⊕ vertical accel) for smooth depth + vertical velocity.
- Reduced-order horizontal-velocity filter (optical flow ⊕ accel, gyro-compensated,
  attitude-rotated) → velocity + bounded position → **POSITION_HOLD** mode.
- Graceful fallback to IMU+baro when flow quality drops (it will, often).
- External magnetometer for absolute heading (fixes the one weak attitude axis).

### Phase 4 — Custom tuning/telemetry dashboard (optional)
- Lightweight UI over WiFi/MAVLink for RPM/health, thrust-curve calibration, and a
  precision joystick — supplementing QGC.

---

## Sources
- ESP32 bidir DShot limits: qqqlab/ESP32_DSHOT, derdoktor667/DShotRMT.
- RP2040 PIO DShot: bastian2001/pico-bidir-dshot, josephduchesne/pico-dshot-bidir.
- ESC telemetry: KISS/BLHeli onewire telemetry; Oscar Liang ESC telemetry.
- Thrust model: ArduPilot `motor-thrust-scaling` / `MOT_THST_EXPO`; BlueRobotics T200
  propeller characterization (thrust ∝ RPM², ±25 µs deadband).
- RPM filter purpose: Betaflight DSHOT RPM Filtering (vibration, not precision).
- Estimation: ArduPilot EKF3 optical-flow fusion; PX4 PMW3901; BlueRobotics "Optical
  Flow for ROV" thread (underwater = experimental).
- MTF-01P: MicoAir MTF-01P; ArduPilot MTF-01 docs.
- DVL / position hold: Water Linked DVL A50; ArduSub R&D DVL; ROV non-GPS nav.


---


## Progress log

> Chronological record. Older entries may name files that have since been **consolidated**
> — `context.md`/`memory.md` → `ARCHITECTURE.md` + `HARDWARE.md`; `CONTROL_LOOP.md`/
> `tuning_guide.md` → `ARCHITECTURE.md`; `WIRING.md` → `HARDWARE.md`; `plan.md`/
> `ideas_and_progress.md` → this file; `pico_thruster/` → `src/pico/`.

### 2026-07-31 — The LoRa link had no CRC, and two parameters could not be tuned

Round 4 of the audit — see `AUDIT.md` R17–R23. Seven defects from a second field report.

The headline: **the SX127x hardware CRC was never enabled on either radio.** `RxPayloadCrcOn` is 0
at power-on and `LoRa.enableCrc()` was called nowhere, so corrupt payloads went straight into the
MAVLink parser. MAVLink's own CRC rejects the bad message, but the parser is already desynchronised
by then, so the next good messages are eaten too — one corrupt packet costs several. "Misses the
attitude" and "parameter saves are unreliable" were the same fault seen from the two ends of the
link.

Also fixed: the ground station announced `STABILIZE` before it had received anything; the uplink
queue dropped writes silently when Bondor paced a LoRa hop as if it were USB; the thruster voltage
had no field in the LoRa frame at all, so it was structurally unreachable over radio; `PM1_VMULT`
silently discarded small values, making it impossible to calibrate; `PM2_VMULT` was read by no code
whatsoever; and the OLED's `--` meant three unrelated things at once.

**Flash both LoRa boards together.** The frame grew 39 → 41 bytes for the new `aux_mv` field, and
the CRC change is not backward-compatible. A mismatched pair fails safe — the link goes silent
rather than passing corruption — but it does go silent.

### 2026-07-30 — Parameters never persisted: the NVS partition was too small all along

From a field report: `"set but NOT saved (NVS full?)"` over USB **and** LoRa, parameters not loading,
payload mode not changeable, thruster voltage absent. **One root cause explains most of it, and it
predates everything in this session.** Details in [AUDIT.md](AUDIT.md) R14–R16.

- **R14 (root cause) — NVS could not hold the parameter set.** `Preferences::putFloat` stores each
  float as a **blob** (~3 NVS entries, not 1), so 227 params + calibration keys needed **~681 entries
  against 504 available**. Writes failed once it filled: values read back as defaults, `SERVOn_ROLE`
  would not stick, and `ESPNOW_EN`/`MOT_BAT_V_MAX` not saving left the thruster-voltage link down.
  Fixed with a custom `partitions_hengla.csv` — NVS 20 KB → **128 KB**, 3906 usable entries, **5.7×
  headroom** — verified by decoding the built `partitions.bin` with ESP-IDF's `gen_esp32part.py`.
  `params::init()` now recovers an unreadable NVS (`nvs_flash_init` → erase → retry) and announces a
  reformat as CRITICAL, since the sensor calibration shares the partition. `saveAll()`/`set()` also
  now skip values already stored — a full rewrite of 193 params per save was churning ~579 entries of
  garbage and driving continuous garbage collection.
  **Correction:** the earlier diagnosis that `PARAM_DEFAULTS_VER` bumps were losing tuning was
  incomplete. Bumps do discard params, but this was the deeper cause. The R10 "not saved" warning did
  not create the fault — it made a long-silent one visible.
- **R15 — on/off payload channels were unreachable over MAVLink.** `DO_SET_RELAY` maps a relay
  *instance* to channels 9–16 only, and `DO_SET_SERVO`'s pulse was discarded for role-2 channels — so
  a switch channel in the 1–8 range answered to **neither**. `DO_SET_SERVO` now treats the pulse as a
  level on role-2 channels (≥1500 µs = ON), making every channel addressable by its own number.
- **R16 — PM1 read a non-linear ADC through a linear multiplier.** Switched to
  `analogReadMilliVolts()` (factory eFuse calibration); `PM1_VMULT` becomes the divider **ratio**,
  with a stale-value detector so a pre-change backup cannot silently report a flat pack.
- **Corrected mid-implementation:** the plan claimed the OLED never displayed the thruster voltage,
  based on a declared-but-never-drawn `aux_v` field. Wrong — it reaches the display as `pm2`
  (`PM2_SRC` defaults to 2) and the top-left box already prints it, dashing when stale. It read `--`
  because the link was down, because `ESPNOW_EN` had not persisted. The dead field was removed rather
  than wired up; no new readout was added because none was needed.
- **Flashing this build needs a one-time erase** — see PARAMETERS.md. Export params **first**.
- All six envs build.

### 2026-07-30 — Round-2 audit: 13 more defects, including an inverted depth PID

Three parallel audit passes over the areas the first round had not read closely (comms/protocol,
control/tasks, drivers + the three secondary MCUs). Full evidence per finding in
[AUDIT.md](AUDIT.md) round 2. **The headline is R1 and it is the worst defect found in this
codebase so far.**

- **R1 (critical) — the depth PID's sign was inverted.** It ran `PID(target, measured_depth)`, but
  depth is positive-DOWN while heave is positive-UP, so a target *deeper* than the current depth
  produced an *ascend* command. `DEPTH_HOLD` was divergent, not mistuned. Far worse: the **SURFACE
  failsafe** — leak, low thruster battery, GCS loss — sets the depth target to 0 through this same
  loop, so the one mechanism meant to save a leaking vehicle **drove it down**. Verified numerically
  across the whole chain (setpoint → PID → mixer → physical direction): at 3 m the old code commanded
  DOWN. Fixed by running the PID in altitude (`−depth`), negating the *inputs* so the conditional
  anti-windup and derivative-on-measurement stay self-consistent. It survived this long only because
  the Bar30 had never been fitted — the loop had never once run closed.
- **R2** — autotune's depth relay had the same inversion, independently written: it drove *away* from
  the reference, giving monotonic divergence instead of the limit cycle it needs to measure.
- **R3** — STUNT and PATTERN had no `abort()`, so a panic-disarm mid-spin left the state machine
  loaded and the thrusters completed the remaining rotation on the next arm, with no command issued.
- **R4** — PATTERN's `TURN_360`/`HEADROOM`/`RETURN` steps had **no timeout** and PATTERN/STUNT were
  **outside the safety monitor entirely** — the two modes that drive full authority with no pilot in
  the loop were the only automatic modes with no guard. Added a 30 s per-step ceiling and brought
  both under the monitor (STUNT exempt from the angle guard only). PATTERN is also now refused
  without a depth sensor, alongside DEPTH_HOLD and AUTO.
- **R5** — a NaN in any `COMMAND_LONG` param passed straight through `constrain()` (built from `<`
  and `>`, both false against NaN) and reached `(int16_t)NaN` on an armed thruster. Now rejected for
  every command at the single dispatch choke point.
- **R6** — `MANUAL_CONTROL` axes were unclamped; a bad packet became a full-authority burst after
  `PILOT_EXPO` cubed it. Clamped at entry.
- **R7** — the `CAL_*` shadow cache (added earlier this session) was zero-initialised, so a first-read
  lock miss on a *scale* row reported 0.0 — which, exported and re-imported, destroys the accel/mag
  calibration. Now seeded from each row's declared default.
- **R8** — the safety monitor's own rate and depth guards **passed NaN silently** (a comparison
  against NaN is false, so `if (x > limit) fail` never fires). Both now fail closed.
- **R9** — preempting a move never resolved the displaced command, hanging its action. **R10** —
  `params::set()` reported success on a failed NVS write. **R11–R13** — zero-guards on two
  GCS-editable divisors, and two ACK-correctness fixes.
- **Deferred, needing hardware or a two-sided change** (documented in AUDIT.md): the Pico's e-stop
  line fails *permissive* (`INPUT_PULLDOWN`, active-HIGH — a broken wire reads as "run"); the LoRa
  mission-waypoint upload has no CRC and the SX127x PHY CRC is never enabled, making it the only
  wire protocol in the tree without corruption detection.
- All six envs build. Docs: `ALGORITHMS.md` §4 now spells the depth sign convention out explicitly,
  since that is what was misread.
- **Verify on hardware before any dive:** depth hold with the Bar30 actually fitted — hand-move the
  sensor and confirm it drives the *correct* way — and specifically confirm the SURFACE failsafe
  ascends. Nothing about this loop has ever been exercised closed.

### 2026-07-30 — Thruster-voltage link brought up; low-battery failsafe debounced (B12)

The 2nd board's ESP-NOW sender works on hardware: `THR 15.10 V | knob 253 deg | power ON |
tx 131 fail 0`. The SROT receive path was already complete (`espnow_link` → `aux_voltage` →
`PM2_SRC=2` → `BATTERY_STATUS` id 1 → Bondor "Battery 2", plus the OLED and `in.thr_volts`), so
bring-up is configuration: `ESPNOW_EN=1`, `MOT_BAT_V_MAX=16.8`, `FS_BAT_VOLTAGE=13.6` for the 4S
LiPo. Documented as a procedure in `HARDWARE.md`.

- **B12 (high) — the low-battery failsafe had no debounce.** It tested the **raw** 4 Hz voltage
  every 500 Hz cycle, so one low sample switched the vehicle to SURFACE. A 4S pack driving eight
  T200s sags over a volt under load, so a full-throttle burst against a 13.6 V threshold would
  have surfaced the vehicle mid-manoeuvre on a healthy pack — and the feature would have looked
  broken on its first dive. Now requires 3 s continuous (`FS_BAT_HOLD_MS`), any sample above
  resetting the timer. Leak and GCS-loss stay instantaneous: neither is analogue.
  This had never fired because `thr_volts` was always 0 with no sender, so the defect and its
  trigger arrived in the same change.
- The battery `STATUSTEXT` now **quotes the voltage** — without a number a spurious trip cannot be
  told apart from a flat pack or a miscalibrated divider.
- **ESP-NOW link up/lost notice** (`task_lora_sd.cpp`), same edge-triggered pattern as the IMU and
  Pico-link notices. Link loss degrades safely but *silently*: the voltage compensation stops and
  the failsafe goes inert again, and the pilot had no way to know the compensation they tuned
  around had switched itself off.
- Docs: `HARDWARE.md` bring-up section (incl. that `fail 0` on the sender only means the MAC
  accepted the frame — broadcast is unacknowledged, so the receiver's STATUSTEXT is the only
  proof), `PARAMETERS.md` `FS_BAT_*` note, `docs/T200_PROFILE.md` worked 4S example, `AUDIT.md`
  B12.
- **Calibrate the divider before trusting any of it:** the 2nd board's `PM1_VOLT_MULT` is
  inherited and the ESP32 ADC is non-linear above ~2.5 V. Check against a multimeter at the pack.

### 2026-07-30 — Firmware named Hengla; full audit (9 defects fixed); 2nd-board env

**Naming.** The board is **SROT**; the firmware is now **HENGLA**. Minimal by intent: `config.h`
§0 identity, the boot banner, the OLED version line and `AUTOPILOT_VERSION` (vendor `'SR'` =
board, product `'HG'` = firmware, `flight_custom_version = "HENGLA"`). Deliberately **unchanged**:
every parameter name, `MAV_CMD_SROT_MOVE` and its 31000 wire id, the NVS namespaces
(`srot_prm`/`srot_cal` — renaming those silently wipes params + calibration), the LoRa frame
magic, Bondor's `protocol.ts`/`srotParams.ts`, and the repo names.

**De-ArduSub.** Deleted the `APM_COMPAT_*` macros — they were **dead code nothing referenced**,
yet `config.h` §0 still claimed "to QGC we advertise ArduSub 4.1.0". The heartbeat has reported
`MAV_AUTOPILOT_GENERIC` for a while, so the disguise was already gone in behaviour and only alive
in the comments. Dropped the "ArduSub-clone" headers and rewrote the lineage comments to describe
behaviour. Parameter IDs keep the ArduPilot convention on purpose (descriptive, and it keeps
`.params` backups valid) — stated honestly in `PARAMETERS.md` rather than implied.

**Audit — see [AUDIT.md](AUDIT.md) for evidence per finding.** Nine real defects, all fixed:
- **B1 (high):** a `SROT_MOVE` that completed during a parameter download **never sent its
  terminal ACK** — `updateMove()` sat below the download throttle and only latched while
  `mv_active`. The handler had already replied `IN_PROGRESS`, so a ROS action hung for ever. Also
  surfaced a second hang: the handler forces AUTO but the loop *refuses* AUTO with no depth
  sensor, so the move never started and nothing resolved it — now an explicit `FAILED`.
- **B2 (high):** `RPM_LOOP` was **half-wired** — the ESP32 gated on compile-time
  `THR_RPM_CLOSED_LOOP` while the Pico got the runtime param, so setting it to 1 did nothing on
  this side. Both now read the param; the `#define` is just its boot default.
- **B3 (high):** **AUTO + `STYLE` disarmed itself every time** — a commanded 360° roll trips the
  70° tumbling guard immediately. The angle guard is now suppressed for a commanded spin; rate,
  depth and RPM guards still apply.
- **B4-B6 (PID, `pid.h` rewritten, 18 host assertions):** no anti-windup — the integrator kept
  filling while the output was saturated, leaving a **permanent 4.5%-of-full-torque bias per
  axis** that an integrator can never decay (measured; now 0.000%). No derivative filter — a raw
  500 Hz difference multiplies gyro noise ×500, which is *why* D could not be raised; now a
  20 Hz one-pole (`PID_D_FILT_HZ`), with old/new agreeing <2% on smooth input so the tune is
  preserved. And one NaN **permanently poisoned** an integrator (`constrain(NaN,…)` returns NaN)
  — depth was the unguarded path.
- **B7 (latent):** the CoB auto-trim only ever *added* to its trim and never bled the PID
  integrator, so the same charge was re-transferred every cycle and the trim pinned at its clamp
  in under a second. Now a true conservative hand-off via `bleedIntegral()`.
- **B8 (latent):** AUTO was excluded from the drag/cross-coupling feedforward despite running the
  same cascade — the model terms silently skipped the companion's mode.
- **B9:** the AUTO depth-runaway guard was a **permanent no-op** (`depth0 = current depth`), so
  AUTO had no depth protection at all. Now guards against `depth::target()`.
- Plus stale comments corrected (200 Hz claims that were really 400/500, a stray **"Mongla"**
  reference, the `DEF_ST_RPM_MAX` justification).

**New `env:second-board`** — `example/2nd_board_firmware/` was 240 lines compiled by nothing.
Promoted to `src/second_board/` and given the **ESP-NOW voltage sender it never had**: it only
ever *received* (a C3 Mini's service voltage, purely to draw on its OLED). That is why
`mixer::setBatteryVoltage()` and the low-thruster-battery failsafe were **both inert** — they
consume a thruster-pack voltage nothing transmitted, and the control board's own ADC reads the
*electronics* pack. Now broadcasts `magic/kill/voltage` at 4 Hz. Display code removed (the OLED
lives on SROT), and the C3 receive dropped with it. Wire format moved to
`shared/espnow_proto.h` so both boards compile one definition instead of a hand-copied comment.

**New docs:** [AUDIT.md](AUDIT.md), [docs/VS_ARDUSUB.md](docs/VS_ARDUSUB.md) (including a
substantial section on where **ArduSub is still clearly ahead** — no EKF, no position
estimation, no replay logging, no redundancy, far more field testing), and
[docs/BLUEOS.md](docs/BLUEOS.md).

**Recommended upgrades, ranked by value per unit of risk:**
1. ~~B1-B3~~ (done).
2. ~~PID D-filter + anti-windup~~ (done) — now actually try raising `ATC_RAT_*_D` in the pool;
   it was previously unusable.
3. **`SET_MESSAGE_INTERVAL` (511) + a per-message rate table.** Currently rates are fixed, so a
   companion cannot turn them *down* for a LoRa link. Cheap, and the standard mechanism.
4. **SD binary logging at loop rate.** The single biggest debugging gap versus ArduSub: there is
   no way to analyse a dive after the fact. Everything else on this list is easier once it exists.
5. **A depth-*rate* inner loop.** DIVE currently ramps a position setpoint at `MOVE_DEPTH_RATE`;
   a velocity loop would track a commanded descent rate properly and make `p3` meaningful.
6. **`THR_POLE_PAIRS` as a runtime param** (compile-time today; 7 is T200-specific).
7. **Gyro notch filter driven by the measured RPM.** This is the *classic* use of bidirectional
   DShot and the one thing the RPM data is not yet used for — it is what would let the rate gains
   go meaningfully higher.
8. **Motor-current sensing on the 2nd board** for power-based thruster health (a fouled prop
   draws current without RPM).
9. Optional: complementary yaw-drift correction gated on thruster current, as the honest
   successor to the one-shot magnetic alignment.

- Build: all six envs SUCCESS; Bondor untouched (typecheck clean).
- **Verify on hardware:** B1 (start a short move, trigger a param download, confirm the terminal
  ACK still arrives); B2 (`RPM_LOOP=1` closes the loop with no rebuild); B3 (`STYLE` in AUTO does
  not disarm); the 2nd-board link end-to-end (`ESPNOW_EN=1` + `MOT_BAT_V_MAX` → real thruster
  voltage in Bondor, stale within 2 s of powering it down, knob toggles `KILL`).

### 2026-07-30 — Param backup/restore, explicit heading lock, one-shot magnetic yaw reference
Three problems that surfaced from actually flying the board.

- **Params were being lost on reflash — and it was our own doing.** NVS survives an upload, but
  `PARAM_DEFAULTS_VER` has been bumped **four times**, and each bump makes `params::init()`
  rewrite every parameter from its `DEF_*`. There was no way to back tuning up. Fixed with
  **Export/Import in Bondor** (`Parameters` tab): a QGC-compatible `.params` file, a **diff
  dialog** before anything is written, a throttled write **confirmed per-parameter against the
  `PARAM_VALUE` echo** with retries, and named failures (a half-applied restore reporting
  success would be the worst outcome). Export is blocked until the full list has downloaded —
  a partial file would silently restore defaults for what was missing.
  New `bondor/src/renderer/src/utils/{files,paramFile}.ts`, `writeParams()` in the store.
- **Calibration is now in the backup too.** `g_state.cal` (accel/mag/level trim + the 8
  MOTOR_DETECT directions) lives in `NVS_NS_CAL` and was unreachable over MAVLink — the most
  painful thing to redo, since mag cal means spinning the vehicle. Added **26 `CAL_*`
  parameters** as a **view**, not a copy: `Desc.cal_id` + `calGet`/`calSet` read/write
  `g_state.cal` under `mtx_cal` and persist via the existing deferred `persist_pending` →
  `calibration::saveToNVS()` path. They own **no key in `NVS_NS_PARAMS`** (one source of truth;
  NVS `0x5000` has no room), and are excluded from the defaults-bump rewrite, from
  `resetAllToDefaults()`, and from `saveAll()`. A lock miss on read returns the last known value
  rather than 0 — reporting 0 would let a GCS write a zeroed cal back over the real one.
  Also added `SYS_PARAM_VER` **[i]** so an exported file records the schema it came from.
  **`PARAM_DEFAULTS_VER` deliberately NOT bumped** — these are additions, so absent NVS keys
  fall back to their defaults and existing tuning survives this update.
- **Heading lock in AUTO is now explicit, and a real TURN bug is fixed.** Heading hold already
  worked (a centred yaw demand latches `s_yaw_target`), but nothing *guaranteed* it. Added
  `attitude::holdYaw()`; `movement` raises a one-shot `Demand.yaw_lock` so **every translate leg
  (`FWD`/`BACK`/`LEFT`/`RIGHT`/`HOLD`/`STOP`/`DIVE`) locks the heading it started with**, held
  through cruise and braking. **Bug:** a completed `TURN` inherited `measured_yaw` at the instant
  the ±0.03 rad (~1.7°) completion test passed, so "turn to 90°" settled anywhere in 88.3–91.7°
  and then held that error — compounding across a sequence of turns. It now locks the
  **commanded** `s_target_yaw`. `ARC` excluded (it commands yaw rate). `MANUAL` unchanged: raw
  passthrough, no holds — it must stay a genuine unstabilized escape hatch.
- **One-shot magnetic yaw reference** (`control/yaw_ref.{h,cpp}`, `MAG_YAW_REF` default **0**).
  Attitude stays on `GAME_ROTATION_VECTOR` (mag never fused, so thruster current can't move it);
  the mag is read **once** at boot — disarmed, still, ≥100 samples over ≥1 s — to compute
  `offset = magnetic_heading − game_yaw`, applied at the single point where yaw is published so
  heading-hold, absolute `TURN`, `ATTITUDE` and `VFR_HUD` cannot disagree. Tilt-compensated
  (a bare `atan2(my,mx)` is only valid level); headings averaged as **unit vectors** (an angle
  mean lands 180° wrong across ±π), with the resultant length doubling as the stability gate.
  **Fails safe:** low mag accuracy, field outside 25–65 µT, or unstable samples all *refuse*,
  leaving offset 0 = today's relative yaw. Every outcome, refusals included, goes out as
  `STATUSTEXT`. `MAG_ALIGN` re-triggers, refused while armed. `BNO_USE_MAG` back to **1** — that
  enables the mag *report* only, never the fusion.
  **Honest limitation, documented everywhere:** this fixes the *reference*, not the *drift*
  (~0.5–3 °/min remains). Fixing drift needs a continuous mag correction, i.e. exactly the motor
  sensitivity the 6-axis fusion exists to avoid.
- Docs: `PARAMETERS.md` (backup warning, `MAG_*`, `CAL_*`), `JETSON_COMMS.md`
  (**sign-conventions table** — depth positive-down internally, `VFR_HUD.alt = −depth` on the
  wire — plus the heading-lock guarantee), `ALGORITHMS.md` §1/§3, `ARCHITECTURE.md`,
  `bondor/README.md` (the reflash routine).
- Build: ESP32 **SUCCESS** (RAM 24.2%, Flash 29.1%); Pico unaffected; Bondor typecheck + build clean.
- **Verify on hardware:** export→bump `PARAM_DEFAULTS_VER`→flash→import round-trip incl. a
  `CAL_*` and `CAL_MDIR*`; absolute `TURN 90` settling at 90.0°; mag alignment at four compass
  headings (watch for a **mirrored** tilt-comp axis) and that yaw does *not* jump under full
  thrust; and the refusal path with the mag deliberately disturbed.

### 2026-07-28 — ESP32 4-way ESC flasher env (flash 4 ESCs at once); dropped the Uno env
- The Arduino Uno 4-way firmware (BrushlessPower 328P) **would not handshake** with esc-configurator
  reliably (SoftwareSerial/reset). Replaced it with an **`esp32_4way` env** (`src/esp32_4way/`): the
  ESP32 version of BrushlessPower/BlHeli-Passthrough, BLE stripped, presenting MSP+4Way over USB like
  a Betaflight FC. **SROT addition: 4-ESC routing** — MSP motor/ESC count set to 4, and the 4-Way
  `DeviceInitFlash`/`DeviceReset` ESC index routes to **GP16/17/18/19** via `selectEsc()` (single-pin
  half-duplex per ESC, **no resistor**). So esc-configurator can read/flash Bluejay/config **4 ESCs at
  once** over USB. Uses the ESP32 core's `EspSoftwareSerial`. Removed `env:uno`, `src/uno_4way/`,
  `lib/SoftwareSerial/`, and the `lib_ignore = SoftwareSerial` lines. Docs: `src/esp32_4way/README.md`,
  `docs/ESC_FLASHING.md`.
- Builds clean: all four envs (`esp32doit-devkit-v1`, `pico`, `groundstation-esp32`, `esp32_4way`).

### 2026-07-27 — Selectable DShot mode (bidir vs normal) for any ESC + Pico USB debug
- **`DSHOT_BIDIR` param** — the Pico now supports **normal (non-bidirectional) DShot** alongside the
  default bidir. `1` = bidirectional (inverted, RPM telemetry → detection/closed-loop/constant-
  distance, needs a bidir ESC); `0` = **normal DShot** via `DShotX4` (4 ch/PIO), which drives **any
  DShot ESC** (stock BLHeli_S, ESCs previously on a Pixhawk) so they beep/arm/spin — no RPM. Pushed
  via `tl_cfg_t.dshot_bidir`; the Pico recreates its PIO drivers on change **while disarmed** (both
  driver classes have destructors). Bidir path unchanged. Set it in Bondor → Motor RPM loop → "DShot
  mode". (Root cause: our board is DShot-only and was bidir-only; a stock/PWM-configured BLHeli_S
  can't speak bidir DShot, so it wouldn't beep/detect — normal mode lets it run without Bluejay.)
- **Pico USB debug** (~2 Hz, `Serial` @115200): `link/armed/rpm_mode/loop` + per-motor `rpm`/`pres` to
  see whether an ESC answers.
- Builds clean: `pico`, `esp32doit-devkit-v1`, Bondor.

### 2026-07-27 — Uno 4-way flasher env + LoRa param gap-fill
- **`env:uno`** — a **4th env** in the project's `platformio.ini` (`src/uno_4way/`) that flashes an
  **Arduino Uno** as a BLHeli **4-way interface** for esc-configurator.com (source vendored from
  BrushlessPower/BlHeli-Passthrough 328P; `lib/SoftwareSerial` patched to RX buffer 300 — `lib_ignore`d
  on the ESP32/Pico envs). `pio run -e uno -t upload`. Wire ESC signal→D11, 1 kΩ D11↔D10, common GND,
  ESC on its own power. Builds clean (Flash 24%). Docs: `src/uno_4way/README.md`, `docs/ESC_FLASHING.md`.
- **LoRa param download fixed (stalled ~50/189):** downlink param frames drop on the lossy link with
  no recovery. Now the C3 has a 24-slot **uplink queue** (not last-wins) so bursts of requests get
  through, and Bondor **re-requests missing param indices** when the stream stalls
  (`PARAM_REQUEST_READ` by index, 8 at a time every 0.9 s) until all arrive — standard MAVLink
  gap-fill, self-healing on any lossy link.
- Builds clean: `groundstation-esp32`, Bondor, `tools/uno_4way`.

### 2026-07-27 — ESC safety (open-loop fallback) + Bluejay guide + LoRa config-plane
- **Loop mode is now a user choice (`RPM_LOOP` param), not an auto-fallback:** `1` = closed-loop RPM
  (PI — the main target, needs bidir-DShot/Bluejay), `0` = open-loop (feedforward, works with any
  ESC). Pushed live to the Pico via `tl_cfg_t.loop_mode`; set it from Bondor (Parameters → Motor RPM
  loop → "Control loop"). Closed-loop keeps **anti-windup** (integrator only advances on a real
  measurement) so a missing sample can't wind it to full — that's correct control, not a mode switch.
- **Bluejay flashing guide** (`docs/ESC_FLASHING.md`) — flash Bluejay via an **Arduino Uno as a
  4-way interface** (BLHeliSuite "Make Arduino Interface" → esc-configurator → Bluejay + bidir DShot).
  Explains why it's needed (bidir DShot = RPM telemetry = detection + closed loop + constant distance).
- **Params/config over LoRa:** low-rate control-plane MAVLink (`PARAM_VALUE`, `COMMAND_ACK`,
  `STATUSTEXT`, `AUTOPILOT_VERSION`) is now **tapped in `mav::tx` into a 16 KB ring and relayed over
  LoRa** (raw `0xFD` frames, ~2/beacon) — only while a LoRa GCS is active. The C3 classifies downlink
  by first byte (`0xC5` telem → bridge; `0xFD` → straight to USB), so Bondor's Parameters / Motors /
  Payload / Tuning tabs now work over LoRa (full load ~10–15 s). **GCS-lost fixed** — the vehicle now
  treats ANY LoRa uplink as GCS activity (`mav_commands::feedGcs`), not just heartbeats. **"No
  heartbeat" fixed** — the C3 sends a steady cached-value heartbeat to Bondor when telem stalls.
- Builds clean: `esp32doit-devkit-v1` (RAM 23.5%), `pico`, `groundstation-esp32`.

### 2026-07-27 — LoRa uplink fixed: RX-window (parsePacket RX_SINGLE) + C3 USB-CDC DTR
- **Commands didn't reach the vehicle** (arm/mode/cal dead) while downlink telemetry was fine. Two
  bugs: (1) **C3 USB CDC RX** — Bondor opened the port with `dtr:false`; the ESP32-C3 native USB
  needs **DTR asserted** to enable host→device data, so the ground station never got the commands.
  Fixed in `serialLink.ts` (`dtr:true, rts:false`). (2) **Control-board LoRa RX timing** —
  sandeepmistry `parsePacket()` uses one-shot **RX_SINGLE**, so the board only hears a reply if it's
  armed-and-listening exactly then; polling at 20 Hz (and calling `receive()` which the next
  parsePacket clobbers) meant it was usually deaf to the uplink. Fixed by opening a **~70 ms RX
  window** (tight `poll()` loop) right after each telemetry beacon — the master listens for the
  ground station's TDM reply. Removed `LoRa.receive()` from `sendTelemetry`.
- **Diagnostics:** C3 LED now flashes only on a command received from Bondor over USB (instant proof
  of the USB hop); LoRa tab shows USB_RX / UP_TX / UL_RX counters to localise any remaining break.
- Builds clean: `esp32doit-devkit-v1`, `groundstation-esp32`; Bondor. **Reflash the control board**
  (RX-window fix) and rebuild Bondor (DTR).

### 2026-07-27 — LoRa TDM (ELRS-style): fix "nothing works" + GCS-lost + Dive clipping
- **Root cause of the dead link:** both radios transmitted autonomously on one frequency — the C3
  ground station's own heartbeat/forwarding TX **blocked its receive**, so it missed the control
  board's telemetry → little/no telemetry reached Bondor (only stale mode/arm chips remained).
- **Fix — half-duplex TDM (ELRS-inspired):** the control board is the **master**, beaconing telemetry
  at ~8 Hz (120 ms; `task_lora_sd.cpp`) and listening the rest of the time. The ground station is a
  **slave** that transmits **exactly one** uplink packet immediately after receiving each downlink
  frame (`sendUplinkSlot`), so the two never transmit at once. Each slot carries, by priority: a
  queued command (idempotent 3×, SROT_MOVE 1×) → fresh joystick → a GCS keep-alive heartbeat — so the
  vehicle sees the GCS every ~120 ms and **GCS-lost stops**. Uplink only while Bondor USB is active
  (failsafe intact when unplugged). Removed the autonomous 2 Hz heartbeat + immediate-forward TX.
- **GCS-lost debounced** ~1.5 s in `task_buzzer` so a rare blip doesn't beep/log.
- **Bondor Dive clipping fixed** — the fixed-height cockpit made the left column overflow with
  `overflow:hidden`, hiding the command panel; the left column now scrolls internally (page still
  doesn't scroll).
- Builds clean: `esp32doit-devkit-v1`, `groundstation-esp32`; Bondor typecheck+build. **Both boards
  must be reflashed** (shared LoRa config + matched TDM cadence).

### 2026-07-27 — LoRa link stability (mode/arm flicker, GCS lost/found, jitter)
- **Root cause of the mode/arm flicker:** the ground station sent a 1 Hz keep-alive HEARTBEAT with
  hardcoded `mode=0, armed=false`, interleaved with the real 4 Hz telemetry heartbeats → Bondor
  alternated true↔fake (and the Arm toggle sent the wrong command half the time). **Fixed:** the
  bridge now sends **only** frame-derived heartbeats (real cached mode/armed); the fake keep-alive is
  gone.
- **GCS lost/found + drops:** the ground station now synthesises a **steady 2 Hz GCS heartbeat toward
  the vehicle** (gated on Bondor USB being active < 2 s, so failsafe still works when unplugged)
  instead of relaying Bondor's bursty 1 Hz beat; `GCS_FAILSAFE_MS` 3000 → **5000**.
- **Reliability/throughput:** idempotent commands (ARM_DISARM / DO_SET_MODE / SET_MODE) are sent **3×**
  over LoRa (SROT_MOVE et al. once — no duplicate seq); LoRa runs at **250 kHz BW / SF7 / CR4-5**
  (shared macros in `shared/lora_telem_proto.h`, applied on both ends) to halve airtime and cut
  collisions → smoother attitude, fewer dropped beats. `mavlink_msg_command_long_get_command` used to
  classify.
- Builds clean: `esp32doit-devkit-v1`, `groundstation-esp32`.

### 2026-07-27 — Bondor refinements: Dive cockpit, stability fix, bidirectional LoRa
- **Stability (hang) fixed** — the store no longer re-clones per-message inspector maps into React
  state (moved to a non-reactive registry read on a timer), views use selectors, and hot instruments
  read a **throttled 15 Hz snapshot** (`useThrottledTelemetry`) so render rate is decoupled from
  message rate. Param download is coalesced (~80 ms flush).
- **Dive tab** (renamed from Fly) — fixed cockpit layout where **only the message log scrolls**;
  added an HSI-style **heading compass**, **two battery** readouts (PM1+PM2 via `BATTERY_STATUS.id`),
  ROV-style quick mode buttons (Manual/Stabilize/Depth) + joystick, and **inline pilot tuning**
  (JS_GAIN/PILOT_YAW_RATE/SPEED/EXPO live sliders). **Params auto-load on connect.**
- **Analyze** now has dedicated **IMU** and **Motor-RPM** charts; **Payload** tab added (per-channel
  role Off/Servo/Switch + live µs slider / on-off test via `DO_SET_SERVO`); **Motor-Tune Enable** is a
  proper two-way toggle.
- **Bidirectional LoRa** — the ground station moved to an **ESP32-C3 Super Mini** (`lolin_c3_mini`,
  its own pins, USB-CDC) and is now a full bridge: telemetry **down** + **uplink** of MAVLink from USB
  (commands always; MANUAL_CONTROL decimated to ~8 Hz). The control board classifies LoRa packets by
  first byte (`0xFD` MAVLink uplink → `mav_commands::handle` — so arm/mode/**autotune/motor-tune**/
  SROT_MOVE/joystick work over LoRa; `0xC5` telem; else mission). UART0 TX made mutex-safe now that
  two Core-0 tasks write it. Limit: command ACKs/PARAM_VALUE still return over UART0, not LoRa.
- Builds clean: `esp32doit-devkit-v1`, `pico`, `groundstation-esp32` (esp32c3); Bondor typecheck+build.

### 2026-07-27 — Bondor Phases 2–6 + USB link; firmware goes SROT-only; LoRa telemetry
- **USB is now a first-class connection** in Bondor alongside UDP: a native serial transport
  (`serialport`) to the ESP32 UART0/USB MAVLink stream (115200), opened with DTR/RTS de-asserted so
  it doesn't reset the DevKit. Refactored the IO into a transport-agnostic `MavlinkLink`
  (`UdpLink` + `SerialLink`); Connect dialog enumerates COM ports.
- **Firmware went SROT-native (dropped the ArduSub disguise):** heartbeat autopilot
  `ARDUPILOTMEGA → GENERIC`, `AUTOPILOT_VERSION` reports SROT's own version (no ArduSub spoof), and
  the ~87 QGC-compat **dummy params were removed** (`params::count()` 255 → 189). QGC is no longer
  supported — Bondor is the GCS. Docs updated (`PARAMETERS.md`).
- **Bondor Phases 2–5:** Parameters editor (full PARAM download/edit/save + a SROT metadata layer
  giving friendly labels/groups/enums), MAVLink **Inspector** (per-message Hz + fields), **Vehicle
  Setup** (sensor cal flows, motor test/reverse/detect, config groups), **Tuning** (PID groups +
  live attitude chart + AUTOTUNE & MOTOR_TUNE consoles), expanded **Modes** (stunt/pattern/surface),
  **Analyze** black-box (live plots + 20 Hz record → CSV).
- **Phase 6 — LoRa telemetry:** compact `shared/lora_telem_proto.h` frame TX'd from the control
  board (`lora_mission::sendTelemetry`, ~4 Hz from Task_LoRa_SD); a **new `groundstation-esp32`
  PlatformIO env** = a LoRa→MAVLink bridge (RX frame → re-emit MAVLink over USB) so Bondor connects
  to the ground receiver as a normal serial link; Bondor **LoRa page** shows RSSI/frame count.
- Builds clean: `esp32doit-devkit-v1`, `pico`, `groundstation-esp32`; Bondor typecheck + build.
  **Mission (Phase 7) remains queued** (stub tab).

### 2026-07-27 — Bondor ground-control app started (Phase 1) — `bondor/`
- **Bondor** = the SROT-native GCS (own app, in `bondor/`, isolated from the PlatformIO build).
  Electron + React + TS + **MUI Material 3** (light-purple, dark/light). Electron chosen over Tauri
  because the machine has Node but no Rust toolchain (Tauri fallback per the approved plan).
- **Phase 1 shipped:** main-process MAVLink IO over **UDP** (`node-mavlink`, GCS heartbeat @1 Hz,
  generic decode → renderer), preload contextBridge, Connect dialog (Direct UDP; MAVLink2Rest slot
  reserved), telemetry store, **Fly view** (attitude HUD, heading/depth/battery/water, thruster RPM,
  arm/disarm, mode select, reboot), **STATUSTEXT console**, **joystick** (Gamepad→MANUAL_CONTROL
  @25 Hz), and a working **SROT Move console** (SROT_MOVE 31000 + ACK feedback).
- Verified: `npm run typecheck` + `build` clean; a headless codec self-test round-trips a SROT_MOVE
  COMMAND_LONG; ESP32 firmware env still builds (no collision). Phases 2-7 in `bondor/README.md`.
- Contract Bondor speaks is in `JETSON_COMMS.md`. Queued: LoRa telemetry (Phase 6, needs firmware),
  mission upload (Phase 7).

### 2026-07-27 — Constant-distance timed moves (RPM loop on), distance-estimate removed, LoRa retry
- **Enabled the Pico RPM closed loop** (`THR_RPM_CLOSED_LOOP 0→1`) so a speed command is sent as a
  held **target RPM** (`norm·THR_MAX_RPM`) → a **timed** move covers the **same distance every run**
  regardless of battery voltage. Motor-test / MOTOR_TUNE still bypass it (raw DShot). Precondition:
  fit the RPM gains via **MOTOR_TUNE in water** (documented) — defaults track but may over/undershoot.
- **Removed the distance-estimate system** per user (pure timer-based): dropped `MOVE_KV`,
  `MV_DIST` telemetry, the `estDist()`/RPM integral in `control/movement`, `mv_est_dist` state, and
  the calibration doc section. Duration/speed/brake/ramp logic unchanged.
- **LoRa "no module" fix** (`drivers/lora_mission`) — detection was one-shot with no diagnostic.
  Now **retries `LoRa.begin()` every ~2 s** while down (a late/loose/reconnected radio comes up with
  no reboot), and on failure logs the raw **REG_VERSION** to the OLED (`LoRa? v=0x00` MISO/power ·
  `0xFF` floating/no-power · `0x12` present→retry catches it); `LoRa OK` on success.
- Docs updated: `JETSON_COMMS.md`, `ALGORITHMS.md` §11, `PARAMETERS.md`. Builds clean: ESP32 + Pico.

### 2026-07-27 — AUTO movement mode: Jetson-offloaded, RPM-braked motion primitives
- **`FlightMode::AUTO=23`** + **`control/movement`** — one state machine for every high-level verb
  (forward / back / strafe / turn / dive / arc / style / stop / hold). The Jetson ("Mongla") sends
  intent; the ESP32 owns the primitive: heading + depth hold, ramps, and braking.
- **Voltage-independent + on-board braking (no drift)** — cruise runs at an **RPM-held** speed (via
  the Pico loop), so duration×speed = same distance at any battery; at the end the ESP32 applies
  **reverse thrust scaled by cruise speed** to null momentum instead of coasting. Repeatable stop.
- **Smooth dive** (ramped depth setpoint at `MOVE_DEPTH_RATE`, no splash); **shortest-path turns**
  (relative or absolute-to-BNO-heading, rate-limited); **arc** = forward + turn; **style** delegates
  to the spin controller. Every command has a timeout, is **preemptible**, and runs under the safety
  monitor (tumble/spin/NaN → abort + disarm; no depth-runaway since dives are intended).
- **Custom MAVLink move API** — `MAV_CMD_SROT_MOVE (31000)` over `COMMAND_LONG` (no dialect regen);
  auto-enters AUTO, streams `COMMAND_ACK` IN_PROGRESS (~3 Hz, progress %) then ACCEPTED on completion
  (drives the ROS Move-action result), plus `MV_STATE/MV_PROG/MV_DIST/MV_TYPE` telemetry.
- **Params** `MOVE_*` (cruise/accel/brake/depth-rate/yaw-rate/KV); QGC-compat params **kept** for now.
- **Docs** — new **`JETSON_COMMS.md`** (full Jetson→AUV contract: link, arm, mode, MANUAL_CONTROL,
  the `SROT_MOVE` table + ACK contract, telemetry, payload, pymavlink, `MOVE_KV` calibration);
  `ALGORITHMS.md` §11 (move + brake + smooth-dive math); `PARAMETERS.md` MOVE_* group.
- Builds clean: ESP32 + Pico.

### 2026-07-26 — Autotune modes (flight-loop + motor-RPM) + best RPM control
- **AUTOTUNE mode** (`FlightMode::AUTOTUNE=21`) — the relay (Åström–Hägglund) tuner now runs as a
  proper mode: rate→angle→depth, hardened with Schmitt-trigger hysteresis + gain clamps + per-phase
  OLED progress, saves + disarms on completion.
- **Safety monitor** (`control/safety_monitor`) — during any tune (armed), a tumbling angle
  (`ST_ANGLE_MAX`), spin-out rate (`ST_RATE_MAX`), depth runaway (`ST_DEPTH_DELTA`), over-RPM
  (`ST_RPM_MAX`) or NaN **auto-disarms** with a reason.
- **Best Pico RPM control** — per-motor loop upgraded to **feedforward + PI + Dynamic Idle
  (min-RPM) + slew limit**; gains are runtime-tunable via a new **cfg frame** (`tl_cfg_t`,
  ESP32→Pico, magic 0xAA57) resent ~1 Hz (survives a Pico reset).
- **MOTOR_TUNE mode** (`FlightMode::MOTOR_TUNE=22`, gated by `MTUNE_EN`) — ESP32-orchestrated,
  per-motor, **in water**: (1) ramp to find min-spin/idle, (2) fit feedforward `FF_A`, (3) relay-tune
  the PI (ZN); averages across motors → `RPM_KP/KI/FF_A/IDLE` + `MOT_SPIN_MIN`, saves + pushes to the
  Pico. Over-RPM / vehicle-motion aborts + disarms. Research: ArduPilot twitch method + Betaflight
  Dynamic Idle.
- Builds clean: ESP32 + Pico.

### 2026-07-26 — Display continuity, warm-reset freeze, Pico-link buffers
- **OLED periodic blackout fixed** — the 5 s "self-heal" was calling `begin()` (DISPLAYOFF +
  100 ms delay + Adafruit logo). Replaced with a non-destructive `keepAlive()` (DC-DC +
  DISPLAY-ON only). Boot line now shows `g_reset_reason`.
- **EN/brown-out reset freeze fixed** — added **I2C bus recovery** (clock a stuck slave free)
  for both buses before `Wire.begin()`; BNO cold-boot `delay(3000)` → 300 ms on warm resets.
- **Pico-link "restored" flap fixed** — root cause = tiny serial buffers at 2 Mbaud: ESP32
  Serial2 RX **256→1024** (+TX 512), Pico `Serial1` FIFO **32→1024**. Plus
  `PICO_LINK_TIMEOUT_MS 200→500` to ride through brief blips. (A residual idle QGC-disconnect
  is a 5 V dip glitching the Pico + USB-UART bridge — electrical; see HARDWARE.md.)
- **UI redesign** — dashboard (volt boxes, arm circle centred, network P/L/G, depth bucket,
  fixed-position R/P, centred yaw, scrolling log ticker fed by STATUSTEXT + beep reasons).
- Builds clean: ESP32 + Pico.

### 2026-07-26 — Pins, not-detected diagnostic, buzzer melodies, boot splash, ESC beeps
- **Pins:** BNO INT → **GPIO4** (non-strapping), RGB → **GPIO2**, **GPIO0 freed**. New
  HARDWARE.md "Expansion / free GPIO" section (0/12/13/14/15 + I2C/SPI buses free in the PICO
  backend). Strapping-safety comments updated.
- **Thruster "not detected" vs "fault":** Pico now sets `TL_ST_PRESENT` from live telemetry;
  no telemetry = "not detected" (ESC absent or bidir-DShot off), present-but-stalled = fault.
  ESP32 reports each distinctly. A bare bench no longer spams "fault".
- **MOT_n_DIRECTION now works on the Pico backend** (baked into published `norm[]` — was
  dropped in closed-loop RPM mode). Setup-time reversal reverses a thruster in both modes.
- **Buzzer overhaul:** dedicated **200 Hz `Task_Buzzer`** (uniform beep lengths — was jittery
  on the 30 Hz UI task). Rick & Morty startup melody + ocean-feel sweep/warble event tones
  (Pico lost/restored, GCS lost, leak). Per-situation **`BUZZ_MASK`** toggle. 2N2222 base-
  resistor note.
- **ESC beacon beep** (`THR_BEEP_EN`) — Pixhawk-style chirp through the thrusters on
  startup/connect via DShot beacon command (BLHeli's 5 tones; not a melody).
- **OLED "Booting SROT" splash** on power-up.
- Builds clean: ESP32 + Pico.

### 2026-07-26 — Docs reorg (9→5), two-board build, hardware fixes, Phase 2a
- **One project, two boards:** Pico firmware → `src/pico/main.cpp`; one `platformio.ini`
  with `esp32doit-devkit-v1` + `pico` envs (per-env `build_src_filter`). Both build clean
  here (`pio run` / `pio run -e pico`). Fixed a Pico enum-scope
  (`ERPM`→`BidirDshotTelemetryType::ERPM`).
- **Docs consolidated 9→5** (README, HARDWARE, ARCHITECTURE, PARAMETERS, ROADMAP) + a 6th
  by request: **`ALGORITHMS.md`** (every algorithm in full detail with the math).
- **Hardware fixes:** Pico onboard-LED power+status indicator (solid/heartbeat/slow/fast);
  buzzer tones raised into the ~2.5-3.1 kHz loud band; **removed `PIN_THR*` params** (RMT
  fallback uses fixed `THRUSTER_PINS`); **Pico DShot pins → GP6..GP13**; plain-language
  pull-up explanation (external resistor now optional-for-reliability).
- **Phase 2a built** — `control/feedforward.cpp`: angular drag FF (`ATC_DRAG_*`), cross-
  coupling (`XC_YAW2*`), CoB auto-trim (`TRIM_*`), all default off; PID integrator getters
  added. Wired after `computeDemands`, reset on mode entry.
- Builds clean: ESP32 + Pico.

### 2026-07-26 — Phase 2b Stage 2: closed-loop RPM + faults + WIRING.md
- **Pico (`pico_thruster/src/main.cpp`):** added per-motor **RPM PI loop** (rpm mode) —
  IIR-filtered measured RPM, PI(err/RPM_MAX) → signed throttle level → DShot 3D; anti-
  windup; neutral/no-windup below RPM_MIN_TARGET. **Fault detection:** commanded-but-not-
  spinning for >RPM_FAULT_MS → `fault_mask` bit + status ALERT. Raw mode (Stage 1) kept.
- **ESP32:** `ThrusterState.norm[]` (mixer demand) published by the control loop; new config
  `THR_RPM_CLOSED_LOOP` (default 0) + `THR_MAX_RPM`; `task_dshot_rmt` selects raw vs rpm mode
  (forces **raw during motor test** via `test_override`); `thruster_link::send` carries the
  mode; `mav_stream` sends an **edge-triggered `STATUSTEXT "Thruster N fault"`** (RPM already
  in `ESC_STATUS`).
- **New `WIRING.md`** — full ASCII pin diagram + connection tables (ESP32/Pico/ESC/power),
  reflecting `THRUSTER_BACKEND=PICO`; RMT fallback noted.
- ESP32 build clean: RAM 18.7%, Flash 67.0%. (Pico builds on the user's machine.)

### 2026-07-25 — Phase 2b Stage 1: RP2350 (Pico 2) thruster + RPM co-processor
Chosen direction (after validating the model-based-control / RP2040 / BlueOS ideas):
Pico 2 co-processor, **hybrid** role. Stage 1 = link + 8-ch bidir DShot + telemetry
bridge (Stage 2 adds the per-motor closed-loop RPM).
- **`shared/thruster_link_proto.h`** — one wire-protocol definition for both MCUs:
  fixed-length, header + CRC16 frames (`tl_cmd_t` ESP32→Pico, `tl_tlm_t` Pico→ESP32).
- **New Pico project `pico_thruster/`** (RP2350, arduino-pico, `bastian2001/
  pico-bidir-dshot`): core0 = 8-ch bidir DShot (pio0+pio1) + eRPM read; core1 = UART
  link (`Serial1` @2 Mbaud) framing; layered failsafes (link-timeout→stop ≤150 ms,
  `rp2040.wdt`, intrinsic ESC timeout, estop GPIO). **User builds/flashes this;** the
  `getTelemetryErpm` enum may need a minor tweak per the pinned library version.
- **ESP32 side:** `drivers/thruster_link.{h,cpp}` (UART2 TX + telemetry parse + CRC +
  link-alive); `task_dshot_rmt` now backend-aware via `THRUSTER_BACKEND` (RMT | PICO,
  default PICO — RMT retained as fallback); `ThrusterState` gained `rpm[8]/esc_status[8]/
  esc_fault/link_ok`; `mav_stream` emits `ESC_STATUS` (RPM → QGC/BlueOS, 5 Hz).
- **Pins:** the 8 old DShot GPIOs freed; 17/16 = Pico UART2, 27 = estop; 4/12/13/14/15
  spare. `platformio.ini` adds `-I shared`.
- **Prereqs (user):** ESCs in DShot **3D mode** + bidir telemetry; ~2.2 kΩ pull-up per
  signal line; common ground; wire per the plan's wiring guide.
- ESP32 build clean: RAM 18.7%, Flash 67.0%.

### 2026-07-25 — Full-codebase audit: bug fixes + optimization + docs overhaul
Three parallel deep-audit agents combed the whole codebase. Param table (255), NVS keys,
pin map, startup order verified clean. Fixed the real findings:
- **B-1 (CRITICAL):** `mixer::oneToDshot` returned 1049 (min forward) for a centred
  stick → all thrusters crept on arm. Now returns 3D-DShot neutral **1048** (stopped);
  `MOT_SPIN_ARM=0` truly means stopped.
- **B-4 (HIGH):** MANUAL mode never set roll/pitch → 2 DOF dead. Now full passthrough.
- **B-2/B-3 (HIGH):** calibration `saveToNVS` (held `mtx_cal` across ~27 flash writes)
  and autotune `saveAll` ran on the Core-1 flight loop. Now: snapshot-then-unlocked, and
  both routed through a **Core-0 deferred save** (`cal.persist_pending` +
  `params::requestSaveAll/serviceSaveAll`, serviced in `mav_commands::update`).
- **B-5:** yaw heading-hold now captures heading on mode entry (was only after first nudge).
- **B-7** `LoopIn in{}` zero-init; **B-8** NaN guards on wrapPi/yawError; **B-9** clamp
  asinf arg; **B-10** motor test/detect stimulus uses dir=+1; **B-11** LoRa requires 14 B.
- **Comms:** param stream now non-blocking `tx()` (was blocking `txReliable`, ~100 ms RX
  stall); `StateLock` **default timeout bounded to 20 ms** + nullptr guard (was
  portMAX_DELAY in RX handlers); `DO_SET_RELAY` uses `PCA_RELAY_BASE_CH` offset like the
  joystick path.
- **Safety/cleanup:** corrected the stale strapping-pin comment blocks (GPIO12=THR2,
  GPIO15=THR5, GPIO2=BNO_INT) + force GPIO12/15 LOW at boot; stack diag extended 3→6
  tasks (STK_UI/LORA/DSH); `pinOr(output=true)` rejects input-only 34-39 for output pins;
  `classifyType` "SPIN"→"SPIN_CNT" footgun removed.
- **Optimizations:** DShot rebuilds a frame only when its value changed (always re-emits);
  ADC oversample 16→4 (cuts RT-core busy-wait 4×); stale "200 Hz" comments → 500 Hz.
  (Skipped gain-caching — negligible gain vs. live-tuning regression risk.)
- **Docs:** rewrote `PARAMETERS.md` (all 255 params, detailed, R/i/C tagged) and
  `CONTROL_LOOP.md` (full pipeline + math + concurrency); refreshed the rest.
- Build clean: RAM 18.9%, Flash 67.4%.

### 2026-07-24 — Phase 1: precision low-speed control + deep-research ROADMAP.md
Deep sourced research (RPM feedback, thrust control, GPS-denied fusion, "beat Pixhawk")
→ key finding: **micro-movements come from thrust linearization + deadband, NOT RPM
feedback** (which needs an RP2040 co-proc or ESC-telem-wire; ESP32 RMT caps bidir DShot
at 4 motors). User chose "precision control now."
- `src/control/mixer.cpp` — `oneToDshot` now applies **thrust-EXPO linearization**
  (`MOT_THST_EXPO`, ArduPilot inverse-quadratic model) + **spin-min deadband crossing**
  (`MOT_SPIN_MIN`) + armed idle (`MOT_SPIN_ARM`).
- `src/control/attitude_control.cpp` — **rate feedforward** (`ATC_RAT_*_FF`), **stick
  expo** (`PILOT_EXPO`), **fine yaw-rate** (`PILOT_YAW_RATE` deg/s), and **yaw
  heading-hold** (centre stick locks fused-yaw heading, nudge to slew). `stabilize()`
  signature gained `meas_yaw`; all call sites updated.
- `src/tasks/task_control_loop.cpp` — forward/lateral scaled by `PILOT_SPEED` + expo.
- New params (ArduSub-standard where possible → QGC-tunable): MOT_THST_EXPO/SPIN_MIN/
  SPIN_ARM, ATC_RAT_*_FF, PILOT_YAW_RATE/EXPO; wired the previously-inert PILOT_SPEED.
- Docs: **new `ROADMAP.md`** (research + phases: P1 precision [done] · P2 RPM · P3
  fusion/position-hold with the MTF-01P + mag · P4 custom dashboard); updated
  `PARAMETERS.md` + `tuning_guide.md`.
- Sensor insight recorded: board's MTF-01P (optical flow + rangefinder) makes
  horizontal velocity/position observable → real GPS-denied station-keeping is possible
  in Phase 3 (experimental underwater; recommend adding a magnetometer).
- Build clean: RAM 18.9%, Flash 67.3%.

### 2026-07-24 — QGC ArduSub Tuning made real + remaining dummies + CONTROL_LOOP.md
User chose to keep QGC's ArduSub UI and make it fully work (vs a generic identity that
would lose the Motor Test tab). Key insight: our attitude gains were just misnamed —
renamed to the ArduSub-standard `ATC_*` so **QGC's Tuning page now tunes the REAL SROT
controller**.
- `RAT_*_P/I/D` → `ATC_RAT_*_P/I/D`, `ANG_*_P` → `ATC_ANG_*_P` (backing `g_params`
  fields unchanged). Added `ATC_RAT_*_IMAX` (real) → wired into `attitude_control.cpp`
  `loadGains()` (replaces the hardcoded 0.5 rate-integrator limit).
- Added inert dummies for the features we don't implement (QGC Tuning/Safety pages):
  `FS_PILOT_INPUT/TIMEOUT`, `PSC_*` (pos/vel/accel-Z control), `WPNAV_*`, `LOIT_*` →
  silences the remaining "missing params" popups.
- **MPU9250** on QGC's Sensors page is cosmetic (decoded from the dummy `INS_ACC_ID`;
  ArduPilot has no BNO085 devtype) — documented, not fixable.
- Docs: updated `PARAMETERS.md` (ATC_* gains, new dummy groups, MPU9250 note) and
  `tuning_guide.md`; **new `CONTROL_LOOP.md`** (pipeline + honest SROT-vs-ArduPilot
  comparison: SROT wins on latency/core-isolation/DShot, ArduPilot wins on EKF
  fusion/position-nav/failsafes).
- Fundamental constraint recorded: QGC's Motor Test tab + Sub setup pages are
  inseparable from the ArduSub identity and its param demands; can't show "SROT"/
  "BNO085" in QGC's ArduSub UI. SROT identity kept on OLED/BlueOS/custom-version.
- Build clean: RAM 18.8%, Flash 67.2%.

### 2026-07-24 — MOT_n_DIRECTION rename, 16 runtime servo/MOSFET channels, PARAMETERS.md
- **MOT_n_DIRECTION:** renamed params `MOT1_DIR`..`MOT8_DIR` → `MOT_1_DIRECTION`..
  `MOT_8_DIRECTION` (ArduSub-standard) → silences the QGC Motors "missing params"
  popup and makes QGC's reverse toggle work. Backing `g_params.mot_dir[]` unchanged.
- **Runtime servo/MOSFET role:** the PCA9685 channel role was compile-time
  (`PCA_CH_ROLE_INIT`). Added per-channel **`SERVOn_ROLE`** param (0=off, 1=PWM servo,
  2=MOSFET; default ch1-8 servo, ch9-16 switch). `pca9685_aux::writeState` now takes a
  runtime `role[]`; `task_ui_status` builds it from `g_params.servo_role[]`. All 16
  PCA channels are user-assignable as servo or MOSFET and driven via DO_SET_SERVO /
  DO_SET_RELAY / joystick buttons. (Servo param rows grew 4→5 per channel.)
- **QGC 9-16 caveat (documented, not a bug):** QGC's Servo Outputs *page* reserves the
  first 8 outputs for the Sub frame's motors, so it only shows 9-16 — but all 16 work
  via the Parameters list + commands. Can't be changed without dropping the 8-motor
  frame (which the Motor Test tab needs).
- **New doc `PARAMETERS.md`** (project root): every param classified Necessary /
  Informational-unwired / QGC-compat-dummy, from the usage audit — incl. PM-vs-BATT and
  servo-vs-MOSFET explainers, and which QGC groups to ignore.
- Audit surfaced inert-but-defined SROT params: PM2_VMULT, FRAME_CONFIG, PILOT_SPEED,
  MOT_PWM_* (documented as such).
- Build clean: RAM 18.6%, Flash 67.1%.

### 2026-07-24 — QGC ArduSub compatibility (Motor Test tab, sensor cal, descriptions)
Board connects to QGC but its ArduSub plugin expected the standard ArduSub param set,
so it spammed "Parameters are missing," wouldn't mark the accel calibrated (level cal
blocked), hid the Motor Test tab, and Sensors showed "Not installed." Source-verified
QGC research (APMSensorsComponent / APMAutoPilotPlugin / APMParameterMetaData):
- **Motor Test tab** needs reported firmware version **≥ 3.5.3**; we reported 0.1.0.
  Fixed: advertise **ArduSub 4.1.0** via `AUTOPILOT_VERSION.flight_sw_version`
  (`APM_COMPAT_VER_PACKED`) + a boot banner "ArduSub V4.1.0" (`sendBootIdentity`),
  with a second "SROT … (ArduSub-compat)" line for branding. `flight_custom_version`
  stays "SROT".
- **Accel calibrated** = at least one `INS_ACCOFFS_*` ≠ 0. **Compass "not installed"**
  = `COMPASS_DEV_ID*` = 0. Added **~61 ArduSub-compat params** in `params.cpp`
  (`COMPAT[]` backed by `s_compat_vals[]`, appended in `buildTable`): INS accel
  offsets/scales/IDs (calibrated), COMPASS_* = 0 (mag off), AHRS_ORIENTATION,
  BATT_*, FS_* (Sub failsafes), LEAK1_*, FLTMODE1-6/_CH, RC6-16_OPTION. Inert to
  firmware behaviour; wire stays all-REAL32.
- **Descriptions (Phase 2):** QGC uses its BUNDLED, version-keyed metadata for
  ArduPilot vehicles and **ignores COMPONENT_INFORMATION** — so the planned FTP route
  was dropped; standard ArduSub-named params now get descriptions **for free** at
  v4.1.0. Custom SROT names (STUNT_*, PIN_*) can't get QGC descriptions (accepted).
- Identity note: QGC already showed the vehicle as ArduSub (heartbeat); v4.1.0 only
  unlocks features. SROT identity retained on OLED/BlueOS/custom-version field.
- Build clean: RAM 18.4%, Flash 67.1%. Param count +61 (~200 total).

### 2026-07-24 — 500 Hz flight core + BNO085 INT-gated at 400 Hz
Reset storm confirmed fixed by the user (OLED `r1`). Moved to high-rate flight:
- **Discovery:** Adafruit_BNO08x **I2C ignores the INT pin** (`begin_I2C(addr,
  wire, sensor_id)`; only `begin_SPI` takes `int_pin`). The "working example" passed
  its INT as `sensor_id` — the library never used it. So the wired INT is used via
  **our own INT-gated polling**.
- `include/config.h` — `CONTROL_LOOP_HZ` 200 → **500**, `TASK_SENSOR_HZ` 200 → 500,
  `TASK_DSHOT_HZ` 200 → 500 (2 ms tick-exact; 400 Hz isn't a whole-ms tick, so 500
  is the clean max and keeps PID `dt` correct). `I2C0_FREQ` 200 k → **400 kHz**
  (needed for 400 Hz dual reports). `PIN_WS2812B` 2 → **0** (RGB to GPIO0),
  `PIN_BNO_INT` -1 → **2** (H_INTN to GPIO2).
- `src/drivers/bno085.cpp` — `REPORT_FAST_US` 5000 → **2500** (rotation + gyro at
  400 Hz, sensor max); `begin()` sets `pinMode(INT, INPUT_PULLUP)`; `poll()` reads
  only while `digitalRead(INT)==LOW`, with a 100 ms blind-poll safety net so a
  flaky/miswired INT can't make the IMU go silent.
- **Strapping caveats (GPIO0/GPIO2):** flash-download may need BOOT held if the BNO
  drives INT HIGH at reset; RGB on GPIO0 fine at run. I2C0 400 kHz needs decent
  wiring — if `rN` climbs again, drop back to 200 kHz.
- Build clean: RAM 18.0%, Flash 67.0%.

### 2026-07-24 — Reliability: BNO reset storm + param download (Phase 1)
Root-caused the attitude "freeze" (OLED + MAVLink freezing together, then
recovering) via two ArduSub audits + the working `example/bno085 example.cpp`:
the **BNO085 was resetting repeatedly** (OLED `r8`). Each `wasReset()` opens a
recovery window that holds the last-good attitude, so chained resets = a
multi-second frozen angle with no reboot (`rst:poweron` confirmed). The dual-core
design was cleared — Audit 1 found no mutex is ever held across an I2C/serial/PID
call. The reset trigger: we poll BNO08x SHTP over I2C **without the INT line**
(user keeps INT/RST = −1) while enabling 6 reports at 200 kHz — the working
example uses the INT pin. Fixes (software-only, INT stays unwired):
- `src/drivers/bno085.cpp` — trimmed `enableReports()` to 3 essentials
  (GAME_ROTATION_VECTOR, GYROSCOPE_CALIBRATED, GRAVITY); dropped LINEAR_ACCELERATION
  + raw ACCELEROMETER (accel accuracy now taken from the rotation-vector status).
  `RECOVERY_MS` 300 → 150; drain bound 12 → 8.
- **IMU rate raised 50 → 200 Hz.** The BNO reports were 50 Hz in the ORIGINAL
  firmware (`REPORT_US=20000`), independent of the poll task. Now rotation + gyro
  run at **200 Hz** (`REPORT_FAST_US=5000`), gravity stays 50 Hz (cal-only);
  `TASK_SENSOR_HZ` = 200 to match. Attitude is now ≤1 control cycle stale.
  400 Hz is feasible on the ESP32 but pushes the INT-less I2C link toward
  desync/resets — reserved for when the INT pin is wired.
- I2C0 kept at **200 kHz** (higher clock drains SHTP faster — a slower bus makes
  large reads exceed the I2C timeout). Reliability fix = trimmed reports, not a
  slow bus/poll.
- **Regression fixed same session:** an intermediate cut used 100 kHz +
  `Wire.setTimeOut(25)` → the BNO's ~270-byte SHTP advertisement read exceeded
  25 ms at 100 kHz, so `begin_I2C` failed and the IMU produced *no* data. Reverted
  (200 kHz, default 50 ms I2C timeout).
- `src/comms/mav_commands.cpp` — **reverted PARAM_VALUE to all-`REAL32`** on the
  wire (ArduPilot does this; last round's INT types were a likely cause of the
  greyed Motor Test tab / incomplete QGC param download). Param table verified
  hole-free (`getByIndex` valid for every `i < count()`).
- Verified QGC handshake intact: HEARTBEAT SUBMARINE+ARDUPILOTMEGA, sys/comp id
  = 1/1, AUTOPILOT_VERSION caps (MAVLINK2|PARAM_FLOAT|COMMAND_INT|MISSION_INT).
  Motor test/detect confirmed ArduSub-aligned; mixer left as-is (documented; no
  reference table to validate against — validate once Motor Test works).
- Build clean: RAM 18.0%, Flash 67.0%. **Phase 2 (QGC COMPONENT_INFORMATION + FTP
  param descriptions, keeping SROT identity) pending hardware confirmation.**

### 2026-07-23 — P0 Foundation
- Initialized project tracking: `plan.md`, `memory.md`, `context.md`,
  `tuning_guide.md`, this file.
- `include/config.h` — all pin assignments, dual-I2C + VSPI bus params, 8-channel
  DShot map, PCA9685 aux role table, FreeRTOS core/priority/stack allocation,
  SROT identity macros, default tuning constants, NVS namespaces.
- `include/state_types.h` — six mutex-guarded sub-structs (Sensor/Control/
  Thruster/Indicator/Aux/Cal), flight-mode/stunt/pattern/cal/PCA enums, `Vec3f`,
  and a scoped `StateLock` guard.
- `src/main.cpp` — strapping-pin-safe boot (buzzer forced LOW), dual I2C bring-up,
  mutex creation, 6 pinned FreeRTOS tasks.
- `src/tasks/*` — six task stubs, each rate-limited and annotated with the phase
  that fills it.
- `platformio.ini` — SparkFun BNO08x, BlueRobotics MS5837, Adafruit NeoPixel/GFX/
  SH110X/PWM-Servo, DShotRMT; MAVLink build flags; `monitor_speed=115200`.
- `lib/mavlink/README.md` — vendoring instructions for MAVLink v2 ardupilotmega
  dialect + version-pin slot.
- Decisions locked: identity = SROT; full standard MAVLink v2; SparkFun/BlueRobotics/
  Adafruit/DShotRMT stack; scaffolding + task skeletons this pass.
- **Build: SUCCESS** (`pio run`) — RAM 6.8%, Flash 20.9%. Two fixes on the way:
  1. `PcaChannelRole` enumerators collided with the ESP32 HAL `#define DISABLED`
     macro → prefixed them `AUX_*`. (Watch for the same with `INPUT/OUTPUT/HIGH/
     LOW` in future enums.)
  2. Toolchain `esptool` was missing the `intelhex` python module → installed
     into the PlatformIO penv. Not a firmware issue.
- `lib_deps` disabled for the foundation build (stubs include none); the
  `sparkfun/SparkFun BNO08x Arduino Library` id did not resolve on the registry —
  confirm exact ids with `pio pkg search` when enabling P1/P3 deps.
- Not yet done (needs hardware / later phases): flash + serial-core check,
  live HEARTBEAT-as-SROT smoke test (P2), mutex-liveness soak.

### 2026-07-23 — P1–P10 Full firmware implementation
- Built the entire firmware from scratch (ArduSub = inspiration only), continuous
  P1→P10, compile-gated after each phase. Final: **clean build, RAM 8.8%, Flash 31.9%.**
- Module layout: `src/drivers/*` (bno085, bar30, analog_mon, dshot_out,
  pca9685_aux, neopixel_rgb, buzzer, oled, lora_mission, sd_log),
  `src/control/*` (pid, mixer, arming, attitude_control, depth_control,
  calibration, stunt, pattern, autotune), `src/comms/*` (mavlink_bridge, params,
  mav_stream, mav_commands). All six tasks now call into these modules.
- MAVLink v2 (ardupilotmega dialect) vendored under `lib/mavlink/` (commit
  `ee5827f`); packed-member warnings silenced via `#pragma` in `mavlink_bridge.h`.
- **Four library decisions changed during the build (all in code + plan.md):**
  1. BNO085 → **Adafruit BNO08x** (matches `example/bno085 example.cpp`), not SparkFun.
  2. Bar30 → **robtillaart/MS5837** (BlueRobotics' own isn't on the PIO registry).
  3. DShot → **custom legacy-RMT driver** (`driver/rmt.h`): DShotRMT 0.9.5 needs
     the IDF-5 `driver/rmt_rx.h`, but the platform is arduino-esp32 2.0.17 / IDF 4.4.
  4. WS2812B → **custom cycle-timed bit-bang** (`drivers/neopixel_rgb`): Adafruit
     NeoPixel drives the ESP32 via RMT (`rmtInit`/`rmtWrite`), and all 8 RMT
     channels are consumed by DShot.
- Toolchain fix retained: `intelhex` installed into the PlatformIO penv for esptool.
- Two compile bugs fixed along the way: `Vec3f` needed a ctor for list-assignment;
  `-Wno-address-of-packed-member` isn't a valid GCC-8.4 flag (dropped; used a
  targeted `#pragma` instead).

### Deferred to hardware bring-up (compile-gate mode — no board wired yet)
- Flash + monitor: confirm all 6 tasks start on their cores; no strapping-pin boot-loop.
- QGC/BlueOS: vehicle reads as **SROT vX.Y.Z**; live ATTITUDE/depth/dual-battery
  telemetry; param get/set; arm/disarm; motor test; DO_SET_SERVO/RELAY.
- IMU sanity (BNO cold-boot 3 s delay), depth zero, DShot on a bench ESC (3D mode),
  stabilize/depth-hold, stunt/pattern, relay auto-tune convergence, LoRa RX, SD log decode.
- **Known hardware gap:** no GPIO allocated for leak/kill (35/36 are the two
  battery ADCs) — `analog_mon` reports both false; wire a pin (e.g. GPIO39) later.
- Set water density (fresh 0.998 / salt 1.029) in `drivers/bar30` for the venue.
- Set LoRa band in `drivers/lora_mission` (default 433 MHz) for your region/radio.

### 2026-07-23 — ArduSub-parity pass
- **Flight-mode renumber → ArduSub custom_mode values** (STABILIZE=0, ACRO=1,
  ALT_HOLD/DEPTH_HOLD=2, SURFACE=9, MANUAL=19, MOTOR_DETECT=20; STUNT=100,
  PATTERN=101). This is what lets QGC/BlueOS's Sub mode dropdown display and
  command modes correctly. SET_MODE validates + DENIES unsupported ArduSub modes.
- **MAVLink mission upload/download** (`comms/mission`) — full MISSION_* handshake;
  LoRa remains a parallel extra path.
- **Joystick buttons** — `MANUAL_CONTROL.buttons` → `BTNn_FUNCTION` (AP_JSButton
  IDs) for arm/disarm/mode/lights/relay; edge-triggered. JS_GAIN applied to axes.
- **Motor reverse** — `MOT1_DIR..MOT8_DIR` params (QGC-editable) into the mixer.
- **Servo/relay like ArduSub** — `DO_SET_SERVO` (servo/payload release) +
  `DO_SET_RELAY` (digital MOSFET) already; buttons now also trigger relays/lights.
- **Failsafes wired** → controlled SURFACE ascent on leak / low PM1 battery /
  GCS-loss (gated by FS_* params).
- **Focused ArduSub params added:** FRAME_CONFIG, JS_GAIN_DEFAULT, LIGHTS_STEP,
  FS_GCS/LEAK/BAT_ENABLE, FS_BAT_VOLTAGE, PILOT_SPEED, BTN0..15_FUNCTION.
- **BNO085 roll/pitch swap** (`BNO_SWAP_ROLL_PITCH` in config) — swaps roll↔pitch
  angles AND gyro x↔y so the PID stays consistent. RST/INT explicit −1
  (`PIN_BNO_RESET`/`PIN_BNO_INT`) — polled, no reset/interrupt wiring.
- **LoRa/SD are non-blocking:** `begin()` returns quickly if the module/card is
  absent; the task guards every use on `healthy()`, so a board without them just
  skips those features (no wait, no boot stall).
- Build after this pass: clean, RAM 9.6%, Flash 32.3%.
- **Caveat to verify on hardware:** AP_JSButton relay/lights IDs (51–56, 32/33)
  are from memory of ArduPilot's enum — confirm against your QGC joystick-setup
  UI; arm/disarm/mode IDs (2–7,12) are certain.

### 2026-07-23 — Persistence, arming, leak, QGC cal wizard, auto-tune
- **Calibration now auto-persists** to NVS on completion (`calibration::finish()`
  → `saveToNVS()`), plus `CalState.result`/`result_seq` for GCS reporting. The
  bot remembers its level/gyro/baro/accel/mag across reboots.
- **Arming reworked ArduPilot-style:** kill switch removed from `canArm()` (it's a
  display-only thruster kill from the 2nd board, shown on OLED); `ARMING_CHECK`
  param gates pre-arm (IMU health + not calibrating); STATUSTEXT feedback
  ("SROT: Armed/Disarmed", "PreArm: <reason>").
- **Leak sensor → GPIO39** (input-only; external bias needed), read only when
  `LEAK_EN`; removed duplicate `FS_LEAK_ENABLE`.
- **QGC-native calibration wizard:** accel 6-position prompts (STATUSTEXT keywords
  + `ACCELCAL_VEHICLE_POS`, step mapped pos−1→index), compass `MAG_CAL_PROGRESS`
  (~5 Hz) + `MAG_CAL_REPORT`, gyro/baro/level completion text. Emitted from the
  comms core off `g_state.cal` (no cross-core string queue).
- **Auto-tune trigger = `ATUNE` param** (set 1 → run, resets to 0 on completion);
  `MAV_CMD_USER_5` kept as alternate. Procedure documented in `tuning_guide.md`.
- Build: clean, RAM 9.7%, Flash 32.4%.
- **Verify on hardware:** QGC accel-cal prompt strings actually advance its wizard
  (string-matching may need a tweak); compass progress bar; that a level cal
  survives reboot; leak trigger on GPIO39; `ATUNE=1` runs + resets.

### 2026-07-23 — Full-PID auto-tune, mag-off, reboot, P11 ESP-NOW
- **Auto-tune now tunes ALL PIDs** (`control/autotune` phased state machine):
  rate roll/pitch/yaw → angle-P roll/pitch/yaw → depth PID, ~1 min, relay method.
  New signature takes angles + depth and outputs a heave demand for the depth
  phase; control loop routes it. Guards skip a phase with no motion.
- **Magnetometer off** (`BNO_USE_MAG 0`): mag report no longer enabled; attitude
  already used the 6-axis game rotation vector. Yaw is relative (accepted). MAG
  cal is a no-op when mag is off.
  *(Superseded 2026-07-30: `BNO_USE_MAG` is back to 1 so the mag REPORT is available
  to `control/yaw_ref`. The mag is still never fused into attitude.)*
- **MAVLink reboot**: `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` → ack + `ESP.restart()`
  (QGC/BlueOS "Reboot Vehicle").
- **P11 ESP-NOW receive-only** (`drivers/espnow_link`): WiFi STA + fixed channel
  (matches `example/2nd_board_firmware`), RX callback → thruster-kill + aux
  voltage → `SensorState` (folded into `Task_LoRa_SD`, no 7th task). Kill is
  display-only (OLED + `KILL` telemetry). Packet contract documented for the 2nd
  board. `SensorState.aux_voltage` added; shown on OLED.
- Build: clean. **Flash jumped to 66.6%, RAM 16.4%** — the WiFi/ESP-NOW stack.
  Still ample (4 MB flash / 320 KB RAM).
- **Verify on hardware:** full auto-tune convergence (esp. depth-phase thruster
  sign); QGC "Reboot Vehicle"; 2nd-board kill/voltage on OLED + telemetry; yaw
  stable-but-relative with mag off; ESP-NOW channel matches the 2nd board.

### 2026-07-23 — Hardware bring-up: QGC connectivity, bug-fixes, servo setup
First real QGC test. Root-caused and fixed the connect/freeze problems:
- **Boot order (the big one):** `bno085::begin()` ran a 3 s blocking delay in
  `setup()` BEFORE tasks were created → no heartbeat for >3 s; with the USB-DTR
  auto-reset on port open, QGC never saw a timely heartbeat. **Moved BNO/Bar30
  bring-up into `Task_SensorRead`**; tasks (heartbeat) now start within ms.
- **UART buffers:** added `setRxBufferSize(1024)`/`setTxBufferSize(1024)` before
  `Serial.begin` so QGC's bursty param/command traffic can't overflow → no more
  MAVLink desync/freeze. Raised `Task_MAVLink` to 100 Hz; send a HEARTBEAT
  immediately on task start.
- **ESP-NOW OFF by default** (`ESPNOW_ENABLE 0`) — WiFi shared Core 0 with the
  MAVLink task and caused jitter/freezes + doubled flash. Flash back to ~32.6%.
  Flip to 1 when the 2nd board is ready and QGC is stable.
- **Phantom leak fixed:** `LEAK_EN` now defaults **0** (GPIO39 floats without
  external bias). Disable/enable from the QGC parameter tab.
- **Power modules selectable:** `PM1_SRC`/`PM2_SRC` (0=off, 1=ADC, 2=ESP-NOW aux).
  Battery telemetry (`BATTERY_STATUS`, `SYS_STATUS` voltage) is **gated by
  presence** → no phantom battery in QGC. Defaults PM1=ADC, PM2=ESP-NOW.
- **Full ArduSub servo config:** `SERVO9..16_FUNCTION/MIN/MAX/TRIM` params
  (mapped to PCA9685 servo channels 0–7) so QGC's servo-output page populates;
  disabled channels drive off. Param table refactored to build at runtime with a
  separate ≤15-char NVS key (long `SERVOn_FUNCTION` names).
- **ATUNE non-persistent** — reset to 0 in `params::init` so a saved 1 can't
  auto-start tuning on boot.
- **AUDIT — critical latent bug found & fixed:** every calibration completion
  block held `mtx_cal` and then called `finish()`, which re-takes the same
  non-recursive mutex → **deadlock** the first time any calibration completed on
  hardware. Scoped the locks so `finish()` runs unlocked (GYRO/BARO/LEVEL/MAG/
  ACCEL_6PT). Also: disabled PCA servo channels now output off (were holding last
  value).
- Build: clean, RAM 10.5%, Flash 32.6%.
- **Hardware checks for the user:** QGC should connect within ~2 s of plugging in;
  param download completes → setup pages (servo/motor-test) enable; attitude
  streams without freezing; no phantom leak/battery; reboot from QGC works.
  If QGC still stalls mid-download, note which param index and we'll chase it.

### 2026-07-23 — Calibration mapping bug (stuck at "cal 16%")
- **Symptom:** pressing a calibration (e.g. Level) in Mission Planner showed
  "completed" but the OLED stuck at **cal 16% (=1/6)** and nothing was calibrated.
- **Cause:** `MAV_CMD_PREFLIGHT_CALIBRATION` mapped *any* param5≥1 to the full
  6-point accel routine (`ACCEL_6PT`). Mission Planner's "Calibrate Level" sends
  **param5=2** (quick AHRS trim), so we started the 6-point routine, captured one
  face (1/6), and waited forever for position steps that a level cal never sends.
- **Fix:** param5==1 → 6-point accel; **param5==2/4 → LEVEL** (the quick routine,
  ~1 s, stores roll/pitch trim to NVS). Unknown cal → `MAV_RESULT_DENIED`.
  Also: ACCEL_6PT now starts with step=255 (don't grab a face until the GCS sends
  a position) and **times out after 120 s** so it can't hang. Level/gyro/baro all
  self-complete and auto-save.

### 2026-07-23 — Bring-up round 2 (Phase 1): freeze, IMU flicker, 16 servos, ESP-NOW toggle
- **MAVLink freeze — real fix.** `mav::tx()` was a **blocking** `Serial.write()`;
  during QGC's param download it exceeded 115200 baud, filled the TX buffer, and
  blocked the MAVLink task → heartbeat stalled → freeze, and param download never
  finished (so setup tabs stayed disabled). Now `tx()` is **non-blocking**: it
  writes only if the message fits in `availableForWrite()`, else drops (telemetry)
  / defers (params). **Param streaming only advances its index when the send
  succeeds**, pacing the download to the UART with zero lost params → download
  completes → Motor Test / servo tabs enable.
- **IMU 0° flicker (periodic, mid-operation).** The BNO085 periodically resets and
  emits an identity quaternion (=0°) before re-fusing, which we were publishing.
  Now: on `wasReset()` we enter a 300 ms recovery window, **ignore rotation
  vectors and hold the last-good attitude** (no 0 flicker), count resets, and set
  `imu_valid` from freshness. OLED shows `--` only at true cold start, plus `rN`
  (reset count). **I2C0 dropped to 200 kHz** to reduce the resets. NOTE: periodic
  BNO resets are usually a **hardware** symptom — marginal 3.3 V (add bulk caps)
  or weak/long I2C wiring (add 2.2–4.7 kΩ pull-ups). The reset count on the OLED
  tells you if a hardware fix helps.
- **All 16 PCA servo channels** now exposed as `SERVO1..16_FUNCTION/MIN/MAX/TRIM`
  (was 9–16). Motors are on separate DShot pins, so every PCA channel is free.
- **ESP-NOW runtime toggle:** compiled in, started only when `ESPNOW_EN=1`
  (default 0 → WiFi never starts → no MAVLink contention). Answer to the question:
  `PMx_SRC=2` only *selects* ESP-NOW as the battery source; `ESPNOW_EN=1` actually
  brings the link up. (Flash back to ~66% because WiFi is linked in.)
- Build: clean, RAM 17.7%, Flash 66.7%.
- **PLEASE TEST after this phase** (recommended before the big features): QGC
  connects and stays live (no freeze), param download completes + Motor Test
  enables, angle no longer flickers to 0 (watch the OLED `rN` reset count).
- **Still TODO (Phase 2/3, approved):** full in-QGC param metadata
  (COMPONENT_INFORMATION + MAVLink FTP); closed-loop RPM control (bidirectional
  DShot + RPM PID + RPM-gain model) — the latter flagged experimental.

### 2026-07-23 — ROOT CAUSE: periodic reboot (freeze + BNO 0° + disabled tabs were ONE bug)
Two independent audits vs the working `example/bno085 example.cpp` found the
freeze, the BNO "reset to 0°", and the greyed Motor Test tab are all **one thing:
the board periodically reboots** (~seconds). Confirmed on hardware: the OLED BNO
reset counter reads **`r1` always** — a per-boot static counter stuck at 1 means
the whole board keeps rebooting (zeroed each boot, +1 on the BNO's power-on reset).
- **Primary cause:** `drivers/bno085.cpp` drained events with an **unbounded
  `while (getSensorEvent)`** (the example uses `if` = one event). If events keep
  arriving / on an SHTP error storm it never returns → `Task_SensorRead` never
  yields → Core-1 IDLE starves → **Task-Watchdog panic → reboot**, which drops the
  heartbeat (freeze), re-inits the BNO (0°), and restarts QGC's param download so
  the setup tabs never latch. **Fixed:** bounded the drain to 12 events/cycle.
- **Diagnostics added (the key aid):** `esp_reset_reason()` → boot STATUSTEXT
  `RST: <reason>` + OLED "reset:" line; task stack high-water + free heap as
  NAMED_VALUE_FLOAT (`STK_MAV/SEN/CTL`, `HEAP`). Now one reconnect tells us
  TASK_WDT vs BROWNOUT vs PANIC.
- **Hardening:** Bar30 read decimated 50→20 Hz (its busy-wait was ~2-3 ms on the
  200 Hz path); sensor stack 4096→8192, MAVLink stack 6144→8192; sensor task
  priority 5→6; I2C0 already 200 kHz.
- **Reliable TX (fixes hung commands + never-completing download → tabs enable):**
  the earlier "non-blocking tx" silently DROPPED COMMAND_ACK/PARAM_VALUE under
  load. Added `txReliable()` (bounded-blocking, yields, ≤20 ms) for ACK / PARAM /
  STATUSTEXT; telemetry stays best-effort; **telemetry throttles to heartbeat-only
  while a param download is active** so it completes fast.
- **Motor params** added for QGC binding: `MOT_PWM_TYPE`(4=DShot150)/`MIN`/`MAX`.
  ESC cal: DShot needs none; unmatched cal commands are ACK'd gracefully.
- **Docs refreshed:** context.md (I2C0 200 kHz, SERVO1..16, PMx_SRC, ESPNOW_EN,
  MOT_PWM_*, diagnostics; removed 4 unimplemented commands + 2 inbound msgs);
  dropped stale "stub" wording in main.cpp/config.h.
- Build: clean, RAM 17.8%, Flash 66.8%.
- **TEST — report the `RST:` string** shown in QGC on connect (or the OLED "reset:"
  line). `POWERON` = healthy. `TASK_WDT` = the hang is still happening somewhere
  (send stack values). `BROWNOUT` = power/supply issue (add caps, stiffer 3.3 V).
  `PANIC` = a code crash (we'll need the backtrace via a plain serial monitor).

### 2026-07-23 — ArduSub-align + confirmed "--" fix + new pinout + pin params
**Root reframe from hardware answers:** the board is NOT rebooting (OLED keeps
running; only R/P/Y blanked). USB-only, no ESCs → no brownout. So:
- **"--" was a UI mutex-timeout bug** (fixed): the display rebuilt its View from
  zero each refresh under a 2 ms lock; on a miss it printed defaults. Now the View
  is **persistent** (holds last values on a missed lock), timeouts 2→15 ms, and
  "--" uses a real `imu_ever_valid` flag. Plus OLED `setRotation(2)`.
- **Motor Test greyed = QGC param download not completing** (per two ArduSub
  audits). Added **`PRM n/N` on the OLED** so we can watch it reach N (or see where
  it stalls); don't restart the stream on a duplicate PARAM_REQUEST_LIST;
  **integer param types** (real MAV_PARAM_TYPE per param, like ArduSub) so QGC's
  Sub components read FRAME_CONFIG/SERVO*/MOT_* as ints.
- **Motor Test → ArduSub semantics:** COMMAND_INT, param2 throttle type
  (0=%→scale, 1=PWM µs→−1..1, 2=reject), **require ARMED**, **≥2 Hz keep-alive**
  with **auto-disarm** on >600 ms gap.
- **Motor Detect (mode 20) → ArduSub-like:** settle-until-quiet → pulse up 500 ms
  → compare peak gyro to the mixer's expected angular factor → store `MOTn_DIR`.
- **Richer MAVLink status:** base_mode (STABILIZE|MANUAL_INPUT|CUSTOM + SAFETY),
  SYS_STATUS control-subsystem bits, CRITICAL state on leak.
- **New pinout applied:** thrusters 4/12/13/14/15/16/17/27; RGB→GPIO2; buzzer→32;
  SD_CS→33; LoRa RST→EN (−1); LEAK 35 / BATT_VOLT 36 / **BATT_CURR 39** (new
  current channel → BATTERY_STATUS current + `CURR` telemetry).
- **Pins configurable via params:** `PIN_THR1..8`, `PIN_BUZZER/RGB/LEAK/BATTVOLT/
  BATTCURR/SDCS/LORACS/LORADIO` (defaults = wired pinout). Drivers read them at
  begin() via `params::pinOr()` (validates 0..39, skips flash 6-11; RGB guarded
  <32 for the bit-bang). Changing a `PIN_*` → **"Pin changed - reboot required"**
  STATUSTEXT; software reboot via `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`. Bus pins
  (I2C/SPI/UART) stay compile-time (a wrong bus pin would brick without reflash).
- Build: clean, RAM 18.0%, Flash 67.0%.
- **TEST:** (1) does "--" stop (holds last value)? (2) On QGC connect, does the
  OLED **`PRM n/N` reach N**? If yes → Motor Test should enable (arm first, then
  test). If it stalls, report the number. (3) New pinout + a `PIN_*` change →
  reboot → new pin active.
- **Still deferred:** full in-QGC param **descriptions** need COMPONENT_INFORMATION
  + MAVLink-FTP metadata (or an "ArduSub Vx.y.z" banner, which dilutes SROT
  branding — your call); closed-loop RPM.

## Idea backlog

- **ESP-NOW telemetry mesh (P11+).** Mirror the 2nd-board pattern: broadcast
  compact telemetry (attitude, depth, PM1/PM2, leak) to a surface/handheld board
  without flooding the MAVLink link. Needs a fixed Wi-Fi channel; keep it off
  Core 1.
- **SD binary flight log.** Fixed-width packed records at loop rate (timestamp,
  quat, gyro, depth, thruster[8], mode, pm1/pm2). Rotate files; header block with
  the config hash + calibration snapshot. Decode offline to CSV.
- **LoRa mission protocol.** Chunked, CRC'd waypoint upload with ACK/resume, so a
  mission survives dropouts. Consider a compact binary waypoint schema mapped
  onto MAVLink MISSION_ITEM_INT after reassembly.
- **Pattern headroom auto-calc.** Derive `HEADROOM_DEPTH` from current depth +
  Bar30 surface distance so the complex pattern never breaches or bottoms out.
- **Thrust linearization.** Add ArduSub-style thrust-curve linearization in the
  mixer for smoother low-speed authority (helps stunt precision).
- **Leak → auto-surface failsafe.** On leak assert (if `LEAK_EN`), auto-switch to
  a controlled surface ascent + alarm tone + red RGB.
- **Kill-switch as hard motor cut.** GPIO36 sense → immediate DShot disarm in
  `Task_DShot_RMT`, independent of the control loop.
- **Dual-battery smart failsafe.** Separate low-voltage thresholds for PM1
  (thruster) vs PM2 (service); warn/limit-thrust on PM1, controlled-return on PM2.
