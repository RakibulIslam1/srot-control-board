# SROT — Architecture, Control Loop & Tuning

How SROT is built and how it flies. Firmware architecture, the full control loop, flight
modes, the MAVLink/QGC/BlueOS interface, and the tuning & calibration playbook. Hardware
wiring is in `HARDWARE.md`; parameters in `PARAMETERS.md`; status/roadmap in `ROADMAP.md`.

---

## 1. Dual-core architecture

Custom MAVLink-native flight-controller firmware for an ESP32 DevKit V1. **Core 1 is the
flight core** (nothing else runs there); **Core 0 handles comms + UI**. One global
`SystemState g_state` (in `main.cpp`) has six sub-structs, each with its own mutex.

| Task | Core | Prio | Rate | Job |
|---|---|---|---|---|
| `Task_SensorRead` | 1 | 6 | 500 Hz | BNO085 + Bar30 + ADC → `SensorState` |
| `Task_ControlLoop` | 1 | 5 | 500 Hz | modes → cascade PID → mixer → `ThrusterState` |
| `Task_DShot_RMT` | 1 | 5 | 500 Hz | thruster output (RMT DShot, or Pico link) |
| `Task_MAVLink` | 0 | 2 | 100 Hz | QGC/BlueOS RX/TX, params, deferred flash |
| `Task_UI_Status` | 0 | 1 | 30 Hz | OLED (+ "Booting SROT" splash) + PCA9685 aux + RGB |
| `Task_Buzzer` | 0 | 1 | 200 Hz | buzzer melodies/alerts (uniform timing); self-detects events, gated by `BUZZ_MASK` |
| `Task_LoRa_SD` | 0 | 1 | 20 Hz | LoRa mission RX + SD log + ESP-NOW |

Loop period = **2 ms (500 Hz)** — the fastest whole-ms rate on the 1 kHz FreeRTOS tick.

### Shared-state contract
- Take one mutex, copy fast, release. **Never nest locks; never hold a lock across I/O**
  (I2C/serial/SPI/flash) → structurally deadlock-free.
- Real-time Core-1 tasks use short (2 ms) timeouts via the `StateLock` RAII guard and
  **skip a cycle on miss** rather than block. `StateLock`'s default timeout is bounded
  (20 ms), so a comms handler can never freeze forever on a contended mutex.
- **No flash write ever happens on the flight loop** — calibration- and autotune-complete
  set a pending flag; the comms core (Core 0) performs the NVS write.
- `g_params` is written from both cores but only via single aligned 32-bit float stores
  (atomic, lock-free by contract).

---

## 2. The control loop (per 2 ms tick)

```
[Core 1] BNO085 (onboard 9-DOF fusion) ──INT-gated I2C──┐
  Task_SensorRead ── fused quaternion→euler, gyro, gravity, depth, battery
        │ publish → g_state.sensors (short lock)
        ▼
  Task_ControlLoop  (dt = 0.002 s)
    readInputs()   snapshot sticks/attitude/rates/depth/mode (short locks, zero-init)
    calibration::update()            (advances any active cal routine)
    failsafe check                   (leak / low-batt / GCS-loss → SURFACE)
    arming enforcement               (lost prerequisite → disarm)
    motor-test override?  yes → drive one motor raw, skip the mixer
    mode dispatch → 6 body demands (roll,pitch,yaw,throttle,forward,lateral)
    mixer::mix()   6 demands → 8 normalized outputs (vectored-6DOF, uniform saturation)
    mixer::toDshot() thrust-shape + per-motor direction → DShot values
        │ publish → g_state.thrusters
        ▼
  Task_DShot_RMT  →  RMT DShot (or UART to the Pico co-processor)
```

### Sensing
Attitude comes from the **BNO085's onboard sensor fusion** (accel+gyro, mag off) — a ready
GAME_ROTATION_VECTOR quaternion at 400 Hz over I2C, read **INT-gated** (only when the sensor
signals a full packet). SROT does not run its own EKF. Quaternion→euler clamps the `asinf`
argument (NaN guard). Bar30 depth (~20 Hz) and battery/leak ADC (~10 Hz, 4× oversample) are
decimated off the 500 Hz path. On a BNO reset, rotation samples are held for 150 ms.

### The cascade (STABILIZE / DEPTH_HOLD / SURFACE) — `control/attitude_control.cpp`
```
outer (angle):  desired_rate = (target_angle − measured_angle) × ATC_ANG_*_P
inner (rate):   torque       = PID(desired_rate, gyro) + ATC_RAT_*_FF · desired_rate
```
- Roll & pitch are angle-stabilized (stick → target lean, with `PILOT_EXPO`).
- **Yaw**: active stick = fine rate command (`PILOT_YAW_RATE` deg/s); centred stick =
  **heading-hold** (captures heading on mode entry, holds via `ATC_ANG_YAW_P`). Without a
  magnetometer the held heading drifts slowly.
- PID (`control/pid.h`): derivative-on-measurement (no setpoint kick), integrator clamp
  `ATC_RAT_*_IMAX`, output clamp ±1; wrap guards NaN.

### Flight modes & on-arm behaviour
On arm with centred sticks, **all thrusters sit at DShot neutral (1048) = stopped** — no
creep. What it actively does depends on the mode:

| Mode (custom_mode) | Attitude | Depth | Hands-off behaviour |
|---|---|---|---|
| MANUAL (19, default at boot) | none (passthrough) | none | Thrusters stopped; won't drive or self-level |
| STABILIZE (0) | level + heading | none | Holds level/heading; **no depth hold** → non-neutral sub drifts vertically |
| ACRO (1) | none (rate-only) | none | Zero thrust; no self-levelling |
| DEPTH_HOLD / "Alt Hold" (2) | level + heading | **holds captured depth** | Holds attitude **and** the depth it was at on entry |
| SURFACE (9, failsafe) | level + heading | → 0 m | Ascends to the surface |
| MOTOR_DETECT (20) | — | — | Pulses each thruster to detect direction |
| STUNT/PATTERN (100/101) | scripted | scripted | Cmd-triggered rotations/sequences |

Entering **Depth Hold** captures the *current* depth as target (it doesn't dive to a
preset); the throttle stick slews the target and re-locks on release. **No mode holds
horizontal position** (no GPS/DVL/flow fusion yet) — current can drift the sub sideways.

### The mixer & thrust shaping — `control/mixer.cpp`
Six body demands → 8 thrusters via a fixed **SUB_FRAME_VECTORED_6DOF** matrix (M1-4
horizontal = yaw/surge/sway; M5-8 vertical = heave/roll/pitch). On saturation the whole set
scales down uniformly (preserves the mix). Each output → DShot via: direction
(`MOT_n_DIRECTION` × detect sign) → centre→neutral 1048 (or `MOT_SPIN_ARM` idle) → thrust
linearization (`MOT_THST_EXPO`) → deadband lift into `[MOT_SPIN_MIN,1]` → 3D band
(1049-2047 fwd / 1047-48 rev).

### Thruster backend (`config.h THRUSTER_BACKEND`)
- **RMT** — on-board ESP32 DShot on the 8 thruster pins; rebuilds a frame only on change,
  always re-emits; disarmed = 0. No RPM.
- **PICO** (default) — the task sends the 8 values (or, with `THR_RPM_CLOSED_LOOP`, signed
  target RPMs) to a **Pico 2 co-processor** over UART2. The Pico drives bidirectional DShot,
  runs a per-motor **closed-loop RPM PI** (voltage-independent thrust), and returns RPM +
  fault flags → `ESC_STATUS` + a `STATUSTEXT` on a fault. Wiring: `HARDWARE.md`.

### Safety & housekeeping
Failsafes (leak / low-batt / GCS-loss → SURFACE); arming enforcement (the 2nd-board
thruster-kill is display-only, never gates arming); motor-test 2 Hz keep-alive with
auto-disarm; deferred NVS writes on Core 0; stack high-water for all six tasks as `STK_*`.

---

## 3. MAVLink / QGC / BlueOS / companion

SROT presents as **ArduSub 4.1.0** (heartbeat `SUBMARINE`+`ARDUPILOTMEGA`, `AUTOPILOT_VERSION`
= 4.1.0). This makes **QGC and BlueOS treat it as a Sub** (Motor Test tab, sensor-cal UI, live
Tuning page, per-thruster `ESC_STATUS`). ~90 ArduSub-standard params are provided as inert
compat dummies so QGC's plugin is happy; the real tuning params use the standard `ATC_*`
names so QGC's Tuning page edits them live (see `PARAMETERS.md`).

- **Flight modes** use ArduSub custom_mode numbers (STABILIZE 0, ACRO 1, DEPTH_HOLD 2,
  SURFACE 9, MANUAL 19, MOTOR_DETECT 20). Unsupported ArduSub modes → `DENIED`.
- **Teleop** (works now): `MANUAL_CONTROL` (x/y/z/r + buttons) — the joystick path.
- **Being GCS-agnostic:** MAVLink is open — QGC is just one client. Under BlueOS on a Pi,
  the same link routes to any client (pymavlink/MAVSDK/mavros) over the network — e.g. a
  Jetson. **Autonomy** (GUIDED + `SET_POSITION_TARGET_LOCAL_NED` / `SET_ATTITUDE_TARGET`) is
  the Phase-2c roadmap item.
- **Persistence:** NVS (`Preferences`) — params in `srot_prm`, calibration in `srot_cal`.
  Params auto-save on change; calibration auto-saves on completion (deferred to Core 0).

---

## 4. Tuning & calibration

### A. Calibration playbook (run in order on a fresh vehicle; all triggerable from QGC)
1. **Gyro** — hold still; `PREFLIGHT_CALIBRATION p1=1` → `gyro_off`.
2. **6-point accel** — 6 orientations (QGC prompts); `p5=1` → `accel_off/scale`.
3. **Compass** — figure-8; `DO_START_MAG_CAL` (mag is off in normal ops, so this is optional).
4. **Baro/depth-zero** — at surface; `p3=1` → `baro_zero`.
5. **Motor-direction detect** (mode 20 or `DO_MOTOR_TEST`) — store per-motor sign → `motor_dir[8]`.
6. **Level/AHRS trim** — sub physically level; `p5=2` → `level_trim`.
Persist with `PREFLIGHT_STORAGE`.

### B. Cascaded PID tuning (inner first)
1. **Inner rate loop (ACRO):** raise `ATC_RAT_*_P` until crisp, just below oscillation; add
   `ATC_RAT_*_D` to damp overshoot, `ATC_RAT_*_I` for steady-state (clamped by `_IMAX`).
2. **Outer angle loop (STABILIZE):** raise `ATC_ANG_*_P` until attitude holds firmly.
3. Per axis: roll/pitch similar (vertical thrusters); yaw wants higher P / lower I.
4. Optional `ATC_RAT_*_FF` (start 0) sharpens small-command response.

**Hydrodynamic notes:** trim buoyancy/ballast *first* (the `LEVEL` cal only removes sensor
mounting offset). Added mass + drag heavily damp the plant → tolerate higher P, less D than
air. Vertical/horizontal planes are decoupled by the mixer, so tune them independently.

### C. Precision / micro-movements
`MOT_SPIN_MIN` (raise until the smallest input just moves a prop, no lurch) · `MOT_THST_EXPO`
(low-end linearizer, default 0.65) · `PILOT_YAW_RATE` (low = gentle yaw, centre holds
heading) · `PILOT_EXPO`/`PILOT_SPEED` (fine centre resolution + delicate translation).

### D. Relay auto-tune (Åström–Hägglund), `ATUNE=1` or `MAV_CMD_USER_5`
In open water, armed in STABILIZE, set `ATUNE=1`. It relay-excites each loop in turn — RATE
roll→pitch→yaw (`ATC_RAT_*`), ANGLE roll→pitch→yaw (`ATC_ANG_*`), then DEPTH (`DEPTH_*`) —
measures the limit-cycle (`Ku = 4A/(π·a)`, period `Tu`) and sets conservative
Ziegler–Nichols gains (`Kp=0.33·Ku`, `Ki=Kp/(0.5·Tu)`, `Kd=Kp·Tu/3`). ~1 min total; saves to
NVS and resets `ATUNE`. Abort by disarming / `ATUNE=0`. A phase with no motion is skipped.

### E. Stunt / pattern
`STUNT_SPIN_CNT`, `STUNT_RATE` (default 1 × 90°/s), `HEADROOM_DEPTH` (0.5 m). During a stunt
the active axis bypasses the angle limit; the passive axes stay in STABILIZE — tune those first.

---

## 5. SROT vs ArduPilot/ArduSub — honest comparison

| | SROT | ArduPilot / ArduSub |
|---|---|---|
| Attitude | BNO085 **hardware fusion** | **EKF3** software fusion (IMU+baro+GPS+mag) |
| Main loop | 500 Hz on a **dedicated core** | 400 Hz single-thread, time-sliced |
| Comms vs flight | separate cores — comms never preempts flight | same thread, time-budgeted |
| Attitude control | angle-P → rate-PID + FF | AC_AttitudeControl (sqrt + FF + input shaping) |
| Motor output | DShot (RMT or Pico bidir + RPM loop) | PWM / DShot |
| Position/nav | attitude + depth only | full position/velocity + waypoints |

**SROT wins** on latency (no EKF step in the loop), zero scheduler jitter (isolated flight
core), and digital ESC drive — a crisp low-jitter response for a small tethered sub.
**ArduPilot wins** on EKF fusion robustness, advanced attitude control, position/mission
capability, and field-hardened failsafes. Net: SROT trades fusion robustness + nav features
for latency, core isolation, and simplicity.
