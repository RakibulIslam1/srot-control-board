# JETSON ↔ AUV Communications Reference (SROT)

The complete contract the Jetson ("Mongla" ROS2 workspace, `duburi_ws`) uses to command the SROT
flight controller (ESP32). Everything is **MAVLink 2**; nothing here needs a custom dialect — the
one custom verb (`SROT_MOVE`) rides inside a standard `COMMAND_LONG`.

The design goal: the Jetson sends **high-level intent** ("go forward 3 s at 50 %", "turn to 90°",
"dive to 2 m") and the ESP32 owns the **motion primitive** — RPM-held cruise (voltage-independent),
**on-board braking** (no coast/drift), heading + depth hold, smooth ramps, and safety. The Jetson
never runs a per-axis loop and never sends a brake.

---

## 1. Link / transport

| Property        | Value                                                        |
|-----------------|-------------------------------------------------------------|
| Protocol        | MAVLink 2                                                    |
| Physical        | ESP32 **UART0** (`Serial`, TX0/RX0) → BlueOS / companion     |
| Baud            | **115200** (`MAVLINK_BAUD`)                                  |
| Vehicle sysid   | **1** (`MAV_SYSTEM_ID`)                                      |
| Autopilot compid| **1** = `MAV_COMP_ID_AUTOPILOT1` (`MAV_COMPONENT_ID`)        |
| GCS failsafe    | No GCS `HEARTBEAT` for **5 s** (`GCS_FAILSAFE_MS = 5000`) → failsafe (surface) |

The Jetson must send its own `HEARTBEAT` at ≥1 Hz or the AUV trips the GCS failsafe. All commands
target `target_system = 1`, `target_component = 1` (or `0` = broadcast, also accepted).

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpout:192.168.2.2:14550', source_system=255, source_component=190)
m.wait_heartbeat()                      # vehicle is sysid 1
# keep a heartbeat going in a background thread so the AUV doesn't failsafe:
#   m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
```

---

## 2. Arm / disarm — `MAV_CMD_COMPONENT_ARM_DISARM` (400)

`param1` = `1` arm / `0` disarm. Arming runs the pre-arm checks; a rejection comes back as a
`STATUSTEXT` (`"PreArm: ..."`) and a `COMMAND_ACK` of `MAV_RESULT_FAILED`.

```python
m.mav.command_long_send(1,1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0, 1,0,0,0,0,0,0)  # arm
```

---

## 3. Flight mode — `MAV_CMD_DO_SET_MODE` (176)

`param1` = base mode (use `MAV_MODE_FLAG_CUSTOM_MODE_ENABLED` = 1), `param2` = **custom_mode**:

| custom_mode | Mode         | Notes                                             |
|-------------|--------------|---------------------------------------------------|
| 0           | STABILIZE    | attitude hold, manual depth                       |
| 1           | ACRO         | rate mode                                         |
| 2           | DEPTH_HOLD   | attitude + depth hold                             |
| 9           | SURFACE      | ascend / failsafe target                          |
| 19          | MANUAL       | passthrough                                       |
| 20          | MOTOR_DETECT | thruster mapping/direction detect                 |
| 21          | AUTOTUNE     | control-loop PID relay tune                        |
| 22          | MOTOR_TUNE   | RPM-loop tune                                      |
| **23**      | **AUTO**     | **the movement mode — target for `SROT_MOVE`**    |

A `SROT_MOVE` command **auto-switches the vehicle into AUTO**, so you normally don't need to set the
mode yourself; do it explicitly only to pre-arm-hold in AUTO (station-keep) before the first move.

```python
m.set_mode(23)   # or command_long DO_SET_MODE with param1=1, param2=23
```

---

## 4. Direct teleop — `MANUAL_CONTROL` (still available)

`x` = forward, `y` = lateral, `z` = heave/throttle (**500 = neutral**), `r` = yaw. Range **±1000**
(`z` 0..1000). Scaled by `JS_GAIN`. Buttons map to ArduSub joystick functions (lights, relays,
arm). Use this for tele-op / manual override; it works in the manual/stabilize modes, not needed for
AUTO moves.

---

## 5. High-level moves — `MAV_CMD_SROT_MOVE` (31000) ★

One `COMMAND_LONG`, command id **31000**. Auto-enters AUTO and starts the primitive; a new command
**preempts** the running one (a quick brake first). Braking, ramps, heading- and depth-hold are all
on the ESP32.

| param  | Meaning                                                                                   |
|--------|-------------------------------------------------------------------------------------------|
| **p1** | **type** (see table below)                                                                |
| **p2** | **primary**: duration_s (fwd/back/strafe/arc) · degrees (turn) · depth_m (dive) · count (style) |
| **p3** | **speed 0..1** (translation & arc forward) · **yaw rate deg/s** (turn) · *ignored for dive* |
| **p4** | **turn**: `0` relative / `1` absolute-shortest-path · **arc**: signed **yaw rate deg/s** (+right / −left) · else 0 |
| **p5** | **timeout_s** (safety cap; ESP32 brakes out if exceeded, `0` → 60 s default)               |
| p6,p7  | reserved (0)                                                                              |

### p1 — type codes

| p1 | Verb        | primary (p2)  | p3               | p4                    |
|----|-------------|---------------|------------------|-----------------------|
| 0  | **forward** | duration_s    | speed 0..1       | —                     |
| 1  | **back**    | duration_s    | speed 0..1       | —                     |
| 2  | **strafe L**| duration_s    | speed 0..1       | —                     |
| 3  | **strafe R**| duration_s    | speed 0..1       | —                     |
| 4  | **turn**    | degrees       | yaw rate deg/s (0 → `MOVE_YAW_RATE`) | 0 rel / 1 abs |
| 5  | **dive**    | depth_m       | *ignored* — rate is always `MOVE_DEPTH_RATE` | —  |
| 6  | **stop**    | —             | —                | —  (brake to a halt)  |
| 7  | **hold**    | seconds (0 = until timeout) | —  | —  (station-keep: zero translation, hold depth + heading) |
| 8  | **style**   | count of 360° rolls | —          | —  (always ROLL at 90°/s) |
| 9  | **arc**     | duration_s    | forward speed 0..1 | signed yaw rate deg/s |

> **Wire type vs internal enum.** p1 is the *wire* code above (0-based). The firmware stores
> `movement::Type = p1 + 1`, and **`MV_TYPE` reports that internal enum**, not p1. So a `forward`
> (p1 = 0) shows up as `MV_TYPE = 1`. Any p1 outside 0..9 is rejected with `MAV_RESULT_DENIED`.

**Turn modes:** relative (`p4=0`) turns ±`p2`° from the current heading; absolute (`p4=1`) drives to
heading `p2` (0–360°) taking the **shortest** direction — this is the same yaw you read on the BNO /
OLED, so "turn to 90°" lands where the display shows 90°.

**Distance repeatability.** At the end of a translation the ESP32 applies **reverse thrust scaled by
the cruise speed** to null momentum, so it stops in place rather than coasting.

Voltage independence is handled *outside* the attitude loop by two mechanisms — see
`docs/T200_PROFILE.md`:

| Mechanism | Param | Default | Note |
|---|---|---|---|
| Slow per-thruster RPM trim | `THR_TRIM_EN` | **0 (off)** | Learns a bounded gain from measured vs expected RPM. **Enable in water only** — a prop in air spins far faster for the same duty and would drive the gains to their clamps. |
| Battery-voltage feedforward | `MOT_BAT_V_MAX` | **0 (off)** | Needs the *thruster* pack voltage, which arrives from the 2nd board over ESP-NOW. That link is **not implemented yet**, so this is currently inert. |

> ⚠️ **Both default OFF.** Until `THR_TRIM_EN = 1` (in water), "3 s @ 50 %" will travel *less*
> distance on a flat pack than a full one. Budget for this in mission tuning, or enable the trim.
>
> An earlier revision of this document claimed a closed-loop RPM controller provided this. It did —
> and it **oscillated**: an RPM setpoint inside the stabilisation loop made a 1° disturbance produce
> spin-up/stop/spin-up cycling. `THR_RPM_CLOSED_LOOP` is now **0** and RPM is deliberately out of the
> fast path. Do not turn it back on.

```python
CMD = 31000
def srot_move(m, t, primary=0.0, speed=0.0, mode=0.0, timeout=0.0):
    m.mav.command_long_send(1,1, CMD, 0, float(t), float(primary), float(speed),
                            float(mode), float(timeout), 0, 0)

srot_move(m, 0, primary=3.0, speed=0.5)              # forward 3 s @ 50 %
srot_move(m, 4, primary=90,  speed=45, mode=1)       # turn to absolute 90° at 45°/s (shortest)
srot_move(m, 4, primary=-30, speed=0,  mode=0)       # turn 30° left (relative, default rate)
srot_move(m, 5, primary=2.0, speed=0.2)              # smooth dive to 2.0 m at 0.2 m/s
srot_move(m, 9, primary=4.0, speed=0.4, mode=20)     # arc: forward 4 s @ 40 % turning +20°/s
srot_move(m, 8, primary=1)                           # one style maneuver
srot_move(m, 6)                                      # stop / brake now
```

### Feedback contract (drives the ROS Move action)

- Immediately: `COMMAND_ACK(command=31000, result=MAV_RESULT_IN_PROGRESS)`.
- While running: `COMMAND_ACK(..., IN_PROGRESS, progress=0..99)` at **~3 Hz** (`progress` = percent),
  plus live `NAMED_VALUE_FLOAT`s (below).
- On completion: one **`COMMAND_ACK(..., MAV_RESULT_ACCEPTED, progress=100)`** → map to the action
  *result*. A tumble/leak/over-angle safety abort disarms and ends the stream.

```python
while True:
    ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.command == 31000:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED: break   # done
        # ack.progress = percent complete → publish as action feedback
```

---

## 6. Telemetry to read

| Message               | Use                                                                    |
|-----------------------|------------------------------------------------------------------------|
| `HEARTBEAT`           | armed state, `custom_mode` (confirm AUTO=23), liveness                  |
| `ATTITUDE`            | roll/pitch/**yaw** — yaw is the heading for absolute turns (radians)    |
| `VFR_HUD` / `SCALED_PRESSURE` | depth (m) and pressure                                         |
| `SYS_STATUS` / `BATTERY_STATUS` | pack voltage, current                                        |
| `ESC_STATUS`          | per-thruster **RPM** (the feedback the braking relies on)              |
| `STATUSTEXT`          | pre-arm results, faults, cal, aborts (`"Abort: ..."`)                  |
| `NAMED_VALUE_FLOAT`   | move + vehicle scalars (below)                                         |

### Rates, and the one trap

| Message | Rate | Units / notes |
|---|---|---|
| `HEARTBEAT` | 1 Hz | `custom_mode` = FlightMode |
| `SYS_STATUS` | 2 Hz | health bits now reflect real IMU/depth validity |
| `ATTITUDE` | 10 Hz | **radians**, rad/s |
| `SCALED_IMU2` | 10 Hz | mg · mrad/s · mgauss |
| `SCALED_PRESSURE2` | 5 Hz | mbar |
| `VFR_HUD` | 5 Hz | **`alt = −depth`**, heading °, throttle % |
| `BATTERY_STATUS` | 1 Hz | id 0 = PM1 (electronics), id 1 = PM2 (thrusters) — see note |
| `POWER_STATUS` | 1 Hz | |
| `ESC_STATUS` | 5 Hz | two messages, `index` 0 and 4; **RPM only** (voltage/current are 0) |
| `NAMED_VALUE_FLOAT` | 2 Hz | see table below |
| `STK_*`, `HEAP` | 0.5 Hz | task stack high-water + free heap (diagnostics) |

> ⚠️ **During a parameter download the vehicle sends ONLY `HEARTBEAT` (1 Hz) and `ESC_STATUS`.**
> Everything else — attitude, depth, and the `SROT_MOVE` progress ACKs — stops until the
> `PARAM_REQUEST_LIST` completes (~10-15 s for 190+ params). A companion must **not** treat that as a
> telemetry dropout or a stalled move. Either avoid downloading params mid-mission, or suppress the
> watchdog while `PARAM_VALUE` messages are arriving.

> **Battery note:** `BATTERY_STATUS` id 0 is the **SBC/electronics** pack (local ADC); id 1 is the
> **thruster** pack, which arrives from a 2nd board over ESP-NOW. That ESP-NOW sender is not
> implemented yet, so id 1 currently reports nothing.

### `NAMED_VALUE_FLOAT` names

| name       | Meaning                                                             |
|------------|---------------------------------------------------------------------|
| `MV_STATE` | movement phase: 0 idle · 1 cruise · 2 brake · 3 turn · 4 dive · 5 style · 6 done |
| `MV_PROG`  | move progress 0..1                                                   |
| `MV_TYPE`  | active `movement::Type` (1 fwd … 10 arc; 0 = idle)                  |
| `LEAK`     | leak detected (1/0)                                                  |
| `WTEMP`    | water temp °C                                                       |
| `CURR`     | main battery current A (**always 0** — no current sensor fitted)     |
| `KILL`     | kill-switch (1/0)                                                   |
| `GAIN`     | live pilot gain 0.1..1.0 (joystick gain buttons change it)           |
| `STUNT_PRG`| stunt / autotune / motor-tune progress 0..1                          |
| `ATUNE`    | autotune active (1/0)                                                |

---

## 7. Payload (torpedo / dropper / lights)

- Servos: `MAV_CMD_DO_SET_SERVO` (183), `param1` = channel, `param2` = PWM µs — driven out the
  PCA9685 servo extender.
- Relays: joystick button functions `JS_RELAY1/2_ON/OFF/TOGGLE` via `MANUAL_CONTROL.buttons`, or map
  as needed. Lights: `JS_LIGHTS1_*`.

---

## 8. Constant-distance moves + movement params

The system is **purely timer-based** — a move is *duration × speed*, no distance sensor or estimate.
Two things make that repeatable:

1. **On-board braking** (always active): reverse thrust scaled by the cruise speed nulls momentum, so
   the vehicle stops where the timer ends instead of coasting.
2. **Thrust normalisation** (opt-in): a slow per-thruster gain learned from measured vs expected RPM,
   plus battery-voltage feedforward. This is what removes the battery-state dependence.

**To get repeatable distance you must enable the trim, in water:**
```
THR_TRIM_EN = 1      # slow RPM-based thrust normalisation (default 0 — keep 0 with props in air)
RPM_MAX     = 3600   # T200 at 16 V; set for your pack (3075 @ 12 V, 3800 @ 20 V)
DSHOT_BIDIR = 1      # required — the trim needs RPM telemetry
```
Optionally also `MOT_BAT_V_MAX` (thruster-pack full-charge volts) once the ESP-NOW voltage link
exists. Full rationale and the physics in `docs/T200_PROFILE.md`.

**Do not** set `THR_RPM_CLOSED_LOOP = 1` or `RPM_LOOP = 1`. That older design put an RPM setpoint
inside the stabilisation loop and oscillated (spin-up → stop → spin-up on a 1° disturbance); it is
off by default for that reason.

### Movement params (`MOVE_*`)

| param            | Default   | Meaning                                            |
|------------------|-----------|----------------------------------------------------|
| `MOVE_CRUISE_MAX`| 0.80      | max normalized cruise speed (clamps p3)            |
| `MOVE_ACCEL`     | 2.5 /s    | translation speed ramp (smooth start)              |
| `MOVE_BRAKE_GAIN`| 0.55      | reverse-thrust fraction during brake               |
| `MOVE_BRAKE_K`   | 0.60      | brake duration = `K · cruise_speed` (s)            |
| `MOVE_DEPTH_RATE`| 0.20 m/s  | smooth dive/ascend rate (no splash)                |
| `MOVE_YAW_RATE`  | 45 °/s    | default turn rate when p3 = 0                       |

---

## 9. Minimal mission loop (Jetson side)

```python
m.wait_heartbeat()
arm(m); set_auto(m)
srot_move(m, 0, primary=3.0, speed=0.5); wait_done(m)   # forward
srot_move(m, 4, primary=90, speed=45, mode=1); wait_done(m)   # face 90°
srot_move(m, 5, primary=2.0, speed=0.2); wait_done(m)   # dive to 2 m
# ... perception decides the next high-level verb ...
disarm(m)
```

The ESP32 holds heading and depth on straight legs, brakes to a stop with no drift, ramps dives so
the hull doesn't splash, and aborts+disarms on a leak or tumble — so the Jetson only ever decides
*what* to do next, never *how* to drive each thruster.

---

## 10. Complete accepted-command reference

Everything the firmware dispatches. **Anything not listed returns `MAV_RESULT_UNSUPPORTED`.**

| MAV_CMD | id | Params / notes |
|---|---|---|
| `COMPONENT_ARM_DISARM` | 400 | p1 1=arm / 0=disarm. Rejection → `STATUSTEXT "PreArm: …"` + `MAV_RESULT_FAILED` |
| `DO_SET_MODE` | 176 | **p2** = custom_mode (table §3) |
| **`SROT_MOVE`** | **31000** | the movement verb — §5 |
| `DO_MOTOR_TEST` | 209 | p1 motor (1-based), p2 throttle type (0 = percent), p3 throttle, p4 timeout s. **Requires ARMED.** Re-send ≥2 Hz as a keep-alive; window 600–3000 ms |
| `PREFLIGHT_CALIBRATION` | 241 | p1 gyro · p2 mag · p3 baro · p5 1 = accel 6-point, 2/4 = level |
| `ACCELCAL_VEHICLE_POS` | 42429 | p1 = 1..6 face, during the 6-point accel cal |
| `DO_START_MAG_CAL` | 42424 | compass cal |
| `PREFLIGHT_STORAGE` | 245 | **p1 = 1** save params · **p1 = 2** factory reset (params **and** learned motor directions) |
| `DO_SET_SERVO` | 183 | p1 channel (1-based), p2 µs — PCA9685 |
| `DO_SET_RELAY` | 181 | p1 relay instance (0-based), p2 1/0 |
| `USER_1/2/3` | 31010-31012 | yaw / pitch / roll spin (stunt) |
| `USER_4` | 31013 | pattern |
| `USER_5` | 31014 | autotune start |
| `REQUEST_MESSAGE` | 512 | |
| `REQUEST_AUTOPILOT_CAPABILITIES` | 520 | → `AUTOPILOT_VERSION` |
| `PREFLIGHT_REBOOT_SHUTDOWN` | 246 | |

**Messages handled:** `HEARTBEAT`, `PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, `PARAM_SET`,
`COMMAND_LONG`, `COMMAND_INT`, `SET_MODE`, `MANUAL_CONTROL`, and the mission set (`MISSION_COUNT`,
`MISSION_ITEM_INT`, `MISSION_REQUEST_LIST`, `MISSION_REQUEST_INT`, `MISSION_REQUEST`,
`MISSION_CLEAR_ALL`, `MISSION_ACK`).

`COMMAND_INT` maps `x`/`y`/`z` → p5/p6/p7, so an INT-encoded `SROT_MOVE` works (timeout lands in `x`
as an integer).

### Things that will bite you
- **A re-sent `SROT_MOVE` re-runs.** Start is edge-triggered on `mv_seq`, which increments per
  accepted command — there is no idempotency key. Send once and track the ACK.
- **A new move preempts the running one.** There is no queue.
- `MV_STATE` / `mv_active` are only published while the mode is **AUTO**.
- On completion `MV_TYPE` is **not** re-sent; you get `MV_STATE = 0` and `MV_PROG = 1.0`.
- Autotune and motor-tune now **require ARMED**, and **disarming aborts them** (as does leaving the
  mode). Motor test no longer auto-disarms when you stop sending the keep-alive.
