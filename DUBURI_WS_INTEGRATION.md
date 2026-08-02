# Integrating Hengla (the SROT board) into `duburi_ws`

**Audience:** an agent or engineer who has never seen this board, tasked with modifying
[`fh1m/duburi_ws`](https://github.com/fh1m/duburi_ws) to fly on this controller instead of a
Pixhawk/ArduSub.

**Naming:** the *board* is **SROT**; the *firmware* is **Hengla**. `MAV_CMD_SROT_MOVE` keeps its
name and its wire id (31000) — it is board-level, and renaming it would break every client.

**Read first:** [`JETSON_COMMS.md`](JETSON_COMMS.md) — the authoritative wire contract. This document
is the *migration plan*; that one is the *interface*.

---

## ⚠ 0a. Read this before you design anything

Four audit rounds (2026-07-30 and 2026-07-31) found 29 real defects, several of which change what a
companion must assume. Full detail in [`AUDIT.md`](AUDIT.md); the ones that affect **your** design:

1. **The depth PID's sign was inverted, and the SURFACE failsafe drove the vehicle DOWN.** Fixed —
   but the loop had **never run closed**, because the Bar30 was not fitted during development.

   **The two mandatory bench checks (§8) gate every AUTO move — i.e. every `SROT_MOVE` primitive,
   `move_forward` included. There is no depth-free path through AUTO.** `SROT_MOVE` auto-enters
   `AUTO`, and `task_control_loop.cpp` runs `depth::setTarget(...)` + `depth::update(...)` under the
   `AUTO` case for **every** primitive, not just `DIVE`. A "control-only" mission with no `set_depth`
   anywhere still runs the loop that has never run closed.

   This paragraph used to say "every depth-dependent behaviour", which read as *"skip the depth
   checks if you are only driving forward"* — the exact run where a sign error bites first, and in
   water a vertical runaway mid-leg is indistinguishable from a buoyancy problem. Corrected
   2026-08-02; the finding is the companion team's.

   **An in-air `move_forward` is NOT partial validation.** At ~0 m the latched target and the
   measurement agree, so the error is ~0, the loop demands ~0 vertical thrust, and nothing about its
   sign, gain or authority has been exercised. A successful dry move is evidence of nothing.
   (`AUDIT.md` R1.)
2. **A move now always reaches exactly one terminal result** — `ACCEPTED`, `CANCELLED`, `FAILED` or
   `DENIED`. Three separate hang paths were fixed. **If you write the action client to wait only for
   `ACCEPTED`, it will still hang**, because preemption resolves as `CANCELLED` and an unstartable
   move as `FAILED`. See the terminal-result table in `JETSON_COMMS.md` §5.
3. **Preemption is now observable.** Sending a new `SROT_MOVE` while one is running resolves the old
   one `CANCELLED`. This is the documented way to interrupt, and your action server should map it to
   *abort*, not *error*.
4. **Non-finite command parameters are rejected** (`DENIED`) for every command. An uninitialised or
   `NaN` field in a `Move` goal fails cleanly instead of being accepted — it used to reach the ESCs as
   an undefined DShot value. Validate goals Python-side anyway, but you will now get told.
5. **`MANUAL_CONTROL` axes are clamped, not rejected** (`x/y/r` ±1000, `z` 0..1000). Out-of-range no
   longer produces a full-authority burst, but it is silently truncated — so a units bug in your
   conversion will now under-drive rather than over-drive, which is harder to notice. Keep the
   conversion helpers' bounds assertions.
6. **New disarm paths in AUTO:** the safety monitor now fails closed on a NaN gyro or NaN depth. A
   sensor going non-finite mid-move aborts and disarms instead of continuing on garbage. Your
   supervisor must handle an unexpected disarm mid-action.
7. **`PATTERN` is refused without a depth sensor** (like `DEPTH_HOLD` and `AUTO`), falling back to
   `STABILIZE` with a `STATUSTEXT`. If you drive modes directly, handle the refusal.
8. **Still outstanding, do not rely on:** the LoRa mission-waypoint upload path has **no application
   CRC** — it is still the one wire protocol in the tree with no message-level corruption detection,
   so do not use it to carry navigation targets. (The radio's *PHY* CRC **is** now enabled as of
   2026-07-31 — `AUDIT.md` R17 — so corrupt frames are dropped at the SX127x. That shrinks the
   exposure a great deal but does not give the waypoint path a checksum of its own.) And the Pico's
   hardware e-stop line fails *permissive* (a broken wire reads as "run"); it is redundant with two
   other failsafes, but do not count it as your safety story. Both remain deferred in `AUDIT.md`.
9. **Parameters do not survive the current firmware update.** Beyond a `PARAM_DEFAULTS_VER` bump,
   the NVS partition was **moved and enlarged** (it was physically too small to hold the parameter
   set, which is why saves had been failing). Bringing a board up to current firmware needs
   `pio run -t erase` then upload, and **loses both the tune and the `CAL_*` block**. Export from
   Bondor first — see "Parameter backup" in §9.

---

## 0. TL;DR

- SROT is a **custom flight controller** (ESP32 + RP2350 co-processor) that already does attitude
  stabilisation, depth hold, thrust allocation for a vectored-6DOF frame, arming/failsafes, and
  **complete motion primitives with on-board braking**.
- It speaks **MAVLink 2** over serial. `duburi_ws` already speaks MAVLink 2 via **pymavlink** — so
  this is a *transport-and-verbs* swap, not a rewrite.
- The single custom verb, **`MAV_CMD_SROT_MOVE` (31000)**, has the exact shape of a ROS 2 action:
  immediate `IN_PROGRESS`, ~3 Hz progress 0..99, then **one of four terminal results**
  (`ACCEPTED` / `CANCELLED` / `FAILED` / `DENIED`). It maps **1:1 onto `duburi_interfaces/Move`** —
  including preemption, which the board now reports rather than leaving dangling.
- Consequence: most of `motion_*.py` collapses into *"send one command, relay its progress"*, and
  several Python control loops delete entirely.
- **Vision, planner, missions, YASMIN, the `COMMANDS` registry and `Move.action` are untouched.**

---

## 1. Why this is a small change, structurally

`duburi_ws` is unusually well-positioned for this:

| Existing fact | Why it helps |
|---|---|
| No `mavros` — one node owns one `pymavlink` connection (`duburi_control/pixhawk.py`) | One file is the entire hardware boundary |
| The whole control surface is one action, `/duburi/move` | Callers don't change when the backend does |
| `duburi_sensors` already has a HAL pattern (`YawSource` factory + `sources/base.py`) | Copy it for `FlightController` |
| Same physical hardware (8× T200, vectored 6DOF, Bar30, BNO085) | No mechanical or mixer rework |

The one structural gap: **there is no thrust/autopilot HAL.** `Pixhawk` is constructed directly and
its methods are called throughout `motion_*.py`. Creating that interface is step 1.

---

## 2. What SROT replaces

| `duburi_ws` component | Action | Why |
|---|---|---|
| `duburi_control/pixhawk.py` | **Rewrite** → `srot.py` behind a HAL | Different verbs (see §4) |
| `duburi_control/connection_config.py` | **Rewrite** | No Pixhawk USB IDs / BlueOS router; SROT is a direct serial or UDP endpoint |
| `duburi_control/heading_lock.py` | **Delete** | SROT holds heading on-board, and every `SROT_MOVE` translate leg explicitly locks the heading it started with. The reason this existed — an untrusted in-hull compass — is solved differently: SROT's attitude is a BNO085 **game**-rotation vector, so the magnetometer is never fused and thruster current cannot move yaw. See the caveat below |
| `duburi_control/motion_depth.py` (ALT_HOLD dance, prime phase, `SET_POSITION_TARGET_GLOBAL_INT`) | **Delete** | `SROT_MOVE` type 5 (dive) ramps the depth setpoint on-board at `MOVE_DEPTH_RATE` |
| `duburi_control/motion_easing.py` | **Delete** | `MOVE_ACCEL` ramps and `MOVE_BRAKE_*` braking are on-board |
| `motion_{yaw,forward,lateral}.py` | **Collapse** | Each becomes one `SROT_MOVE` + progress relay |
| `duburi_sensors` yaw fusion (`composite_bno_dvl`, `mavlink_ahrs`, `bno085`) | **Optional** | SROT publishes fused attitude in `ATTITUDE`. Keep the DVL for position/velocity; the BNO085-on-ESP32-C3 becomes redundant |
| ArduSub mode machinery (`set_mode`, MANUAL→neutral→disarm) | **Delete** | Different mode numbers, and SROT disarms cleanly |
| `duburi_vision` (all) | **Keep** | Unrelated |
| `duburi_planner` (CLI, missions, YASMIN) | **Keep** | Unrelated |
| `duburi_control/commands.py` (`COMMANDS` registry) | **Keep** | The verb table is backend-agnostic |
| `duburi_interfaces/Move.action`, `DuburiState.msg` | **Keep** | See §5 |
| `duburi_control/payload.py` | **Keep or migrate** | Currently a direct ESP32-over-USB link. SROT can drive the same payload via `DO_SET_SERVO`/`DO_SET_RELAY` on its PCA9685 — migrating removes a USB device |

**Rule of thumb:** anything in `duburi_ws` that closes a loop on vehicle *attitude* or *depth* should
be deleted. Anything that closes a loop on *perception* (vision servo, DVL distance) stays.

---

## 3. Add a `FlightController` HAL first

Do this before touching any `motion_*.py`, so both backends can coexist while you migrate.

Copy the existing pattern from `duburi_sensors`:
- `sources/base.py` → an abstract base with the methods `motion_*.py` actually uses
- `factory.py` → selects by a ROS param

```
duburi_control/
  fc/
    base.py       # FlightController ABC: connect, arm, disarm, set_mode, manual, move, telemetry
    pixhawk.py    # existing implementation, moved behind the ABC (keep working!)
    srot.py       # new
    factory.py    # make_fc(kind: str) -> FlightController
```

Add a ROS param `flight_controller` (`pixhawk` | `srot`) on `duburi_manager`, defaulting to `pixhawk`
until SROT is pool-proven. This lets you A/B the two on the same vehicle.

The ABC should expose **intent**, not channels — that is the whole point of the migration:

```python
class FlightController(ABC):
    def arm(self, timeout: float = 5.0) -> bool: ...
    def disarm(self) -> bool: ...
    def set_mode(self, mode: str) -> bool: ...          # 'STABILIZE' | 'DEPTH_HOLD' | 'AUTO' | ...
    def manual(self, fwd: float, lat: float, up: float, yaw: float) -> None:  # -1..1
    def move(self, verb: str, **kw) -> MoveHandle: ...  # returns a progress-yielding handle
    def telemetry(self) -> Telemetry: ...               # attitude, depth, battery, rpm, armed, mode
```

`MoveHandle` is what makes the action server trivial: it yields progress 0..1 and resolves to a
result. On the SROT backend it is a thin wrapper over the `COMMAND_ACK` stream.

---

## 4. Transport and verb differences

Same pymavlink, same MAVLink 2, `sysid 1 / compid 1` — but different verbs.

### 4.1 Actuation: `RC_CHANNELS_OVERRIDE` → `MANUAL_CONTROL`

`pixhawk.py` today writes 6 RC channels with `percent_to_pwm()` (−100..100 % → 1100..1900 µs,
1500 neutral, `NO_OVERRIDE = 65535`). **SROT does not implement `RC_CHANNELS_OVERRIDE` at all.**

| duburi_ws channel | const | SROT `MANUAL_CONTROL` field | Range |
|---|---|---|---|
| forward | `CH_FORWARD = 4` | `x` | **±1000** |
| lateral | `CH_LATERAL = 5` | `y` | **±1000** |
| yaw | `CH_YAW = 3` | `r` | **±1000** |
| throttle / heave | `CH_THROTTLE = 2` | `z` | **0..1000, 500 = neutral** |
| pitch | `CH_PITCH = 0` | — | not exposed; SROT stabilises pitch |
| roll | `CH_ROLL = 1` | — | not exposed; SROT stabilises roll |
| — | — | `buttons` | 16-bit mask → mapped functions (§4.4) |

Conversion from the existing percent API:

```python
def pct_to_mc(p: float) -> int:      # -100..100  ->  -1000..1000
    return int(max(-100.0, min(100.0, p)) * 10)

def pct_to_mc_z(p: float) -> int:    # -100..100  ->  0..1000 (500 neutral)
    return int(500 + max(-100.0, min(100.0, p)) * 5)
```

> **`NO_OVERRIDE` has no equivalent.** With RC override you could release a channel and let the
> autopilot own it (that is how `motion_depth.py` hands Ch3 to ALT_HOLD). Under `MANUAL_CONTROL` you
> always send all four axes. To "release" depth, **change mode** (`DEPTH_HOLD`) or use a `SROT_MOVE`
> dive — don't try to emulate channel release.

Also note `MANUAL_CONTROL` is scaled by the **live pilot gain** (`GAIN`, 0.1..1.0, default from
`JS_GAIN_DEFAULT`). Set it to 1.0 for programmatic control or your commands will be scaled by 0.5.

### 4.2 Modes

This is the **complete** `FlightMode` enum, not a selection. `HEARTBEAT.custom_mode` can carry any
of these, so map exhaustively or your state publisher will hit an unknown value.

| SROT `custom_mode` | Name | duburi_ws equivalent | Settable by `DO_SET_MODE`? |
|---|---|---|---|
| 0 | STABILIZE | `STABILIZE` | yes |
| 1 | ACRO | `ACRO` | yes |
| 2 | DEPTH_HOLD | `ALT_HOLD` | yes |
| 9 | SURFACE | `SURFACE` | yes |
| 19 | MANUAL | `MANUAL` | yes |
| 20 | MOTOR_DETECT | — | yes (bench only, props off) |
| 21 | AUTOTUNE | — | yes |
| 22 | MOTOR_TUNE | — | yes (bench only, props off) |
| 23 | **AUTO** | *(new — the mode `SROT_MOVE` runs in)* | yes |
| 100 | STUNT | — | no — entered by command only |
| 101 | PATTERN | — | no — entered by command only |

`SROT_MOVE` auto-switches to AUTO, so you rarely set it explicitly. **20, 21, 22, 100 and 101 are
not modes a mission should ever command**, but a supervisor must still recognise them: seeing one
mid-mission means something else put the board there, and 20/22 spin motors.

⚠️ **Without a depth sensor fitted, `DEPTH_HOLD` and `AUTO` are refused** and the board falls back to
STABILIZE with a `STATUSTEXT`. Handle that rejection rather than assuming the mode took.

### 4.3 The move verb

See `JETSON_COMMS.md` §5 for the full parameter table. The mapping onto `Move.action`:

The `p1` wire type is **not** the internal `movement::Type` — the board adds 1. Send the wire value:

| p1 (send this) | Verb | `movement::Type` |
|---|---|---|
| 0 | FWD | 1 |
| 1 | BACK | 2 |
| 2 | LEFT | 3 |
| 3 | RIGHT | 4 |
| 4 | TURN | 5 |
| 5 | DIVE | 6 |
| 6 | STOP | 7 |
| 7 | HOLD | 8 |
| 8 | STYLE | 9 |
| 9 | ARC | 10 |

Anything outside 0..9 is `DENIED`.

| `Move.action` goal field | SROT | Note |
|---|---|---|
| `cmd` ("forward", "turn", …) | p1 wire type 0..9 | `COMMANDS` registry maps the verb string |
| `duration` | p2 (translation/arc/hold) | seconds |
| `target` (heading) | p2 with p4=1 | absolute-shortest-path turn |
| `gain` | p3 | 0..1 speed, or deg/s for turn |
| `distance_m` | **no SROT equivalent** | keep the DVL loop in Python |
| `target_class`, `camera`, `offset_*`, `err_*` | **no SROT equivalent** | vision servo stays in Python |
| `kp_lat/yaw/depth/forward` | **obsolete for SROT verbs** | those loops are on-board now |

Feedback → action feedback:

| SROT | `Move.action` feedback |
|---|---|
| `COMMAND_ACK.progress` 0..99 | `current_value` / progress |
| `MV_STATE` — see the table below | `phase` |
| `MV_PROG` 0..1 | progress |
| terminal `COMMAND_ACK(ACCEPTED, 100)` | `result.success = True` |
| safety abort → disarm + `STATUSTEXT` | `result.success = False`, `message` |

`MV_STATE` values, from `movement.cpp`:

| Value | Phase | Value | Phase |
|---|---|---|---|
| 0 | IDLE | 4 | DIVE |
| 1 | CRUISE | 5 | STYLE |
| 2 | BRAKE | **6** | **HOLD** |
| 3 | TURN | **7** | **DONE** |

> ⚠️ An earlier version of this table said `6 = done`. It does not — **6 is HOLD and 7 is DONE**.
> A client using the old mapping reports a hold leg as finished while the vehicle is still holding
> station. `MV_STATE` is a progress *hint* regardless: the terminal `COMMAND_ACK` is the only
> authority on completion.

**Three traps:** a re-sent `SROT_MOVE` **re-runs** (start is edge-triggered on `mv_seq`; there is no
idempotency key); a new move **preempts** the running one (no queue), so your action server must
serialise goals itself; and `MAV_RESULT_TEMPORARILY_REJECTED` is a **fifth, non-terminal** reply,
returned when the board could not take the control mutex in time. Retry it — do not treat it as a
failure, and do not treat it as one of the four terminal results.

### 4.4 Buttons, payload, params

- Payload: `DO_SET_SERVO` (183) and `DO_SET_RELAY` (181) drive the PCA9685 — this can replace
  `payload.py`'s separate USB ESP32.
- `MANUAL_CONTROL.buttons` maps to the `BTN0..15_FUNCTION` params (arm, disarm, modes, lights,
  relays, gain). Only relevant if a human joystick is in the loop.
- Parameters are standard `PARAM_SET`/`PARAM_REQUEST_*`. **190+ params take 10-15 s to download, and
  during that window the vehicle sends only `HEARTBEAT` and `ESC_STATUS`** — see the trap in
  `JETSON_COMMS.md` §6. Do not download params mid-mission.

---

## 5. Telemetry → `DuburiState`

| `DuburiState` field | SROT source | Note |
|---|---|---|
| `armed` | `HEARTBEAT.base_mode` | |
| `mode` | `HEARTBEAT.custom_mode` | map via §4.2 |
| `yaw_deg` | `ATTITUDE.yaw` | **radians** → degrees. This is fused BNO085; no separate yaw source needed |
| `depth_m` | `VFR_HUD.alt` | **negated** (`alt = −depth`), or `SCALED_PRESSURE2` |
| `battery_voltage` | `BATTERY_STATUS` id 0 | **id 0 is the electronics pack**; id 1 is the thruster pack (see below) |

`/duburi/imu_rates` ← `ATTITUDE.{rollspeed,pitchspeed,yawspeed}` (rad/s) or `SCALED_IMU2`.

**New capability:** `ESC_STATUS` at 5 Hz carries **real per-thruster RPM** (bidirectional DShot from
Bluejay ESCs). `duburi_ws` has never had this. Two messages, `index` 0 and 4. Useful for a fouled-prop
or dead-thruster health check — publish it as a new topic.

---

## 6. The offload boundary

> **The Jetson sends intent. The board owns primitives, braking, holds, and aborts.**

On-board (do **not** reimplement in ROS):
attitude stabilisation · depth hold · thrust allocation (vectored 6DOF) · per-thruster direction ·
motion primitives with ramps and braking · heading hold between moves · arming and pre-arm checks ·
leak / low-battery / GCS-loss failsafes → SURFACE · thruster stall detection · ESC telemetry.

On the Jetson:
perception · mission logic and state machines · *which* verb to run next · DVL-based position loops ·
vision servoing · payload sequencing · logging.

---

## 7. Migration order (each step independently testable)

1. **HAL only.** Extract `FlightController`, move `Pixhawk` behind it, add the `flight_controller`
   param. *Checkpoint: existing missions still run identically on Pixhawk.*
2. **`srot.py` connect + telemetry.** Implement connect/heartbeat/telemetry only.
   *Checkpoint: `/duburi/state` populates from SROT; nothing actuates.*
3. **Arm / disarm / modes.** *Checkpoint: arm and disarm from the CLI, on the bench, props off.*
4. **`manual()`.** Port the `MANUAL_CONTROL` mapping. *Checkpoint: teleop moves the vehicle correctly
   on every axis — verify against `docs/THRUSTER_MAP.md` before trusting any sign.*
5. **`move()` + the ACK stream.** Wire `SROT_MOVE` into the `/duburi/move` action server for
   forward/back/left/right/turn/dive/stop/hold, plus **arc** (p1 = 9) and **style** (p1 = 8), which
   earlier revisions of this list omitted. *Checkpoint: each verb runs and reports progress, and
   all four terminal results resolve the action — test preemption explicitly, it is the one that
   hangs a naive client.*
6. **Delete the redundant loops** (`heading_lock`, `motion_easing`, the ALT_HOLD dance).
   *Checkpoint: missions still complete, with less Python.*
7. **Vision verbs last** — they only need `manual()`, which is already done.

---

## 8. Before the first wet test

- [ ] **Thruster directions verified** against `docs/THRUSTER_MAP.md` using Bondor's motor test. The
      four horizontals are at 45° and each pushes a *different diagonal* — they do not share a
      direction. Fix any reversed one with `MOT_n_DIRECTION`.
- [ ] **Depth sensor fitted.** Without it, `DEPTH_HOLD`/`AUTO`/`PATTERN` are refused and the SURFACE
      failsafe falls back to a fixed open-loop ascent.
- [ ] **⚠ Depth loop verified BY HAND, on the bench, before it is trusted in water.** Its sign was
      inverted until 2026-07-30 and it has never run closed (no Bar30 was fitted during development),
      so no observed vehicle behaviour has ever validated it.
      **This gates every AUTO move — every `SROT_MOVE` primitive, `move_forward` included** — not
      just `DIVE`/`set_depth`/`surface`. `SROT_MOVE` auto-enters `AUTO` and `AUTO` closes the depth
      loop under every primitive. **An in-air `move_forward` is not partial validation**: at ~0 m
      target and measurement agree, so the loop is never exercised.
      Two checks, both mandatory:
      - Enter `DEPTH_HOLD`, then raise and lower the sensor by hand. The vertical thrusters must push
        **back toward** the latched depth, not away from it.
      - Force the SURFACE failsafe (e.g. trip the leak input with `LEAK_EN = 1`) at a simulated depth
        and confirm the demand is **ascend**. This is the path that was driving the vehicle *down*.
- [ ] **Terminal-result handling in the action client** covers `CANCELLED`, `FAILED` and `DENIED`, not
      just `ACCEPTED` — otherwise a preempted or unstartable move hangs the action. `JETSON_COMMS.md` §5.
- [ ] **Goal validation Python-side** rejects `NaN`/`inf` before sending. The board now rejects them
      too (`DENIED`), but failing in your own node gives a far better error.
- [ ] **`THR_TRIM_EN = 1` (in water)** if you need repeatable distance — it is **off** by default and
      must stay off with props in air. See `docs/T200_PROFILE.md`.
- [ ] **`RPM_MAX`** set for your pack (3600 @ 16 V for a T200).
- [ ] **`JS_GAIN_DEFAULT` / `GAIN`** — set to 1.0 for programmatic control.
- [ ] GCS heartbeat ≥1 Hz from the Jetson, or the 5 s failsafe surfaces the vehicle.

## 9. Known gaps (as of this writing)

| Gap | Impact |
|---|---|
| ESP-NOW thruster-battery link is **off by default** *(no longer missing — see below)* | `BATTERY_STATUS` id 1 and the voltage feedforward stay inert until `ESPNOW_EN = 1` **and** `MOT_BAT_V_MAX` is set |
| No per-thruster current sensing, and pack current is not calibrated | `ESC_STATUS` current is always 0. Pack `CURR` reads 0 because `BATT_CURR_MULT` is a **compile-time** 0.0 — it is not a parameter, so enabling it needs a firmware rebuild. Note `BATTERY_STATUS.current_battery` reports `-1` (unknown) while `NAMED_VALUE_FLOAT CURR` reports `0.0` |
| `THR_POLE_PAIRS` is compile-time (7, correct for T200) | A different motor needs a firmware rebuild |
| `STYLE` is always a roll at 90°/s | Not selectable |
| Mission protocol (`MISSION_*`) is handled but minimal | Prefer `SROT_MOVE` sequencing from ROS |
| **Yaw is RELATIVE unless `MAG_YAW_REF = 1`** | An arbitrary power-on zero drifting ~0.5–3 °/min. Heading *hold* is unaffected (it tracks a target in the same frame), but **absolute `TURN` (`p4 = 1`) is only meaningful with the reference on**, and headings are not comparable between dives. See below |

### The thruster pack is now a real source — what changed on 2026-07-31

This section used to say the ESP-NOW link was "not implemented". It is, end to end, and that
changes two pieces of advice above:

- `env:second-board` is a real firmware (`src/second_board/`) that broadcasts the thruster-pack
  voltage. The flight controller receives it (`drivers/espnow_link`), and `BATTERY_STATUS` **id 1
  is emitted** whenever the link is fresh. It is *absent*, never zero, when the link is down —
  so "no id 1 for >2 s" means no data, not a flat pack.
- The mixer's **battery-voltage feedforward is wired to it** (`MOT_BAT_V_MAX`), so a timed move
  travels the same distance on a full or a flat pack. That is now the primary mechanism for
  repeatable distance; the RPM trim (`THR_TRIM_EN`) is complementary, not the fallback.
- There is a **low-thruster-battery failsafe** (`FS_BAT_VOLTAGE`, debounced by `FS_BAT_HOLD_MS`)
  that was inert before this link existed and is live once `ESPNOW_EN = 1`. Your supervisor can
  now be surfaced by a flat *thruster* pack, not just a flat electronics pack.

Both are **off by default** (`ESPNOW_EN = 0`, `MOT_BAT_V_MAX = 0`). Enabling them is a
configuration step, not a firmware change.

Two related parameter changes that will bite an old `.params` file: `PM1_VMULT` changed units
(volts-per-count → divider ratio, ~11) and `PM2_VMULT` went from inert to a live trim on the
ESP-NOW voltage, with a one-shot migration at boot. A backup taken before 2026-07-31 will report
wrong battery voltages until both are recalibrated.

### STATUSTEXT strings worth matching

The board says things a supervisor should react to rather than discover. Beyond the mode-refusal
and yaw-alignment messages named elsewhere in this document:

| Message | Means |
|---|---|
| `"ESPNOW_EN=0 - no thruster-pack voltage (set it to 1)"` | at boot; Battery 2 and the voltage feedforward will stay inert |
| `"ESP-NOW started"` / `"ESP-NOW init failed - retrying"` | the thruster-pack link came up, or the radio will not initialise |
| `"Params reset to build defaults"` | the tune is gone — this is not the vehicle you calibrated |
| `"NVS reformatted - params AND calibration lost, re-import"` | worse: the `CAL_*` block went too |
| `"<NAME> set but NOT saved (NVS full?)"` | a `PARAM_SET` applied live but will not survive a reboot |
| `"PM2_VMULT migrated to 1.0 (it is now an ESP-NOW trim)"` | one-shot units migration ran on this boot |

### The yaw reference, in detail — this one will affect your design

`duburi_ws` grew `heading_lock.py` because the in-hull compass could not be trusted. SROT
answers the same problem by keeping the magnetometer **out of the attitude fusion entirely**
(`SH2_GAME_ROTATION_VECTOR`), so no amount of thruster current moves roll/pitch/yaw. The
trade is that yaw has no earth reference.

`MAG_YAW_REF = 1` enables a **one-shot** alignment: at boot, disarmed and still, the board
reads the magnetometer once, computes `offset = magnetic_heading − game_yaw`, and adds that
offset to yaw from then on — never consulting the mag again. Heading becomes absolute without
reintroducing motor sensitivity.

What this means for a ROS integration:

- **Do not build a Python heading loop.** Translate legs already lock heading on-board, and a
  50 Hz outer loop over a link with a 5 s failsafe is strictly worse than the 500 Hz on-board one.
- **Relative turns (`p4 = 0`) always work.** Prefer them.
- **Absolute turns need `MAG_YAW_REF = 1`** *and* a mag calibration done in the hull with
  electronics powered. Verify against a real compass at four headings before trusting it.
- **It fixes the reference, not the drift.** For a long mission, treat absolute heading as
  slowly degrading. If you need better, that is what the DVL is for — keep it.
- Alignment can be **refused** (bad mag cal, implausible field, vehicle moving); the board then
  falls back to relative yaw and says so over `STATUSTEXT`. A companion that depends on absolute
  headings should **check for that message at startup** rather than assume it succeeded.

### Parameter backup — do this before you reflash anything

The board's parameters survive a firmware upload but are wiped by a `PARAM_DEFAULTS_VER` bump
(which has happened four times) or a chip erase. Bondor's **Parameters → Export** writes a
`.params` file that includes the `CAL_*` sensor calibration — mag/accel/level trim and the
detected motor directions — which lives in separate storage and is otherwise unrecoverable.
Keep an exported file in the `duburi_ws` repo alongside the mission configs; a board that comes
up reporting `"Params reset to build defaults"` is flyable but *not* the vehicle you tuned.

---

## 10. Repository map

| Repo | Contents |
|---|---|
| **srot-control-board** *(this repo)* | ESP32 flight controller + RP2350 thruster co-processor, all firmware docs, this guide |
| **srot-ground-station** | Bondor desktop GCS (Electron) + the ESP32-C3 LoRa bridge firmware |
| **srot-esc-flasher** | ESP32 BLHeli 4-way interface for flashing/configuring the ESCs with Bluejay |

Board-side reference: [`ARCHITECTURE.md`](ARCHITECTURE.md) · [`ALGORITHMS.md`](ALGORITHMS.md) ·
[`HARDWARE.md`](HARDWARE.md) · [`PARAMETERS.md`](PARAMETERS.md) ·
[`docs/THRUSTER_MAP.md`](docs/THRUSTER_MAP.md) · [`docs/T200_PROFILE.md`](docs/T200_PROFILE.md)

Two documents are specifically about this integration and are not linked above:

- [`JETSON_FEEDBACK.md`](JETSON_FEEDBACK.md) — findings sent to the firmware team *by* the companion
  team, ranked by what blocks them. Read it before filing anything.
- [`VISION_API.md`](VISION_API.md) — a **specification, not an implementation**. It proposes moving
  the vision servo loop on-board (bearings in, 500 Hz loop on the board), which would contradict §2
  and §6 of this document. None of it is in the firmware today: there is no `LANDING_TARGET`,
  `VISION_POSITION_ESTIMATE`, `DISTANCE_SENSOR` or `ODOMETRY` handling. Until that changes, **the
  vision servo stays in Python** exactly as §2 says.
