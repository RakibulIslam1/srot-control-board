# Integrating SROT into `duburi_ws`

**Audience:** an agent or engineer who has never seen this board, tasked with modifying
[`fh1m/duburi_ws`](https://github.com/fh1m/duburi_ws) to fly on SROT instead of a Pixhawk/ArduSub.

**Read first:** [`JETSON_COMMS.md`](JETSON_COMMS.md) — the authoritative wire contract. This document
is the *migration plan*; that one is the *interface*.

---

## 0. TL;DR

- SROT is a **custom flight controller** (ESP32 + RP2350 co-processor) that already does attitude
  stabilisation, depth hold, thrust allocation for a vectored-6DOF frame, arming/failsafes, and
  **complete motion primitives with on-board braking**.
- It speaks **MAVLink 2** over serial. `duburi_ws` already speaks MAVLink 2 via **pymavlink** — so
  this is a *transport-and-verbs* swap, not a rewrite.
- The single custom verb, **`MAV_CMD_SROT_MOVE` (31000)**, has the exact shape of a ROS 2 action:
  immediate `IN_PROGRESS`, ~3 Hz progress 0..99, terminal `ACCEPTED`/100. It maps **1:1 onto
  `duburi_interfaces/Move`**.
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
| `duburi_control/heading_lock.py` | **Delete** | SROT holds heading on-board. The reason this existed — an untrusted in-hull compass — is solved differently: SROT uses a BNO085 game-rotation vector with the magnetometer disabled |
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

| SROT `custom_mode` | Name | duburi_ws equivalent |
|---|---|---|
| 0 | STABILIZE | `STABILIZE` |
| 1 | ACRO | `ACRO` |
| 2 | DEPTH_HOLD | `ALT_HOLD` |
| 9 | SURFACE | `SURFACE` |
| 19 | MANUAL | `MANUAL` |
| 23 | **AUTO** | *(new — the mode `SROT_MOVE` runs in)* |

`SROT_MOVE` auto-switches to AUTO, so you rarely set it explicitly.

⚠️ **Without a depth sensor fitted, `DEPTH_HOLD` and `AUTO` are refused** and the board falls back to
STABILIZE with a `STATUSTEXT`. Handle that rejection rather than assuming the mode took.

### 4.3 The move verb

See `JETSON_COMMS.md` §5 for the full parameter table. The mapping onto `Move.action`:

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
| `MV_STATE` (0 idle,1 cruise,2 brake,3 turn,4 dive,5 style,6 done) | `phase` |
| `MV_PROG` 0..1 | progress |
| terminal `COMMAND_ACK(ACCEPTED, 100)` | `result.success = True` |
| safety abort → disarm + `STATUSTEXT` | `result.success = False`, `message` |

**Two traps:** a re-sent `SROT_MOVE` **re-runs** (start is edge-triggered on `mv_seq`; there is no
idempotency key), and a new move **preempts** the running one (no queue). Your action server must
serialise goals itself.

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
   forward/back/strafe/turn/dive/stop/hold. *Checkpoint: each verb runs and reports progress.*
6. **Delete the redundant loops** (`heading_lock`, `motion_easing`, the ALT_HOLD dance).
   *Checkpoint: missions still complete, with less Python.*
7. **Vision verbs last** — they only need `manual()`, which is already done.

---

## 8. Before the first wet test

- [ ] **Thruster directions verified** against `docs/THRUSTER_MAP.md` using Bondor's motor test. The
      four horizontals are at 45° and each pushes a *different diagonal* — they do not share a
      direction. Fix any reversed one with `MOT_n_DIRECTION`.
- [ ] **Depth sensor fitted.** Without it, `DEPTH_HOLD`/`AUTO` are refused and the SURFACE failsafe
      falls back to a fixed open-loop ascent.
- [ ] **`THR_TRIM_EN = 1` (in water)** if you need repeatable distance — it is **off** by default and
      must stay off with props in air. See `docs/T200_PROFILE.md`.
- [ ] **`RPM_MAX`** set for your pack (3600 @ 16 V for a T200).
- [ ] **`JS_GAIN_DEFAULT` / `GAIN`** — set to 1.0 for programmatic control.
- [ ] GCS heartbeat ≥1 Hz from the Jetson, or the 5 s failsafe surfaces the vehicle.

## 9. Known gaps (as of this writing)

| Gap | Impact |
|---|---|
| ESP-NOW thruster-battery link not implemented | `BATTERY_STATUS` id 1 and the voltage feedforward are inert; use the RPM trim for repeatability |
| No per-thruster current sensing | `CURR` is always 0; no power-based health checks |
| `THR_POLE_PAIRS` is compile-time (7, correct for T200) | A different motor needs a firmware rebuild |
| `STYLE` is always a roll at 90°/s | Not selectable |
| Mission protocol (`MISSION_*`) is handled but minimal | Prefer `SROT_MOVE` sequencing from ROS |

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
