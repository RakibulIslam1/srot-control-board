# VISION_API — vision-guided control on Hengla

**Status:** specification, not yet implemented. Written by the `duburi_ws` (Mongla) companion
side against firmware **`9384e6b`**, which is the build currently flashed. Every firmware
claim below carries a `file:line` so you can verify rather than trust.

This is the concrete form of the ask in [`JETSON_FEEDBACK.md`](JETSON_FEEDBACK.md) §11.

---

## 0. Why this exists

The vehicle's control authority and its perception live on opposite sides of a USB cable:

| | Jetson Orin Nano | SROT board |
|---|---|---|
| runs | YOLO11 + tracker, 20–30 Hz | the 500 Hz control loop |
| knows | where the target is | where the vehicle is, and owns all 8 thrusters |
| sees attitude at | 10 Hz (`ATTITUDE`, and there is no `SET_MESSAGE_INTERVAL`) | 500 Hz, natively |

Today the *entire* vision control loop runs on the Jetson at 20 Hz, computing thrust from
pixel error and pushing it down as RC-style commands. The board is told nothing about the
target and just actuates. That is the wrong place to close a position loop, and it is the
main reason vision alignment fights the inertia of a 20 kg hull: by the time a correction is
computed from a 20 Hz observation and lands over a serial link, the hull has already moved.

**The fix is to move the loop, not to tune it.** The Jetson should say *"the target is 8°
right and 3° low, and it subtends 12°"* — and the board should decide what the thrusters do,
at 500 Hz, with the IMU in hand.

### What makes this cheap

**The dialect is already vendored.** `src/comms/mavlink_bridge.h:17`:

```c
#include <ardupilotmega/mavlink.h>
```

`ardupilotmega` supersets `common`, so `lib/mavlink/common/mavlink_msg_landing_target.h`
(and `vision_position_estimate`, `distance_sensor`, `odometry`) **already compile into the
binary today** — verified by listing that directory. No custom message, no dialect
regeneration, no XML. Ingest is *one `case`* in `mav_commands::handle()`.

This also dodges a trap we hit from the other direction: `ESC_STATUS` (291) was removed from
upstream `common`, so no pymavlink dialect can decode it and every RPM packet is silently
dropped companion-side. `LANDING_TARGET` (149) is standard and decodes everywhere — we
verified it in our own pymavlink build.

---

## 1. The wire contract

**Message:** `LANDING_TARGET`, id **149**, Jetson → board, streamed at detector rate
(**20–30 Hz** with a TensorRT engine; ~3–4 Hz on a raw `.pt`, which is a Jetson-side problem
we already handle).

**One message per frame, for the ONE currently-selected target.** The board never sees
candidate boxes and never does data association — see §2.

| Field | Type | Carries | Convention |
|---|---|---|---|
| `time_usec` | u64 | capture timestamp (Jetson clock) | **advisory only** — see §1.2 |
| `target_num` | u8 | class index | small interned int, not a label string |
| `frame` | u8 | `MAV_FRAME_BODY_FRD` | bearings are body-frame |
| `angle_x` | f32 | **bearing to target, rad, + = target is RIGHT** | see §1.1 |
| `angle_y` | f32 | **elevation to target, rad, + = target is BELOW** | image Y grows down |
| `distance` | f32 | range (m), **`0` = unknown** | monocular estimate; do not trust for control yet |
| `size_x` | f32 | **angular width of the target, rad** | the standoff measure |
| `size_y` | f32 | **angular height, rad** | |
| `x`,`y`,`z`,`q` | — | **unused, zero** | |
| `type` | u8 | `LANDING_TARGET_TYPE_VISION_OTHER` | |
| `position_valid` | u8 | **`0`** — angle-only, no position solution | |

**Absence of a message means "no target this frame."** There is no explicit "lost" flag: a
detector that sees nothing simply stops sending. Staleness is therefore the *only* loss
signal, which is why §4.1 is mandatory and not an optimisation.

### 1.1 Why angles and not pixels — the load-bearing decision

Our current internal quantity is a normalized pixel error,
`ex = (cx − W/2)/(W/2)`. It is dimensionless but **not physical**: it depends on resolution,
lens and crop. Every gain we have tuned against it is silently a function of the camera, and
swapping a lens invalidates the tune with no warning.

The Jetson converts once, using the camera's known FOV:

```
angle_x = ex     * (HFOV / 2)        # rad
angle_y = ey     * (VFOV / 2)        # rad
size_x  = w_frac * HFOV              # rad
size_y  = h_frac * VFOV              # rad
```

Consequences that matter to you:

- **Your gains have units** — thrust per radian — and are a property of the *vehicle*, not of
  the camera. A camera change, a resolution change or a crop does not retune the board.
- It is what `LANDING_TARGET` already means, so we are using the standard message correctly
  rather than as a convenient container.
- A bearing is directly comparable to a yaw error in radians, so the vision yaw term and the
  existing heading loop speak the same language.

We will publish the FOV we use in `duburi_ws`'s camera config and state it in
`.claude/context/vision-control-split.md`. Treat FOV as **our** responsibility; you never need
image dimensions.

### 1.2 Timestamps — do not trust `time_usec`

`time_usec` is the Jetson's capture clock. The two clocks are not synchronised and we are not
proposing to synchronise them. **Derive freshness from your own `millis()` at receipt**, the
same way every other external input in this firmware does (§4.1).

`time_usec` is still worth sending: it is strictly monotonic per frame, so it is a reliable
**frame-identity** signal if you ever need to distinguish "a new observation" from "the same
observation re-read". We use exactly that distinction companion-side to gate payload firing.

---

## 2. The split: Jetson selects, board controls

There is a fair design where the Jetson streams *all* detections and the board picks. We are
**not** proposing that, for a reason worth stating.

Target selection is not a simple "biggest box" rule. It is: a class filter, a confidence
floor distinct from the detector's own, largest-area acquisition, then a **continuity lock**
that switches to nearest-to-last-centre once locked (so a second torpedo hole cannot steal
the aim at close range), plus a Kalman **coast** that keeps steering the predicted box of the
locked track id for a bounded time through a detection dropout. That is ~150 lines of policy
that is already written, already tested, and already tuned against real pool failures.

It also needs the tracker's state, which lives with the tracker.

Reimplementing it in C++ on the flight controller would be a large amount of subtle,
safety-adjacent code to duplicate — for no gain, because none of it needs the 500 Hz loop.
**Selection is perception. Stabilisation is control.** The cut goes there.

### What stays on the Jetson, permanently

Target selection, tracking and coast, model/class switching, the reactive `detected()` /
`where()` layer, the mission DSL and its search-pattern fallbacks, and **payload fire gating**
(it rides a stable-frame counter that needs frame identity). The board never learns what a
"gate" is.

### What the board owns, exclusively

Everything from a bearing to a thruster: the control law, the mixer, depth and heading, all
failsafes, and the arbitration between vision and the pilot.

---

## 3. The command and the primitive

### 3.1 `MAV_CMD_SROT_VISION` = **31001**

Next after `MAV_CMD_SROT_MOVE` (31000). Same `COMMAND_LONG` shape, same four-terminal ACK
contract.

| Param | Meaning |
|---|---|
| `p1` | **axis bitmask** — `1` yaw, `2` lateral, `4` depth, `8` forward/standoff. Combine. |
| `p2` | **target angular size** (rad) for the standoff axis. `0` = no forward axis. |
| `p3` | **speed cap**, 0..1 — a hard maximum on commanded thrust, not a target speed |
| `p4` | **hold seconds** — after first alignment, keep actively correcting for this long, then finish |
| `p5` | **timeout seconds** — overall budget; `0` → your 60 s default |

Two axes deserve comment:

- **yaw vs lateral are independent on purpose.** A vectored-6DOF hull can null a bearing by
  rotating *or* by strafing. Rotating changes what the camera sees; strafing does not. Close
  in on a small target we usually want lateral only, with heading held — so the mask must
  allow `lateral` without `yaw`.
- **`p4` (hold) is what makes a shot possible.** Reaching alignment is not enough; the hull
  coasts. The hold phase keeps correcting so the vehicle is *stationary on target* when the
  payload fires.

### 3.2 Implement it as a `movement::Type`, not a new `FlightMode`

A new `FlightMode` is **eight** separate touch points — the enum
(`include/state_types.h:33-45`), `isSupportedMode()` (`src/comms/mav_commands.cpp:122-137`),
the `computeDemands()` switch (`src/tasks/task_control_loop.cpp:145-222`), the mode-entry
reset (`:360-394`), the ARMED→DISARMED abort list (`:277-303`), the safety-monitor gate
(`:449-483`), the depth-refusal list (`:317-329`), and the completion→STABILIZE convention
(`:677`). Miss any one and it fails **silently** (a missing `computeDemands` case falls
through `default:` to STABILIZE and simply "does nothing").

Appending to `movement::Type` (`src/control/movement.h:25-27`) instead inherits, for free:

- the four-terminal ACK machine (`mav_stream.cpp:456-530`) that our action server depends on,
- progress reporting, preemption and abort,
- AUTO's existing depth hold and heading lock underneath (`task_control_loop.cpp:188-210`).

The primitive only has to produce a `Demand{fwd, lat, yaw, depth_target}` and let
`attitude::stabilize()`, `depth::update()` and `mixer::mix()` do their jobs. **Do not write a
new attitude or heave controller.**

> ⚠️ **Append, never insert.** `movement::Type` is currently
> `NONE, FWD, BACK, LEFT, RIGHT, TURN, DIVE, STOP, HOLD, STYLE, ARC` (`movement.h:25-27`) and
> the wire mapping is `mv_type = wire + 1` with the bound
> `if (wire < 0 || wire > 9) return MAV_RESULT_DENIED;` (`mav_commands.cpp:285`). Our
> `test_srot_protocol_drift.py` pins this ordering against your header and **will fail** if
> anything is renumbered. Add `VISION` at the end (wire `10`) and raise the bound to `10` in
> the same commit.

If you choose the `SROT_VISION` command form instead of a `SROT_MOVE` subtype, keep the ACK
semantics identical — and note that `MAV_RESULT_TEMPORARILY_REJECTED` is a real fifth outcome
on a mutex miss (`mav_commands.cpp:287`) that we now treat as terminal.

### 3.3 Ingest plumbing

The path already exists; you are adding one `case`:

```
UART0 → mav::poll()                     mavlink_bridge.cpp:104-113
      → mav_commands::handle()          mav_commands.cpp:636-677   ← add MAVLINK_MSG_ID_LANDING_TARGET
      → onLandingTarget(msg)            decode → StateLock(mtx_control) → write vis_* fields
      → readInputs()                    task_control_loop.cpp:60-125
```

**Use the `onManualControl()` pattern verbatim** (`mav_commands.cpp:589-628`): it is already a
20–30 Hz Core-0 writer into `ControlState` under the default 20 ms `StateLock`, while the
500 Hz loop reads with a 2 ms timeout and skips on miss. A vision writer is architecturally
identical — **no new threading pattern, no new mutex.**

Suggested carrier in `ControlState` (`state_types.h:138-182`), following the `mv_*` block
convention:

```cpp
float    vis_bearing  = 0;   // rad, + = right   (angle_x)
float    vis_elev     = 0;   // rad, + = below   (angle_y)
float    vis_size     = 0;   // rad, angular size (sqrt(size_x*size_y) or size per axis)
float    vis_range    = 0;   // m, 0 = unknown
uint32_t vis_stamp_ms = 0;   // millis() AT RECEIPT — 0 = never
uint32_t vis_seq      = 0;   // ++ per accepted message; rising edge = new observation
bool     vis_valid    = false;
```

`vis_stamp_ms` must be `millis()` taken **on Core 0 at receipt**, not derived from
`time_usec` (§1.2). Both cores share the `esp_timer` base, so it is comparable in the loop —
same as `aux_stamp_ms` / `baro_stamp_ms`.

> **NaN guard is mandatory and is easy to miss.** `dispatchCommand()` rejects non-finite
> params (`mav_commands.cpp:248-253`) and `onParamSet()` does too (`:548-551`) — but a new
> message handler is a **third entry point that bypasses both**. `constrain(NaN, -1, 1)`
> returns NaN, and the existing comment at `:240-247` spells out where that ends up:
> `(int16_t)NaN` in `mixer::oneToDshot()` = an arbitrary DShot value on an armed thruster.
> Reject the whole message if any consumed field is non-finite.

---

## 4. Safety requirements — these are the spec, not an appendix

### 4.1 Staleness — the single most important requirement

Every external input in this firmware carries `value` + `stamp_ms` + `valid` and is tested
against a window: `ESPNOW_STALE_MS 2000` (`config.h:562`), `DEPTH_STALE_MS 1500` (`:496`),
`PICO_LINK_TIMEOUT_MS 500` (`:194`), `GCS_FAILSAFE_MS 5000` (`:511`).

The rationale is already written in your own header (`state_types.h:118-120`):

> *"espnow_link returns aux_v even when the link is stale, so without this a lost link would
> leave the last voltage in place forever."*

**That sentence applies verbatim to a bearing**, and worse: a stale bearing does not merely
mislead, it commands thrust toward where the target *used to be*.

Requirements:

- Define `VISION_STALE_MS`, suggested **300–500 ms** (6–15 dropped frames at 20–30 Hz).
- When stale: **zero the translation demands** (`fwd`, `lat`) and the vision yaw term. Hold
  heading and depth. Do **not** keep driving on the last bearing.
- This mirrors the IMU-stale handling, which zeroes the gyro rates rather than letting the
  PIDs wind up against a frozen value (`task_control_loop.cpp:602`).
- **Beware the lock-miss trap:** `LoopIn in` is declared outside the loop
  (`task_control_loop.cpp:229`) and `readInputs()` only overwrites on lock success — so a
  missed lock silently reuses last cycle's values. **Never infer freshness from the value
  changing; always use `vis_stamp_ms`.**
- Prolonged staleness (say > 2 s) should end the primitive with a terminal ACK, not hang.
  Companion-side we treat "no terminal ACK" as the failure mode we cannot recover from
  ([`JETSON_FEEDBACK.md`](JETSON_FEEDBACK.md) §1).

### 4.2 Depth must ramp, or the vehicle disarms itself

`safety_monitor` trips on `ST_DEPTH_DELTA = 2.0 m` between the measured depth and
`depth::target()` (`task_control_loop.cpp:459`, `config.h:449`), and a trip **disarms**
(`:477-478`). A vision depth axis that writes a setpoint 2 m away in one step disarms the
vehicle mid-task.

`movement` already solves this by ramping the *setpoint* at `MOVE_DEPTH_RATE` 0.20 m/s
(`config.h:485`, `movement.cpp:130-133`) so the guard's reference moves with the vehicle. The
vision primitive must ramp the same way.

Also note `depth::setTarget()` has **no clamp** (`depth_control.cpp:20`), so the primitive must
clamp its own target to a sane band ≥ 0 — see [`JETSON_FEEDBACK.md`](JETSON_FEEDBACK.md) §7.

> ⛔ **The depth loop has never run closed.** The sign was inverted until 2026-07-30 and the
> Bar30 was not fitted while it was written (`AUDIT.md:35-73`). The vision depth axis sits
> *downstream* of it. Ship the vision yaw/lateral axes first and gate the depth axis behind
> the two bench checks in `DUBURI_WS_INTEGRATION.md:295-301`.

### 4.3 The forward/standoff axis is ONE-SIDED

Drive forward while the target is smaller than the commanded angular size; go **neutral** at
or past it. **Never reverse.** A two-sided term turns a slightly-close approach into a
reverse kick, and — worse — makes the approach oscillatory right where the vehicle is nearest
the structure it is aiming at. We learned this on the torpedo task; our host-side law has the
same one-sided rule with a small deadband.

### 4.4 `feedforward learn` must be forced false — a real latent bug

`feedforward::apply(..., learn)` is gated on the pilot sticks being centred
(`task_control_loop.cpp:692-693`). A vision mode never drives `sp_*`, so **`learn` would be
permanently `true`** and the CoB auto-trim would learn against vision-commanded effort —
slowly biasing the vehicle's trim toward whatever the last vision task demanded.

This is exactly the reasoning already applied to AUTO, which is deliberately excluded from
trim learning because it commands its own translation. Force `learn = false` in the vision
primitive for the same reason.

### 4.5 Failsafe precedence — say what the operator will see

A vision primitive is silently overridden and silently killed by two higher-authority paths:

| Authority | Effect on a vision task |
|---|---|
| safety monitor (`:449-483`) | **disarms**, `STATUSTEXT "Disarmed: <why>"` |
| leak / low-batt / GCS-loss failsafe (`:489-569`) | forces `SURFACE`, overriding the mode |

Two follow-ons: add the vision mode to the **safety-monitor gate list** (it is an automatic
manoeuvre with no pilot in the loop — exactly the category STUNT/PATTERN were added for), and
to the **ARMED→DISARMED abort list** (`:277-303`), or a latched target survives a panic-disarm
and resumes on re-arm. That failure mode is documented in your own comment at `:281-285`.

Note also that the failsafe path currently does **not** call `movement::abort()`, which is why
a move stranded on `IN_PROGRESS` is our §1 complaint. Please do not inherit that here.

---

## 5. The control law — and why each term exists

You own the law; the 500 Hz loop and the IMU are yours and we do not have them. But these
terms exist because of specific, reproducible pool failures, and we would rather hand you the
scar tissue than have it re-learned in the water. Treat this as the *behaviour* to reproduce,
not the code to transliterate.

Let `e_yaw = vis_bearing` (rad), `e_lat = vis_bearing`, `e_dep = vis_elev`,
`s = vis_size`, `s*` = commanded size, and let `A ∈ [0,1]` be a freshness authority (§5.5).

### 5.1 The base terms

```
yaw_cmd   = clamp(Kp_yaw * e_yaw, ±cap)                  # deadbanded, see 5.3
lat_cmd   = clamp(Kp_lat * e_lat * R(s), ±cap) * A       # see 5.2
depth_rate= clamp(Kp_dep * e_dep * R(s), ±rate) # stepped + frozen in band, see 5.4
fwd_cmd   = (s* - s) > band ? clamp(Kp_fwd * (s* - s), 0, cap) * A : 0    # one-sided, 4.3
```

With the 500 Hz loop and gyro available, a **D term on yaw is now possible and is the single
biggest improvement over our host loop** — we could never run one, because differentiating a
20 Hz bearing over a 10 Hz attitude stream is pure noise. You can damp with measured yaw rate
instead of differentiating the bearing at all, which is strictly better. Reuse `PID`
(`src/control/pid.h`) — it already has conditional-integration anti-windup, a filtered
derivative-on-measurement at `PID_D_FILT_HZ` 20 Hz, and NaN containment.

### 5.2 Range-dependent gain — `R(s)`

**The failure:** a gain tuned at distance oscillates the hull up close.

**Why:** with a bearing error, the *lateral* correction needed for a given angular error
scales with range — so a fixed bearing→thrust gain has an effective loop gain that rises as
you approach. At the torpedo hole the same gain that tracked smoothly at 3 m makes a 20 kg
hull hunt.

**Shape:** scale the lateral (and depth) gain *down* as the target's angular size grows —
full gain below a small angular size, ramping to a floor (~0.3×) once it fills the view.
`s` is a direct proxy for 1/range, so this needs no range estimate.

**Not applied to yaw** — rotating in place has no range dependence.

### 5.3 Yaw stiction floor with taper

**The failure:** the hull stops just outside the deadband and never finishes the last degree,
so a hole-lock stalls forever.

**Why:** T200s have a breakaway threshold. Below it the commanded thrust does not turn the
prop at all, so pure proportional yaw dies before the error does.

**Shape:** impose a minimum yaw command magnitude — but **taper it to zero across an approach
band above the deadband**, not as a hard floor. A hard floor is a relay and limit-cycles the
hull. Gate the floor on the target being *close* (large angular size); far-field it is not
needed and the relay behaviour is worse than the stall.

This is the same fix already applied to `motion_yaw` and the heading lock on our side.

### 5.4 Deadband-frozen, stepped depth

**The failure:** z-wobble — the vehicle bobs while "aligned".

**Why:** the bounding box jitters frame to frame. If the depth *setpoint* tracks that jitter,
the depth PID chases a moving target and never settles.

**Shape:** move the depth setpoint in bounded steps at a **lower rate than the control loop**
(we use 5 Hz), and **freeze it entirely inside the deadband**. Let the depth PID settle
between steps. The step size is the operator's depth-rate knob.

### 5.5 Freshness authority `A` — decay, don't cliff

**The failure:** at low detector FPS the loop re-commands the same bearing many times before
a new observation arrives, over-driving by roughly `Kp · e · T_frame`.

**Shape:** full authority while the observation is fresh (≲100 ms), decaying linearly to zero
by ~400 ms, well before the staleness cutoff of §4.1. **Applies to translation only** — not
to yaw (a rate the loop bleeds naturally) and not to depth (a hold).

The decay is what makes the transition to "lost" continuous rather than a step, and it is why
a brief dropout produces a graceful glide rather than a lurch.

### 5.6 Arrival is not alignment

Being in the deadband for one frame is not "aligned": a hull strafing *through* centre
satisfies the band mid-pass, and if you exit there it coasts straight off target.

Require **N distinct observations** in band (not N loop ticks — at low FPS one lucky frame
re-read 25 times must not qualify; this is what `vis_seq` is for), and optionally require the
error to have stopped changing much, not just to be small. Only then start the hold phase.

---

## 6. Parameters and telemetry

### 6.1 New `VIS_*` parameters — **no `PARAM_DEFAULTS_VER` bump**

Suggested: `VIS_YAW_P`, `VIS_YAW_D`, `VIS_LAT_P`, `VIS_DEP_P`, `VIS_FWD_P`,
`VIS_RGAIN_FLOOR`, `VIS_YAW_MIN`, `VIS_DEADBAND` (rad), `VIS_STALE_MS`, `VIS_DEPTH_RATE`.

Adding parameters does **not** require a defaults bump — `config.h:534-536` is explicit that a
key absent from NVS falls back to its `DEF_*` while existing values are preserved. Please keep
it that way: a bump discards user tuning, and per `PARAMETERS.md:52-57` that has already cost
four pool sessions. Names ≤16 chars (MAVLink), NVS keys ≤15.

Read them live from `g_params` each cycle, as `attitude_control.cpp:36-45` does, so we can
tune from Bondor without a reboot.

### 6.2 Telemetry back to the Jetson

Add as `NAMED_VALUE_FLOAT` (≤10-char names): `VIS_OK` (0/1 fresh), `VIS_AGE` (s since last
observation), `VIS_ERR` (current bearing error, rad), `VIS_SIZE` (angular size). ~32 B each;
4 at 2 Hz costs 256 B/s ≈ 2 % of the link.

> **Hoist it above the param-download blackout.** `mav_stream.cpp:554-558` returns early
> during a param download, killing all telemetry for 10–15 s. `updateMove()` was deliberately
> placed above that guard (`:541-546`) so a GCS opening its Setup tab could not strand a move.
> Vision feedback needs the same treatment, or an operator opening Bondor's Setup tab blinds
> the vision task mid-run.

---

## 7. Budgets — arithmetic shown, please challenge it

**Bandwidth.** Link is 115200 baud ≈ 11 520 B/s each way. `LANDING_TARGET` with extensions is
~60 B payload + 14 B MAVLink v2 framing ≈ 74 B.

```
25 Hz × 74 B = 1 850 B/s = 16 % of the link
```

This is on the **RX** direction, which today carries only `HEARTBEAT` (1 Hz) and
`MANUAL_CONTROL`. TX is separately ~2 660 B/s ≈ 23 %. Both fine.

(For comparison: `ODOMETRY` (331) at 25 Hz would be ~6 100 B/s ≈ 53 % — rejected. If you want
a smaller message later, a packed custom struct at ~20 B payload would be 850 B/s ≈ 7 %.)

**RX buffer caution:** the ESP32 Arduino default UART RX buffer is 256 B. At ~1.9 kB/s a
10 ms `Task_MAVLink` gap accumulates ~19 B, so there is margin — but `Task_MAVLink` is
priority 2 on Core 0 (`config.h:284`), and a burst plus a delayed tick could overflow. If you
see dropped frames, call `setRxBufferSize()` before `begin()`.

**Latency, board-side ingest:**

```
wire time      74 B × 10 bits / 115200   = 6.4 ms
RX poll jitter (TASK_MAVLINK_HZ 100)     ≤ 10  ms
loop pickup    (500 Hz)                  ≤ 2   ms
                                    typical ≈ 10 ms, worst ≈ 18 ms
```

Under one frame period at 25 Hz (40 ms) — and against the status quo, where the loop closes at
20 Hz over a 10 Hz attitude stream, it is a large net win.

---

## 8. Suggested build order

Each stage is independently useful and independently verifiable. Please do not skip to 4.

| Stage | Build | Bench check |
|---|---|---|
| **1** | Decode `LANDING_TARGET` → `vis_*` + `VIS_*` telemetry echo. **No control.** | We stream from the Jetson; you echo `VIS_ERR`/`VIS_AGE` and we confirm the numbers match ours. Validates the contract, the FOV convention and the budgets with zero risk. |
| **2** | Staleness + failsafe wiring (§4.1, §4.5), still no actuation | Unplug the camera mid-stream; `VIS_OK` drops, nothing moves, a terminal ACK arrives. |
| **3** | Yaw axis only, props **off**, vehicle restrained | Move a target across the camera; the correct thrusters are commanded in the correct direction. Verify the deadband and the stiction taper. |
| **4** | Yaw + lateral, in water | Alignment converges without hunting. Tune `VIS_RGAIN_FLOOR` here. |
| **5** | Forward/standoff axis | Approaches to the commanded angular size and **holds**, never reverses. |
| **6** | Depth axis | **Gated on the two mandatory depth-loop bench checks** (§4.2). |
| **7** | Hold phase + alignment criteria (§5.6) | Vehicle is stationary on target for the full hold. This is what makes the torpedo shot possible. |

We can support stage 1 immediately — the uplink is one-directional and the board ignoring an
unknown message is harmless, so we can stream at you before any of this is implemented.

---

## 9. Open questions for you

These are genuinely yours to decide; we have no strong view and would rather you choose from
the 500 Hz side.

1. **`movement::Type` subtype vs a `SROT_VISION` command** — we lean subtype (§3.2) for the
   free ACK machine, but you own the state machine.
2. **Yaw damping source** — measured gyro rate (our recommendation) vs differentiating the
   bearing. We cannot do the former; you can.
3. **Should the board expose a "vision hold" that outlives a single command**, i.e. a mode
   that keeps station on a target until told otherwise? Our missions are currently phrased as
   bounded commands, but a persistent hold would be strictly more useful for a manipulator
   task.
4. **Range.** We can send a monocular `distance`, but we do not trust it for control. If you
   ever want a real range, a known-size target plus angular size is better than our depth net.
5. **Multi-target.** `target_num` is there, but we currently only ever track one. Say if you
   want that reserved differently.

---

## 10. What we commit to on the Jetson side

- Publish `LANDING_TARGET` at detector rate for the selected target, with correct FOV-derived
  angles, and **stop publishing when there is no target** (no stale repeats).
- Keep selection, tracking, coast, class switching and fire gating on our side.
- Keep our timeout ladder consistent with yours and documented in
  `.claude/context/vision-control-split.md`.
- Maintain `test_srot_protocol_drift.py`, which reads **your headers directly** and fails if
  the wire constants, mode ints or `movement::Type` ordering ever diverge from our copy. When
  you add `VISION`, that test is what tells us.
- Not change any shared wire constant without a matching change on your side. See
  [`AGENTS.md`](AGENTS.md).

Questions, corrections and outright rejections of anything above are welcome — especially the
budgets in §7 and the control-law rationale in §5, which come from a 20 Hz host loop and may
not survive contact with a 500 Hz one.
