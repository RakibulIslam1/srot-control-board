# Thruster map — which motor pushes which way

The mixer (`src/control/mixer.cpp`) is the ArduSub **Vectored 6DOF** matrix (BlueROV2-Heavy
layout): **M1-M4 horizontal at 45°**, **M5-M8 vertical**. This page is the physical build
reference — mount and verify against it.

> **Common misconception:** the thrusters do **not** all share one "normal thrust direction".
> The four horizontals sit at 45° and each pushes a *different diagonal*. Only the four
> verticals share a direction (all push **down** on a positive command).

## Frame layout (viewed from above, bow at top)

```
              BOW (forward, +X)
                     ^
                     |
     M2 \                     / M1        M1  front-RIGHT horizontal, 45°
        \   [M6]     [M5]    /            M2  front-LEFT  horizontal, 45°
         \                  /             M5  front-RIGHT vertical
          +----------------+              M6  front-LEFT  vertical
          |                |              M7  rear-RIGHT  vertical
   PORT   |     VEHICLE    |  STARBOARD   M8  rear-LEFT   vertical
   (-Y)   |                |    (+Y)      M3  rear-RIGHT  horizontal, 45°
          +----------------+              M4  rear-LEFT   horizontal, 45°
         /                  \
        /   [M8]     [M7]    \            [Mn] = vertical thruster
     M4 /                     \ M3        Mn / = horizontal, angled 45°
                     |
                     v
                  STERN
```

## What a positive command does (this is what Motor Test sends)

Motor Test deliberately forces `dir = +1`, so it always drives a thruster in its **native**
direction, ignoring `MOT_n_DIRECTION`. Hold each button and check against this:

| Motor | Position | A positive command pushes the vehicle… |
|---|---|---|
| **M1** | Front-Right horizontal | **aft + starboard** (toward the rear-right corner) |
| **M2** | Front-Left horizontal | **aft + port** (rear-left corner) |
| **M3** | Rear-Right horizontal | **forward + starboard** (front-right corner) |
| **M4** | Rear-Left horizontal | **forward + port** (front-left corner) |
| **M5** | Front-Right vertical | **DOWN** |
| **M6** | Front-Left vertical | **DOWN** |
| **M7** | Rear-Right vertical | **DOWN** |
| **M8** | Rear-Left vertical | **DOWN** |

Note the horizontals are in **opposing diagonal pairs**: M1/M2 push the vehicle backwards
(outward at the bow), M3/M4 push it forwards. That is what lets the four of them produce
surge, sway and yaw independently.

## Axis sign conventions

| Axis | Positive means |
|---|---|
| forward (surge) | toward the **bow** |
| lateral (sway) | toward **starboard** (right) |
| yaw | nose **right** (clockwise viewed from above) |
| throttle (heave) | **ascend** (toward the surface) |
| roll | **right side down** |
| pitch | **nose up** |

## Per-axis motor outputs

Pure demand on one axis, all others zero (`norm[]`, before direction is applied):

| Demand | M1 | M2 | M3 | M4 | M5 | M6 | M7 | M8 |
|---|---|---|---|---|---|---|---|---|
| forward +1 | −1 | −1 | +1 | +1 | 0 | 0 | 0 | 0 |
| lateral +1 (starboard) | +1 | −1 | +1 | −1 | 0 | 0 | 0 | 0 |
| yaw +1 (nose right) | +1 | −1 | −1 | +1 | 0 | 0 | 0 | 0 |
| throttle +1 (ascend) | 0 | 0 | 0 | 0 | −1 | −1 | −1 | −1 |
| roll +1 (right down) | 0 | 0 | 0 | 0 | +1 | −1 | +1 | −1 |
| pitch +1 (nose up) | 0 | 0 | 0 | 0 | −1 | −1 | +1 | +1 |

## Bring-up procedure

1. **Wire** ESC *n* → Pico **GP(5+n)**, i.e. M1→GP6 … M8→GP13.
2. **Arm** in Bondor → Setup → Motors. (Releasing a test button stops that motor but keeps
   the vehicle armed, so you can walk through all eight in one session.)
3. **Hold each M*n*** at ~15 % and observe which way the vehicle is pushed (or which way the
   prop blows water/air). Compare to the table above.
4. **If a thruster pushes the opposite way**, set that motor's **`MOT_n_DIRECTION` = -1**
   (Bondor → Parameters). Do *not* re-mount or swap motor phases — the parameter exists for
   exactly this.
5. Re-check: with `MOT_n_DIRECTION` applied, a forward stick should move the vehicle forward.

### `Run Motor Detect` — what it actually does
It works out each thruster's direction **automatically, by feeling the vehicle rotate** — it is
not a spin-and-watch. Per motor, in sequence (~1.5 s each, ~12 s total):

1. **Settle** — all motors neutral until the gyro is quiet (|ω| < 0.15 rad/s for 500 ms).
2. **Thrust** — pulse *that one motor* to **30 % for 500 ms**, recording the peak signed gyro.
3. **Detect** — look up the motor's row in the mixer, take the axis it should rotate the
   vehicle around most, and compare the measured sign with the expected sign:
   match → `+1`, opposite → `−1`. If the vehicle barely moved (< 0.05 rad/s) it gives up and
   assumes `+1`.

Motors **1-4 are judged on yaw**; motors **5-8 on roll** (M5/M7 expect one sign, M6/M8 the
other). Because the verticals are distinguished only by roll, detect determines **direction,
not position** — it cannot tell a swapped M5/M7 (both expect the same roll sign) apart.

**Requirements: ARMED, in water, and free to rotate.** On a bench it reads noise and defaults
everything to `+1`. It refuses to start while disarmed, and returns to STABILIZE when done,
reporting e.g. `MotorDetect: + + - + + + - +`.

> ⚠️ It writes `g_state.cal.motor_dir[]` (persisted separately in NVS), **not** the
> `MOT_n_DIRECTION` parameters — and the two **multiply**. A detected `−1` combined with a
> parameter `−1` cancels back to `+1`. **Use one or the other**, not both: for a hand-built
> frame, verifying with Motor Test and setting `MOT_n_DIRECTION` is the more predictable route.
>
> If flipping `MOT_n_DIRECTION` appears to do nothing, a stale detected `−1` is the usual
> reason. Send **`MAV_CMD_PREFLIGHT_STORAGE` with param1 = 2** — it resets the parameters
> *and* clears all learned motor directions back to `+1`, giving you a clean slate.

**Motor Test applies `MOT_n_DIRECTION`**, so the tab shows what the vehicle will really do.
(MOTOR_TUNE deliberately does not — it characterises the motor itself, not the frame.)

## Depth sensor

Depth-dependent behaviour is gated on a live Bar30 (`DEPTH_STALE_MS`). Without one:
- **DEPTH_HOLD and AUTO are refused** (fall back to STABILIZE, with a GCS message).
- **SURFACE failsafe** ascends **open-loop** at `DEPTH_LOST_ASCENT` instead of doing nothing.
- **Autotune skips its depth phase.**
- MANUAL and STABILIZE are unaffected.

Water density is currently hard-coded to fresh water (`src/drivers/bar30.cpp`) — adjust there
for salt water.

## Related
[`ESC_FLASHING.md`](ESC_FLASHING.md) · [`T200_PROFILE.md`](T200_PROFILE.md) ·
[`../PARAMETERS.md`](../PARAMETERS.md)
