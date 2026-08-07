# PID tuning guide — Hengla / SROT

Written after the 2026-08-07/08 water tests, where several tunes went wrong for reasons that
had nothing to do with the gains. Read the first section before touching a number.

---

## 0. The three mistakes that cost us the most time

**1. Do not copy ArduSub gain values across. The loop structures differ.**

The single worst example: `PSC_POSZ_P = 3.0` on ArduSub maps **metres → m/s**, and feeds
`PSC_VELZ_P` → `PSC_ACCZ_P/I`. A three-stage cascade whose *velocity* loop supplies damping.
Our `DEPTH_P` maps **metres → normalised throttle in one step**. Copying `3.0` made 0.33 m of
error saturate full throttle with no damping anywhere — a guaranteed bounce.

Matching a *number* across two topologies is not matching behaviour. Always ask what the gain
multiplies and what its output feeds.

The rate loops *are* directly comparable (both are rate → normalised torque), which is why the
ArduSub rate values transferred fine and the depth value did not.

**2. Tune in the condition you fly in.** A neutrally buoyant hull at the surface lifts thrusters
clear of the water. The relay then measures a ventilating wallow, autotune converts it into
confident gains, and you undo them by hand afterwards. Submerged, still water, hull free.

**3. Change one thing, then fly it.** Every gain here is live-settable over UDP with no reflash,
so a single-parameter test is cheap. Three of our bad tunes came from changing several at once
and being unable to attribute the result.

---

## 1. Record the flight

Bondor's blackbox is the tool for this and it now runs from any tab (it used to stop the moment
you left **Analyze**, which is exactly when you start flying).

- **Analyze → Record.** A pulsing red `REC` chip stays in the sidebar wherever you navigate.
- Fly. Change modes, move sticks, let it misbehave.
- **Stop → Export.** You get two CSVs sharing one `t = 0`:
  - `bondor-log-*.csv` — 20 Hz samples: mode, armed, roll/pitch/yaw, depth, all 8 RPMs, and
    **every** `NAMED_VALUE_FLOAT` (`DEPTH_CMD/ERR/OUT`, `MIX_*`, `AT_*`, `YAW_REF`, `BARO_P2P`…).
  - `bondor-events-*.csv` — mode changes, arm/disarm, every STATUSTEXT, every command sent.

Open them together. "Which mode was it in when that happened" is the question they exist for.

---

## 2. Order of operations

Never tune an outer loop before the inner one under it is solid.

```
1. motor directions          <- MOTOR_DETECT, in water, armed
2. rate loops   ATC_RAT_*    <- roll, then pitch, then yaw
3. angle loops  ATC_ANG_*_P  <- only once rate is calm
4. depth        DEPTH_*
```

A wobble in STABILIZE with a good rate loop is an **angle** problem. A wobble that is also
present in ACRO is a **rate** problem. That single check saves a lot of guessing.

---

## 3. Rate loops — `ATC_RAT_{RLL,PIT,YAW}_{P,I,D,IMAX,FLTD,FLTT}`

These map body rate → normalised torque. Directly comparable to ArduSub.

Known-good on this hull (from the operator's Pixhawk, ArduSub 4.5.3):

| | P | I | D | IMAX | FLTD | FLTT |
|---|---|---|---|---|---|---|
| roll / pitch | 0.135 | 0.090 | 0.0036 | 0.444 | 30 | 30 |
| yaw | 0.180 | 0.018 | 0 | 0.222 | 5 | 5 |

**Procedure** (roll first; this frame is symmetric so copy to pitch and verify rather than
repeating the search):

1. Set `I = 0`, `D = 0`. They mask what P is doing.
2. Raise `P` ~25 % at a time. Disturb the hull by hand between steps. Stop at the first value
   that gives a **sustained fast oscillation** — that is `P_osc`.
3. Set `P = 0.5 × P_osc`.
4. Raise `D` in small steps to damp the overshoot. Stop at the first sign of **high-frequency
   buzz** and drop back one step — that is D amplifying gyro noise.
5. Raise `I` last, toward roughly `0.5 × P`. It removes steady drift. Too much gives a slow
   wallow that takes seconds to settle.

**Filters.** `FLTD` filters the derivative; `FLTT` filters the *target*. Yaw is filtered far
harder (5 Hz) because it is the noisiest axis with the least useful high-frequency content.
If a tune buzzes at high `D`, lower `FLTD` before lowering `D`.

---

## 4. Angle loops — `ATC_ANG_{RLL,PIT,YAW}_P`

P-only by design. Maps angle error → rate demand, feeding the rate loop above.

Default **6.0** — this is ArduSub's value. (We shipped 4.5 for a while, which is
ArduCOPTER's; the Copter number on a submarine left the attitude loop a third weak.)

Raise for crisper attitude hold, lower if the vehicle bounces past level or hunts. If STABILIZE
wobbles and ACRO does not, **this is the knob**, not the rate gains.

---

## 5. Depth — `DEPTH_{P,I,D}`

⚠ **Single-stage: metres → normalised throttle.** Not ArduSub's cascade. Do not import
`PSC_*` values.

Known-good on this hull, measured in water:

```
DEPTH_P 0.5     1 m of error asks for half throttle
DEPTH_I 0.1     removes steady buoyancy offset
DEPTH_D 0.01    velocity damping
```

`D` is small on purpose: it differentiates a **baro-derived** depth, i.e. a noisy sensor, so it
needs far less gain than the loop's scale suggests. `D = 0.2` bounced badly; `0.01` holds well.

Tuning order: get `P` holding roughly, add `D` until the overshoot stops, then the least `I`
that removes the steady offset. If it overshoots, raise `D`. If it sags below target, raise `P`
before touching `I`.

---

## 6. Autotune

Relay (Åström–Hägglund): drives a square wave, measures the limit cycle's amplitude `a` and
period `Tu`, derives `Ku = 4A/(π·a)`.

**Preconditions — it cannot work otherwise:**
- **Submerged**, still water, hull free to rotate. Not at the surface.
- Armed, props on.
- Export your parameters first.

It reports its measurement per phase, pass or fail:

```
RollRate n12 a0.31 T0.82 ok88%
```

| symptom | meaning | action |
|---|---|---|
| `amplitude too large` | thrashing, usually a thruster leaving the water | go deeper; if still large, lower `A_TORQUE` |
| `amplitude too small` | relay is not exciting the axis | raise `A_TORQUE` |
| `no limit cycle` (low `n`) | phase timed out | raise `PHASE_TIMEOUT` |
| low `ok%` | motion genuinely irregular | calmer/deeper water |

`A_TORQUE` and `PHASE_TIMEOUT` are compile-time in `src/control/autotune.cpp` and need a flash.

Watch `AT_N`, `AT_AMP`, `AT_TU`, `AT_OKPCT` live in Bondor while it runs.

**Autotune holds its own depth** through the rate and angle phases (rev 12+), so you do not
need to pin the vehicle by hand — which would make the relay measure the restraint instead of
the vehicle.

---

## 7. Saving

Wait for the **`"Params saved to flash"`** statustext, not the `COMMAND_ACK`. The ACK fires when
the command is parsed; the NVS write is deferred. Match on `"Params saved"`, not `"saved"` —
the calibration line is emitted first and matching loosely returns on the wrong one.

---

## 8. Related settings that are not gains but feel like them

| param | note |
|---|---|
| `JS_GAIN_DEFAULT` | pilot stick authority. 0.5 on the proven config; at 1.0 everything feels twice as aggressive before any gain is involved. |
| `MOT_SPIN_MIN` | minimum output for a **non-zero** command. At 0.02 the mixer issued commands too small to break stiction and logged `Thruster N STALLED: dshot=146 rpm=0`. 0.15 on this hull. |
| `MOT_SPIN_ARM` | idle spin **on arm**. Deliberately **0** here — nothing turns until commanded. |
| `FRAME_REVERSE` | whole-frame axis flip. For "every axis is backwards"; use `MOT_n_DIRECTION` for individual mis-wired thrusters. Note a successful MOTOR_DETECT makes `FRAME_REVERSE = 1` wrong — run detect, then set it to 0. |
| `PILOT_EXPO` | stick expo. Improves fine centre resolution without touching any loop. |
