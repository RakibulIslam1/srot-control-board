# Water test — CONTROL ONLY. Reference and command sheet

**Firmware:** `SROT_FW_BEHAVIOUR_REV 4` (`srot-control-board` @ `439a454`)
**Companion:** `duburi_ws` branch `srot`, after PR #4
**Scope:** thrusters and depth only. **No vision, no payload actuation, no mission autonomy.**

> This sheet is deliberately narrow. It is the first time this vehicle's depth loop closes in
> water, and that loop has **never run closed anywhere**. Everything here exists to make that one
> unknown observable before it matters.

---

## ⛔ The gate — do not skip, do not reorder

`SROT_MOVE` **auto-enters AUTO**, and AUTO closes the depth loop underneath **every** primitive.
There is no depth-free path through it. So these two checks gate **every move, `move_forward`
included** — not just `set_depth` and `surface`.

**An in-air `move_forward` is NOT partial validation.** At ~0 m the latched target and the
measurement agree, the error is ~0, the loop demands ~0 vertical thrust, and nothing about its
sign, gain or authority has been exercised. A successful dry run is evidence of nothing.

## 🛑 BLOCKER — resolve this BEFORE the vehicle touches water

**On arming, props off, with nothing commanded, the four VERTICAL thrusters spun to
~3000–3180 RPM** while the horizontals idled correctly at 85–170 RPM. A subsequent small
`MANUAL_CONTROL` +Z then took all eight to 0 RPM. Both are unexplained. Observed 2026-08-02;
the vehicle was disarmed immediately and no further armed runs were made.

**This matters MORE because the signs are correct, not less.** The controller sign and the mixer
sign are both verified below — so the demand path is right, which means *something is producing a
heave demand nobody asked for*. On a vehicle whose depth loop has never run closed, unexplained
vertical thrust is exactly the failure this gate exists to catch. In water it would be a vehicle
that dives or surfaces the moment it arms.

**Diagnose it DISARMED first. Do not characterise it by repeating armed runs.**

| # | Read | Why |
|---|---|---|
| 1 | Which mode does it arm into? | If `DEPTH_HOLD`/`AUTO`, `depth::update()` runs and a standing target error drives the verticals exactly like this |
| 2 | Is the depth target latched to *current* depth on the disarmed→armed edge? | A target left at 0 with non-zero measured depth is a permanent error |
| 3 | `DEPTH_ERR` / `DEPTH_OUT` while armed | Separates "the controller demanded it" from "something downstream did". These exist for precisely this. |
| 4 | `MOT_SPIN_ARM` | Probably *not* it — it applies to all eight, and the horizontals idled correctly |
| 5 | Why all eight went to 0 on a +Z | A failsafe cutting thrusters and telemetry dropping are very different faults |

**Clearing this is item zero for tomorrow.** Everything below assumes it is understood.

---

### ✅ Two of the four links in this gate are now closed — bench, disarmed

The **controller sign** has been verified for the first time (2026-08-02), without arming and
without spinning anything, using the `DEPTH_CMD` telemetry added for exactly this purpose:

```
target 0.10 m deeper than the surface
DEPTH_CMD          mean -0.844   (NEGATIVE = descend toward it)   correct
correlation(depth, DEPTH_CMD) = +0.719                            correct
                   (a deeper measurement raises the demand, i.e. reduces descent)
```

**And the MIXER sign, same method** (`mixer::mix()` is a pure function, so it previews exactly):

```
demand throttle = +1.0 (ASCEND)
MIX_VERT  = -1.0    all four verticals commanded negative = push up    correct
MIX_VSGN  = 4       all four agree, so no mixer-matrix typo            correct
```

**What that proves:** the chain is

```
depth error -> heave demand -> MIXER -> motor output -> MOT_n_DIRECTION -> spin
     CLOSED                    CLOSED                    BLOCKED (see above)
```

The inversion `AUDIT.md` R1 warns about is **not** in the controller and **not** in the mixer.
That is the two-negation trap in `THRUSTER_MAP.md` — a `-1` throttle column *plus* "a positive
motor command pushes DOWN" — read correctly.

**What it does NOT prove, and why the checks below still stand:**
- that the demand reaches the **thrusters** with the right sign (mixer columns and
  `MOT_n_DIRECTION` are downstream of this and are a separate opportunity to invert);
- that the loop is **stable** closed, with real water, real mass and real buoyancy.

So: the sign question is answered, the actuation and stability questions are not.

**Bench, props OFF, before the vehicle is near water:**

1. **Depth-hold sign.** Enter `DEPTH_HOLD`. Raise and lower the vehicle by hand.
   The vertical thrusters must push **back toward** the latched depth.
   *Pushing away means the sign is still inverted. STOP. Do not put it in water.*
2. **SURFACE failsafe direction.** At a simulated depth, trip the leak input (`LEAK_EN=1`).
   The demand must be **ascend**. This is the path that once drove the vehicle down.

Both pass → continue. Either fails → stop and report.

---

## Pre-dive checks (read-only, ~5 minutes)

| # | Check | Expected |
|---|---|---|
| 1 | `bringup_check --srot` reports behaviour rev | **≥ 4** |
| 2 | `MAG cal PRESENT` in the boot log | present, `\|off\|` non-zero |
| 3 | `Mag yaw ref: LOCKED - heading is absolute` | present |
| 4 | Heading vs a handheld compass | agree within ~10° |
| 5 | Depth in air | within ±0.1 m of 0 |
| 6 | `SCALED_PRESSURE2` + `WTEMP` arriving | both present, temp plausible |
| 6b | `DEPTH_CMD` at the surface | **negative** (descend toward the 0.10 m preview target) |
| 7 | `BATTERY_STATUS id=1` (thruster pack) | present, matches a multimeter |
| 8 | `LEAK` | dry |
| 9 | `GAIN` | **1.0** — 0.5 means half authority on every manual input |
| 10 | Thruster directions vs `docs/THRUSTER_MAP.md` | each verified individually |

**A missing value is NOT zero.** From rev 4, `WTEMP` and `SCALED_PRESSURE2` are *suppressed* when
the barometer is unhealthy. If they are absent, depth is not trustworthy — do not dive.

---

## Command sheet — control only

Run from `duburi_ws`. Tethered, shallow, **hand on the kill switch**, one leg at a time.

```bash
ros2 launch duburi_manager bringup.launch.py            # expect: rev 4, srot backend
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_mode --target_name DEPTH_HOLD
```

**Order matters — each step gates the next. Stop at the first surprise.**

| # | Command | Watch for |
|---|---|---|
| 1 | `duburi set_depth --depth -0.3` | descends, settles, **holds** without oscillating |
| 2 | *(hand-push it 0.2 m off depth)* | returns to the setpoint, does not run away |
| 3 | `duburi move_forward --duration 2 --gain 40` | moves, **stops on its own**, holds depth throughout |
| 4 | `duburi stop` | **decelerates** — does not coast (rev ≥ 2 brakes on-board) |
| 5 | `duburi yaw_left --degrees 90` | turns ~90°, holds depth |
| 6 | `duburi turn --heading 0` | goes to **absolute** north, not 90° from wherever it started |
| 7 | `duburi surface` | ascends, disarms at the surface |

**Step 6 is new in rev 4** and is the one to watch: absolute `turn` never worked before, because it
requires `MAG_YAW_REF=1` which shipped off. If it turns to a heading relative to where it booted,
the alignment did not lock — check for `LOCKED` in the log.

**Abort at any point:** `duburi stop`, then kill switch. `stop` now brakes on-board; if the hull
coasts, the board is running firmware older than rev 2 and **must not be flown**.

---

## What is deliberately NOT in this test

- **Payload actuation.** Channel roles are firmware parameters (Bondor); the host only addresses
  channels 0–15. Nothing is fired during a control-only test.
- **Vision.** `LANDING_TARGET` ingest is spec'd, not built. Blocked on measured camera FOV.
- **Mission autonomy / DVL distance moves.** One primitive at a time, by hand.

---

## If something goes wrong

| Symptom | Meaning | Action |
|---|---|---|
| Hull coasts on `stop` | firmware < rev 2 | surface, reflash, do not continue |
| Depth runs away | the sign check was skipped or failed | **kill switch**, surface, stop the session |
| Heading drifts between runs | mag alignment not locking | check `Mag yaw ref` in the log |
| `WTEMP`/pressure vanish mid-dive | barometer went unhealthy | surface; depth is no longer trustworthy |
| `Thruster pack link LOST` | ESP-NOW dropout | voltage comp + low-batt failsafe are inert while down |
| Vehicle surfaces unexpectedly | a failsafe fired | read the STATUSTEXT — it names the reason |

Every failsafe announces itself with a reason. **Read the STATUSTEXT before theorising.**
