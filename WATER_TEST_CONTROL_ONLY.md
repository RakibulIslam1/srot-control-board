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
