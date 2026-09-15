# DIST: a measured-distance move, closed on the board

From the `duburi_ws` (Mongla) companion side, 2026-09-15. Written against `main` at `f1d3ba9`.
**Compiled** (`pio run -e esp32doit-devkit-v1`: RAM 24.5 %, flash 29.8 %, +~1.3 kB flash,
+~0.3 kB RAM over `f1d3ba9`), and **the control and state-machine code was executed**:
`dist_leg.cpp` and `movement.cpp` built natively and driven against a hull model. The
harness lives in `duburi_ws` at `bumblebee-study/rounds/dist_leg/`.

## Why

Every translate verb is timed. A timed leg's distance depends on battery, trim, current and
hull drag. Nothing measures how far the vehicle actually went. Task after task depends on
"go 2 m, then look". The companion now measures velocity over ground: the bottom camera
reads 3.6 % against tape, fused with a thrust model through an invariant EKF. Following the
split we agreed, the **Pi measures and the board closes the loop at 500 Hz**, instead of the
host trying to close it through a 115200 link with 35 ms p2p arrival jitter.

## What

- **`VISION_SPEED_ESTIMATE` (103) ingest** (`mav_commands.cpp`): body velocity x/y, stored
  as a value + receipt-stamp pair, like `sp_stamp_ms`.
  ⛔ This is a **third entry point past the `isfinite()` sweep**, which only covers the
  COMMAND path. So every consumed field is checked in the handler: non-finite values,
  non-positive variances and |v| > 3 m/s are all dropped.
- **`SROT_MOVE` wire type 10 = `Type::DIST`** (enum 11, per `wire + 1`). The bound is now
  `wire > 10`. Parameters: p2 = signed metres, p3 = speed cap, p4 = 0 surge / 1 sway,
  p5 = timeout. p2 is covered by the existing `isfinite()` sweep (COMMAND_LONG path,
  `mav_commands.cpp:290-295`), so a NaN distance is DENIED before `start()`.
- **`control/dist_leg`**: a 1-D Kalman filter [p, v, accel bias] along the leg axis.
  - It predicts every cycle with the BNO `lin_accel` and corrects on each companion velocity,
    weighted by **the sender's own variance** (floored at 0.01 m/s).
  - Control is cascaded: `v_ref = min(v_max, sqrt(2·a·|e|), kp·|e|)`, then a velocity PI with
    feed-forward and conditional-integration anti-windup.
  - Six gains are parameters (`PARAMETERS.md`).
- **Honest terminal results.** The leg reports FAILED (never ACCEPTED at 100 %) when:
  - no fresh velocity exists at start (refused before any thrust);
  - the feed goes stale for more than 400 ms mid-leg (brakes, then fails);
  - it times out with more than 3 cm still to go.

  A refusal inside `start()` never becomes active, so it is latched to `mv_failed_seq`
  directly. Otherwise the companion would wait on IN_PROGRESS for ever.
- **Telemetry:** `MV_REMAIN` (signed metres left, the residual at the end) and `DIST_ACCOK`.
- **Accel fault guard.** If five velocity updates in a row fail a 4σ gate, the estimate has
  left the world. The likeliest cause is a wrong `lin_accel` axis or sign. The filter then:
  - adopts the measured velocity;
  - drops the accelerometer for the rest of the leg;
  - finishes velocity-only, with `DIST_ACCOK = 0` and a STATUSTEXT.

## How it was chosen (read this before tuning)

One harness, scored against **truth**:
- **Hulls:** 9 models (τ 0.6–1.6 s × gain 0.5–1.2 m/s per unit demand).
- **Distances:** 0.5 / 2 / 4 m.
- **Actuator:** this board's measured relay (a demand of 0.005 jumps to 16 % of band).
- **Feeds:** 10 Hz / 30 ms up to 5 Hz / 250 ms, with noise, plus accel bias of −17 mg
  (measured on this board).

| candidate | result |
|---|---|
| handoff to the existing timed brake | **overshoot up to 72 cm**: the brake ends while a high-drag hull is still moving |
| PD | accurate, but no current rejection |
| linear ADRC (Gao bandwidth ESO) | accurate, 2–3× more thrust reversals than PID, no gain |
| super-twisting sliding mode | accurate, **300–486 thrust reversals per stop** on the relay actuator |
| **cascaded PID + KF (this PR)** | **all runs settle; worst 5.6 cm nominal, 6.8 cm in a 0.15 m/s current, 9.6 cm on a 10 m leg against a current** |

Error is set by the velocity feed, not the control law: every family lands within a few cm
of the others on the same feed. That is why this is a PID and not something fancier.

**Two things were built and REMOVED on measurement:**
1. **Delay compensation** (Larsen 1998, innovation against the belief at capture time) made
   things worse. With a 250 ms feed it settled 31/54 runs against 51/54 without it. It also
   cost 2.6 kB of RAM. The cause is a hypothesis (past innovation, present gain), not verified.
2. **A zero-command band near the goal** did nothing: the reversals happen on the approach.

**Guards that carry load, each proven by removing it:**

| removed | what the harness sees |
|---|---|
| the integral | 0.15 m/s current settles 36/54 |
| anti-windup | 10 m leg vs current overshoots **2.7 m** |
| adopt-on-lockout | 40 m runaway |
| bias sign flipped | 8 scenarios breach |
| start refusal, stale fail, timeout fail, axis swap, sign drop | each fails a state-machine test |

## What we need to verify with you, disarmed first

1. **`lin_accel` axes and sign** against body FRD. The guard survives a wrong sign (worst
   33 cm), but it should not have to.
2. `DIST_FF`: demand per m/s. Measure the cruise speed at a known demand in water and set
   it to 1 / that speed.
3. Wet run: 2 m legs against a tape, reading `MV_REMAIN` and the tape side by side.

## Not in this PR

On-board de-rotation of the `LANDING_TARGET` bearing. The companion now stamps the capture
instant on **your** clock (`time_usec`, flagged `z = 1`), so it is ready when #18 lands.
It is a separate PR so this one stays reviewable.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

## How this sits with the open PRs

- **#11 (OPTICAL_FLOW_RAD ingest) and #22 (thruster as velocity sensor)** are other
  velocity sources. This PR takes velocity from `VISION_SPEED_ESTIMATE` (103) only,
  weighted by its own variance. If either lands, it feeds the same `correct()` call; no
  second filter is needed.
- **#15 (AUTO never exits)**: DIST enters AUTO like every other move, so it inherits that
  defect and that fix. Nothing here depends on the fix.
- **#18 (LANDING_TARGET stage 1)**: independent. On-board de-rotation of the bearing is
  a separate PR that builds on #18.
- **#8 (false complete)**: DIST reports FAILED, not ACCEPTED, when it stops short.
