# Hengla — firmware audit

Two rounds. **Round 2 is below and includes the most serious defect found in either** — the depth
PID's sign was inverted, which made the leak / low-battery / GCS-loss failsafe drive the vehicle
*down* instead of surfacing it. Round 1 (further down) covered the comms and control paths I read
by hand.

---

# Round 2 (2026-07-30) — full-tree sweep

Three independent audit passes over areas round 1 had not read closely: the comms/protocol layer,
the control/task layer, and the drivers plus the three secondary MCUs. **13 further findings.**
Every one below was re-verified against the source before being acted on.

| # | Finding | Severity | Status |
|---|---|---|---|
| R1 | **Depth PID sign inverted — the SURFACE failsafe dived instead of surfacing** | **Critical** | Fixed |
| R2 | Autotune's depth relay had the same inversion, independently written | High | Fixed |
| R3 | STUNT/PATTERN had no `abort()`; resumed mid-manoeuvre on re-arm | High | Fixed |
| R4 | PATTERN steps had no timeout and no safety-monitor coverage | High | Fixed |
| R5 | NaN in a `COMMAND_LONG` param reached `(int16_t)NaN` on an armed thruster | High | Fixed |
| R6 | `MANUAL_CONTROL` axes unclamped — a bad packet = full-authority burst | High | Fixed |
| R7 | `CAL_*` shadow cache could report a 0.0 scale and corrupt a calibration | High | Fixed |
| R8 | Safety monitor's rate and depth guards silently passed NaN | Medium | Fixed |
| R9 | Preempting a move never resolved the displaced command's action | Medium | Fixed |
| R10 | `params::set()` reported success on a failed NVS write | Medium | Fixed |
| R11 | `PILOT_YAW_RATE` / `RPM_MAX` used as divisors with no zero guard | Low | Fixed |
| R12 | `ACCELCAL_VEHICLE_POS` ACKed success when nothing changed | Low | Fixed |
| R13 | `PREFLIGHT_STORAGE` burned an NVS write on a read request | Low | Fixed |

Plus three that need a hardware or two-sided change and are **documented, not fixed** — see
*Deferred* at the end of this round.

## R1 — Depth PID sign inverted (critical)

`src/control/depth_control.cpp`. The loop ran `s_pid.update(s_target, meas_depth, dt)`, giving
`err = target − measured`. Depth is **positive down**, and positive heave **ascends**. So a target
*deeper* than the current depth produced a *positive* error and therefore an *ascend* command — the
exact opposite of what was wanted, on every axis of the depth loop.

The old comment is where the slip happened: *"positive error → push down (negative heave, since the
mixer's throttle column is −1)"*. The `−1` column combined with "a positive motor command pushes the
vehicle DOWN" means positive throttle **ascends**. Two negatives, read as one.

The convention is unambiguous and corroborated four independent ways: `docs/THRUSTER_MAP.md`'s axis
table ("throttle (heave): positive means ascend") and its mixer row (`throttle +1 (ascend)` →
M5–M8 = `−1`), `DEPTH_LOST_ASCENT` being applied as a **positive** value to surface without a
sensor, and MANUAL mode passing the pilot's throttle straight through (`MANUAL_CONTROL` z > 500 = up).

**Why this is the worst finding.** `DEPTH_HOLD` would have been divergent rather than merely
mistuned — any perturbation amplified until the output saturated. But the real hazard is the
**SURFACE failsafe**: leak, low thruster battery and GCS-loss all force `SURFACE`, which drives
`depth::setTarget(0)` through this same loop. Verified numerically — at 3 m depth the old code
commanded **DOWN**. The one mechanism meant to save a leaking or dying vehicle would have sunk it.

**Fix.** Run the PID in **altitude** (`−depth`), the frame its output already lives in:
`s_pid.update(-s_target, -meas_depth, dt)`.

Negating the inputs rather than the output is a *readability* choice, not a correctness one. My
first write-up claimed flipping the sign outside the call would desynchronise the conditional
anti-windup and the derivative-on-measurement; review showed that is **not true for this PID** —
both are linear, and negating everything swaps the `push_high`/`push_low` tests symmetrically, so
the integrate-or-hold decision is unchanged. The two forms are provably identical from zero state.
Altitude is still the better choice, because error, output, integrator and derivative then share one
frame — which is exactly the confusion that caused the bug — but the stronger claim was wrong and is
corrected here rather than left standing.

**Why it was never caught in the water:** the Bar30 had not been fitted, so the loop had never once
run closed. Nothing about the vehicle's observed behaviour could have revealed it.

Proven with a numeric check of the whole chain (setpoint → PID → mixer → physical direction) across
four cases including the failsafe; old code wrong on 3 of 4, new code correct on all 4.

## R2 — Autotune's depth relay, same inversion

`src/control/autotune.cpp`. `sig = depth − s_ref`; the relay did `(sig > 0) ? -A_HEAVE : A_HEAVE`.
`sig > 0` means deeper than the reference, so it needs to **ascend** — the code drove further away.
A relay autotune needs *negative* feedback to produce the sustained limit cycle it measures; this
gave monotonic divergence, so the depth phase either measured nothing usable or ran until the
`ST_DEPTH_DELTA` guard disarmed it. Independently written from R1, same mistake.

## R3 — STUNT/PATTERN resumed mid-manoeuvre on re-arm

`src/control/stunt.{h,cpp}`, `pattern.{h,cpp}`, `src/tasks/task_control_loop.cpp`. The
ARMED→DISARMED edge aborts `motor_tune`, `autotune` and `movement` — its comment even explains why
("a tune/move interrupted by a disarm silently continued and then drove the motors again on the next
arm"). Neither `stunt` nor `pattern` had an `abort()` to call. Disarming does not change the GCS mode
selector, so the mode-entry reset never re-fired either.

Panic-disarm 200° into a 360° spin and the state machine kept `s_done_deg = 200`; on the next arm —
for any reason, possibly unrelated — the thrusters completed the remaining 160° immediately, with no
command issued. Both now have `abort()`, wired into the disarm edge and the safety-abort path.

## R4 — PATTERN could drive full authority indefinitely

`src/control/pattern.cpp`. Only the `TASK` step had a timeout. `TURN_360`, `HEADROOM` and `RETURN`
advanced *only* on their goal condition (turn accumulated, depth converged, heading converged) — so
any condition that never converged (a current holding the vehicle off depth, a stuck sensor, an
unreachable headroom) wedged the step forever at full authority.

Compounding it: the safety monitor only ran for `at_active || mt_active || auto_armed`, so **STUNT
and PATTERN — the two modes that drive full authority with no pilot in the loop — were the only
automatic modes with no guard at all.** No depth-runaway, no rate, no tumble check.

Fixed both ways: a 30 s per-step ceiling (generous — a backstop, not a schedule), and STUNT/PATTERN
now get the same safety-monitor coverage as a tune or a move. STUNT is exempted from the *angle*
guard only (a 360° rotation is its purpose), exactly as STYLE already was.

## R5 — NaN in a command parameter reached the ESCs

`src/comms/mav_commands.cpp`. Arduino's `constrain()` is built from `<` and `>`, both false against
NaN, so `constrain(NaN, -1, 1)` returns **NaN unchanged**. A NaN in `DO_MOTOR_TEST`'s throttle
therefore survived into `mixer::oneToDshot()`, where `n >= 0.0f` is false for NaN — taking the
*reverse* band — and ended at `(int16_t)NaN`, an undefined cast, on an already-armed thruster.

Fixed at the single choke point: `dispatchCommand()` now rejects any non-finite parameter for
**every** command with `MAV_RESULT_DENIED`, which closes the whole class rather than one instance.
Safe as a blanket rule because no command this firmware implements uses NaN as a "leave unchanged"
sentinel (some ArduPilot NAV/REPOSITION commands do; none are implemented here). This also protects
`SROT_MOVE`'s depth target, which feeds `depth::target()` and hence the runaway guard.

## R6 — `MANUAL_CONTROL` axes were unclamped

`src/comms/mav_commands.cpp`. `x/y/z/r` are raw `int16_t` on the wire and MAVLink does not enforce
the ±1000 convention, so a glitching bridge could send 32767 → `sp_forward = 32.7`, which
`PILOT_EXPO` then cubes (≈35000) before the mixer's uniform scale-down. The per-motor clamp bounds
the *output* but not the *behaviour*: a small intended nudge became an instant full-authority burst.
Now clamped at entry, where `ControlState` documents the −1..1 contract.

## R7 — The `CAL_*` shadow cache could corrupt a calibration

`src/comms/params.cpp` — a defect in code added earlier this session. The shadow cache exists so a
`mtx_cal` lock miss serves the last known value instead of 0 (its own comment: *"returning 0 there
would… destroy the real one"*). But it was zero-initialised and only ever populated on a successful
read — so on the **first** post-boot param download, a lock miss on a *scale* row (`CAL_ACC_S*`,
`CAL_MAG_S*`, real default 1.0) reported **0.0**. Exported and later imported, a zero scale silently
destroys the accel/mag calibration. Now seeded from each row's declared default in `buildTable()`,
so the `def` field on those rows finally does something instead of being decorative.

## R8 — The safety monitor's own guards passed NaN

`src/control/safety_monitor.cpp`. `roll`/`pitch` had an explicit `isfinite` fail-fast, but the rate
check did not — and a comparison against NaN is false, so `if (rate > limit) fail` **silently
passed** a NaN gyro. Worse, the depth check was written `if (isfinite(depth) && ...)`, making a NaN
depth skip the runaway guard entirely and report *safe*. Both now fail closed, which is the only
defensible default for a component whose entire job is catching a broken state.

Note this adds two disarm paths that did not exist. Consistent with the existing "attitude NaN"
behaviour, and it only applies to automatic manoeuvres — the monitor never runs in MANUAL or
STABILIZE, so a hand-flying pilot is unaffected.

## R9 — Preempting a move stranded the displaced command

`src/comms/mav_stream.cpp`. `updateMove()` tracks one sequence; a preempting `SROT_MOVE` reassigned
it, so the displaced command never got a terminal ACK — its ROS action hung. Same failure mode as
round 1's B1, reached through the *documented* preemption behaviour rather than through timing. The
tracked sender is now cached (`ControlState`'s `mv_src_*` is overwritten the moment the new command
lands, so the address of the command being cancelled is otherwise already gone) and the old
sequence is resolved `MAV_RESULT_CANCELLED` before the new one is adopted.

## R10 — `params::set()` reported success on a failed NVS write

`src/comms/params.cpp`. `putFloat`'s return was discarded, unlike `saveAll()` which checks it. On a
full or worn namespace the value went live in RAM, the GCS saw `PARAM_VALUE` echoed back as
accepted, and it vanished on the next boot with nothing said. "Applied" and "persisted" are now
reported separately — the return still means applied (correct: the value *is* live), and a new
optional out-param carries persistence, with a `STATUSTEXT` warning naming the parameter.

## R11–R13 — Smaller items

- **`PILOT_YAW_RATE` and `RPM_MAX` as unguarded divisors.** Both are GCS-editable with no lower
  clamp. At 0, `movement.cpp` produced Inf (survivable) or NaN, which `stabilize()` reads as a
  centred stick — so a commanded TURN/ARC silently never turned. `motor_tune.cpp` collapsed to
  `Ku = 0` and wrote an all-zero PI tune. Both now guarded the way `attitude_control.cpp` and
  `thrust_trim.cpp` already guard the same parameters.
- **`ACCELCAL_VEHICLE_POS` always ACKed `ACCEPTED`**, including for an out-of-range face or a missed
  lock, so the GCS's calibration wizard advanced while the vehicle did not and the two desynced.
- **`PREFLIGHT_STORAGE` set `cal.persist_pending` for any `param1 != 2`**, including 0 ("read") —
  an NVS write cycle for a command that asked for nothing.

## Deferred — need a hardware or two-sided change

- **The Pico's hardware e-stop fails permissive.** `PIN_ESTOP` is `INPUT_PULLDOWN` with active-HIGH
  assertion, so a broken wire, unseated connector or dead ESP32 all read as "run permitted". That
  inverts standard e-stop practice. It is redundant with `TL_FLAG_ESTOP` over the CRC-checked UART
  and with the 150 ms link timeout, so the vehicle is not undefended — but the *one* scenario a
  discrete stop line exists for (a wedged or rebooting ESP32) is the scenario it does not cover.
  **Fixing it properly means inverting the signal in hardware** (pull-up, active-LOW), so it is
  flagged rather than changed unilaterally.
- **LoRa mission-waypoint upload has no integrity check.** `src/drivers/lora_mission.cpp` accepts
  and ACKs a 14-byte chunk with no CRC, and the SX127x's PHY CRC is never enabled — it is the only
  wire protocol in the tree without corruption detection (both other shared protocols are
  CRC16-verified). An RF bit-flip in a float byte is accepted, stored and ACKed, so the sender never
  retries. Fixing it changes the wire format and needs the ground-station end updated in lockstep;
  worth doing, but not as an unannounced change. Enabling `LoRa.enableCrc()` on both ends would be
  a cheap partial mitigation.
- **`SECOND_KILL_ACTIVE_LOW` only inverts the reported bit, not the MOSFET drive.** Someone using it
  to "fix inverted wiring" would correct the telemetry while leaving the knob's ON/OFF zones
  physically backwards. Harmless for thrust (the flag is indication-only by design) but misleading.

## Verified sound (recorded so a future pass need not re-tread)

The DShot 3D band mapping on both the ESP32 and Pico sides, including the bidir library's ±47 offset
which cancels correctly; eRPM→RPM conversion and pole pairs (7 is right for a 14-pole T200); the
Pico's four failsafe layers, with the link timeout being staleness-based and therefore still valid
if core1 hangs; struct packing and CRC coverage on the thruster-link, ESP-NOW and LoRa-telemetry
protocols, compared field-by-field at both ends; the mission `MISSION_*` state machine's bounds
checks; the `PARAM_REQUEST_LIST` streamer's re-entry refusal and 30 s hard stop; the LoRa tap ring
buffer; all 16-char param-name copies; servo/relay/motor index bounds checks; the AS5600 error path
and hysteresis on the 2nd board (including the wrap-through-zero branch, checked algebraically); and
lock discipline throughout — no `StateLock` is held across a blocking call, and the historical
"state busy" fix in `task_sensor_read.cpp` is intact.

---

# Round 1 (2026-07-30)

A read-through of the whole control/comms/driver stack looking for defects, with
`src/esp32_4way/` (the ESC flasher) excluded by request. **11 findings: 9 real defects, 2
categories of misleading-but-harmless code.** All 9 are fixed; the evidence and reasoning for
each is recorded below so the next reader can check the work rather than trust it.

Scope read: `src/comms/`, `src/control/`, `src/drivers/`, `src/tasks/`, `src/pico/`,
`include/config.h`, `shared/`.

| # | Finding | Severity | Status |
|---|---|---|---|
| B1 | Move ACK lost if the move completes during a param download | **High** — hangs a companion | Fixed |
| B2 | `RPM_LOOP` only half-wired; runtime setting did nothing | **High** — feature dead | Fixed |
| B3 | AUTO + `STYLE` disarms itself every time | **High** — unusable primitive | Fixed |
| B4 | No PID anti-windup on output saturation | Medium | Fixed |
| B5 | No PID derivative filter | Medium | Fixed |
| B6 | One NaN permanently poisons a PID integrator | Medium | Fixed |
| B7 | CoB auto-trim is a one-way ramp, not the documented bleed | Medium (latent) | Fixed |
| B8 | AUTO gets no drag / cross-coupling feedforward | Low (latent) | Fixed |
| B9 | AUTO depth-runaway guard is a permanent no-op | Medium | Fixed |
| B10 | Dead ArduSub spoof macros that the config still documented | Cosmetic | Removed |
| B11 | Comments contradicting the code | Cosmetic | Corrected |
| B12 | Low-battery failsafe had no debounce against pack sag | **High** — spurious mid-dive surface | Fixed |

---

## B1 — A move completing during a parameter download never got its terminal ACK

**High.** Reproducible, and easy to hit: opening Bondor's Setup tab triggers a parameter
download.

`mav_stream::update()` early-returned before `updateMove()` whenever
`mav_commands::paramDownloadActive()`. Separately, `updateMove()` only began tracking a
command while `mv_active` was true. Put those together and a move that started *and* finished
inside a download window was never observed: by the time `updateMove()` ran again `mv_active`
was false, so `s_seq` stayed 0 and the function returned immediately.

The command handler had already replied `MAV_RESULT_IN_PROGRESS` synchronously, so the
companion was left waiting on a terminal result **that would never be sent**. A ROS 2 action
would hang indefinitely.

**Fix.** `updateMove()` now runs *above* the download throttle — it is one `COMMAND_ACK` at
3 Hz (~40 bytes), so the bandwidth reasoning that justified suppressing everything else never
applied to it. Tracking now latches on an `mv_seq` change alone.

Two things that fell out of doing this properly:

- Dropping the `mv_active` requirement naively would have made the *first* cycle of every move
  look complete (`!mv_active` is trivially true before the control loop has run). A
  `s_seen_active` latch prevents that: `!mv_active` only means "done" once the move has been
  observed running. `mv_done_seq` is latched by the control loop, so a completion missed
  entirely is still recoverable however late the ACK path gets there.
- A pre-existing hang came to light next door: the command handler forces `AUTO`, but the
  control loop **refuses** `AUTO` with no depth sensor and falls back to `STABILIZE`. The move
  then never starts and nothing would ever resolve it. There is now an explicit
  `MAV_RESULT_FAILED` after 500 ms in that case, instead of `IN_PROGRESS` for ever.

`src/comms/mav_stream.cpp`

## B2 — `RPM_LOOP` was only half-wired

**High.** `task_dshot_rmt.cpp` chose between raw-DShot and target-RPM frames using the
**compile-time** `THR_RPM_CLOSED_LOOP` (`0`), while the config frame pushed the **runtime**
`g_params.rpm_loop` to the Pico as its loop-enable. One feature, two switches, disagreeing:
setting `RPM_LOOP = 1` told the Pico to close a loop while this side kept sending raw DShot
values *tagged raw*. The runtime parameter silently did nothing on the ESP32.

This matters because choosing open- vs closed-loop at runtime was an explicit requirement, not
a nicety — a compile-time fallback is exactly what was asked against.

**Fix.** Both sides now read `g_params.rpm_loop`. `THR_RPM_CLOSED_LOOP` is demoted to the
parameter's boot default (`DEF_RPM_LOOP` is defined from it), so there is one switch and the
`#define` can no longer drift away from the param.

`src/tasks/task_dshot_rmt.cpp`, `include/config.h`

## B3 — AUTO + `STYLE` disarmed itself

**High.** `safety_monitor::ok()` enforces `ST_ANGLE_MAX` (70°) whenever an AUTO move is armed.
`SROT_MOVE` type `STYLE` commands a **360° roll**. Roll therefore passed 70° within a fraction
of a turn and the vehicle disarmed with `Disarmed: angle limit` — every single time. The
primitive could never have worked.

**Fix.** `safety_monitor::ok()` takes `allow_inverted`, which suppresses **only** the
tumbling-angle check. The rate, depth-runaway, over-RPM and NaN guards all still apply — those
are what actually distinguish a commanded roll (rate-limited by the spin controller) from a
real tumble.

`src/control/safety_monitor.{h,cpp}`, `src/tasks/task_control_loop.cpp`

## B4–B6 — PID: anti-windup, derivative filter, NaN containment

`src/control/pid.h` rewritten. All three verified with 18 host-side assertions against a
faithful transcription of the old and new algorithms.

**B4 — no anti-windup.** `_integ` was clamped to ±`_imax`, but integration continued while the
*output* sat pinned at ±`_outmax`. The mixer scales all thrusters down uniformly on
saturation, so saturation is routine here, not exotic.

Measured, with the real roll-rate gains (`kp 0.135, ki 0.09, imax 0.5`): after two seconds of
saturated demand the old integrator sits exactly at its 0.5 clamp. When the error then goes to
zero it **cannot decay** — an integrator only moves when there is an error — so it leaves a
**permanent 4.5%-of-full-torque bias on that axis**. New code: 0.000%. Conditional integration
(ArduPilot `AC_PID`'s approach) simply does not integrate further into a rail it is already
against; it resumes normally the moment the output is back inside its limits, which the tests
confirm in both directions.

> Worth stating plainly: the textbook "less step overshoot" demonstration does **not** show up
> here, because `ki · imax` caps the integrator's authority at 4.5% and a step response is
> dominated by P. The real, measurable win is the eliminated persistent bias. A plant contrived
> to produce an overshoot number would have been manufactured evidence, so it was left out.

**B5 — no derivative filter.** `-(meas − prev)/dt` at 500 Hz multiplies gyro noise by 500. That
is why D could never be raised: the term was mostly noise and buzzed the motors long before it
damped anything. There is now a one-pole low-pass at `PID_D_FILT_HZ` (20 Hz, `config.h`).
Measured on stationary noise the D output drops to well under a third of the unfiltered
magnitude, while a genuine constant-rate ramp still yields the correct derivative to within
1%. On smooth, non-saturating input old and new agree to <2%, so the **existing tune is
preserved**.

The cutoff is compile-time deliberately: the parameter table was to be left untouched, so
changing it needs a rebuild.

**B6 — NaN poisoned an integrator permanently.** `constrain(NaN, lo, hi)` returns NaN, so a
single non-finite sample made `_integ` NaN and every subsequent `constrain()` passed it
straight through — nothing short of `reset()` recovered. Attitude and gyro are guarded
upstream, but **depth is not**: a bad Bar30 read reached `depth::update()` directly. Non-finite
inputs (and `dt <= 0`) are now rejected without touching stored state, and a poisoned
integrator self-heals. Tests confirm the old code stays NaN for ever after one bad sample and
the new code does not.

## B7 — The CoB auto-trim was a one-way ramp

**Medium, latent** (`TRIM_EN` defaults 0).

The comment claimed it "slowly bleeds the steady rate-PID integrator into a persistent
feedforward trim, so the integrator returns toward zero". It did not: it only *added*
`leak · rateIntegral()` to the trim and never touched the integrator. An integrator only decays
when the error reverses, so the same charge was transferred over and over. At
`TRIM_LEAK = 0.002` × 500 Hz the trim gains ~1.0/s and pins at its ±`TRIM_MAX` (0.30) clamp in
well under a second — after which the PID is fighting its own trim.

**Fix.** `PID::bleedIntegral()`, exposed as `attitude::bleedRateIntegral()` and
`depth::bleedIntegral()`, removes exactly the amount the clamp actually accepted. Total effort
is now conserved, which is what the design always claimed.

`src/control/pid.h`, `attitude_control.{h,cpp}`, `depth_control.{h,cpp}`, `feedforward.cpp`

## B8 — AUTO got no drag or cross-coupling feedforward

**Low, latent** (all model gains default 0). `feedforward::apply()`'s `stabilized` set excluded
`AUTO`, although AUTO runs the *same* `attitude::stabilize()` cascade — so the model-based
terms silently skipped the one mode a companion computer uses. AUTO is now in `stabilized`, and
deliberately **not** in `angle_hold`: it commands its own translation, so centred pilot sticks
are not evidence of a steady trim state and it must not learn.

## B9 — The AUTO depth-runaway guard was a permanent no-op

**Medium.** For AUTO the guard was passed `depth0 = in.depth` — the *current* depth — so
`|depth − depth0|` was always ~0 and the check could never fire. It was deliberate and
commented (a commanded dive is intended, not a runaway), but the consequence is that AUTO had
**no depth protection of any kind**: an overshooting dive, or a depth sensor drifting while the
vehicle held station, was never caught.

**Fix.** Guard against `depth::target()` — the active setpoint — so `ST_DEPTH_DELTA` now means
"the vehicle is this far from where it was told to be". A tune still checks against where it
started, which is correct for a manoeuvre that must not change depth at all.

## B12 — The low-battery failsafe had no debounce

**High**, and found while bringing the 2nd board's ESP-NOW voltage link up — i.e. found at
exactly the moment it became reachable.

The branch was:

```cpp
if (fs_bat_enable > 0 && in.thr_volts > 1.0f && in.thr_volts < fs_bat_voltage) → SURFACE
```

evaluated every 500 Hz cycle against the **raw** published voltage, not the mixer's 0.5 Hz
filtered copy. A single low sample switched the vehicle to `SURFACE`.

A 4S LiPo driving eight T200s sags well over a volt under load. With a 15.1 V resting pack and a
13.6 V threshold, a full-throttle burst later in the dive is entirely capable of dipping below the
line for a moment — surfacing the vehicle mid-manoeuvre on a pack that is completely healthy. The
feature would have looked broken on its first real dive.

This had never fired because `in.thr_volts` was always 0 (no sender existed, so the `> 1.0f` guard
made the whole branch inert). It went live for the first time the moment the 2nd board started
broadcasting — so the defect and its trigger arrived together.

**Fix.** The voltage must stay below `FS_BAT_VOLTAGE` continuously for `FS_BAT_HOLD_MS` (3 s,
`config.h`); any sample above resets the timer. The 2nd board already averages its ADC over 500 ms
and sends at 4 Hz, so 3 s is ~12 consecutive low readings — far too long for a transient, still
early enough that a genuinely flat pack surfaces with reserve.

Leak and GCS-loss are deliberately **not** debounced: neither is an analogue measurement and
neither has a sag equivalent.

The `STATUSTEXT` now also quotes the measured voltage
(`"Failsafe: surfacing (low thruster battery 13.4 V)"`). Without a number, a spurious trip is
undiagnosable after the fact — you cannot distinguish a flat pack from a load sag from a
miscalibrated divider.

`src/tasks/task_control_loop.cpp`, `include/config.h`

## B10 — Dead ArduSub spoof, still documented as live

`APM_COMPAT_VER_MAJOR/MINOR/PATCH/PACKED` and `APM_COMPAT_BANNER` were referenced **nowhere**
in the firmware — the heartbeat has reported `MAV_AUTOPILOT_GENERIC` for some time. Yet
`config.h` §0 still told the reader "to QGC we advertise ArduSub 4.1.0 (see the `APM_COMPAT_*`
macros below)". A reader trying to understand the identity story would have been actively
misled. Macros deleted, §0 rewritten to describe what the code really does.

## B11 — Comments contradicting the code

- `bno085.cpp`: "Attitude + rate run FAST (200 Hz) — matched to the 200 Hz control loop." The
  reports are 400 Hz and the loop is 500 Hz. The same block warned that 400 Hz needs the INT
  pin — which *is* wired — so the warning read as if unheeded.
- `task_sensor_read.cpp`: "Decimation counters (200 Hz base)" — the base is 500 Hz. The
  arithmetic was right; only the comment was wrong.
- `mav_stream.cpp`: referred to a **"Mongla"** move-action result — a project name from
  somewhere else entirely.
- `config.h` / `main.cpp`: called the firmware an "ArduSub-clone".
- `DEF_ST_RPM_MAX`'s comment justified 4500 as ">= THR_MAX_RPM" (4000) while the live
  `RPM_MAX` default is 3600 — harmless, but the stated reason no longer matched the numbers.

All corrected.

---

## Advisory — noted, not changed

- **`classifyType()` in `params.cpp` is dead.** Its own comment says so: `sendParam()`
  advertises `REAL32` for every parameter. `params.h`/`params.cpp` were deliberately left
  untouched in this pass (the parameter table had to stay byte-identical so existing `.params`
  backups keep working), so it stays. Safe to delete whenever the table is next opened.
- **No `REQUEST_DATA_STREAM` (66) or `SET_MESSAGE_INTERVAL` (511) handling.** Telemetry rates
  are fixed in `mav_stream::update()`. Harmless today — a GCS asking for a change is simply
  refused — but `SET_MESSAGE_INTERVAL` is the standard way for a companion to turn rates *down*
  over a LoRa link, and it is cheap to add. See `ROADMAP.md`.
- **`THR_POLE_PAIRS` is compile-time.** Correct for a T200 (7); a different motor needs a
  rebuild.
- **A leak that starts while already in `SURFACE` mode emits no failsafe `STATUSTEXT`.** The whole
  failsafe block sits inside `if (in.mode != FlightMode::SURFACE)` — pre-existing, and mostly
  moot since the mode-forcing would be a no-op anyway. Not a blind spot in practice: `HEARTBEAT`
  forces `MAV_STATE_CRITICAL` while `leak` is true, a `LEAK` `NAMED_VALUE_FLOAT` streams every
  500 ms, and the OLED and buzzer react to raw sensor state — none of them mode-gated. Only the
  one-shot CRITICAL line is lost. Worth tidying when that block is next touched.
- **Single IMU, single baro, no redundancy.** Inherent to the hardware, not a code defect, but
  it bounds how much any of the above can protect you. See `docs/VS_ARDUSUB.md`.

## What was checked and found sound

Worth recording, so a future audit does not re-tread it: the mutex discipline (every
`StateLock` is scoped, timed, and never held across `vTaskDelayUntil` — the bug that caused the
old "state busy" disarms is genuinely gone); lock ordering in the new `yaw_ref` path (it takes
`mtx_cal`/`mtx_control` *before* `mtx_sensors` is acquired, so no nesting); the DShot 3D band
mapping on both the ESP32 and Pico sides (both bands run low→high within themselves, matching);
the Pico's layered failsafes (150 ms link timeout, hardware watchdog at 250 ms, intrinsic ESC
timeout, plus the e-stop line); the arm-edge and disarm-edge controller resets; and the
`ESPNOW_STALE_MS` / `DEPTH_STALE_MS` freshness gating, which correctly refuses to trust a held
last-value.
