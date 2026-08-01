# Hengla — firmware audit

---

# Round 6 (2026-08-01) — the contract the companion is built on

Not our findings. The `duburi_ws` companion team read this firmware in full and wrote
[`JETSON_FEEDBACK.md`](JETSON_FEEDBACK.md), filtered to one question: *what does the companion
hit that it cannot fix from its own side?* Six of their eleven items are real defects that five
rounds of our own auditing missed — because we were auditing the vehicle in isolation and these
only bite across the wire.

Every claim below was re-verified against the source before it was fixed, and an adversarial
review then re-verified the verification (and caught two errors in it — see the end).

## R35 — a move was stranded on `IN_PROGRESS` for ever when a failsafe displaced AUTO

**The highest-severity defect in the codebase, and it fired only when the vehicle was already in
trouble.** `AGENTS.md` invariant #3 promises every command reaches exactly one terminal ACK.

The `mv_active` / `mv_done_seq` publish was gated on `mode == AUTO`. A leak, low-battery or
GCS-loss failsafe forces `mode = SURFACE` without ending the move — and once that publish stops
running, `mv_active` freezes `true` and `mv_done_seq` never advances. `mav_stream`'s
done-condition is `(mv_done_seq == s_seq) || (s_seen_active && !mv_active)`: **both terms are
then permanently false.** The un-started escape hatch requires `!s_seen_active`, so it cannot
rescue a move that had already begun. Nothing else in the tree writes those fields. Result:
`COMMAND_ACK IN_PROGRESS` at 3 Hz, indefinitely. Same on any operator mode change mid-move,
including one from Bondor.

Fixed with two changes, both needed:

- **`movement::cancel()`** — a new terminal that goes straight to idle, called when AUTO is
  taken away. Deliberately *not* `abort()`: abort parks the command in `PH_BRAKE`, which only
  advances inside `movement::update()`, which is only reached from the AUTO branch we are
  leaving. Using abort here would have swapped one permanent non-terminal state for another —
  a fix that looks right and changes nothing.
- **The `mv_*` publish is now unconditional.** The falling edge that latches `mv_done_seq` has
  to be observable from outside AUTO or the cancel means nothing.

## R36 — `MOVE_STOP` applied zero braking thrust

`JETSON_COMMS.md` documented stop as "brake to a halt". `movement::start()` zeroed `s_uf`,
`s_ul` and `s_speed` **before** the type switch, so `PH_BRAKE` computed `-0 × gain × 0` = zero.
The brake *duration* was correct — it used the leftover ramped speed — so the vehicle held zero
translation for exactly the right interval and coasted.

The asymmetry that hid it: `movement::abort()` never touched those fields, so **internal
preemption braked correctly the whole time**. Only the wire-reachable verb was broken, which is
why five audit rounds of reading the module never caught it.

Fixed by capturing the outgoing leg's axis and ramped speed before the reset and restoring them
in the `STOP` case. On a hull with no position or velocity estimate a coast is unrecoverable
state — nothing on board or on the host knows how far it travelled.

> **Coordinated across repos.** The companion carries a host-side reverse-leg brake as a
> workaround, and their `test_srot_protocol_drift.py` **fails when this lands** — by design, so
> they drop their brake rather than double-kick the hull.

## R37 — the SURFACE failsafe kept driving the last pilot translation and yaw

`computeDemands()` derives surge/sway from the pilot setpoints unconditionally for every mode,
and the SURFACE case passed pilot yaw straight through. Nothing zeroed them on a failsafe — the
only zeroing was on the ARMED→DISARMED edge, and no failsafe disarms.

So on **GCS loss** — the one case where the last sticks are stale *by definition* — the vehicle
ascended while continuing to translate and yaw on whatever the final packet said, indefinitely.
A vehicle that has lost its operator should not still be driving somewhere.

Two fixes, because they cover different failures:

- SURFACE now zeroes translation and yaw outright. That covers a sender still talking while the
  vehicle surfaces for an unrelated reason — a leak with the pilot still on the sticks.
- **`MANUAL_CONTROL` got the freshness triple.** It was the only external input in this firmware
  without one, which is notable given `AGENTS.md` requires it of every external input and the
  recurring bug class in this codebase is exactly "a stale input that keeps being used". Full
  authority for `MANUAL_FRESH_MS`, then a linear ramp to neutral by `MANUAL_DECAY_MS` — a ramp,
  not a cliff, so a single dropped packet does not make piloting lurch. Applied once in
  `readInputs()` so all six mode branches inherit it without needing to know.

## R38 — a failsafe left the vehicle armed at the surface for ever

The failsafe forces SURFACE but never disarms, so a permanent link loss ended with the vehicle
armed and station-keeping at 0 m until the battery ran out. There was no end state at all.

Now disarms once depth has stayed above `FS_SURFACE_DEPTH_M` continuously for
`FS_SURFACE_HOLD_MS`. Gated on a working depth sensor: without one the ascent is open-loop and
"am I at the surface?" is unanswerable, so it keeps pushing up rather than disarming at an
unknown depth and sinking. The hold is what stops a wave slapping the sensor from disarming a
vehicle that is still deep.

## R39 — `DIVE` was not clamped to at-or-below the surface

`JETSON_COMMS.md` claimed DIVE `p2` was "clamped at ≥ 0 — you cannot command above the surface".
It was not: `movement.cpp` had a bare assignment, `depth::setTarget()` has no clamp, and the one
inside `depth::update()` sits in a stick-deadband branch that AUTO never enters.

A negative target therefore produced sustained ascent — and because the runaway guard references
`depth::target()`, **the guard tracked the bad setpoint instead of catching it.** Now clamped at
the source. (The companion rejects `target > 0` host-side, so this was defence in depth — but it
was a documented safety property that did not exist, on the one loop that has never run closed.)

## R40 — `MV_PROG` carried no information for TURN and DIVE

`progress()` had cases for cruise, brake, style and hold; `PH_TURN` and `PH_DIVE` fell through to
`default: return 0.5f`. Those are the two verbs a mission most wants to wait on.

They are closed-loop, so there is no duration to divide by — now they latch the span at start
(angle or depth to cover) and divide the live remaining error, which both completion tests
already compute. Capped at 0.99 so only the terminal state reports 1.0.

## R41 — per-thruster RPM was undecodable by the companion

`ESC_STATUS` (291) is a WIP message that upstream MAVLink **removed from `common`**. pymavlink
silently discards any message whose id is missing from its CRC-extra table — no error, no
callback. So the board could be reporting RPM perfectly while the companion read nothing, and it
looked exactly like an ESC or Bluejay fault. They verified it against four dialects.

Now also emits `ESC_TELEMETRY_1_TO_4` (11030) and `ESC_TELEMETRY_5_TO_8` (11031) — already in
the dialect we vendor, exactly 4 ESCs each, and they decode everywhere. `ESC_STATUS` is kept
alongside because QGC and Bondor render it.

## R42 — no way to change a stream rate

Neither `SET_MESSAGE_INTERVAL` (511) nor `REQUEST_DATA_STREAM` (66) existed, so the fixed 10 Hz
`ATTITUDE` was the hard ceiling on every loop the companion runs — un-raisable for a control
loop, un-lowerable for a slow link. The rates were hardcoded literals.

Now a small table with a live interval and a compiled default per stream, 511 and 510
implemented against it. Requests are clamped to a 20 ms floor so a companion cannot ask for
1 kHz and starve the `PARAM_VALUE` and `COMMAND_ACK` traffic missions depend on. `HEARTBEAT`
cannot be disabled — a GCS that turned it off would look identical to a dead vehicle.

## R43 — smaller items

- **A failed parameter persist was a WARNING.** Raised to `ERROR`: `PARAM_VALUE` still echoes
  the accepted value, so from the companion's side a failed write looks fine and then silently
  reverts at the next boot. They lost a pool session to exactly this.
- **`ATTITUDE` is now exempt from the param-download blackout**, alongside `HEARTBEAT` and
  `ESC_STATUS`. A 10–15 s hole in the companion's control-loop input because an operator opened
  Bondor's Setup tab is not acceptable. The rule is now explicit: anything a *mission* depends
  on goes above the guard, anything only a human reads goes below.
- **`feedforward`'s trim-learning gate is inverted.** It enumerated the modes allowed to learn,
  so an autonomous mode that never drives the sticks left `learn` permanently true and the CoB
  auto-trim learned against machine-commanded effort. AUTO was excluded by name; the next
  autonomous mode would have had to remember. Now derived from "is a pilot mode", so a new mode
  is excluded until someone deliberately includes it.
- **Docs corrected**: `MV_STATE` 6 is HOLD and 7 is DONE (the table said 6 = done, so every
  station-keep leg read as complete); `TEMPORARILY_REJECTED` documented as the fifth, *non*-
  terminal reply; the preemption "quick brake first" claim removed, because there is no brake
  phase between commands; parameter count corrected to 227; UART2 corrected from 2 Mbaud to
  1 Mbaud in four places against `config.h`.

## R44 — every completed move re-sent its terminal ACK for ever (found on the bench)

**Not from the feedback, and not from reading — found by actually running it.** Measured on
hardware at ~14 Hz, indefinitely, after every completed move.

`updateMove()` used `s_seq = 0` as its "nothing being tracked" sentinel. But `mv_seq` in the
snapshot stays at the last command's value, so the very next cycle saw `mv_seq != s_seq` and
**re-adopted the same completed command as if it were new** — whereupon `mv_done_seq == s_seq`
was still true, so it re-sent the terminal ACK and zeroed `s_seq` again. A closed loop.

A single `DIVE` produced ~100 `ACCEPTED` acks in 7 seconds. On a 115200 link shared with all
telemetry, that is continuous waste between every pair of mission legs, and any client that
keys off "a terminal arrived" gets told the same thing a hundred times.

Fixed by replacing the sentinel with an explicit `s_resolved` latch, so the re-adopt test only
fires for a genuinely new sequence. The preemption ACK is also now suppressed for an
already-resolved command — ACKing it again as `CANCELLED` would contradict the `ACCEPTED` the
companion had already acted on.

**Pre-existing since at least Round 2** (`git log` confirms this round did not touch that
logic). Five audit rounds read the function and missed it; twelve seconds of bench traffic
found it. Worth remembering the next time a review feels thorough.

## Found by review of this round's own work

- The first `movement::cancel()` was written as `abort()`, which would have been inert for the
  reason described in R35 — caught while tracing the brake phase, before it shipped.
- The first pass at R36 removed the `s_uf`/`s_ul` reset entirely, which would have leaked the
  previous leg's travel axis into every subsequent verb. Narrowed to capture-and-restore.
- The adversarial review found the proposed `FS_GCS_SYSID` fix for the GCS-failsafe-source
  problem **cannot work**: `JETSON_COMMS.md` tells companions to connect as 255/190 and our own
  LoRa bridge synthesises 255/190, so a parameter naming the required source matches both and
  the dead-Jetson case survives untouched. **Deferred rather than shipped**, because a fix that
  looks like it closes a failsafe gap and does not is worse than no fix. Resolving it needs a
  distinct component id for the companion — a coordinated three-repo change.
- It also downgraded the `feedforward` item from defect to robustness: a second gate already
  excluded AUTO, so nothing was actually mislearning today.

---

# Round 5 (2026-08-01) — autotune, and a ground station that lied about the arm state

A full read of the flight firmware and the ground station, with the relay auto-tuner as the
focus. Eleven findings. The autotune ones share a single theme: **the relay measurement was
never validated**, so a phase that failed to oscillate was indistinguishable from one that
worked — and wrote gains anyway.

## R24 — the ground station asserted a stale ARM state for ever

`s_have_state` was write-once. After the first decoded frame the bridge synthesised a
HEARTBEAT at 2 Hz carrying the last known mode and arm flag, **indefinitely**, and Bondor's
only liveness test is heartbeat age (< 3 s). So a vehicle that went out of range — or lost
power — while armed in AUTO left the GCS showing *connected, ARMED, AUTO* with total
confidence, permanently. Attitude and depth froze, but the two things an operator acts on
kept being asserted.

This is the exact inverse of R18, which stopped the bridge fabricating state *before* the
first frame. The gate added then was never given a back edge.

Fixed: the cached heartbeat now expires at `LINK_DEAD_MS` (5 s), after which the bridge goes
**silent** — Bondor's own heartbeat test then drops the link. Silence is the honest signal;
repeating a stale "ARMED" is worse than saying nothing. A `STATUSTEXT` announces the loss on
the edge (naming the arm state at the time) and the recovery.

## R25 — the autotune gain clamps were calibrated for the wrong loop

One clamp set — `KP_MAX 20`, `KI_MAX 10`, `KD_MAX 2` — was applied to all three loop
families. Those are sane for the **angle** loop (default P 4.5) and meaningless for the
**rate** loops (defaults 0.135 / 0.090 / 0.0036), where they sit **148× / 111× / 555×** above
the default. A single clamp across families whose natural scales differ by ~30× is not a
safety clamp at all: for the rate loops the P and D limits could never be reached, and the I
limit, when it did bind, bound at 111× the default.

Fixed with a per-family `LIMITS[]` table at roughly 7–11× each family's compiled default —
loose enough never to truncate a genuine tune, tight enough that a bad measurement cannot
write something unflyable. Hitting a clamp is now reported, because it means the measurement
wanted somewhere the envelope refused.

## R26 — a phase that never oscillated still wrote a full PID

`measure()` exited on `periods >= MIN_PERIODS` **or** an 8 s timeout, and gains were then
applied on `s_period_n > 0` — so **one** counted half-cycle out of a required six was enough.
There was no distinction between "measured a clean limit cycle" and "drifted across the
hysteresis band once".

Worked through: a single ~4 s interval gives `Tu = 8 s` and `Kd = Kp·Tu/3 = 2.67·Kp`, which
for a rate loop lands at the old clamp — **555× the default D gain**, into a derivative term
on a 500 Hz loop. The mirror case, a fast noise-driven crossing, drives `Ki` to its clamp
instead. Both are reachable on a bench in air, which is where anyone would first try this.

Fixed by `measurementIsValid()`: the full half-cycle count, an amplitude that clears the
Schmitt hysteresis by `AMP_MARGIN`, and a period spread within `PERIOD_SPREAD_MAX` (a real
limit cycle has a consistent period; drift and noise do not).

## R27 — the measured amplitude was the entry transient

`s_amp` was a running max over the entire phase, reset only at phase entry. The relay steps
to full amplitude the instant a phase opens, so the opening excursion is a step response —
typically the largest swing of the run — and it set the amplitude used for `Ku = 4A/(πa)`.

Relay tuning wants the *steady-state* amplitude. Measuring the transient biases `a` high and
`Ku` low, and makes two runs on the same vehicle disagree depending on how violent the entry
happened to be. Fixed by discarding the first `SETTLE_CROSSINGS` half-cycles, amplitude
included.

## R28 — success and total failure were indistinguishable

Autotune emitted exactly one message, ever: `"Disarmed: autotune finished"`. No gains
reported, no per-phase result, and **no warning when a phase produced nothing usable** —
`finishPhase()` silently skipped the write and moved on.

`motor_tune` in the same tree already reports both its results and its failure
(`"MTune: no motor spun - check ESCs/arming"`), so this was an omission, not a stance.

Now: one line per phase with `Tu` and the resulting P/I/D (plus `CLAMP` when the envelope
bound), one line per failure with the reason, and a summary naming how many phases failed.
All sized under 50 characters — MAVLink's `STATUSTEXT.text` is `char[50]` and the
cross-core queue slot is 64 B, so the messages that explain the tune were the ones most at
risk of arriving truncated.

## R29 — a failed rate phase silently poisoned every phase after it

`applyGains()` writes straight into `g_params`, and `attitude::rateAxis()` calls
`loadGains()` on **every** invocation, so a gain is live on the next control cycle. The angle
phases relay a rate *command* through the rate PID, and the depth phase holds attitude with
it — so a bad rate result made every later measurement meaningless while the tune reported
success at the end.

A failed RATE phase now aborts the whole tune, with a reason, and does not save.

## R30 — ATUNE latched and fired on the next arm, in any mode

`ATUNE = 1` (and `MAV_CMD_USER_5`) set `autotune_active` unconditionally, and the gate is
`(in.autotune || mode == AUTOTUNE) && armed` — **the flight mode is irrelevant**. Set it
while disarmed and it persisted (the clear runs only on an armed→disarmed edge, which never
occurs if you were already disarmed), so the next arm for any purpose started a full-authority
relay tune while the GCS and OLED still showed MANUAL.

Autotune now forces `mode = AUTOTUNE` on start, whichever way it was triggered, so what the
vehicle is doing is visible everywhere the mode is, and the start notice says the thrusters
will drive.

## R31 — a safety abort reported the tune as finished

`at_active` was latched before the safety-monitor block, so on an abort the tune branch still
ran, saw "not running", and queued `"Disarmed: autotune finished"` immediately after the
safety reason. Fixed by clearing the local and edge latches in the abort path — which also
removes a redundant abort and duplicate "Autotune stopped" one cycle later.

## R32 — every bulk parameter save was unconditionally silent

`serviceSaveAll()` discarded `saveAll()`'s return value. That covers the GCS "Save to flash"
button, autotune and motor_tune — **all three paths that persist a whole tune**. On a full or
worn NVS the write failed, `PREFLIGHT_STORAGE` returned `ACCEPTED`, and the tuning was gone at
the next boot. The single-parameter path already reported this; the bulk path had no
equivalent. `serviceSaveAll()` now returns the outcome and the caller reports it.

## R33 — the mixer coupled two mechanically independent motor groups

The mix matrix is block-diagonal: motors 1–4 (horizontal) are non-zero only in
yaw/forward/lateral, motors 5–8 (vertical) only in roll/pitch/throttle. The two groups share
no axis and no thruster. But saturation used **one** `maxabs` across all eight, so a
saturating *forward* command scaled down *roll and pitch* — thrusters nowhere near their
limits.

Worked example: `forward = 1.0` with `yaw = 0.5` drives motor 2 to −1.5, giving a global
scale of 0.667 — a hard forward burst silently cost a third of the vehicle's roll/pitch
authority, in the manoeuvre where you want it most. Now normalised per group.

## R34 — smaller items

- **Pre-arm was IMU-only.** Added a leak check (gated on `LEAK_EN`) and a thruster-pack
  check (gated on `FS_BAT_ENABLE`, so a pre-arm is never stricter than the failsafe it
  anticipates). Arming into a leak previously succeeded, then surfaced — safe, but an
  unexplained surface is a fault to diagnose where a refusal is a fact to act on.
- **Ground station `VFR_HUD.heading` was not normalised.** `yaw_cd/100` is signed, the field
  is `uint16` 0..360, so every westerly heading read as ~65400 — **over LoRa only**. The
  control board normalises, so the same vehicle disagreed with itself depending on the link.
- **Ground station `ESC_STATUS` timestamp was in milliseconds**, not µs. Same
  divergence-between-links class.
- **Ground station diagnostics streamed at 8 Hz**, four times the control board's own
  cadence — nine `NAMED_VALUE_FLOAT`s per frame, ~2 KB/s of an 11.5 KB/s link, competing
  directly with the relayed `PARAM_VALUE` traffic the LoRa path is already short of.
  Throttled to 2 Hz.
- **`isStateCmd()` tagged `SET_MODE` and `DO_SET_MODE` differently**, so a mode change sent
  one way would not supersede a queued one sent the other. They now share a supersede class;
  arm/disarm keeps its own.
- **Dead code**: `loraSendMsg()`'s `reps` parameter and its `delay(4)` — every call site
  passed 1. Loss tolerance is handled by queueing three copies, which spreads them across
  three TDM slots and is strictly better against a burst fade.

## Found by review of this round's own changes

- The new pre-arm battery check ignored `FS_BAT_ENABLE`, so it would have refused to arm on
  a threshold the pilot had explicitly disabled. Gated.
- Three new `STATUSTEXT` messages exceeded MAVLink's 50-character field and would have been
  truncated — including two of the ones added specifically to explain a failure. Measured
  and shortened.
- The safety-abort fix cleared the local latches but not the edge latches, leaving a
  redundant abort and a duplicate message one cycle later. Cleared both.
- `params.h` still documented `FS_BAT_VOLTAGE` as a PM1 threshold, which R-round-2 had
  already established was the wrong battery. Corrected.

---

# Round 4 (2026-07-31) — the LoRa link, and two parameters that could not be tuned

From a second field report: the mode display flickers to a wrong value for well under a second,
attitude "goes missing", parameter saves work sometimes and not others, the thruster voltage in
Bondor never changes, the OLED always shows `--`, and changing `PM1`/`PM2` multipliers does
nothing visible.

Six separate defects. The first is the substrate under most of the LoRa symptoms.

## R17 — the SX127x hardware CRC was never enabled, on either radio

`RxPayloadCrcOn` is **0 at power-on**, and `LoRa.enableCrc()` — present in the vendored library —
was called nowhere. The radio therefore handed **corrupted payloads straight up**: into
`mavlink_parse_char()` on the uplink and into the `LoraTelem` decoder on the downlink.

MAVLink's own CRC rejects the damaged message, so this does not produce *wrong* commands. It
produces **missing** ones: garbage bytes desynchronise the parser, so the next one or two *good*
messages are consumed as the tail of a bogus frame and lost too. One corrupt packet costs several
good ones. That is the mechanism behind both "misses the attitude" and "the parameter save is not
reliable" — the same corruption, on the two directions of the same link.

Fixed in `src/drivers/lora_mission.cpp` and `src/groundstation/main.cpp`. Explicit-header mode
carries a CRC-present flag so this is self-describing, but a receiver without it still accepts a
corrupt frame: **both boards must be reflashed together.**

## R18 — the ground station announced STABILIZE before it had heard anything

`s_last_mode` initialised to **0**, which is `STABILIZE` — not a neutral value — and the
synthesised keep-alive heartbeat fired as soon as `now - s_last_frame_ms > 1000`, which is true
from boot. Powering the bridge up before the vehicle therefore told Bondor "STABILIZE, disarmed"
with full confidence, before a single frame had been received. The file's own comment promised
"never send fabricated 0/false to Bondor"; the initialiser contradicted it.

Separately, `t.mode` was copied into the heartbeat **with no range check**, so any frame that
slipped the CRC16 could paint an arbitrary mode for one frame — ~125 ms, which is exactly the
"wrong mode for less than a second" that was reported.

Fixed: `s_have_state` gates the keep-alive, and `modeIsKnown()` rejects unrecognised mode bytes
and keeps the previous value. Everything else in a frame is still used — a bad mode byte does not
invalidate the attitude beside it.

## R19 — the uplink queue dropped parameter writes silently

The TDM scheme carries **one uplink packet per received downlink slot** (~8/s at 120 ms pacing).
Bondor paced writes on `status.kind === 'serial'` — and the LoRa bridge **is** a USB serial port
to Bondor, indistinguishable by link kind — so it fired ~20 writes/s into an 8/s pipe. The ground
station's 24-deep queue overflowed and `ulqPush()` dropped the excess with no counter and no
message (`if (!ulqFull())`, no `else`). "Sometimes it saves and sometimes it doesn't" was that,
exactly.

Fixed at both ends:
- Ground station counts drops and streams `UL_DROP` alongside the existing `USB_RX`/`UP_TX`/`UL_RX`
  named values. It must read 0 during a parameter import.
- Bondor detects the bridge by the `LORA_RSSI`/`LORA_RX` named values that only the ground station
  emits — no protocol change — and paces to ~3.3 writes/s with a longer settle, leaving headroom
  for the `PARAM_VALUE` confirm-and-retry to work instead of being swamped.

Also made mode/arm commands **last-wins** in the uplink queue. Those are sent 3× for loss
tolerance, and the replay is unconditional, so a quick disarm-then-arm could replay the **disarm
after the arm**. Not a symptom that was reported — the flicker was display-only — but it is a real
hazard on its own merits.

## R20 — the thruster voltage was structurally unreachable over LoRa

`LoraTelem` had **no thruster-pack field**. `batt_mv` is `pm1_voltage`, the *electronics* pack, and
the ground station emits only `SYS_STATUS`, which carries one voltage that any GCS maps to Battery
1. So over LoRa there was no path by which the thruster voltage could arrive, no matter how healthy
the ESP-NOW link was. What was being watched in Bondor was PM1 the whole time — and PM1 looked
frozen for the separate reason in R21.

Fixed by adding `aux_mv` (uint16 mV) to the frame, filled from `pm2_voltage` and **0 when not
fresh**, with the ground station emitting `BATTERY_STATUS` id 1 at 1 Hz to match the USB path.

**Co-owned wire change** (`AGENTS.md` rule 1): the struct is duplicated in both repos and the frame
grows 39 → 41 bytes. The receiver requires an exact size match, so a mismatched pair does not
corrupt — the link goes **silent**, which looks exactly like being out of range. Reflash both.

## R21 — my own PM1_VMULT guard made the parameter impossible to calibrate

R16 changed `PM1_VMULT` from volts-per-ADC-count to a divider ratio and added a guard that
**silently substituted the default** for anything below 0.5, warning only at boot. The intent was
to protect against an imported pre-change value. The effect was that every small value typed was
discarded in silence: the reading never moved, and there was no way to tell a rejected value from
one that had never arrived. A parameter you cannot observe responding is a parameter you cannot
calibrate.

It was also wrong on the merits — a ratio below 0.5 is a legitimate setting for a divider this
firmware does not know about.

Fixed: the value set is the value used (only ≤ 0 or NaN falls back). The threshold survives as a
**warning only**, now emitted **when the parameter is set** as well as at boot, because
calibration is an interactive loop and advice that appears only at startup arrives too late.

## R22 — PM2_VMULT was in the table, echoed, and persisted, but read by nothing

`pm2_voltage` came straight from the ESP-NOW aux voltage; `g_params.pm2_vmult` appeared in no
expression anywhere in the firmware. It was documented as inert in `PARAMETERS.md` §B, which is
honest but not sufficient: from the GCS it looked exactly like a live parameter — it accepted
values, echoed them and stored them — so "I change it and see no effect" was the only possible
experience. It is the other half of the report.

Fixed by making it live as a **trim on the ESP-NOW voltage** (1.0 = as reported), applied at the
single ingest point so the OLED, Battery 2, the `FS_BAT_*` failsafe, the mixer's voltage
linearisation and the new LoRa `aux_mv` can never disagree about the pack voltage. This lets the
thruster reading be calibrated from the GCS without reflashing the 2nd board.

The units change is handled by a **one-shot migration**, not a clamp: a stored value below 0.05
cannot be a trim, so it is rewritten to 1.0 in NVS and announced. Writing it back matters — a clamp
at the point of use would leave the param list showing `0.009` while the code used `1.0`, which is
precisely the failure mode of R21. No `PARAM_DEFAULTS_VER` bump; that would wipe an entire tune to
fix one row.

## R23 — `--` on the OLED meant three different things

The thruster-voltage box showed `--` for "`ESPNOW_EN` = 0", for "`espnow_link::begin()` failed",
and for "no packets arriving" — three causes whose fixes are unrelated. Worse, `begin()` was
retried **every loop (20 Hz), for ever, with its failure never reported**, so a radio that could
not initialise was indistinguishable from one with nothing transmitting to it.

Fixed: the OLED shows `OFF` when the source is switched off and `--` only when it is enabled but
not fresh; a boot `STATUSTEXT` says so when `ESPNOW_EN = 0` with `PM2_SRC = 2`; and `begin()` backs
off to 2 s and reports failure on the first attempt and every ~30 s after. `--` now always has an
explanation available.

(Fixing the retry cadence also removed a 20 Hz WiFi-init call from the Core-0 comms task.)

## Found by review of this round's own changes — before any of it shipped

Two defects in the fixes above, caught by an adversarial pass over the working tree.

**R22a — I reimplemented R21's mistake in the same commit that declared it wrong.** The PM2_VMULT
migration was gated on the *value* (`pm2_vmult < 0.05`), which is not a one-shot condition: it
re-evaluates every boot. An operator who deliberately set a small trim after the migration would
have it silently overwritten on the next power cycle — and overwritten **in NVS**, which is
strictly worse than the point-of-use substitution R21 removed, because it destroys the stored
value rather than ignoring it. Three paragraphs above it, I had written that exact reasoning down
as the thing not to do.

Fixed by gating on a persistent marker (`NVS_KEY_PM2_MIGRATED`) instead of the value, so it runs
once per board and every value set afterwards is the operator's to keep. Also added the set-time
warning for `PM2_VMULT` that R21 added for `PM1_VMULT` — it had been left out, so a small PM2
value was accepted with no comment at all and then quietly reverted at the next boot.

**R19a — the LoRa detection sampled the one moment it is guaranteed to be wrong.** `overLora` was
read once at the top of `writeParams()`. But `LORA_RSSI`/`LORA_RX` only start flowing after the
bridge decodes its **first downlink frame** — so importing parameters with the bridge plugged in
and the vehicle not yet powered, which is the ordinary pre-dive order of operations, saw an empty
`named`, chose the **fast profile**, and reproduced the exact overrun R19 exists to prevent. It was
also never re-evaluated, so telemetry appearing mid-import changed nothing.

Fixed by re-reading the profile before every batch, and by treating "no telemetry at all" as LoRa
rather than as USB: a directly-connected board streams its own named values within ~500 ms, so an
empty `named` means nothing has been heard from and the fast link must not be assumed. The
asymmetry is deliberate — guessing slow on a fast link costs a minute on an import that succeeds;
guessing fast on a slow link loses parameters.

---

# Round 3 (2026-07-30) — parameter persistence: the partition was too small all along

Found from a field report ("parameter set but NOT saved (NVS full?)" over both USB and LoRa,
parameters not loading, payload mode not changeable). **This is the root cause of several
long-standing complaints, and it predates every change made in this session.**

## R14 — the NVS partition physically cannot hold the parameter set

`Preferences::putFloat()` calls `putBytes()` → `nvs_set_blob()`
(`framework-arduinoespressif32/libraries/Preferences/src/Preferences.cpp:255`). Every float is
stored as a **blob**, costing ~3 NVS entries (blob index + item header + data), not 1.

```
nvs partition (huge_app.csv) : 0x5000 = 20 KB = 5 pages
usable entries               : (5 − 1 reserved for GC) × 126 = 504
needed                       : (121 scalar + 80 servo + 26 cal) × 3 = 681
```

**681 > 504.** Once the partition filled, writes failed. Downstream symptoms, all one cause:
values that never persisted read back as defaults ("not all parameters load"); a `SERVOn_ROLE`
change would not stick ("cannot change payload mode"); and `ESPNOW_EN` / `MOT_BAT_V_MAX` failing to
save left the thruster-voltage link down, so the OLED's PM2 box showed `--` and the voltage looked
absent or frozen.

**Correction to an earlier diagnosis in this same document.** When the operator reported parameters
resetting after a flash, that was attributed to `PARAM_DEFAULTS_VER` bumps discarding tuning. Bumps
*do* discard tuning and that part stands — but it was not the whole story, and the deeper cause was
this. The R10 "not saved" warning added earlier did not introduce the fault; it made a
long-silent failure visible for the first time. Recorded here rather than quietly amended.

**Fix.** New `partitions_hengla.csv`: NVS becomes **128 KB (32 pages → 31 × 126 = 3906 usable
entries, 5.7× headroom)**, placed **after** the app at `0x310000`. Verified by decoding the generated
`partitions.bin` with ESP-IDF's own `gen_esp32part.py` rather than trusting the CSV.

> ### ⚠ This fix bricked the board on its first attempt. Recorded, because the reason is a trap.
>
> The first version grew NVS in place (`0x5000` → `0xE000`) and moved `app0` from `0x10000` to
> `0x20000` to make room. **The board then did nothing at all — no display, no serial — and
> reflashing could not fix it.**
>
> Cause: **PlatformIO does not read the app offset from the partition CSV.**
> `platform-espressif32/builder/main.py` does
> `ESP32_APP_OFFSET = board.get("upload.offset_address", "0x10000")`, so it flashed `firmware.bin`
> to a hardcoded `0x10000` while the bootloader — reading the new table — looked for the app at
> `0x20000` and found erased flash. Every subsequent flash repeated the same mismatch, which is
> exactly why reflashing appeared to do nothing.
>
> The board was never damaged: the bootloader at `0x1000` stayed intact, and flashing a table whose
> app offset matches where PlatformIO actually writes recovers it immediately.
>
> **Rule for anyone editing that file: `app0` must start at `0x10000`.** Squeezing NVS in below the
> app is capped at 28 KB anyway by the `0x9000` table boundary, so placing NVS *after* the app is
> both safe and far roomier — partition order is free, since `nvs_flash_init()` finds the partition
> by label, not position. The `0x9000`–`0x10000` region is left deliberately unallocated.

One consequence, handled explicitly:

- **Existing NVS is invalidated** — NVS has moved to a different address entirely, so the old
  of the app, so those pages are neither blank nor valid NVS. `params::init()` now runs
  `nvs_flash_init()` and, **on `ESP_ERR_NVS_NO_FREE_PAGES` or `ESP_ERR_NVS_NEW_VERSION_FOUND`
  specifically**, `nvs_flash_erase()` + retry. Those two mean "the contents are unusable"; erasing
  on *any* error would risk wiping a good calibration in response to something an erase cannot fix
  (a missing partition, an allocation failure). Without this guard every
  `Preferences::begin()` would fail forever and no parameter could ever be saved again — the exact
  failure the partition change exists to fix. A reformat is announced as a **CRITICAL** `STATUSTEXT`
  because it also wipes the sensor calibration, which shares the partition.

**Also reduced the write churn that filled it.** `saveAll()` rewrote all 201 non-cal parameters on
every save — and autotune, motor_tune and every GCS "Save to flash" call it — churning ~603 entries
of garbage per save and driving continuous garbage collection. `saveAll()` and `set()` now skip
values already stored. A save is now nearly free in the common case.

> **Count correction.** The first write-up of this finding said 113 scalar params / 657 entries.
> Review caught it: `SCALARS[]` actually has 147 rows, 26 of them `CAL_*` views that own no NVS key,
> leaving **121** non-cal scalars. Real totals are **227 keys / 681 entries**, and headroom after the
> fix is **2.41×**, not 2.49×. The conclusion is unchanged — 681 > 504 just as 657 did — and the fix
> is still comfortably sufficient, but the arithmetic is the whole point of this entry, so the
> corrected numbers are what stand.

**A related gap closed at the same time.** The skip-if-already-stored compare reads an absent key
back as its `NAN` default, so a genuinely-NaN value written to a never-before-written key would
*match*, take the skip path, and report `persisted = true` for a key that was never stored — then
silently revert to its compiled default on the next boot. `PARAM_SET` is a separate entry point from
`dispatchCommand()`, so R5's blanket non-finite rejection did not cover it. `onParamSet()` now
rejects non-finite values the same way.

## R15 — on/off payload channels were unreachable over MAVLink

`DO_SET_RELAY`'s param1 is a relay *instance*, mapped to `PCA_RELAY_BASE_CH + instance` = **channels
9–16 only**. `DO_SET_SERVO` writes `servo_us[ch]`, which `Task_UI_Status` deliberately zeroes for a
role-2 channel. A channel in the **1–8** range set to on/off was therefore addressable by **neither**
command — which is why neither the servo nor the relay responded over USB in that mode. One bug, two
symptoms.

**Fix.** `DO_SET_SERVO` now branches on `SERVOn_ROLE`: on a role-2 channel the pulse width is a level
(≥ 1500 µs = ON); otherwise it writes `servo_us` as before. Every channel is now addressable by its
own channel number whatever its role. `DO_SET_RELAY` is unchanged, so the joystick relay buttons that
share its mapping still behave identically.

## R16 — PM1 read a non-linear ADC through a linear multiplier

`analog_mon.cpp` did `analogRead(pin) × volt_mult` at 11 dB attenuation. The ESP32 ADC is markedly
non-linear there (worst above ~2.5 V and below ~0.15 V); no single multiplier fits it, and after a
battery divider the error is easily several hundred millivolts.

**Fix.** Use `analogReadMilliVolts()`, which applies this chip's factory eFuse calibration curve.
`PM1_VMULT` consequently changes meaning from volts-per-ADC-count (~0.009) to the **divider ratio**
(~11). Because a `.params` backup written before this change carries the old value — and applying
~0.009 as a ratio would report a flat battery on a full pack — a value below 0.5 is detected as
stale, replaced by the default, and reported over `STATUSTEXT`. Current is left on raw counts: its
multiplier is uncalibrated either way and `CURR` is display-only.

## Corrected while investigating — the OLED was not the problem

The plan for this round claimed the OLED never displayed the thruster voltage, based on `oled.h`
declaring an `aux_v` field that `oled.cpp` referenced zero times. **That diagnosis was wrong.** The
voltage reaches the display as `pm2` (since `PM2_SRC` defaults to 2) and the top-left box already
prints it, showing `--` when the link is stale. Nothing was missing; the reading was absent because
the link was down, because `ESPNOW_EN` had not persisted — R14 again.

The dead `aux_v` field was removed rather than wired up: it was plumbed from `Task_UI_Status` and
never drawn, and its presence is precisely what made the display path look absent. No new readout
was added, because none was needed.

---

Two earlier rounds. **Round 2 is below and includes the most serious defect found in either** — the depth
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
