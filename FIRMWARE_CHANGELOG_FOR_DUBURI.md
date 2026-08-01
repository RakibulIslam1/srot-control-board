# Firmware changes for `duburi_ws` — 2026-08-01 (Round 6)

**Written for the companion team.** This is our reply to
[`JETSON_FEEDBACK.md`](JETSON_FEEDBACK.md). Every item you raised is answered below: fixed,
deferred with a reason, or bounced back as a question. Full detail in [`AUDIT.md`](AUDIT.md)
R35–R43.

Short version: **your §1, §2, §3, §5, §6, §7, §8 and §9 are all fixed.** §4 is *deliberately
not* fixed and the reason matters — read it. §10 is corrected. §11 is deferred to L3, on
purpose, with the scope note at the end.

> ⚠️ **Two of these are coordinated changes that will break a workaround of yours.** They are
> marked **[COORDINATE]**. Nothing here changes a wire constant, so `test_srot_protocol_drift.py`
> should still pass on constants — but the *behavioural* assertions in it will move.

---

## What changed, mapped to your findings

### §1 — terminal-ACK guarantee — **FIXED** [COORDINATE]

You were right, and it was worse than "sometimes": once a move had started, **neither** disjunct
of the done-condition could ever become true again. There was no rescue path anywhere in the
tree.

Two changes, both required:

- New `movement::cancel()` — an immediate idle terminal — called when AUTO is displaced by a
  failsafe *or* by any operator/GCS mode change. Notably **not** `abort()`: abort parks the
  command in `PH_BRAKE`, which only advances inside `movement::update()`, which is only reached
  from the AUTO branch being left. Using abort would have swapped one permanent non-terminal
  state for another.
- **The `mv_*` publish is now unconditional**, not gated on `mode == AUTO`. The falling edge that
  latches `mv_done_seq` has to be observable from outside AUTO or the cancel means nothing.

**What you should do:** your leg-duration ACK deadline (`srot_fc._ack_budget_s`) is now
redundant. It is *harmless* — a strict improvement over waiting forever — so there is no version
skew hazard and you can retire it whenever convenient. No rush, no ordering requirement.

**Behaviour change to expect:** `MV_STATE` / `mv_active` now arrive outside AUTO too. If you
gated any parsing on "we are in AUTO", you can drop that.

### §2 — `MOVE_STOP` coasted — **FIXED** [COORDINATE — this one has ordering]

Exactly your diagnosis. `start()` zeroed `s_uf`/`s_ul`/`s_speed` before the type switch, so
`PH_BRAKE` computed `-0 × gain × 0`. Fixed by capturing the outgoing leg's axis and ramped speed
before the reset and restoring them in the `STOP` case.

**⚠️ You must remove `SrotFC._brake_last_leg` when you take this firmware**, or the hull gets
braked twice — once by your reverse leg, once by ours. Your drift test is designed to fail here;
that failure is the signal, not a bug.

Suggested sequence: take this firmware on the bench, confirm `stop` decelerates, then delete the
host brake in the same session. Measure coast distance before and after — we would genuinely
like the number for `docs/T200_PROFILE.md`.

### §3 — SURFACE kept driving stale sticks — **FIXED**

Two independent fixes, because they cover different failures:

- SURFACE now zeroes translation **and** yaw outright.
- **`MANUAL_CONTROL` got the freshness triple** it was missing — it was the only external input
  in the firmware without one. Full authority for 1000 ms, then a linear ramp to neutral by
  1500 ms. A ramp, not a cliff, so a single dropped packet does not make piloting lurch.

**Relevant to you:** you disable the 5 Hz neutral-RC heartbeat on SROT (correctly). That means
during an AUTO move nothing sends `MANUAL_CONTROL`, so the setpoints age out to neutral. This is
harmless for AUTO translation (the movement primitive overrides `fwd`/`lat`) but **`STYLE`/spin
passes `sp_throttle` through live**. If you ever want to hold depth manually during a
`style_roll`, you must keep streaming `MANUAL_CONTROL` — otherwise the throttle decays to
neutral within 1.5 s. Previously it held the last value indefinitely.

### §4 — GCS failsafe not source-specific — **NOT FIXED, and here is why**

We tried. The obvious fix — a `FS_GCS_SYSID` parameter naming the required source — **cannot
work**, and shipping it would have been worse than shipping nothing:

- `JETSON_COMMS.md:30` tells companions to connect as `source_system=255, source_component=190`.
- Our LoRa bridge synthesises its filler heartbeat as **255/190** — the same identity.

So a parameter naming "the Jetson" as 255/190 also matches the bridge, and your exact scenario
(dead Jetson, live Bondor holding the failsafe open) survives untouched — while everyone
believes the gap is closed. We would rather leave a known hole than ship a fake fix.

**This needs a decision from you, and it is a three-repo change:**

> **Proposal: the companion adopts a distinct component id.** Something in the onboard-computer
> range (e.g. `MAV_COMP_ID_ONBOARD_COMPUTER = 191`) rather than sharing 190 with every GCS and
> relay. Then `FS_GCS_SYSID`/`FS_GCS_COMPID` becomes meaningful, we can track liveness per
> source, and the bridge's filler heartbeat stops satisfying a failsafe that is supposed to be
> watching *you*.
>
> Cost on your side: one constant in `srot_protocol.py`. Cost on ours: the liveness map plus two
> params. Cost on Bondor: none.
>
> Is there a reason you need 190? If you would rather we key on the physical link instead
> (USB vs LoRa), say so — that is also implementable and needs nothing from you, but it cannot
> distinguish two companions on the same link.

Note the exposure is narrower than first stated: our bridge only transmits its filler heartbeat
while Bondor's USB has been active within 2 s. So the failure case is specifically **Jetson dies
while Bondor stays connected**, not "the LoRa link holds it open indefinitely".

### §5 — `SET_MESSAGE_INTERVAL` — **IMPLEMENTED**

`MAV_CMD_SET_MESSAGE_INTERVAL` (511) and `GET_MESSAGE_INTERVAL` (510) both work now. The
previously hardcoded rates became a table with a live interval and a compiled default per
stream.

Adjustable: `HEARTBEAT`, `SYS_STATUS`, `ATTITUDE`, `SCALED_IMU2`, `SCALED_PRESSURE2`, `VFR_HUD`,
`BATTERY_STATUS`, `POWER_STATUS`, `ESC_STATUS`, `NAMED_VALUE_FLOAT`.

Semantics are the standard ones: `param2` > 0 sets the interval in µs, `0` restores our default,
`< 0` disables the stream. Two house rules:

- **Clamped to a 20 ms floor** (50 Hz). Without it a companion can ask for 1 kHz `ATTITUDE` and
  starve the `PARAM_VALUE` and `COMMAND_ACK` traffic your missions depend on. Budget check:
  `ATTITUDE` at 50 Hz is ~1.8 kB/s, about 16 % of the 115200 link.
- **`HEARTBEAT` cannot be disabled** — a request to do so returns `DENIED` rather than being
  accepted and ignored. A vehicle that stopped heartbeating would be indistinguishable from a
  dead one.

`ATTITUDE` at 50 Hz is now yours for the asking. Your `MESSAGE_RATES` pinning block can be
re-enabled on the srot backend — the `srot-integration.md` gotcha that says to skip it is now
out of date.

Still **no `COMMAND_CANCEL`**. With §2 fixed, `stop` genuinely stops, which we think covers it —
tell us if it does not.

### §6 — `MV_STATE` off-by-one and dead progress — **FIXED (both halves)**

- Docs corrected: **6 is HOLD, 7 is DONE**. `JETSON_COMMS.md` was the one still wrong; we had
  already fixed `DUBURI_WS_INTEGRATION.md`.
- **`progress()` now returns real values for TURN and DIVE.** They latch the span at start (angle
  or depth to cover) and divide the live remaining error — which both completion tests already
  computed. Capped at 0.99 so only the terminal state reports 1.0, letting you distinguish
  "nearly there" from "done".

### §7 — `DIVE` clamp — **FIXED**

Clamped at the source in `movement::start()`. The documented property now exists. Your
host-side rejection of `target > 0` is still worth keeping as defence in depth.

`p3` is still ignored for DIVE — the descent rate is still `MOVE_DEPTH_RATE`. That needs the
depth-rate loop, which is deferred with the rest of the velocity work.

### §8 — `ESC_STATUS` undecodable — **FIXED**

Excellent catch, and it explains a bench symptom we had not connected. We now emit
**`ESC_TELEMETRY_1_TO_4` (11030)** and **`ESC_TELEMETRY_5_TO_8` (11031)** alongside `ESC_STATUS`
(kept because QGC and Bondor render it).

Two things to know:

- RPM in `ESC_TELEMETRY` is `uint16`, so we send **magnitude**. The sign lives in the commanded
  direction, which you already know. `ESC_STATUS` still carries signed `int32` if you prefer it
  and can decode it.
- Temperature / voltage / current are **0 = not instrumented**, not "measured zero". The Pico
  link does not report them yet.

`/duburi/esc_rpm` should build now.

### §9 — Round-3 notes — **FIXED**

- **Failed parameter persist is now `MAV_SEVERITY_ERROR`**, not WARNING. You were right that a
  companion has no other way to see it: `PARAM_VALUE` echoes the accepted value regardless.
- Re your `JS_GAIN_DEFAULT = 1.0`: **please re-write and re-verify it.** The NVS partition was
  enlarged in Round 3, so writes work now, but anything written during the full-partition era
  should be assumed lost. `GAIN` is streamed as a `NAMED_VALUE_FLOAT` — check it reads 1.0.
- `PARAMETERS.md` notes the `PM1_VMULT` units change.

### §10 — documentation drift — **FIXED**

Corrected in `JETSON_COMMS.md`: the `MV_STATE` table; `TEMPORARILY_REJECTED` documented as the
fifth, **non-terminal** reply with an explicit "retry it"; the preemption "a quick brake first"
claim removed (there is no brake phase between commands — a preempting **`stop`** brakes, a
preempting *move* does not); parameter count corrected to **227**; UART2 corrected from 2 Mbaud
to 1 Mbaud in four places.

Also corrected, and this one is new information for you: the param-download blackout list.
**`ATTITUDE` is now exempt**, alongside `HEARTBEAT` and `ESC_STATUS`/`ESC_TELEMETRY` and the
move ACKs. An operator opening Bondor's Setup tab no longer blinds your control loop for 10–15 s.
`VFR_HUD` (depth) is still suppressed — if you need depth during a download, avoid the download.

The rule we now apply, so you can predict it: **anything a mission depends on is exempt; anything
only a human reads is throttled.**

### §11 / `VISION_API.md` — **deferred, deliberately**

Your spec is the best-specified thing anyone has handed this firmware and we intend to implement
it close to as written. It is not in this round because of a scope decision, not a disagreement.

Two answers to your open questions while they are fresh:

- **Q2, yaw damping source: measured gyro rate, definitively.** We have it at 500 Hz with zero
  transport latency. Differentiating a 20 Hz bearing would be strictly worse and you were right
  to flag that you cannot do it.
- **Q1, subtype vs new command: `movement::Type` subtype**, for exactly the reason you give — the
  ACK machine, progress, preemption and abort all come free, and a new `FlightMode` is eight
  touch points that fail silently if you miss one.

---

## ⛔ READ FIRST — your drift test will NOT catch the `MOVE_STOP` fix

We ran your `test_srot_protocol_drift.py` against this firmware with
`SROT_FW_DIR` pointed at it. **All 7 tests pass — including
`test_stop_still_does_not_apply_reverse_thrust`.**

That test is green and the behaviour has changed underneath it. It greps the `Type::STOP`
case for `abort()`; we fixed MOVE_STOP by restoring the outgoing leg's axis and speed, which
`start()` had zeroed before the switch.

**So: remove `SrotFC._brake_last_leg` before flying this, or the hull brakes twice.** Your
tripwire cannot tell you.

Worth knowing for the record: routing `Type::STOP` through `abort()` — the fix your docstring
suggests — would **not** have worked on its own. `start()` zeroes `s_uf`/`s_ul`/`s_speed`
before the type switch and `abort()` does not restore them, so `PH_BRAKE` would still have
computed `-0 × gain × 0`. The reset had to move regardless of which entry point you use.

### The fix for the fix: assert on behaviour, not on our source text

`include/config.h` now carries an explicit revision, bumped deliberately whenever observable
behaviour a partner has worked around changes:

```c
#define SROT_FW_BEHAVIOUR_REV   2
```

Suggested replacement for the brittle assertion:

```python
# in srot_protocol.py, parsed from the firmware header like your other constants
FW_BEHAVIOUR_REV = 2

# in test_srot_protocol_drift.py
def test_host_brake_workaround_is_still_needed():
    """REV 2 fixed MOVE_STOP. Above it, our host brake is a double brake."""
    assert sp.FW_BEHAVIOUR_REV < 2, (
        'firmware REV >= 2 brakes on MOVE_STOP -- remove SrotFC._brake_last_leg')
```

Rev meanings are documented next to the define. We will bump it for anything you might have a
workaround for; a plain source-text grep cannot survive us fixing something a different way
than you predicted, and this one already did not.

## One extra fix you did not ask for, because we found it on the bench

**Every completed move was re-sending its terminal ACK at ~14 Hz, for ever** (`AUDIT.md` R44).
Pre-existing since at least Round 2, not a Round-6 regression — we found it by running the
firmware against real hardware rather than reading it.

`updateMove()` used `s_seq = 0` as its idle sentinel, so the next cycle re-adopted the same
completed command as new, saw it was already done, and re-sent the terminal — a closed loop. One
`DIVE` produced ~100 `ACCEPTED` acks in 7 seconds.

**Check your action server tolerates this**, because you have been receiving it all along:
- If you take the first terminal and ignore the rest, you were fine and now you get less traffic.
- If anything keys off "a terminal arrived" (a counter, a state transition, a log line), it has
  been firing ~100× per leg.

Bench-verified before and after: `terminals seen = ['ACCEPTED' × ~100]` → `['ACCEPTED']`.

## Bench results from this firmware (COM19, 2026-08-01)

Run against the real board, disarmed throughout. What we could and could not prove:

| Check | Result |
|---|---|
| `ESC_TELEMETRY_1_TO_4` / `_5_TO_8` decode | **PASS** — both at 5 Hz. `UNKNOWN_291` still present (ESC_STATUS, kept for QGC/Bondor); pymavlink confirms `291 in mavlink_map = False`, `11030 = True` |
| `SET_MESSAGE_INTERVAL(ATTITUDE, 20000)` | **PASS** — 11.0 → 50.8 Hz |
| restore default (`interval = 0`) | **PASS** — 50.8 → 10.2 Hz |
| `GET_MESSAGE_INTERVAL` | **PASS** — returns `MESSAGE_INTERVAL msgid=30 interval=20000us` |
| `HEARTBEAT` disable | **PASS** — `DENIED`, as intended |
| single terminal ACK per move | **PASS** — one `ACCEPTED`, was ~100 |
| move resolves when AUTO is refused | **PASS** — `FAILED` + `"No depth sensor - mode refused"`, no hang |
| `DIVE` negative clamp | **INFERRED, not directly observed.** Unclamped, `depth::target()` would be −2.0 and the runaway guard (`ST_DEPTH_DELTA` 2.0) would have tripped and disarmed. It did not, and no runaway STATUSTEXT appeared |
| `MV_PROG` sweep on TURN | **NOT PROVEN.** On a static bench the hull cannot rotate, so remaining == span and progress correctly reads 0. That is the honest answer where it used to be a fabricated 0.5 — but the sweep itself needs the vehicle to actually move. **Please verify this one in water.** |

Also observed, and worth knowing: **`GAIN` reads 0.500 on this board.** Your §9 hypothesis was
right — the `JS_GAIN_DEFAULT = 1.0` write never persisted, so `MANUAL_CONTROL` has been running
at half authority. NVS writes work now (Round 3 enlarged the partition); please re-write it and
confirm `GAIN` reads 1.0.

## What we did NOT do, and what we would want first

**No velocity or position work.** Not because we disagree — because there is no firmware-facing
sensor to stand on, and we would rather say that than build on an aspiration:

- No hardware optical-flow sensor exists in either repo.
- The DVL's status is self-contradictory across six of your own documents (`dvl-integration.md`
  says SHIPPED; `hardware-setup.md` says STUB; `vehicle-spec.md` says both, eleven lines apart).
  More decisively, it is host-side only and never reaches the firmware, and **`Dubomini 2.0` —
  the competition body — is documented as having none.**
- **No 2026 mission calls a distance verb.** They were rewritten to bbox-fill precisely because
  distance is unavailable.

So `move_forward_dist` / `move_back_dist` / `move_lateral_dist` stay refused, and that is the
right answer today.

**If you want that to change, the useful thing is not more firmware — it is one of these:**

1. **A hardware flow sensor on our UART1** (GPIO13/14 are free, non-strapping). We would do
   rotation compensation with our own 500 Hz gyro, which is the one thing you structurally
   cannot do well over a 10 Hz `ATTITUDE` stream. `OPTICAL_FLOW_RAD` (106) is already vendored
   and is the ideal ingest message — it carries flow integrated over a window **plus the gyro
   integral over that same window**, which is exactly the concurrency the split destroys.
2. **A validated DVL on the competition vehicle**, publishing `VISION_SPEED_ESTIMATE` (103) — also
   already vendored. Then we close the loop on-board and the distance verbs land.
3. **Reconciling the DVL documentation.** Six documents disagreeing is itself a blocker: we
   cannot plan against it. One line saying what is actually installed and tested would help.

Our headroom is waiting: free UART1, a spare VSPI CS, 2.16 MB flash and ~75 % RAM.

---

## Suggestions for `duburi_ws`, offered not imposed

These come from reading your stack; push back freely.

1. **Retire the two workarounds** — `_brake_last_leg` (required, see §2) and `_ack_budget_s`
   (optional). Both were correct calls at the time and both are now dead weight.
2. **Re-enable `MESSAGE_RATES` on the srot backend** and pin `ATTITUDE` to 50 Hz. Your
   `srot-integration.md` gotcha "SROT rates are fixed on-board; skip the pinning" is now wrong.
   This is probably the single biggest cheap win available to your host-side loops.
3. **`move_back` is not a firmware gap.** `MOVE_BACK = 1` works on the wire; the `AttributeError`
   is host-side. Removing it from `UNSUPPORTED_VERBS` should be a small change.
4. **`lock_heading` may not need firmware either.** SROT already locks the heading each translate
   leg starts with, on-board — which is what `heading_lock.py` was invented to fake against an
   untrusted ArduSub compass. The one real gap is `task_full_2026`'s `lock_heading(0.0)`, an
   *absolute* heading, which needs `MAG_YAW_REF = 1`. Our read is that this maps to an absolute
   `TURN` plus the implicit per-leg hold. Confirm before we build anything.
5. **`arc` is a real firmware gap** — ours takes a signed yaw *rate* with no heading lock; yours
   holds an absolute heading while curving. Tell us which you want and we will add it (append-only,
   wire 10) or document ours as rate-only.
6. **`style_yaw`** — ours is hardcoded ROLL at 90°/s. Making the axis and rate command parameters
   is small; say if it is worth doing.
7. **Consider extending the drift test to parameter defaults**, not just wire constants. Three of
   the ten doc-drift items you found were wrong default values in `PARAMETERS.md`. The same
   read-their-headers trick would catch those.

---

## Bench checklist for this firmware

Please run these before trusting it in water — they are the gates from our side, and several
need your action to verify:

| # | Check | Whose |
|---|---|---|
| 1 | Start `move_forward`, trip a leak mid-leg → **terminal ACK arrives**, not `IN_PROGRESS` for ever. Repeat with a Bondor mode change mid-move | yours |
| 2 | `turn` and `dive` → `MV_PROG` sweeps 0→1, not stuck at 0.5 | yours |
| 3 | Arm, hold a stick, unplug the Jetson → ascends with **zero** translation and yaw | either |
| 4 | `stop` after a translate leg **decelerates**; remove your host brake in the same session | yours |
| 5 | `DIVE` to a negative target → refused/clamped, not ascent | yours |
| 6 | `/duburi/esc_rpm` populates | yours |
| 7 | `SET_MESSAGE_INTERVAL(ATTITUDE, 20000)` → 50 Hz arrives | yours |
| 8 | Pilot **selects** SURFACE deliberately → **must NOT auto-disarm**. Only a failsafe-forced SURFACE disarms at the top | ours + yours |

⛔ **Unchanged standing gate:** the depth loop has still never run closed. The two bench checks in
`DUBURI_WS_INTEGRATION.md` gate every dive-dependent verb, and §7's clamp does not change that.
