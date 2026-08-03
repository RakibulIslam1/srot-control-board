# Jetson-side feedback on Hengla v0.2.0

Written by the **`duburi_ws` (Mongla)** side — the ROS 2 companion that drives this board
over MAVLink. Read alongside `DUBURI_WS_INTEGRATION.md` and `JETSON_COMMS.md`, which are
the contract this document reports against.

**Reviewed at:** `8cb4203` (Round 3). Firmware read in full: `mav_commands.cpp`,
`mav_stream.cpp`, `movement.cpp`, `task_control_loop.cpp`, `arming.cpp`,
`depth_control.cpp`, `attitude_control.cpp`, `mixer.cpp`, `pico/main.cpp`, `config.h`,
`state_types.h`, plus every root doc.

**Framing.** This is not a firmware quality review — the audits are already doing that
well, and Rounds 1-3 fixed real, hard-to-find defects (the inverted depth PID especially).
Everything below is filtered to one question: **what does the companion computer hit that
it cannot fix from its own side?** Ranked by that, not by severity in the abstract.

Where we have already worked around something, it says so — a workaround is a cost we are
paying every mission, not a reason to deprioritise. Each item names the file:line we read.

---

## 1. A move can be stranded on `IN_PROGRESS` forever — the terminal-ACK guarantee is not held

**Severity: highest.** `JETSON_COMMS.md:157` states *"Every command reaches exactly one
terminal result. That is now guaranteed."* Our whole action-server design rests on that
sentence, and it is not currently true.

The move-state publish is gated on the mode:

```cpp
// task_control_loop.cpp:731-741
if (in.mode == FlightMode::AUTO) {
    g_state.control.mv_active = now_active;
    ...
    if (mv_was_active && !now_active) g_state.control.mv_done_seq = prev_mv_seq;
}
```

The leak / low-battery / GCS-loss failsafes all set `mode = SURFACE` (`:542-544`) **without
calling `movement::abort()`**. So `mv_active` freezes at `true`, and in
`mav_stream.cpp:514` the done-condition `(mv_done_seq == s_seq) || (s_seen_active &&
!mv_active)` can never become true again. The un-startable escape hatch at `:507` requires
`!s_seen_active`, so it cannot fire either. The result is `MAV_RESULT_IN_PROGRESS` at 3 Hz,
indefinitely.

The same happens on **any** operator or GCS mode change during a move — including one from
Bondor. (The safety-monitor path at `:474-482` is correct: it calls `movement::abort()` and
leaves the mode at AUTO, so that one resolves.)

Why it matters more than it looks: this fires in exactly the situations where the vehicle
is already in trouble, and it makes the **host deadline the only terminator rather than a
backstop**. A companion that trusted the guarantee would hang until its own timeout.

**Suggested fix (either):** call `movement::abort()` wherever a failsafe or mode change
displaces AUTO; or publish the `mv_*` block unconditionally and let `updateMove()` see the
transition. The first is cheaper and also fixes item 2's sibling case.

**Our workaround:** we now derive an ACK deadline from the leg's own expected duration
rather than trusting `p5` (`srot_fc._ack_budget_s`). It bounds the damage; it does not
restore the guarantee.

---

## 2. `MOVE_STOP` applies **zero** braking thrust — "brake to a halt" is a coast

`JETSON_COMMS.md:108` documents **stop** as *"(brake to a halt)"*. It does not brake.

`movement::start()` zeroes the axis signs and the speed **before** the type switch:

```cpp
// movement.cpp:56,58
s_speed = constrain(speed, 0.0f, g_params.move_cruise_max);   // speed=0 -> 0
s_uf = s_ul = 0; s_yaw_rate = 0;
// :90
case Type::STOP: s_phase = PH_BRAKE; s_brake_ms = brakeMs(s_cur_speed); return;
```

`PH_BRAKE` then computes `d.fwd = -s_uf * g * s_speed` = `-0 * g * 0` = **0** (`:152-153`).
The brake *duration* is correct (it uses the leftover `s_cur_speed`), but no reverse thrust
is ever applied — the vehicle holds zero translation for the brake window and coasts.

The revealing detail: **`movement::abort()` (`:109-114`) does not touch `s_uf`/`s_speed`,
so the internal preemption path brakes correctly.** Only the wire-reachable verb is broken.
That asymmetry is presumably why the audits didn't catch it — the code that gets exercised
internally is the good one.

This lands on a vehicle with **no position or velocity estimate**, so a coast is
unrecoverable state: nothing on board or on the host knows how far it travelled.

**Suggested fix:** route `Type::STOP` through `abort()`, or move the `s_uf/s_ul/s_speed`
reset to after the switch so `STOP` keeps the outgoing leg's axis and speed.

**Our workaround:** every abort/cancel/stop now sends an explicit short **reverse
`SROT_MOVE` leg** before `MOVE_STOP` (`srot_fc._brake_last_leg`, gain/duration mirroring
`MOVE_BRAKE_GAIN`/`MOVE_BRAKE_K`). We have a drift test that **fails when you fix this**, so
we drop our brake instead of double-kicking the hull.

---

## 3. The SURFACE failsafe keeps driving the last pilot translation and yaw

`computeDemands()` computes surge/sway from the sticks **unconditionally, for every mode**,
before the switch:

```cpp
// task_control_loop.cpp:134-143
fwd = ((1-pe)*f + pe*f*f*f) * ps;   // f = in.sp_forward
lat = ((1-pe)*l + pe*l*l*l) * ps;
// :171 — the SURFACE case passes pilot yaw straight through
attitude::stabilize(0, 0, in.sp_yaw, ...);
```

Nothing zeroes `sp_forward` / `sp_lateral` / `sp_yaw` on a failsafe — only the
ARMED→DISARMED edge does (`:293-302`), and none of the three failsafes disarm.

Compounding it, there is **no `MANUAL_CONTROL` staleness watchdog at all** (`:172` in our
reading of `mav_commands.cpp:563-602` — setpoints persist until overwritten).

So on a **GCS-loss failsafe** — the one case where the last-received sticks are stale *by
definition* — the vehicle ascends while continuing to translate and yaw on whatever the
last packet said, indefinitely. A vehicle that has lost its operator should not still be
driving somewhere.

**Suggested fix:** zero the pilot setpoints on failsafe entry, and add a
`MANUAL_CONTROL` timeout (~0.5-1 s) that decays them to neutral.

---

## 4. The GCS failsafe does not monitor the companion specifically

```cpp
// mav_commands.cpp:615-621 — fed by ANY heartbeat that is not the vehicle's own
```

Two consequences on our topology:

- Under any routed setup, a router's or QGC's own heartbeat keeps the failsafe satisfied
  **even if the Jetson has died**.
- The Bondor LoRa bridge **synthesises its own 255/190 GCS heartbeat** as slot filler
  (`srot-ground-station/src/groundstation/main.cpp:91,140-143`). So on a dual-link vehicle,
  the LoRa link alone will hold the failsafe open with a dead companion.

Also: the failsafe only acts while armed (`:534`) and forces SURFACE **without disarming**,
so a permanent link loss leaves the vehicle armed and station-keeping at 0 m indefinitely.

**Suggested fix:** track liveness per `(sysid, compid)` and let a parameter name which
source is the *required* one (a `FS_GCS_SYSID`), defaulting to current behaviour.

---

## 5. `SET_MESSAGE_INTERVAL` (511) — already #3 on the roadmap, please keep it there

`ATTITUDE` at a fixed 10 Hz is the ceiling for every host-side loop we run, and our vision
loop is 20 Hz. We cannot turn it up for a control loop or down for the LoRa link. The
absence of both 511 and `REQUEST_DATA_STREAM` (66) means there is no standard mechanism at
all. `ROADMAP.md:302-303` already ranks this correctly — this is a vote, not a new finding.

Related: **no `COMMAND_CANCEL`.** The only way to cancel a move is to send another
`SROT_MOVE` — and per item 2, sending `STOP` does not actually stop.

---

## 6. `MV_STATE` is off by one past 5, so a `HOLD` leg reads as "done"

```cpp
// movement.cpp:12
enum { PH_IDLE=0, PH_CRUISE, PH_BRAKE, PH_TURN, PH_DIVE, PH_STYLE, PH_HOLD, PH_DONE };
```

`HOLD = 6`, `DONE = 7`. Both `JETSON_COMMS.md:277` and `DUBURI_WS_INTEGRATION.md:217`
document **`6 = done`**. A client decoding `MV_STATE` reads every station-keep leg as
complete — and station-keep is precisely what a companion holds a position with while it
aligns a camera or fires a payload.

Related, same area: **`progress()` returns a constant `0.5` for `PH_TURN` and `PH_DIVE`**
(`:204-214` — both fall to `default`). Those are the two verbs a mission most wants to wait
on, so `MV_PROG` and `COMMAND_ACK.progress` carry no information exactly where they matter.

**Suggested fix:** correct the two docs (cheapest), and give TURN/DIVE a real progress
fraction from the angle/depth remaining — both are already computed for the completion test.

---

## 7. `DIVE` — `p3` ignored, and the target is not clamped ≥ 0 despite the docs

`JETSON_COMMS.md:247` states DIVE `p2` is *"**Clamped at ≥ 0** — you cannot command above
the surface."* It is not:

- `movement.cpp:84` is a bare `s_depth_goal = primary;`
- `depth_control.cpp:32` clamps only inside the `fabsf(stick_throttle) > STICK_DEADBAND`
  branch, and AUTO calls `depth::update(0, ...)` (`task_control_loop.cpp:206`), so that
  branch never runs
- `depth::setTarget()` (`depth_control.cpp:20`) has no clamp

A negative DIVE target therefore produces sustained ascent — and because the runaway guard
uses `d0 = depth::target()` (`task_control_loop.cpp:459`), **the guard tracks the bad
setpoint instead of catching it**.

We do reject `target > 0` host-side, so this is defence-in-depth rather than a live bug for
us. But it is a documented safety property that isn't there, on the one loop that has
**never run closed** (`AUDIT.md:35-73`) — worth closing on principle.

Separately, `DIVE`'s `p3` is ignored (rate is always `MOVE_DEPTH_RATE`). `ROADMAP.md:306`
already has the depth-rate loop as #5; that would make `p3` meaningful.

---

## 8. `ESC_STATUS` (291) cannot be decoded by a standard pymavlink companion

This one is not a firmware defect, but it silently costs you the entire RPM feature on the
companion, and Bondor was bitten by the same thing from the other direction.

Upstream MAVLink **removed the WIP messages 290/291 from `common`**. We verified on
pymavlink 2.4.49 that msgid 291 is absent from *every* dialect we can load
(`common`, `ardupilotmega`, `all`, `development`). pymavlink **silently discards** any
message whose id is missing from its CRC-extra table — no error, no callback. So the board
can be reporting per-thruster RPM perfectly while the companion reads nothing, and it looks
exactly like an ESC or Bluejay fault.

Bondor hit the identical class of bug and documented it well
(`srot-ground-station/bondor/src/main/mavlink/connection.ts:40-51`).

**Suggested fix:** emit **`ESC_TELEMETRY_1_TO_4` (11030)** and **`ESC_TELEMETRY_5_TO_8`
(11031)** instead of, or alongside, `ESC_STATUS`. They are in the `ardupilotmega` dialect
today, carry exactly 4 ESCs each — a precise fit for the 8 thrusters — and they also have
the temperature/voltage/current fields that `ESC_STATUS` is currently sending as zeros
(`mav_stream.cpp:155`).

We have added the 11030/11031 read path on our side already, so this works the moment the
board sends it. Until then `/duburi/esc_rpm` cannot be built.

---

## 9. Round-3 notes (`8cb4203`) — two things worth flagging back

- **R14 (NVS too small) has a consequence for every companion that sets a parameter.**
  Because writes silently failed once the partition filled, we must assume our
  `JS_GAIN_DEFAULT = 1.0` write **never persisted** — and `pilotGain()` is runtime-only and
  only *lazily* adopts the param (`mav_commands.cpp:176-182`), so `MANUAL_CONTROL` may have
  been running at half authority throughout. It would be very useful if `PARAM_SET`'s
  `STATUSTEXT` on a failed persist were **`MAV_SEVERITY_ERROR`** rather than a warning — a
  companion cannot see a failed write any other way. (The R10 `"set but NOT saved"` text
  exists; making it loud is the ask.)
- **R15 (role-2 channels unreachable) is a genuinely good fix** and it resolved an open
  question on our side — we no longer have to know whether a payload channel is a servo or a
  MOSFET to command it, since `DO_SET_SERVO` now treats ≥1500 µs as ON for role 2. Thank you.
- **R16 (PM1 non-linear ADC)** explains a reading we saw on the bench and could not account
  for: `battery_voltage: 3.35 V` on a live pack. Worth noting in `PARAMETERS.md` that any
  `PM1_VMULT` from before this commit is in the old units.

---

## 10. Documentation drift (cheap to fix, and it actively misled us)

We built our backend from the docs first and the code second. These are the places that
cost us time, all verified against `8cb4203`:

| Claim | Where | Reality |
|---|---|---|
| "SROT presents as ArduSub 4.1.0" | `ARCHITECTURE.md:134` | `MAV_AUTOPILOT_GENERIC` (`mav_stream.cpp:178`); AUDIT B10 deleted the spoof |
| "Every command reaches exactly one terminal result" | `JETSON_COMMS.md:157` | Item 1 above |
| "stop … (brake to a halt)" | `JETSON_COMMS.md:108` | Item 2 above |
| "DIVE p2 … Clamped at ≥ 0" | `JETSON_COMMS.md:247` | Item 7 above |
| "6 = done" for `MV_STATE` | `JETSON_COMMS.md:277`, `DUBURI_WS_INTEGRATION.md:217` | `HOLD=6`, `DONE=7` |
| "a new command preempts the running one (a quick brake first)" | `JETSON_COMMS.md:86` | `task_control_loop.cpp:398-402` calls `movement::start()` with no `abort()` between — no null-momentum phase |
| `RPM_LOOP` default 1, `RPM_MAX` 4000, `MOT_SPIN_MIN` 0.10 | `PARAMETERS.md:105-109,135` | 0, 3600, 0.02 (`config.h:380,445,464`) |
| ~~"constant distance comes from the Pico RPM closed loop (now enabled)"~~ **FIXED 2026-08-03** | `PARAMETERS.md`, `ALGORITHMS.md §11.1`, `ARCHITECTURE.md` | You were right, and it was worse than a wrong mechanism: `RPM_LOOP`, `THR_TRIM_EN` **and** `MOT_BAT_V_MAX` are all 0 at defaults, so **no** voltage compensation is active and a timed move really does travel further on a full pack. All three docs now say so and list the two supported routes. |
| "191 params" / "190+" / "113+26+80" | `mav_commands.cpp:48`, `JETSON_COMMS.md:231`, `PARAMETERS.md:11` | **227** (147 scalar rows incl. 26 CAL + 80 servo) |
| "UART2 … 2 Mbaud" | `HARDWARE.md:218`, `thruster_link_proto.h:7`, `pico/main.cpp:6` | 1 Mbaud (`config.h:191`) |
| `docs/T200_PROFILE.md` | referenced by `docs/ESC_FLASHING.md` step 3 and by the flasher repo | **file does not exist** |

Also: the **fifth** dispatch outcome, `MAV_RESULT_TEMPORARILY_REJECTED` (state-mutex miss at
`mav_commands.cpp:287`, `:376`, and now `:422` on the payload path), is not in the
four-result table in `JETSON_COMMS.md:164-172`. A client that treats only the documented
four as terminal will wait out its full deadline on a command that was simply refused. We
found this by reading `dispatchCommand()`, not the docs.

---

## 11. For v3 — the one architectural ask → **now a spec: [`VISION_API.md`](VISION_API.md)**

> **Update.** This ask has been written up as a full implementation spec —
> **[`VISION_API.md`](VISION_API.md)** — with the wire contract, the plumbing, the control law
> and its rationale, the safety requirements, bandwidth/latency budgets with the arithmetic
> shown, and a staged build order with a bench check per stage. Read that instead of this
> section; the paragraphs below are kept as the original motivation.
>
> **Two findings from writing it that you should know regardless:**
>
> 1. **The dialect already has everything.** `src/comms/mavlink_bridge.h:17` includes
>    `ardupilotmega`, which supersets `common`, so `mavlink_msg_landing_target.h`,
>    `vision_position_estimate`, `distance_sensor` and `odometry` **already compile into the
>    binary**. Ingest is one `case` in `mav_commands::handle()` — no custom message, no dialect
>    regeneration. This was the main thing we assumed would be expensive and isn't.
>
> 2. **`feedforward learn` is a latent bug for any autonomous mode.** It is gated on the pilot
>    sticks being centred (`task_control_loop.cpp:692-693`). A mode that never drives `sp_*` —
>    which is every autonomous mode, including a future vision mode — leaves `learn`
>    permanently **true**, so the CoB auto-trim learns against machine-commanded effort. AUTO
>    is already deliberately excluded from trim learning for exactly this reason; the exclusion
>    should probably be "any non-pilot mode" rather than an enumerated list.

**A position/velocity ingest path.** There is no estimator (`docs/VS_ARDUSUB.md:100-103`
is admirably direct about this) and, more importantly, no *consumer* for one:
no `VISION_POSITION_ESTIMATE`, no `DISTANCE_SENSOR`, no `ODOMETRY`, no `GPS_INPUT`.

The consequence is structural rather than a bug: **every DVL and vision loop is permanently
host-side**, closed over a 10 Hz `ATTITUDE` and a USB link, while the board runs a 500 Hz
loop with the actual thruster authority. That is the wrong place to close a position loop,
and it is the main reason our vision alignment fights inertia.

We are not asking for an estimator. We are asking for the **ingest side** — accept a
position/velocity estimate from the companion and let the board close the loop on it (the
existing `POSITION_HOLD` idea in Phase 3 is exactly the shape). Even accepting
`DISTANCE_SENSOR` for a downward-facing altitude would unlock bottom-following.

Whatever the answer, it is worth making it a **deliberate decision** recorded in
`ARCHITECTURE.md` rather than an emergent property — right now the split is implicit, and
we keep re-deriving it.

---

## What we changed on our side (so you know what is already absorbed)

| Item | Our mitigation |
|---|---|
| 1 (no terminal ACK) | ACK deadline derived from the leg's expected duration, not `p5` |
| 2 (STOP coasts) | Explicit reverse-leg brake before every `MOVE_STOP`; drift test fails when you fix it |
| 5th ACK result | `TEMPORARILY_REJECTED` treated as terminal, reported as retryable |
| 7 (DIVE clamp) | `set_depth` target > 0 refused host-side |
| 8 (`ESC_STATUS`) | 11030/11031 read path added, ready when the board sends it |
| Depth sign | Fixed a double negation on our side; `VFR_HUD.alt` is correct as sent |

We also added `test_srot_protocol_drift.py`, which reads **this repo's headers directly**
and fails if `MAV_CMD_SROT_MOVE`, the `movement::Type` ordering, the `FlightMode` ints,
`PCA_RELAY_BASE_CH`, `MAVLINK_BAUD` or `GCS_FAILSAFE_MS` ever diverge from our copy. It
skips cleanly when this repo isn't checked out. If you renumber something deliberately,
that test is what will tell us.

Happy to take any of the above as PRs against this repo instead of issues — say which.

---

# Round 6 reply — 2026-08-01, against `b213e50` / `SROT_FW_BEHAVIOUR_REV 2`

Written after reading `FIRMWARE_CHANGELOG_FOR_DUBURI.md`, `AUDIT.md` R35–R45, and merging
your PR into `duburi_ws`. **Nine of eleven answered, and the two that were not are the two
that should not have been.** §4 you correctly refused to fake, §11 you deferred by agreement.

Both host-side workarounds are gone: `_brake_last_leg` removed (§2), `_ack_budget_s`
demoted to a backstop (§1). Rate pinning is re-enabled, `move_back` un-refused,
`/duburi/esc_rpm` built on 11030/11031.

## Verified on our side

`payload_fire_map` and the MAVLink-2 binding were both real, and we reproduced both:

- **The dialect trap is worse than "srot_fc didn't set it".** Measured here:
  pymavlink's default is `dialects.v10.ardupilotmega`, where `11030 not in mavlink_map`;
  only v20 has it. It worked *by import order alone* — `duburi_control/__init__` reaches
  `pixhawk.py`, which sets `MAVLINK20` at line 44. Anything importing `fc.srot_fc` alone
  got MAVLink 1. Good catch, and it was found in-vehicle rather than by reading, which is
  the only way it could have been found.
- **`ESC_STATUS` 291 is in neither dialect** — re-confirmed. Your decision to emit
  `ESC_TELEMETRY_*` alongside is what makes RPM reachable at all.

## Three gaps we closed after the PR (all in `srot_protocol.py`, our one copy)

Not criticism — they are ours to maintain, and your changelog is what surfaced two of them.

1. **`MV_STATE` was still `6 = done` here.** Corrected against `movement.cpp:12` —
   `PH_HOLD = 6`, `PH_DONE = 7`. It never stuck a mission (the terminal ACK is the
   authority, exactly as your doc now says) which is precisely why it survived.
2. **`STUNT` (100) / `PATTERN` (101) were missing from `MODE_NAMES`**, so they surfaced as
   `UNKNOWN(100)` on `/duburi/state` — which reads like a comms fault rather than a real
   mode moving the hull. Added, but excluded from `mode_int()`: `DO_SET_MODE` cannot enter
   them, so accepting one would burn our 8 s poll and report a misleading
   "mode stayed STABILIZE".
3. **Our drift test was one-directional** — it compared the modes we already declare, so it
   stayed green through both additions above. It now asserts the firmware enum **as a set**,
   plus the phase enum.

That third one is the same shape as the `MOVE_STOP` miss you diagnosed, and it generalises:
**a tripwire that encodes our prediction of your code tests us, not you.** Your
`SROT_FW_BEHAVIOUR_REV`-on-the-wire is the right correction and we have adopted it as
specified — read at connect *and* inside `arm()`, fail-closed on a known-old rev, loud but
permissive on silence.

## Your three open decisions — answered

### 1. Companion component id → **yes, we will move to 191**

`MAV_COMP_ID_ONBOARD_COMPUTER`. Cheaper on our side than you assumed: `SOURCE_SYSID` /
`SOURCE_COMPID` in `srot_protocol.py` were *declared but never applied* — the connection
uses pymavlink's 255/190 default — so it is one constant plus one `source_component=`
kwarg, and nothing else in our tree keys on 190.

**Please build `FS_GCS_SYSID` / `FS_GCS_COMPID`.** Your framing is right: a dead Jetson
with Bondor connected currently holds the failsafe open, and the vehicle station-keeps when
it should surface. Prefer this over keying on the physical link — link-keying cannot
distinguish two companions, and we would rather the failsafe watch an *identity* than a
cable. We will land 191 on our side first so the parameter has something true to point at.

### 2. LEAK off `NAMED_VALUE_FLOAT` → **yes, `SYS_STATUS` health bits**

pymavlink caches one message per msgid, so with `MV_STATE`/`LEAK`/`WTEMP`/`GAIN`
multiplexed onto one, whether we ever observe a leak depends on arrival order. That is
probabilistic detection of a flooding hull, which is not a tradeoff worth having. Sensor
health bits are the right carrier — they are a latched state, not a sample, which is what
a leak actually is.

### 3. `arc` and `style_yaw` → **defer, and here is the check**

We grepped rather than guessed. `arc` appears only in `missions/demo_arc.py`; `style_yaw`
only in `missions/robosub_gate_rescue.py`. **No 2026 competition task chunk calls either.**
Not worth firmware time ahead of the vision API. If `arc` does land later we want the
absolute-heading form, since ours holds a heading while curving — but that is a
nice-to-have, not a blocker.

## One correction to the changelog, because the nuance matters

> "**No 2026 mission calls a distance verb.**"

True for the **competition path** and we confirmed it: the `task_*.py` chunks and the five
2026 FSM plans (`slalom`, `bin_drop`, `torpedo_fire`, `return_gate`, `full_competition`)
call **zero** distance verbs. But `move_forward_dist` *is* called by
`gate_flare_prequal.py`, `gate_prequal.py` and `gate_flare_autonomous.py`, and `distance_m`
is passed by the `gate_flare`, `prequal` and `gate_then_bin` FSM plans.

Those are legacy/prequal runs, not 2026 chunks — so your conclusion (distance verbs stay
refused) is unchanged and correct. Worth stating precisely because your PR's
`has_distance_moves` gate now makes those plans **fail loudly on srot instead of silently**,
which is the right behaviour and a real improvement, but it is a behaviour change someone
will meet on pool day if we record it as "nothing calls these".

## DVL documentation — reconciled, as you asked

You were right that six of our documents disagreed, and that it was a planning blocker.
The authoritative statement, now propagated:

> **The Nucleus1000 driver code exists and is unit-tested** (`nucleus_dvl.py`,
> `nucleus_parser.py`: TCP auth, AHRS heading, bottom-track integration, backoff reconnect).
> **It has never been validated in water, it is not fitted to the competition body, and it
> is not on the SROT wire at all.** `Dubomini 2.0` has no DVL. Treat DVL-derived distance as
> **unavailable** for 2026 planning.

So of your three options for unblocking distance: **(1) a hardware flow sensor on UART1 is
the one we would pursue**, and `OPTICAL_FLOW_RAD` (106) is the right ingest for exactly the
reason you give — it carries the gyro integral over the *same* window as the flow, which is
the concurrency a 10 Hz `ATTITUDE` stream over USB structurally destroys. Not before the
vision API, though.

## Also on our side this round

The **ESP32-C3 + BNO085 USB board has been removed from the hull** — so your
`auv-architecture-2026.md` inference was correct, and we can now confirm the half you said
you could not see. `bringup.launch.py` no longer defaults `yaw_source:=dvl`; it defaults to
`mavlink_ahrs`, which reads your fused `ATTITUDE`. `duburi_sensors` stays in the tree as a
fallback but nothing selects it, so its 50 Hz reader thread and 5 s boot calibration stop
costing the Orin anything.

## Still blocked on us, not you

**Camera FOV.** You were right that it does not exist anywhere in our repo and that it is
the critical path for `VISION_API.md`. It is a bench measurement, and it is ours to make.
Nothing downstream of it can be built honestly until it is — a guessed FOV would put a
constant scale error inside the board's gains, where it would read as a tuning problem
rather than a units problem.

⛔ **The depth loop still has never run closed**, on either side. Unchanged gate.
